/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   NavStateFuse.cpp
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

// MOLA & MRPT:
#include <mola_navstate_fuse/NavStateFuse.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

// Custom factors:
#include "FactorAngularVelocityIntegration.h"
#include "FactorConstAngularVelocity.h"
#include "FactorTrapezoidalIntegrator.h"

using namespace mola;

using gtsam::symbol_shorthand::F;  // Frame of reference origin pose (Pose3)
using gtsam::symbol_shorthand::P;  // Position                       (Point3)
using gtsam::symbol_shorthand::R;  // Rotation                       (Rot3)
using gtsam::symbol_shorthand::V;  // Lin velocity                   (Point3)
using gtsam::symbol_shorthand::W;  // Ang velocity                   (Point3)

// -------- GtsamImpl -------

struct NavStateFuse::GtsamImpl
{
    GtsamImpl()  = default;
    ~GtsamImpl() = default;

    gtsam::NonlinearFactorGraph fg;
    gtsam::Values               values;
};

// -------- NavStateFuse::State -------
NavStateFuse::State::State() : impl(mrpt::make_impl<NavStateFuse::GtsamImpl>())
{
}

NavStateFuse::frameid_t NavStateFuse::State::frame_id(
    const std::string& frame_name)
{
    if (auto it = known_frames.find_key(frame_name);
        it != known_frames.getDirectMap().end())
    {
        return it->second;
    }
    else
    {
        const auto newId = static_cast<frameid_t>(known_frames.size());
        known_frames.insert(frame_name, newId);
        return newId;
    }
}

// -------- NavStateFuse -------
NavStateFuse::NavStateFuse()
{
    this->mrpt::system::COutputLogger::setLoggerName("NavStateFuse");
}

void NavStateFuse::initialize(const mrpt::containers::yaml& cfg)
{
    reset();

    // Load params:
    params_.loadFrom(cfg);
}

void NavStateFuse::reset()
{
    // reset:
    state_ = State();
}

void NavStateFuse::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, const std::string& odomName)
{
    THROW_EXCEPTION("TODO");
#if 0
    // temporarily, this will work well only for simple datasets:
    if (state_.last_odom_obs && state_.last_pose)
    {
        const auto poseIncr = odom.odometry - state_.last_odom_obs->odometry;

        state_.last_pose->mean =
            state_.last_pose->mean + mrpt::poses::CPose3D(poseIncr);

        // and discard velocity-based model:
        state_.last_twist = mrpt::math::TTwist3D(0, 0, 0, 0, 0, 0);
    }
    // copy:
    state_.last_odom_obs = odom;
#endif
}

void NavStateFuse::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    THROW_EXCEPTION("TODO");
    (void)imu;
}

void NavStateFuse::fuse_pose(
    const mrpt::Clock::time_point&         timestamp,
    const mrpt::poses::CPose3DPDFGaussian& pose, const std::string& frame_id)
{
    auto lck = mrpt::lockHelper(state_mtx_);

    // numerical sanity:
    for (int i = 0; i < 6; i++) ASSERT_GT_(pose.cov(i, i), .0);

    PoseData d;
    d.frameId = state_.frame_id(frame_id);
    d.pose    = pose;

    state_.data.insert(std::pair(timestamp, d));
}

void NavStateFuse::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist)
{
    (void)timestamp;
    THROW_EXCEPTION("TODO");
}

#if 0
void NavStateFuse::force_last_twist(const mrpt::math::TTwist3D& twist)
{
    state_.last_twist = twist;
}
#endif

std::optional<NavState> NavStateFuse::estimated_navstate(
    const mrpt::Clock::time_point& timestamp)
{
    build_and_optimize_fg(timestamp);

    NavState ret;
    return ret;
}

void NavStateFuse::build_and_optimize_fg(
    const mrpt::Clock::time_point queryTimestamp)
{
    auto lck = mrpt::lockHelper(state_mtx_);

    if (state_.data.empty() || state_.known_frames.empty()) return;

    state_.impl->values.clear();
    state_.impl->fg = {};

    // Build the sequence of time points:
    // FG variable indices will use the indices in this vector:
    auto itQuery = state_.data.insert({queryTimestamp, QueryPointData()});

    using map_it_t =
        std::multimap<mrpt::Clock::time_point, PointData>::value_type;

    std::vector<const map_it_t*> entries;
    for (const auto& it : state_.data) entries.push_back(&it);

    // add const vel kinematic factors between consecutive KFs:
    if (entries.size() >= 2)
    {
        for (size_t i = 1; i < entries.size(); i++)
        {
            mola::FactorConstVelKinematics f;
            f.from_kf_   = i - 1;
            f.to_kf_     = i;
            f.deltaTime_ = mrpt::system::timeDifference(
                entries[i - 1]->first, entries[i]->first);

            addFactor(f);
        }
    }

    // Init values:
    for (size_t i = 0; i < entries.size(); i++)
    {
        state_.impl->values.insert<gtsam::Point3>(P(i), gtsam::Z_3x1);
        state_.impl->values.insert<gtsam::Rot3>(R(i), gtsam::Rot3::Identity());
        state_.impl->values.insert<gtsam::Point3>(V(i), gtsam::Z_3x1);
        state_.impl->values.insert<gtsam::Point3>(W(i), gtsam::Z_3x1);
    }
    for (const auto& [frameName, frameId] : state_.known_frames.getDirectMap())
    {
        state_.impl->values.insert<gtsam::Pose3>(
            F(frameId), gtsam::Pose3::Identity());
    }
    // Unary prior for first frameId only:
    state_.impl->fg.addPrior(F(0 /*first one*/), gtsam::Pose3::Identity());

#if 1
    state_.impl->fg.print();
    state_.impl->values.print();
#endif

    gtsam::LevenbergMarquardtOptimizer lm(state_.impl->fg, state_.impl->values);

    const auto& optimal = lm.optimize();

#if 1
    optimal.print("Optimized:");
#endif

    // delete temporary entry:
    state_.data.erase(itQuery);
}

/// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
void NavStateFuse::addFactor(const mola::FactorConstVelKinematics& f)
{
    MRPT_LOG_DEBUG_STREAM(
        "[addFactor] FactorConstVelKinematics: "
        << f.from_kf_ << " ==> " << f.to_kf_ << " dt=" << f.deltaTime_);

    // Add const-vel factor to gtsam itself:
    const double dt = f.deltaTime_;
    ASSERT_GE_(dt, 0.);

    // errors in constant vel:
    const double std_linvel = params_.sigma_random_walk_acceleration_linear;
    const double std_angvel = params_.sigma_random_walk_acceleration_angular;

    auto noise_linVelModel =
        gtsam::noiseModel::Isotropic::Sigma(3, std_linvel * dt);
    auto noise_angVelModel =
        gtsam::noiseModel::Isotropic::Sigma(3, std_angvel * dt);

    if (dt > params_.time_between_frames_to_warning)
    {
        MRPT_LOG_WARN_FMT(
            "A constant-velocity kinematics factor has been added for a "
            "dT=%.03f s.",
            dt);
    }

    // 1) Add GTSAM factors for constant velocity model
    // -------------------------------------------------

    auto Pi  = gtsam::Point3_(P(f.from_kf_));
    auto Pj  = gtsam::Point3_(P(f.to_kf_));
    auto Ri  = gtsam::Rot3_(R(f.from_kf_));
    auto Rj  = gtsam::Rot3_(R(f.to_kf_));
    auto Vi  = gtsam::Point3_(V(f.from_kf_));
    auto Vj  = gtsam::Point3_(V(f.to_kf_));
    auto bWi = gtsam::Point3_(W(f.from_kf_));
    auto bWj = gtsam::Point3_(W(f.to_kf_));

    const auto kPi  = P(f.from_kf_);
    const auto kPj  = P(f.to_kf_);
    const auto kVi  = V(f.from_kf_);
    const auto kVj  = V(f.to_kf_);
    const auto kRi  = R(f.from_kf_);
    const auto kRj  = R(f.to_kf_);
    const auto kbWi = W(f.from_kf_);
    const auto kbWj = W(f.to_kf_);

    // See line 3 of eq (4) in the MOLA RSS2019 paper
    state_.impl->fg.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(
        kVi, kVj, gtsam::Z_3x1, noise_linVelModel);

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    state_.impl->fg.emplace_shared<FactorConstAngularVelocity>(
        kRi, kbWi, kRj, kbWj, noise_angVelModel);

    // 2) Add kinematics / numerical integration factor
    // ---------------------------------------------------
    auto noise_kinematicsPosition = gtsam::noiseModel::Isotropic::Sigma(
        3, params_.sigma_integrator_position);

    auto noise_kinematicsOrientation = gtsam::noiseModel::Isotropic::Sigma(
        3, params_.sigma_integrator_orientation);

    // Impl. line 2 of eq (1) in the MOLA RSS2019 paper
    state_.impl->fg.emplace_shared<FactorTrapezoidalIntegrator>(
        kPi, kVi, kPj, kVj, dt, noise_kinematicsPosition);

    // Impl. line 1 of eq (4) in the MOLA RSS2019 paper.
    state_.impl->fg.emplace_shared<FactorAngularVelocityIntegration>(
        kRi, kbWi, kRj, dt, noise_kinematicsOrientation);
}
