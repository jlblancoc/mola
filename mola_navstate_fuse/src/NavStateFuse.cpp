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
#include <mrpt/core/get_env.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
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

const bool NAVSTATE_PRINT_FG = mrpt::get_env<bool>("NAVSTATE_PRINT_FG", false);

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
NavStateFuse::State::~State() = default;

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

NavStateFuse::~NavStateFuse() = default;

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

    delete_too_old_entries();
}

void NavStateFuse::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    THROW_EXCEPTION("TODO");
    (void)imu;

    delete_too_old_entries();
}

void NavStateFuse::fuse_pose(
    const mrpt::Clock::time_point&         timestamp,
    const mrpt::poses::CPose3DPDFGaussian& pose, const std::string& frame_id)
{
    // numerical sanity:
    for (int i = 0; i < 6; i++) ASSERT_GT_(pose.cov(i, i), .0);

    PoseData d;
    d.frameId = state_.frame_id(frame_id);
    d.pose    = pose;

    state_.data.insert({timestamp, d});

    delete_too_old_entries();
}

void NavStateFuse::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist)
{
    (void)timestamp;
    THROW_EXCEPTION("TODO");

    delete_too_old_entries();
}

#if 0
void NavStateFuse::force_last_twist(const mrpt::math::TTwist3D& twist)
{
    state_.last_twist = twist;
}
#endif

std::optional<NavState> NavStateFuse::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, const std::string& frame_id)
{
    return build_and_optimize_fg(timestamp, frame_id);
}

std::set<std::string> NavStateFuse::known_frame_ids()
{
    std::set<std::string> ret;
    for (const auto& [name, id] : state_.known_frames.getDirectMap())
        ret.insert(name);

    return ret;
}

std::optional<NavState> NavStateFuse::build_and_optimize_fg(
    const mrpt::Clock::time_point queryTimestamp, const std::string& frame_id)
{
    using namespace std::string_literals;

    // Return an empty answer if we don't have data, or we would need to
    // extrapolate too much:
    if (state_.data.empty() || state_.known_frames.empty()) return {};
    {
        const double tq_2_tfirst = mrpt::system::timeDifference(
            queryTimestamp, state_.data.begin()->first);
        const double tlast_2_tq = mrpt::system::timeDifference(
            state_.data.rbegin()->first, queryTimestamp);
        if (tq_2_tfirst > params_.max_time_to_use_velocity_model ||
            tlast_2_tq > params_.max_time_to_use_velocity_model)
            return {};
    }

    // shortcuts:
    auto& fg     = state_.impl->fg;
    auto& values = state_.impl->values;

    values.clear();
    fg = {};

    // Build the sequence of time points:
    // FG variable indices will use the indices in this vector:
    auto itQuery = state_.data.insert({queryTimestamp, QueryPointData()});

    using map_it_t =
        std::multimap<mrpt::Clock::time_point, PointData>::value_type;

    std::vector<const map_it_t*> entries;
    std::optional<size_t>        query_KF_id;
    for (const auto& it : state_.data)
    {
        if (it.first == itQuery->first) query_KF_id = entries.size();

        entries.push_back(&it);
    }
    ASSERT_(query_KF_id.has_value());

    // add const vel kinematic factors between consecutive KFs:
    ASSERT_(entries.size() >= 2);

    for (size_t i = 1; i < entries.size(); i++)
    {
        mola::FactorConstVelKinematics f;
        f.from_kf_   = i - 1;
        f.to_kf_     = i;
        f.deltaTime_ = mrpt::system::timeDifference(
            entries[i - 1]->first, entries[i]->first);

        addFactor(f);
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
        // F(0): this variable is not used.
        // We only need to estimate F(i), the SE(3) pose of the frame_id "i" wrt
        // "0" (see paper diagrams!)
        if (frameId == 0) continue;

        state_.impl->values.insert<gtsam::Pose3>(
            F(frameId), gtsam::Pose3::Identity());
    }

    // Unary prior for initial twist:
    const auto& tw = params_.initial_twist;
    fg.addPrior(
        V(0), gtsam::Vector3(tw.vx, tw.vy, tw.vz),
        gtsam::noiseModel::Isotropic::Sigma(
            3, params_.initial_twist_sigma_lin));

    fg.addPrior(
        W(0), gtsam::Vector3(tw.wx, tw.wy, tw.wz),
        gtsam::noiseModel::Isotropic::Sigma(
            3, params_.initial_twist_sigma_ang));

    // Process pose observations:
    // ------------------------------------------
    for (size_t kfId = 0; kfId < entries.size(); kfId++)
    {
        // const auto  tim = entries.at(kfId)->first;
        const auto& d = entries.at(kfId)->second;

        if (d.pose.has_value())
        {
            if (d.pose->frameId == 0)
            {
                // Pose observations in the first frame are just priors:
                // (see paper!)

                gtsam::Pose3   p;
                gtsam::Matrix6 pCov;
                mrpt::gtsam_wrappers::to_gtsam_se3_cov6(d.pose->pose, p, pCov);

                fg.addPrior(
                    P(kfId), p.translation(),
                    gtsam::noiseModel::Gaussian::Covariance(
                        pCov.block<3, 3>(3, 3)));

                fg.addPrior(
                    R(kfId), p.rotation(),
                    gtsam::noiseModel::Gaussian::Covariance(
                        pCov.block<3, 3>(0, 0)));
            }
            else
            {
                // Pose observations in subsequent frames are more complex:
                // (see paper!)
                THROW_EXCEPTION("todo");
            }
        }
    }

    // FG is built: optimize it
    // -------------------------------------
    if (NAVSTATE_PRINT_FG)
    {
        fg.print();
        state_.impl->values.print();
    }

    gtsam::LevenbergMarquardtOptimizer lm(fg, state_.impl->values);

    const auto& optimal = lm.optimize();

    MRPT_LOG_DEBUG_STREAM(
        "[build_and_optimize_fg] LM ran for "
        << lm.iterations() << " iterations, " << fg.size() << " factors, RMSE: "
        << std::sqrt(fg.error(state_.impl->values) / fg.size()) << " => "
        << std::sqrt(fg.error(optimal) / fg.size()));

    if (NAVSTATE_PRINT_FG)
    {
        optimal.print("Optimized:");
        std::cout << "\n query_KF_id: " << *query_KF_id << std::endl;
    }

    // Extract results from the factor graph:
    // ----------------------------------------------
    NavState out;

    // SE(3) pose:
    auto poseResult = gtsam::Pose3(
        optimal.at<gtsam::Rot3>(R(*query_KF_id)),
        optimal.at<gtsam::Point3>(P(*query_KF_id)));
    out.pose.mean =
        mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(poseResult));

    gtsam::Marginals marginals(fg, optimal);

    // Pose SE(3) cov: (in mrpt order is xyz, then yaw/pitch/roll):
    gtsam::Matrix6 cov_inv    = gtsam::Matrix6::Zero();
    cov_inv.block<3, 3>(0, 0) = marginals.marginalInformation(P(*query_KF_id));
    cov_inv.block<3, 3>(3, 3) = marginals.marginalInformation(R(*query_KF_id));
    out.pose.cov_inv          = cov_inv;

    // Twist:
    const auto linVel = optimal.at<gtsam::Vector3>(V(*query_KF_id));
    const auto angVel = optimal.at<gtsam::Vector3>(W(*query_KF_id));
    out.twist.vx      = linVel.x();
    out.twist.vy      = linVel.y();
    out.twist.vz      = linVel.z();
    out.twist.wx      = angVel.x();
    out.twist.wy      = angVel.y();
    out.twist.wz      = angVel.z();

    // Twist cov:
    gtsam::Matrix6 tw_cov_inv = gtsam::Matrix6::Zero();
    tw_cov_inv.block<3, 3>(0, 0) =
        marginals.marginalInformation(V(*query_KF_id));
    tw_cov_inv.block<3, 3>(3, 3) =
        marginals.marginalInformation(W(*query_KF_id));
    out.twist_inv_cov = tw_cov_inv;

    // delete temporary entry:
    state_.data.erase(itQuery);

    // Honor requested frame_id:
    // ----------------------------------
    ASSERTMSG_(
        state_.known_frames.hasKey(frame_id),
        "Requested results in unknown frame_id: '"s + frame_id + "'"s);

    // if this is the first frame_id, we are already done, otherwise, recover
    // and apply the transformation:
    if (const frameid_t frameId = state_.known_frames.direct(frame_id);
        frameId != 0)
    {
        // Apply F(frameId) transformation on the left:
        const auto T = optimal.at<gtsam::Pose3>(F(frameId));

        THROW_EXCEPTION("TODO");
    }

    return out;
}

/// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
void NavStateFuse::addFactor(const mola::FactorConstVelKinematics& f)
{
    MRPT_LOG_DEBUG_STREAM(
        "[addFactor] FactorConstVelKinematics: "
        << f.from_kf_ << " ==> " << f.to_kf_ << " dt=" << f.deltaTime_);

    // Add const-vel factor to gtsam itself:
    double dt = f.deltaTime_;

    // trick to easily handle queries on exactly an existing keyframe:
    if (dt == 0) dt = 1e-5;

    ASSERT_GT_(dt, 0.);

    // errors in constant vel:
    const double std_linvel = params_.sigma_random_walk_acceleration_linear;
    const double std_angvel = params_.sigma_random_walk_acceleration_angular;

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
        kVi, kVj, gtsam::Z_3x1,
        gtsam::noiseModel::Isotropic::Sigma(3, std_linvel * dt));

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    state_.impl->fg.emplace_shared<FactorConstAngularVelocity>(
        kRi, kbWi, kRj, kbWj,
        gtsam::noiseModel::Isotropic::Sigma(3, std_angvel * dt));

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

void NavStateFuse::delete_too_old_entries()
{
    if (state_.data.empty()) return;

    const double newestTime =
        mrpt::Clock::toDouble(state_.data.rbegin()->first);
    const double minTime = newestTime - params_.sliding_window_length;

    for (auto it = state_.data.begin(); it != state_.data.end();)
    {
        const double t = mrpt::Clock::toDouble(it->first);
        if (t < minTime)
        {
            // remove it:
            it = state_.data.erase(it);
        }
        else { ++it; }
    }
}
