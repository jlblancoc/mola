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
 * @file   NavStateFuse.h
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

// this package:
#include <mola_navstate_fuse/NavState.h>
#include <mola_navstate_fuse/NavStateFuseParams.h>

// MOLA:
#include <mola_imu_preintegration/RotationIntegrator.h>
#include <mola_kernel/factors/FactorConstVelKinematics.h>

// MRPT:
#include <mrpt/containers/bimap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/COutputLogger.h>

// std:
#include <mutex>
#include <optional>

namespace mola
{
/** Sliding window Factor-graph data fusion for odometry, IMU, GNNS, and SE(3)
 * pose/twist estimations.
 *
 * Frame conventions:
 * - There is a frame of reference for each source of odometry, e.g.
 *   there may be one for LiDAR-odometry, another for visual-odometry, or
 *   wheels-based odometry, etc. Each such frame is referenced with a "frame
 *   name" (an arbitrary string).
 * - Internally, the first frame of reference will be used as "global"
 *   coordinates, despite it may be actually either a `map` or `odom` frame, in
 *   the [ROS REP 105](https://www.ros.org/reps/rep-0105.html) sense.
 * - IMU readings are, by definition, given in the robot body frame, although
 *   they can have a relative transformation between the vehicle and sensor.
 *
 * Main API methods and frame conventions:
 * - `estimated_navstate()`: Output estimations can be requested in any of the
 *    existing frames of reference.
 * - `fuse_pose()`: Can be used to integrate information from any "odometry" or
 *   "localization" input, as mentioned above.
 * - `fuse_gnns()`: TO-DO.
 * - `fuse_imu()`: TO-DO.
 *
 * Usage:
 * - (1) Call initialize() or set the required parameters directly in params_.
 * - (2) Integrate measurements with `fuse_*()` methods. Each CObservation
 *       class includes a `timestamp` field which is used to estimate the
 *       trajectory.
 * - (3) Repeat (2) as needed.
 * - (4) Read the estimation up to any nearby moment in time with
 *       estimated_navstate()
 *
 * Old observations are automatically removed.
 *
 * A constant SE(3) velocity model is internally used, without any
 * particular assumptions on the vehicle kinematics.
 *
 * For more theoretical descriptions, see the papers cited in
 * https://docs.mola-slam.org/latest/
 *
 * \ingroup mola_navstate_fuse_grp
 */
class NavStateFuse : public mrpt::system::COutputLogger
{
   public:
    NavStateFuse();
    ~NavStateFuse() = default;

    /** \name Main API
     *  @{ */

    NavStateFuseParams params_;

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const mrpt::containers::yaml& cfg);

    /** Resets the estimator state to an initial state.
     *  \sa currentIntegrationState
     */
    void reset();

    /** Integrates new SE(3) pose estimation of the vehicle wrt frame_id
     */
    void fuse_pose(
        const mrpt::Clock::time_point&         timestamp,
        const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string&                     frame_id);

    /** Integrates new wheels-based odometry observations into the estimator.
     *  This is a convenience method that internally ends up calling
     *  fuse_pose(), but computing the uncertainty of odometry increments
     *  according to a given motion model.
     */
    void fuse_odometry(
        const mrpt::obs::CObservationOdometry& odom,
        const std::string&                     odomName = "odom_wheels");

    /** Integrates new IMU observations into the estimator */
    void fuse_imu(const mrpt::obs::CObservationIMU& imu);

    /** Integrates new twist estimation (in the odom frame) */
    void fuse_twist(
        const mrpt::Clock::time_point& timestamp,
        const mrpt::math::TTwist3D&    twist);

    /** Computes the estimated vehicle state at a given timestep using the
     * observations in the time window. A std::nullopt is returned if there is
     * no valid observations yet, or if requested a timestamp out of the model
     * validity time window (e.g. too far in the future to be trustful).
     */
    std::optional<NavState> estimated_navstate(
        const mrpt::Clock::time_point& timestamp);

#if 0
    std::optional<mrpt::math::TTwist3D> get_last_twist() const
    {
        return state_.last_twist;
    }

    void force_last_twist(const mrpt::math::TTwist3D& twist);
#endif

    /** @} */

   private:
    // everything related to gtsam is hidden in the public API via pimpl
    struct GtsamImpl;

    using frameid_t = uint8_t;

    // an observation from fuse_pose()
    struct PoseData
    {
        PoseData() = default;

        mrpt::poses::CPose3DPDFGaussian pose;
        frameid_t                       frameId;
    };

    // Dummy type representing the query point.
    struct QueryPointData
    {
        QueryPointData() = default;
    };

    struct PointData
    {
        PointData(const PoseData& p) : pose(p) {}
        PointData(const QueryPointData& p) : query(p) {}

        std::optional<PoseData>       pose;
        std::optional<QueryPointData> query;

       private:
        PointData() = default;
    };

    struct State
    {
        State();
        ~State();

        mrpt::pimpl<GtsamImpl> impl;

        /// A bimap of known "frame_id" <=> "numeric IDs":
        mrpt::containers::bimap<std::string, frameid_t> known_frames;

        /// Returns the existing ID, or creates a new ID, for a frame:
        frameid_t frame_id(const std::string& frame_name);

        /// The sliding window of observation data:
        std::multimap<mrpt::Clock::time_point, PointData> data;

#if 0
        std::optional<mrpt::Clock::time_point>         last_pose_obs_tim;
        std::optional<mrpt::obs::CObservationOdometry> last_odom_obs;
        std::optional<mrpt::poses::CPose3DPDFGaussian> last_pose;
        std::optional<mrpt::math::TTwist3D>            last_twist;
#endif
    };

    State                state_;
    std::recursive_mutex state_mtx_;  /// to access state_

    void build_and_optimize_fg(const mrpt::Clock::time_point queryTimestamp);

    /// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
    void addFactor(const mola::FactorConstVelKinematics& f);
};

}  // namespace mola
