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

#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_navstate_fuse/NavStateFuseParams.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <optional>

namespace mola
{
/** Fuse of odometry, IMU, and SE(3) pose/twist estimations.
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
 * \note This implementation of mola::NavStateFilter ignores the passed
 *       "frame_id"
 *
 * \sa IMUIntegrator
 * \ingroup mola_imu_preintegration_grp
 */
class NavStateFuse : public mola::NavStateFilter

{
   public:
    NavStateFuse()  = default;
    ~NavStateFuse() = default;

    /** \name Main API
     *  @{ */

    NavStateFuseParams params_;

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const mrpt::containers::yaml& cfg) override;

    /** Resets the estimator state to an initial state.
     *  \sa currentIntegrationState
     */
    void reset() override;

    /** Integrates new SE(3) pose estimation of the vehicle wrt frame_id
     */
    void fuse_pose(
        const mrpt::Clock::time_point&         timestamp,
        const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string&                     frame_id) override;

    /** Integrates new wheels-based odometry observations into the estimator.
     *  This is a convenience method that internally ends up calling
     *  fuse_pose(), but computing the uncertainty of odometry increments
     *  according to a given motion model.
     */
    void fuse_odometry(
        const mrpt::obs::CObservationOdometry& odom,
        const std::string& odomName = "odom_wheels") override;

    /** Integrates new IMU observations into the estimator */
    void fuse_imu(const mrpt::obs::CObservationIMU& imu) override;

    /** Integrates new twist estimation (in the odom frame) */
    void fuse_twist(
        const mrpt::Clock::time_point&     timestamp,
        const mrpt::math::TTwist3D&        twist,
        const mrpt::math::CMatrixDouble66& twistCov) override;

    /** Computes the estimated vehicle state at a given timestep using the
     * observations in the time window. A std::nullopt is returned if there is
     * no valid observations yet, or if requested a timestamp out of the model
     * validity time window (e.g. too far in the future to be trustful).
     */
    std::optional<NavState> estimated_navstate(
        const mrpt::Clock::time_point& timestamp,
        const std::string&             frame_id) override;

    std::optional<mrpt::math::TTwist3D> get_last_twist() const
    {
        return state_.last_twist;
    }

    /** @} */

   private:
    struct State
    {
        State()  = default;
        ~State() = default;

        std::optional<mrpt::obs::CObservationOdometry> last_odom_obs;
        std::optional<mrpt::Clock::time_point>         last_pose_obs_tim;
        std::optional<mrpt::poses::CPose3DPDFGaussian> last_pose;
        std::optional<mrpt::math::TTwist3D>            last_twist;
        bool pose_already_updated_with_odom = false;
    };

    State state_;
};

}  // namespace mola
