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
 * @file   NavStateFGParams.h
 * @brief  Parameters for NavStateFG
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>

namespace mola
{
/** Parameters needed by NavStateFG.
 *
 * \ingroup mola_navstate_fuse__grp
 */
class NavStateFGParams
{
   public:
    NavStateFGParams()  = default;
    ~NavStateFGParams() = default;

    /// Loads all parameters from a YAML map node.
    void loadFrom(const mrpt::containers::yaml& cfg);

    /** Valid estimations will be extrapolated only up to this time since the
     * last incorporated observation. If a request is done farther away, an
     * empty estimation will be returned.
     */
    double max_time_to_use_velocity_model = 2.0;  // [s]

    /// Time to keep past observations in the filter
    double sliding_window_length = 5.0;  // [s]

    /// If the time between two keyframes is larger than this, a warning will be
    /// emitted; but the algorithm will keep trying its best.
    double time_between_frames_to_warning = 3.0;  // [s]

    double sigma_random_walk_acceleration_linear  = 1.0;  // [m/s²]
    double sigma_random_walk_acceleration_angular = 1.0;  // [rad/s²]
    double sigma_integrator_position              = 0.10;  // [m]
    double sigma_integrator_orientation           = 0.10;  // [rad]

    double robust_param = 0.0;  // 0: no robust
    double max_rmse     = 2.0;

    mrpt::math::TTwist3D initial_twist;
    double               initial_twist_sigma_lin = 20.0;  // [m/s]
    double               initial_twist_sigma_ang = 3.0;  // [rad/s]
};

}  // namespace mola
