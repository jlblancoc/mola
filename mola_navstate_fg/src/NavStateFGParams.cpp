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
 * @file   NavStateFGParams.cpp
 * @brief  Parameters for NavStateFuse
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_navstate_fg/NavStateFGParams.h>

using namespace mola;

void NavStateFGParams::loadFrom(const mrpt::containers::yaml& cfg)
{
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_REQ(cfg, sliding_window_length);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);

    MCP_LOAD_OPT(cfg, time_between_frames_to_warning);

    MCP_LOAD_OPT(cfg, initial_twist_sigma_lin);
    MCP_LOAD_OPT(cfg, initial_twist_sigma_ang);

    MCP_LOAD_OPT(cfg, max_rmse);
    MCP_LOAD_OPT(cfg, robust_param);

    if (cfg.has("initial_twist"))
    {
        ASSERT_(
            cfg["initial_twist"].isSequence() &&
            cfg["initial_twist"].asSequence().size() == 6);

        auto&      tw  = initial_twist;
        const auto seq = cfg["initial_twist"].asSequenceRange();
        for (size_t i = 0; i < 6; i++) tw[i] = seq.at(i).as<double>();
    }
}
