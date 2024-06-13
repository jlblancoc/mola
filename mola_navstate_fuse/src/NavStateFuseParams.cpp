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
 * @file   NavStateFuseParams.cpp
 * @brief  Parameters for NavStateFuse
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_navstate_fuse/NavStateFuseParams.h>

using namespace mola;

void NavStateFuseParams::loadFrom(const mrpt::containers::yaml& cfg)
{
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_REQ(cfg, sliding_window_length);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);

    MCP_LOAD_OPT(cfg, time_between_frames_to_warning);
}
