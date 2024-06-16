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
 * @file   NavStateFilter.cpp
 * @brief  State vector for SE(3) pose + velocity
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_kernel/interfaces/NavStateFilter.h>

#include <Eigen/Dense>
#include <sstream>

using namespace mola;

NavStateFilter::NavStateFilter()
{
    this->mrpt::system::COutputLogger::setLoggerName("NavStateFilter");
}

NavStateFilter::~NavStateFilter() = default;

std::string NavState::asString() const
{
    std::ostringstream ss;
    ss << "pose  : " << pose;
    ss << "twist : " << twist.asString() << "\n";
    ss << "twist inv_cov diagonal: "
       << twist_inv_cov.asEigen().diagonal().transpose() << "\n";

    return ss.str();
}
