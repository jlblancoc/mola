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
 * @file   test-relocalization-se2-kitti.cpp
 * @brief  Unit tests for mola_relolization
 * @author Jose Luis Blanco Claraco
 * @date   Apr 2, 2024
 */

#include <mola_relocalization/relocalization.h>

using namespace std::string_literals;

static void test1()
{
    // TODO:
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test1();

        std::cout << "Test successful." << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
