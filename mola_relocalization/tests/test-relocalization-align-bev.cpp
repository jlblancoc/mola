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
 * @file   test-relocalization-align-bev.cpp
 * @brief  Unit tests for mola_relocalization
 * @author Jose Luis Blanco Claraco
 * @date   Apr 23, 2024
 */

#include <mola_relocalization/relocalization.h>
#include <mrpt/system/filesystem.h>

static void test1(const std::string& refMap, const std::string& localMap)
{
    mp2p_icp::metric_map_t rMap, lMap;

    bool gLoadOk = rMap.load_from_file(refMap);
    ASSERT_(gLoadOk);

    bool lLoadOk = lMap.load_from_file(localMap);
    ASSERT_(lLoadOk);

    mola::AlignBEV::Input in;

    in.reference_map = rMap;
    in.local_map     = lMap;

    const auto out = mola::AlignBEV::run(in);
}

int main(int argc, char** argv)
{
    try
    {
        ASSERT_(argc == 3);

        test1(argv[1], argv[2]);

        std::cout << "Test successful." << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
