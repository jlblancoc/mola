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
 * @file   relocalization.cpp
 * @brief  Algorithms for localization starting with large uncertainty.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 2, 2024
 */

#include <mola_relocalization/relocalization.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/version.h>

std::map<double, mrpt::math::TPose2D> mola::find_best_poses_se2(
    const mrpt::poses::CPosePDFGrid& grid, const double percentile)
{
    ASSERT_GT_(percentile, 0.0);
    ASSERT_LT_(percentile, 1.0);

#if MRPT_VERSION < 0x020c01
    const size_t nX   = grid.getSizeX();
    const size_t nY   = grid.getSizeY();
    const size_t nPhi = grid.getSizePhi();

    double maxLik = .0;
    for (size_t iX = 0; iX < nX; iX++)
        for (size_t iY = 0; iY < nY; iY++)
            for (size_t iPhi = 0; iPhi < nPhi; iPhi++)
            {
                const double& cell = *grid.getByIndex(iX, iY, iPhi);
                mrpt::keep_max(maxLik, cell);
            }
#else
    const double maxLik =
        *std::max_element(grid.data().begin(), grid.data().end());
#endif

    const double threshold = percentile * maxLik;

    std::map<double, mrpt::math::TPose2D> best;

#if MRPT_VERSION < 0x020c01
    for (size_t iX = 0; iX < nX; iX++)
    {
        const double x = grid.idx2x(iX);
        for (size_t iY = 0; iY < nY; iY++)
        {
            for (size_t iPhi = 0; iPhi < nPhi; iPhi++)
            {
                const double& cell = *grid.getByIndex(iX, iY, iPhi);
                if (cell < threshold) continue;

                const double y   = grid.idx2y(iY);
                const double phi = grid.idx2phi(iPhi);

                best[cell] = {x, y, phi};
            }
        }
    }
#else
    const size_t N = grid.data().size();
    for (size_t i = 0; i < N; i++)
    {
        const double cell = grid.data()[i];
        if (cell < threshold) continue;

        const auto [ix, iy, iphi] = grid.absidx2idx(i);
        const double x            = grid.idx2x(ix);
        const double y            = grid.idx2y(iy);
        const double phi          = grid.idx2phi(iphi);
        best[cell]                = {x, y, phi};
    }
#endif
    return best;
}
