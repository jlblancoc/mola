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
#include <mrpt/version.h>

#include <optional>

/** \defgroup mola_relocalization_grp mola-relocalization
 * Algorithms for localization starting with large uncertainty.
 */

mola::Relocalization_SE2::Output mola::Relocalization_SE2::run(const Input& in)
{
    mola::Relocalization_SE2::Output result;

    const double t0 = mrpt::Clock::nowDouble();

    ASSERT_(!in.reference_map.layers.empty());

    result.likelihood_grid = mrpt::poses::CPosePDFGrid(
        in.corner_min.x, in.corner_max.x, in.corner_min.y, in.corner_max.y,
        in.resolution_xy, in.resolution_phi, in.corner_min.phi,
        in.corner_max.phi);

    auto& grid = result.likelihood_grid;

    const size_t nX   = grid.getSizeX();
    const size_t nY   = grid.getSizeY();
    const size_t nPhi = grid.getSizePhi();

    const size_t nCells = nX * nY * nPhi;
    ASSERT_(nCells > 0);

    // evaluate over the grid:
    std::optional<double> minW, maxW;

    for (size_t iX = 0, iGlobal = 0; iX < nX; iX++, iGlobal++)
    {
        const double x = grid.idx2x(iX);
        for (size_t iY = 0; iY < nY; iY++)
        {
            const double y = grid.idx2y(iY);
            for (size_t iPhi = 0; iPhi < nPhi; iPhi++)
            {
                const double phi = grid.idx2phi(iPhi);

                const auto pose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                    x, y, 0, phi, 0, 0);

                for (const auto& [layerName, map] : in.reference_map.layers)
                {
                    ASSERT_(map);
                    const double logLik = map->computeObservationsLikelihood(
                        in.observations, pose);

                    double* cell = grid.getByIndex(iX, iY, iPhi);
                    ASSERT_(cell);
                    *cell = logLik;

                    if (!minW || logLik < *minW) minW = logLik;
                    if (!maxW || logLik > *maxW) maxW = logLik;
                }
            }
        }
    }

    // normalizeWeights and convert log-lik ==> likelihood
    for (size_t iX = 0; iX < nX; iX++)
        for (size_t iY = 0; iY < nY; iY++)
            for (size_t iPhi = 0; iPhi < nPhi; iPhi++)
            {
                double& cell = *grid.getByIndex(iX, iY, iPhi);
                cell -= *maxW;
                cell = std::exp(cell);
            }
    *minW -= *maxW;

    // Normalize PDF:
    grid.normalize();

    result.time_cost          = mrpt::Clock::nowDouble() - t0;
    result.max_log_likelihood = 0;  // by definition of the normalization above
    result.min_log_likelihood = *minW;

    return result;
}

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
