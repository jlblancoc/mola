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

#include <mola_relocalization/relocalization.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/slam/CGridMapAligner.h>

#include <mrpt/maps/bonxai/bonxai.hpp>

namespace
{

class DensityGridMap
{
   public:
    DensityGridMap(
        double resolution, uint8_t inner_bits = 2, uint8_t leaf_bits = 3)
        : grid_(resolution, inner_bits, leaf_bits)
    {
    }

    void insert_point_cloud(const mrpt::maps::CPointsMap& p)
    {
        const auto&  xs = p.getPointsBufferRef_x();
        const auto&  ys = p.getPointsBufferRef_y();
        const size_t N  = xs.size();

        for (size_t i = 0; i < N; i++)
        {
            const auto pt = mrpt::math::TPoint3Df(xs[i], ys[i], .0f);

            auto* cell = accessor_.value(
                Bonxai::PosToCoord({pt.x, pt.y, pt.z}, grid_.inv_resolution),
                true /*create*/);
            cell->pointCount++;

            mrpt::keep_max<uint32_t>(maxDensity_, cell->pointCount);

            if (!bbox_)
            {
                bbox_.emplace();
                bbox_ = mrpt::math::TBoundingBoxf(pt, pt);
            }
            else
                bbox_->updateWithPoint(pt);
        }
    }

    mrpt::maps::COccupancyGridMap2D::Ptr get_as_occupancy_grid()
    {
        ASSERT_(bbox_.has_value());
        ASSERT_(maxDensity_ > 0);

        auto og = mrpt::maps::COccupancyGridMap2D::Create();
        og->setSize(
            bbox_->min.x, bbox_->max.x, bbox_->min.y, bbox_->max.y,
            grid_.resolution, .01f);

        const float nPts_inv = 1.0f / maxDensity_;

        grid_.forEachCell(
            [&](Node& v, const Bonxai::CoordT& c)
            {
                const auto pt = Bonxai::CoordToPos(c, grid_.resolution);

                const float density = v.pointCount * nPts_inv;
                og->setPos(pt.x, pt.y, density);
            });

        return og;
    }

   private:
    struct Node
    {
        uint32_t pointCount = 0;
    };

    Bonxai::VoxelGrid<Node>                  grid_;
    Bonxai::VoxelGrid<Node>::Accessor        accessor_{grid_};
    uint32_t                                 maxDensity_ = 1;
    std::optional<mrpt::math::TBoundingBoxf> bbox_;
};

}  // namespace

mola::AlignBEV::Output mola::AlignBEV::run(const Input& in)
{
    mola::AlignBEV::Output result;

    const double t0 = mrpt::Clock::nowDouble();

    ASSERT_(!in.reference_map.layers.empty());
    ASSERT_(!in.local_map.layers.empty());

    auto lambdaProcessMap = [](DensityGridMap&                 g,
                               const mp2p_icp::metric_map_t&   m,
                               const std::vector<std::string>& inLayers)
    {
        std::vector<std::string> layers = inLayers;
        if (layers.empty())
            for (const auto& [l, ml] : m.layers) layers.push_back(l);

        for (const auto& layer : layers)
        {
            auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
                m.layers.at(layer));
            if (!pts) continue;
            g.insert_point_cloud(*pts);
        }
    };

    // Build density maps:
    DensityGridMap gGrid(in.resolution_xy);
    DensityGridMap lGrid(in.resolution_xy);

    lambdaProcessMap(gGrid, in.reference_map, in.ref_map_layers);
    lambdaProcessMap(lGrid, in.local_map, in.local_map_layers);

    const auto gOccGrid = gGrid.get_as_occupancy_grid();
    const auto lOccGrid = lGrid.get_as_occupancy_grid();

#if 1
    gOccGrid->saveAsBitmapFile("og_reference.png");
    lOccGrid->saveAsBitmapFile("og_local.png");
#endif

    mrpt::slam::CGridMapAligner ga;
    ga.options.dumpToTextStream(std::cout);

    ga.options.feature_descriptor   = mrpt::vision::descORB;
    ga.options.debug_save_map_pairs = true;

    auto resPose = ga.Align(gOccGrid.get(), lOccGrid.get(), {});

    std::cout << "res: " << resPose->getMeanVal() << "\n";

    // end
    result.time_cost = mrpt::Clock::nowDouble() - t0;

    return result;
}
