/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorConstVelKinematics.h
 * @brief  Constant-velocity model factor
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/factors/FactorBase.h>
#include <mrpt/core/exceptions.h>

namespace mola
{
/** Abstract representation of a constant-velocity kinematic motion model factor
 * between two key frames.
 *
 * \ingroup mola_kernel_grp
 */
class FactorConstVelKinematics : public FactorBase
{
    DEFINE_SERIALIZABLE(FactorConstVelKinematics, mola)

   public:
    FactorConstVelKinematics() = default;

    /** Creates relative pose constraint of KF `to` as seem from `from`. */
    FactorConstVelKinematics(id_t kf_from, id_t kf_to, double deltaTime)
        : from_kf_(kf_from), to_kf_(kf_to), deltaTime_(deltaTime)
    {
    }

    id_t from_kf_ = INVALID_ID, to_kf_ = INVALID_ID;

    /** Elapsed time between "from_kf" and "to_kf" [seconds] */
    double deltaTime_ = .0;

    std::size_t edge_count() const override { return 2; }
    mola::id_t  edge_indices(const std::size_t i) const override;
};

}  // namespace mola
