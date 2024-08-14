/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorConstVelKinematics.cpp
 * @brief  Constant-velocity model factor
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */

#include <mola_kernel/factors/FactorConstVelKinematics.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

// arguments: classname, parent class, namespace
IMPLEMENTS_SERIALIZABLE(FactorConstVelKinematics, FactorBase, mola);

mola::id_t FactorConstVelKinematics::edge_indices(const std::size_t i) const
{
    switch (i)
    {
        case 0:
            return from_kf_;
        case 1:
            return to_kf_;
        default:
            THROW_EXCEPTION("Out of bounds");
    }
}

// Implementation of the CSerializable virtual interface:
uint8_t FactorConstVelKinematics::serializeGetVersion() const { return 0; }
void    FactorConstVelKinematics::serializeTo(
       mrpt::serialization::CArchive& out) const
{
    baseSerializeTo(out);
    out << from_kf_ << to_kf_ << deltaTime_;
}
void FactorConstVelKinematics::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    baseSerializeFrom(in);

    switch (version)
    {
        case 0:
        {
            in >> from_kf_ >> to_kf_ >> deltaTime_;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}
