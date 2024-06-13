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
 * @file   FactorAngularVelocityIntegration.h
 * @brief  GTSAM factor
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2024
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

namespace mola
{
/**
 * Factor for constant angular velocity model, equivalent to expression:
 *
 *   gtsam::rotate(Ri, bWi) - gtsam::rotate(Rj, bWj) = errZero
 *
 */
class FactorConstAngularVelocity
    : public gtsam::ExpressionFactorN<
          gtsam::Point3 /*return type*/, gtsam::Rot3, gtsam::Point3,
          gtsam::Rot3, gtsam::Point3>
{
   private:
    using This = FactorConstAngularVelocity;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3, gtsam::Rot3, gtsam::Point3, gtsam::Rot3, gtsam::Point3>;

   public:
    /// default constructor
    FactorConstAngularVelocity()           = default;
    ~FactorConstAngularVelocity() override = default;

    FactorConstAngularVelocity(
        gtsam::Key kRi, gtsam::Key kWi, gtsam::Key kRj, gtsam::Key kWj,
        const gtsam::SharedNoiseModel& model)
        : Base({kRi, kWi, kRj, kWj}, model, {0, 0, 0})
    {
        this->initialize(This::expression({kRi, kWi, kRj, kWj}));
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    // Return measurement expression
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Expression<gtsam::Rot3>   Ri_(keys[0]);
        gtsam::Expression<gtsam::Point3> bWi_(keys[1]);
        gtsam::Expression<gtsam::Rot3>   Rj_(keys[2]);
        gtsam::Expression<gtsam::Point3> bWj_(keys[3]);
        return {gtsam::rotate(Ri_, bWi_) - gtsam::rotate(Rj_, bWj_)};
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string& s, const gtsam::KeyFormatter& keyFormatter =
                                  gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorConstAngularVelocity("
                  << keyFormatter(Factor::keys_[0]) << ","
                  << keyFormatter(Factor::keys_[1]) << ","
                  << keyFormatter(Factor::keys_[2]) << ","
                  << keyFormatter(Factor::keys_[3]) << ")\n";
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9)
        const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol);
    }

    /** implement functions needed to derive from Factor */

    /** number of variables attached to this factor */
    // std::size_t size() const;

   private:
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        // **IMPORTANT** We need to deserialize parameters before the base
        // class, since it calls expression() and we need all parameters ready
        // at that point.
        ar& BOOST_SERIALIZATION_NVP(measured_);
        ar& boost::serialization::make_nvp(
            "FactorConstAngularVelocity",
            boost::serialization::base_object<Base>(*this));
    }
};

}  // namespace mola
