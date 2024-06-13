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
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

namespace mola
{
/**
 * Factor for constant angular velocity model, equivalent to expression:
 *
 *   Pi + 0.5 * dt * (Vi + Vj) - Pj = errZero
 *
 */
class FactorTrapezoidalIntegrator
    : public gtsam::ExpressionFactorN<
          gtsam::Point3 /*return type*/, gtsam::Point3, gtsam::Point3,
          gtsam::Point3, gtsam::Point3>
{
   private:
    using This = FactorTrapezoidalIntegrator;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3 /*return type*/, gtsam::Point3, gtsam::Point3,
        gtsam::Point3, gtsam::Point3>;

    double dt_ = .0;

   public:
    /// default constructor
    FactorTrapezoidalIntegrator()           = default;
    ~FactorTrapezoidalIntegrator() override = default;

    FactorTrapezoidalIntegrator(
        gtsam::Key kPi, gtsam::Key kVi, gtsam::Key kPj, gtsam::Key kVj,
        const double dt, const gtsam::SharedNoiseModel& model)
        : Base({kPi, kVi, kPj, kVj}, model, /* error=0 */ {0, 0, 0}), dt_(dt)
    {
        this->initialize(This::expression({kPi, kVi, kPj, kVj}));
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
        gtsam::Expression<gtsam::Point3> Pi_(keys[0]);
        gtsam::Expression<gtsam::Point3> Vi_(keys[1]);

        gtsam::Expression<gtsam::Point3> Pj_(keys[2]);
        gtsam::Expression<gtsam::Point3> Vj_(keys[3]);

        return {Pi_ + 0.5 * dt_ * (Vi_ + Vj_) - Pj_};
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string& s, const gtsam::KeyFormatter& keyFormatter =
                                  gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorTrapezoidalIntegrator("
                  << keyFormatter(Factor::keys_[0]) << ","
                  << keyFormatter(Factor::keys_[1]) << ","
                  << keyFormatter(Factor::keys_[2]) << ","
                  << keyFormatter(Factor::keys_[3]) << ")\n";
        gtsam::traits<double>::Print(dt_, "  dt: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9)
        const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(e->dt_, dt_, tol);
    }

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
        ar& BOOST_SERIALIZATION_NVP(dt_);
        ar& boost::serialization::make_nvp(
            "FactorTrapezoidalIntegrator",
            boost::serialization::base_object<Base>(*this));
    }
};

}  // namespace mola
