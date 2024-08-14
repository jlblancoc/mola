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
 * @file   test-navstate-basic.cpp
 * @brief  Unit tests for NavStateFG
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2024
 */

#include <mola_navstate_fg/NavStateFG.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>  // required by "matrix * scalar"
#include <functional>
#include <iostream>
#include <map>

using namespace std::string_literals;

namespace
{
const char* navStateParams =
    R"###(# Config for NavStateFGParams
sliding_window_length: 5.0 # [s]
max_time_to_use_velocity_model: 2.0  # [s]
time_between_frames_to_warning: 2.0  # [s]
sigma_random_walk_acceleration_linear: 1.0 # [m/s²]
sigma_random_walk_acceleration_angular: 1.0 # [rad/s²]
sigma_integrator_position: 0.10 # [m]
sigma_integrator_orientation: 0.10 # [rad]
robust_param: 0
max_rmse: 2
)###";

using namespace mrpt::literals;  // _deg
using mrpt::math::CMatrixDouble66;

class Data
{
   private:
    Data() = default;

   public:
    static Data& Instance()
    {
        static Data o;
        return o;
    }

    const CMatrixDouble66 I6_2cm =
        CMatrixDouble66(CMatrixDouble66::Identity() * 0.02);

    const mrpt::poses::CPose3D p0 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
    const mrpt::poses::CPose3DPDFGaussian pdf0{p0, I6_2cm};

    const mrpt::poses::CPose3D p1 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        0.5, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
    const mrpt::poses::CPose3DPDFGaussian pdf1{p1, I6_2cm};

    const mrpt::poses::CPose3D p2 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        1.0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
    const mrpt::poses::CPose3DPDFGaussian pdf2{p2, I6_2cm};

    // circle:
    const mrpt::poses::CPose3D pc0 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
    const mrpt::poses::CPose3DPDFGaussian pdf_c0{pc0, I6_2cm};

    const double wc = 0.2, vc = 10.0, Rc = vc / wc;

    const mrpt::poses::CPose3D pc1 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        Rc * sin(wc * 0.1), Rc*(1 - cos(wc * 0.1)), 0, wc * 0.1, 0.0_deg,
        0.0_deg);
    const mrpt::poses::CPose3DPDFGaussian pdf_c1{pc1, I6_2cm};

    const mrpt::poses::CPose3D pc2 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        Rc * sin(wc * 0.2), Rc*(1 - cos(wc * 0.2)), 0, wc * 0.2, 0.0_deg,
        0.0_deg);
    const mrpt::poses::CPose3DPDFGaussian pdf_c2{pc2, I6_2cm};

    const mrpt::poses::CPose3D pc3 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        Rc * sin(wc * 0.3), Rc*(1 - cos(wc * 0.3)), 0, wc * 0.3, 0.0_deg,
        0.0_deg);
};

// --------------------------------------
void test_init_state()
{
    mola::NavStateFG nav;
    nav.initialize(mrpt::containers::yaml::FromText(navStateParams));

    ASSERT_(nav.known_frame_ids().empty());

    const auto ret = nav.estimated_navstate(mrpt::Clock::now(), "odom");
    ASSERT_(!ret.has_value());
}

// --------------------------------------
void test_one_pose()
{
    const auto& _ = Data::Instance();

    mola::NavStateFG nav;
    nav.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const auto t0 = mrpt::Clock::fromDouble(.0);

    nav.fuse_pose(t0, _.pdf0, "odom");

    const auto ret = nav.estimated_navstate(t0, "odom");
    ASSERT_(ret.has_value());

    // std::cout << "Result:\n" << ret->asString() << std::endl;

    ASSERT_NEAR_(
        mrpt::poses::Lie::SE<3>::log(ret->pose.mean - _.pdf0.mean).norm(), 0.0,
        1e-4);
}

// --------------------------------------
void test_one_pose_extrapolate()
{
    const auto& _ = Data::Instance();

    mola::NavStateFG nav;
    nav.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const auto t0 = mrpt::Clock::fromDouble(.0);
    const auto t1 = mrpt::Clock::fromDouble(.5);

    nav.fuse_pose(t0, _.pdf0, "odom");

    const auto ret = nav.estimated_navstate(t1, "odom");
    ASSERT_(ret.has_value());

    // std::cout << "Result:\n" << ret->asString() << std::endl;

    ASSERT_NEAR_(
        mrpt::poses::Lie::SE<3>::log(ret->pose.mean - _.pdf0.mean).norm(), 0.0,
        1e-4);

    ASSERT_GT_(
        std::sqrt(1.0 / ret->twist_inv_cov(0, 0)),
        nav.params_.initial_twist_sigma_lin);
}

// --------------------------------------
void test_2_poses()
{
    const auto& _ = Data::Instance();

    mola::NavStateFG nav;
    nav.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const auto t0 = mrpt::Clock::fromDouble(0.0);
    const auto t1 = mrpt::Clock::fromDouble(0.5);

    const auto t2 = mrpt::Clock::fromDouble(0.6);
    const auto t3 = mrpt::Clock::fromDouble(0.25);

    nav.fuse_pose(t0, _.pdf0, "odom");
    nav.fuse_pose(t1, _.pdf1, "odom");

    const auto ret2 = nav.estimated_navstate(t2, "odom");
    ASSERT_(ret2.has_value());

    const auto ret3 = nav.estimated_navstate(t3, "odom");
    ASSERT_(ret3.has_value());

    // std::cout << "Result:\n" << ret2->asString() << std::endl;

    const auto expected2 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        0.6, 0.0, 0.0, .0_deg, .0_deg, .0_deg);
    ASSERT_NEAR_(
        mrpt::poses::Lie::SE<3>::log(ret2->pose.mean - expected2).norm(), 0.0,
        1e-2);

    const auto expected3 = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        0.25, 0.0, 0.0, .0_deg, .0_deg, .0_deg);
    ASSERT_NEAR_(
        mrpt::poses::Lie::SE<3>::log(ret3->pose.mean - expected3).norm(), 0.0,
        1e-2);
}

// --------------------------------------
void test_2_poses_too_late()
{
    const auto& _ = Data::Instance();

    mola::NavStateFG nav;
    nav.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const auto t0 = mrpt::Clock::fromDouble(0.0);
    const auto t1 = mrpt::Clock::fromDouble(0.5);

    // too late/early to extrapolate!! must return nullopt:
    const auto t2 = mrpt::Clock::fromDouble(
        nav.params_.max_time_to_use_velocity_model + 0.5 + 0.1);
    const auto t3 = mrpt::Clock::fromDouble(
        0.0 - 0.1 - nav.params_.max_time_to_use_velocity_model);

    nav.fuse_pose(t0, _.pdf0, "odom");
    nav.fuse_pose(t1, _.pdf1, "odom");

    const auto ret1 = nav.estimated_navstate(t2, "odom");
    ASSERT_(!ret1.has_value());

    const auto ret2 = nav.estimated_navstate(t3, "odom");
    ASSERT_(!ret2.has_value());
}

// --------------------------------------
void test_3_poses()
{
    const auto& _ = Data::Instance();

    mola::NavStateFG nav;
    nav.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const auto t0 = mrpt::Clock::fromDouble(0.0);
    const auto t1 = mrpt::Clock::fromDouble(0.1);
    const auto t2 = mrpt::Clock::fromDouble(0.2);
    const auto t3 = mrpt::Clock::fromDouble(0.3);

    nav.fuse_pose(t0, _.pdf_c0, "odom");
    nav.fuse_pose(t1, _.pdf_c1, "odom");
    nav.fuse_pose(t2, _.pdf_c2, "odom");

    const auto ret2 = nav.estimated_navstate(t3, "odom");
    ASSERT_(ret2.has_value());

    // std::cout << "Result:\n" << ret2->asString() << std::endl;

    ASSERT_NEAR_(
        mrpt::poses::Lie::SE<3>::log(ret2->pose.mean - _.pc3).norm(), 0.0,
        1e-1);

    ASSERT_NEAR_(ret2->twist.vx, _.vc, 0.1);
    // ASSERT_NEAR_(ret2->twist.vy, .0, 0.2);
    ASSERT_NEAR_(ret2->twist.vz, .0, 0.2);

    ASSERT_NEAR_(ret2->twist.wx, .0, 0.01);
    ASSERT_NEAR_(ret2->twist.wy, .0, 0.01);
    ASSERT_NEAR_(ret2->twist.wz, _.wc, 0.05);
}

// --------------------------------------
void test_noisy_straight()
{
    const auto& _ = Data::Instance();

    mola::NavStateFG nav;
    nav.initialize(mrpt::containers::yaml::FromText(navStateParams));

    auto& rng = mrpt::random::getRandomGenerator();

    const size_t nSteps = 10;
    const double vx     = 8.0;  // m/s
    const double T      = 0.100;  // s

    const double stdXYZ = 0.01;
    const double stdYPR = mrpt::DEG2RAD(0.1);

    for (size_t i = 0; i < nSteps; i++)
    {
        const double tt = T * i;
        const auto   t  = mrpt::Clock::fromDouble(tt);

        const mrpt::poses::CPose3D p =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                vx * tt + rng.drawGaussian1D(0, stdXYZ),
                rng.drawGaussian1D(0, stdXYZ), +rng.drawGaussian1D(0, stdXYZ),
                rng.drawGaussian1D(0, stdYPR), rng.drawGaussian1D(0, stdYPR),
                rng.drawGaussian1D(0, stdYPR));
        const mrpt::poses::CPose3DPDFGaussian pdf{p, _.I6_2cm};

        nav.fuse_pose(t, pdf, "odom");
    }

    // query:
    const double tt_q = T * nSteps;
    const auto   t_q  = mrpt::Clock::fromDouble(tt_q);

    const auto ret = nav.estimated_navstate(t_q, "odom");
    ASSERT_(ret.has_value());

    // std::cout << "Result:\n" << ret->asString() << std::endl;

    ASSERT_NEAR_(ret->twist.vx, vx, 0.1);
    ASSERT_NEAR_(ret->twist.vy, .0, 0.1);
    ASSERT_NEAR_(ret->twist.vz, .0, 0.1);

    ASSERT_NEAR_(ret->twist.wx, .0, 0.01);
    ASSERT_NEAR_(ret->twist.wy, .0, 0.01);
    ASSERT_NEAR_(ret->twist.wz, .0, 0.01);
}

}  // namespace

int main(int argc, char** argv)
{
    const std::map<std::string, std::function<void()>> tests = {
        {"test_init_state", test_init_state},
        {"test_one_pose", test_one_pose},
        {"test_one_pose_extrap", test_one_pose_extrapolate},
        {"test_2_poses", test_2_poses},
        {"test_2_poses_too_late", test_2_poses_too_late},
        {"test_3_poses", test_3_poses},
        {"test_noisy_straight", test_noisy_straight},
    };

    int runOnlyIdx = -1;
    if (argc == 2) runOnlyIdx = std::stoi(argv[1]);

    bool anyFail = false;

    int index = 0;
    for (const auto& [name, f] : tests)
    {
        index++;

        if (runOnlyIdx >= 0 && index != runOnlyIdx) continue;

        const auto sPrefix = mrpt::format(
            "[ (%3i / %3zu) %20s ]", index, tests.size(), name.c_str());
        try
        {
            std::cout << sPrefix << " Running..." << std::endl;
            f();

            mrpt::system::consoleColorAndStyle(
                mrpt::system::ConsoleForegroundColor::GREEN);
            std::cout << sPrefix << " OK." << std::endl;
        }
        catch (std::exception& e)
        {
            mrpt::system::consoleColorAndStyle(
                mrpt::system::ConsoleForegroundColor::RED);
            std::cout << sPrefix << " ERROR: " << std::endl
                      << e.what() << std::endl;
            anyFail = true;
        }

        mrpt::system::consoleColorAndStyle(
            mrpt::system::ConsoleForegroundColor::DEFAULT);
    }

    return anyFail;
}
