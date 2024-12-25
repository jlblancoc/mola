/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   VizInterface.h
 * @brief  Virtual visualization interface (see MolaViz)
 * @author Jose Luis Blanco Claraco
 * @date   Sep 9, 2020
 */
#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/rtti/CObject.h>

#include <future>
#include <memory>

// Fwrd decl to avoid pushing gui dependencies to all clients of mola-kernel
// Was: #include <mrpt/gui/CDisplayWindowGUI.h>  // nanogui
// clang-format off
namespace nanogui { class Window; }
namespace mrpt::opengl { class CSetOfObjects; }
// clang-format on

namespace mola
{
/** Virtual visualization interface (see MolaViz)
 *
 * \ingroup mola_kernel_interfaces_grp */
class VizInterface
{
   public:
    VizInterface()          = default;
    virtual ~VizInterface() = default;

    using Ptr = std::shared_ptr<VizInterface>;

    // ===============================
    // See class MolaViz for docs
    // ===============================

    virtual std::future<nanogui::Window*> create_subwindow(
        const std::string& title, const std::string& parentWindow = "main") = 0;

    virtual std::future<void> subwindow_grid_layout(
        const std::string& subWindowTitle, const bool orientationVertical,
        int resolution, const std::string& parentWindow = "main") = 0;

    virtual std::future<void> subwindow_move_resize(
        const std::string&                subWindowTitle,
        const mrpt::math::TPoint2D_<int>& location,
        const mrpt::math::TPoint2D_<int>& size,
        const std::string&                parentWindow = "main") = 0;

    virtual std::future<bool> subwindow_update_visualization(
        const mrpt::rtti::CObject::Ptr& obj, const std::string& subWindowTitle,
        const std::string& parentWindow = "main") = 0;

    virtual std::future<bool> update_3d_object(
        const std::string&                                  objName,
        const std::shared_ptr<mrpt::opengl::CSetOfObjects>& obj,
        const std::string& viewportName = "main",
        const std::string& parentWindow = "main") = 0;

    virtual std::future<bool> update_viewport_look_at(
        const mrpt::math::TPoint3Df& lookAt,
        const std::string&           viewportName = "main",
        const std::string&           parentWindow = "main") = 0;

    virtual std::future<bool> update_viewport_camera_azimuth(
        const double azimuth, bool absolute_falseForRelative = true,
        const std::string& viewportName = "main",
        const std::string& parentWindow = "main") = 0;

    virtual std::future<void> enqueue_custom_nanogui_code(
        const std::function<void(void)>& userCode) = 0;

    virtual std::future<bool> output_console_message(
        const std::string& msg, const std::string& parentWindow = "main") = 0;
};

}  // namespace mola
