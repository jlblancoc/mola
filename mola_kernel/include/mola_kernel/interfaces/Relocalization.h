/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Relocalization.h
 * @brief  Virtual interface for relocalization offered by MOLA modules
 * @author Jose Luis Blanco Claraco
 * @date   Jul 29, 2024
 */
#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mola
{
/** Virtual interface for relocalization offered by MOLA modules
 * \ingroup mola_kernel_grp */
class Relocalization
{
   public:
    Relocalization();
    virtual ~Relocalization();

    /** @name Virtual interface of Relocalization
     *{ */

    /** Re-localize near this pose, including uncetainty.
     *  \param[in] pose The pose, in the local map frame.
     *  There is no return value from this method.
     */
    virtual void relocalize_near_pose_pdf(
        const mrpt::poses::CPose3DPDFGaussian& p) = 0;

    /** Re-localize with the next incoming GNSS message.
     *  There is no return value from this method.
     */
    virtual void relocalize_from_gnss() = 0;

    /** @} */
};

}  // namespace mola
