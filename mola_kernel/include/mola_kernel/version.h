/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   version.h
 * @brief  Provides macros for checking MOLA version
 * @author Jose Luis Blanco Claraco
 * @date   Aug 18, 2024
 */
#pragma once

/// To be used in #if() checks for >= minimum MOLA versions
#define MOLA_VERSION_CHECK(major, minor, patch)                         \
    ((MOLA_MAJOR_VERSION > (major)) ||                                  \
     (MOLA_MAJOR_VERSION == (major) && MOLA_MINOR_VERSION > (minor)) || \
     (MOLA_MAJOR_VERSION == (major) && MOLA_MINOR_VERSION == (minor) && \
      MOLA_PATCH_VERSION >= (patch)))
