/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MapServer.h
 * @brief  Map saving/loading services offered by MOLA modules
 * @author Jose Luis Blanco Claraco
 * @date   Aug 18, 2024
 */
#pragma once

#include <cstdlib>
#include <string>

namespace mola
{
/** Virtual interface for map server services offered by MOLA modules
 * \ingroup mola_kernel_grp */
class MapServer
{
   public:
    MapServer();
    virtual ~MapServer();

    /** @name Virtual interface of MapServer
     *{ */

    struct ReturnStatus
    {
        ReturnStatus() = default;

        bool        success = false;
        std::string error_message;
    };

    /** Loads a map from file(s) and sets it as active current map.
     * Different implementations may use one or more files to store map as
     * files.
     *
     *  \param[in] path File name(s) prefix for the map to load. Do not add file
     * extension.
     */
    virtual ReturnStatus map_load([[maybe_unused]] const std::string& path)
    {
        return {};
    }

    /** Saves a map from file(s) and sets it as active current map.
     * Different implementations may use one or more files to store map as
     * files.
     *
     *  \param[in] path File name(s) prefix for the map to save. Do not add file
     * extension.
     */
    virtual ReturnStatus map_save([[maybe_unused]] const std::string& path)
    {
        return {};
    }

    /** @} */
};

}  // namespace mola
