/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   NavStateFilter.h
 * @brief  Virtual base class for algorithms to fuse odometry, twist, IMU, etc.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/topography/data_types.h>

#pragma once

namespace mola
{
/** The state returned by NavStateFilter
 *
 * \ingroup mola_kernel_grp */
struct NavState
{
    NavState()  = default;
    ~NavState() = default;

    /** SE(3) pose estimation, including information matrix, given
     *  in the requested frame_id.
     */
    mrpt::poses::CPose3DPDFGaussianInf pose;

    /** Linear and angular velocity estimation, given in the local vehicle
     *  frame. */
    mrpt::math::TTwist3D twist;

    /** Inverse covariance matrix (information) of twist,
     *  with variable order in the matrix: [vx vy vz wx wy wz]
     */
    mrpt::math::CMatrixDouble66 twist_inv_cov;

    std::string asString() const;
};

/** Unified API for kinematic state filtering algorithms,
 *  fusing information from multiple odometry or twist sources.
 *
 * \ingroup mola_kernel_grp */
class NavStateFilter : public mrpt::system::COutputLogger
{
   public:
    NavStateFilter();
    ~NavStateFilter();

    /** Resets the estimator state to an initial state */
    virtual void reset() = 0;

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    virtual void initialize(const mrpt::containers::yaml& cfg) = 0;

    /** Integrates new SE(3) pose estimation of the vehicle wrt frame_id
     */
    virtual void fuse_pose(
        const mrpt::Clock::time_point&         timestamp,
        const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string&                     frame_id) = 0;

    /** Integrates new wheels-based odometry observations into the estimator.
     *  This is a convenience method that internally ends up calling
     *  fuse_pose(), but computing the uncertainty of odometry increments
     *  according to a given motion model.
     */
    virtual void fuse_odometry(
        const mrpt::obs::CObservationOdometry& odom,
        const std::string&                     odomName = "odom_wheels") = 0;

    /** Integrates new IMU observations into the estimator */
    virtual void fuse_imu(const mrpt::obs::CObservationIMU& imu) = 0;

    /** Integrates new GNSS observations into the estimator */
    virtual void fuse_gnss(const mrpt::obs::CObservationGPS& gps) = 0;

    /** Integrates new twist estimation (in the odom frame) */
    virtual void fuse_twist(
        const mrpt::Clock::time_point&     timestamp,
        const mrpt::math::TTwist3D&        twist,
        const mrpt::math::CMatrixDouble66& twistCov) = 0;

    /** Computes the estimated vehicle state at a given timestep using the
     * observations in the time window. A std::nullopt is returned if there is
     * no valid observations yet, or if requested a timestamp out of the model
     * validity time window (e.g. too far in the future to be trustful).
     */
    virtual std::optional<NavState> estimated_navstate(
        const mrpt::Clock::time_point& timestamp,
        const std::string&             frame_id) = 0;

    /** Must be invoked with the mp2p_icp metric map geo-referencing information
     *  of the map in order to have GNSS observations correctly fused.
     */
    void set_georeferencing_params(
        /** The geodetic coordinates (on WGS-84) of the metric map ENU frame of
         * reference. */
        mrpt::topography::TGeodeticCoords geo_coord,
        /** The SE(3) transformation from the ENU (earth-north-up) frame
         * to the metric map local frame of reference.
         * If this is the identity (default) it means the map is already in
         * ENU coordinates (i.e. +X is East, +Y is North, +Z is up) and
         * the point (0,0,0) is the one having the geodetic coordinates
         * geo_coord
         */
        mrpt::poses::CPose3DPDFGaussian T_enu_to_map)
    {
        auto& g = geoRefParams_.emplace();

        g.geo_coord    = geo_coord;
        g.T_enu_to_map = T_enu_to_map;
    }

   protected:
    struct GeoReferenceParams
    {
        mrpt::topography::TGeodeticCoords geo_coord;
        mrpt::poses::CPose3DPDFGaussian   T_enu_to_map;
    };
    std::optional<GeoReferenceParams> geoRefParams_;
};

}  // namespace mola
