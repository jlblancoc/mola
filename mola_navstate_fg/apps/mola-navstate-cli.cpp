/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mola_navstate_fg/NavStateFG.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CRawlog.h>

#include <iostream>

namespace
{
void run_navstate(const std::string& paramsFile, const std::string& rawlogFile)
{
    using mrpt::obs::CObservationGPS;
    using mrpt::obs::CObservationIMU;
    using mrpt::obs::CObservationRobotPose;

    mola::NavStateFG nav;

    std::cout << "Initalizing from: " << paramsFile << std::endl;

    const auto cfg = mrpt::containers::yaml::FromFile(paramsFile);
    nav.initialize(cfg);

    std::cout << "Reading dataset from: " << rawlogFile << std::endl;

    mrpt::obs::CRawlog dataset;
    dataset.loadFromRawLogFile(rawlogFile);

    const std::string frame_id = "map";

    std::cout << "Read entries: " << dataset.size() << std::endl;

    for (size_t i = 0; i < dataset.size(); i++)
    {
        const auto o = dataset.getAsObservation(i);
        if (!o) continue;

        if (auto oGPS = std::dynamic_pointer_cast<CObservationGPS>(o); oGPS)
        {
            nav.fuse_gnss(*oGPS);
        }
        else if (auto oPose =
                     std::dynamic_pointer_cast<CObservationRobotPose>(o);
                 oPose)
        {
            nav.fuse_pose(oPose->timestamp, oPose->pose, frame_id);
        }
        else if (auto oImu = std::dynamic_pointer_cast<CObservationIMU>(o);
                 oImu)
        {
            nav.fuse_imu(*oImu);
        }
        else
        {
            std::cout << "[Warning] Ignoring observation #" << i << ": '"
                      << o->sensorLabel
                      << "' of type: " << o->GetRuntimeClass()->className
                      << "\n";
        }

    }  // for each entry
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: " << argv[0] << " params.yaml dataset.rawlog"
                      << std::endl;
            return 1;
        }

        run_navstate(argv[1], argv[2]);

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
