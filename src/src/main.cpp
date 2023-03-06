/*
 * Copyright 2023 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <iostream>
#include "Topics.h"
#include "data_loaders/DataLoaderController.h"
#include "data_loaders/CameraDataLoader.h"
#include "data_loaders/LidarDataLoader.h"
#include "data_loaders/ImuDataLoader.h"
#include "data_loaders/GnssDataLoader.h"
#include "data_loaders/RadarDataLoader.h"


int main(int argc, char **argv) {
    AtlasFusion::Log::Init();

    std::string datasetPath = "/media/standa/174A20FD45B9BA09/BUD/3_1_1_2/";
    //std::string datasetPath = "/home/standa/Desktop/BUD/3_1_3_3/";

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions().use_intra_process_comms(true);

    auto rgbCameraLSDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraLeftSideDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraLeftSide,
            AtlasFusion::Topics::kCameraLeftSideDataLoader,
            nodeOptions
    );
    executor.add_node(rgbCameraLSDataLoader);

    auto rgbCameraLFDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraLeftFrontDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraLeftFront,
            AtlasFusion::Topics::kCameraLeftFrontDataLoader,
            nodeOptions
    );
    executor.add_node(rgbCameraLFDataLoader);

    auto rgbCameraRFDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraRightFrontDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraRightFront,
            AtlasFusion::Topics::kCameraRightFrontDataLoader,
            nodeOptions
    );
    executor.add_node(rgbCameraRFDataLoader);

    auto rgbCameraRSDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraRightSideDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraRightSide,
            AtlasFusion::Topics::kCameraRightSideDataLoader,
            nodeOptions
    );
    executor.add_node(rgbCameraRSDataLoader);

    auto irCameraDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraIrDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraIr,
            AtlasFusion::Topics::kCameraIrDataLoader,
            nodeOptions
    );
    executor.add_node(irCameraDataLoader);

    auto lidarLeftDataLoader = std::make_shared<AtlasFusion::DataLoader::LidarDataLoader>(
            "LidarLeftDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::LidarIdentifier::kLeftLidar,
            AtlasFusion::Topics::kLidarLeftDataLoader,
            nodeOptions
    );
    executor.add_node(lidarLeftDataLoader);

    auto lidarCenterDataLoader = std::make_shared<AtlasFusion::DataLoader::LidarDataLoader>(
            "LidarCenterDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::LidarIdentifier::kCenterLidar,
            AtlasFusion::Topics::kLidarCenterDataLoader,
            nodeOptions
    );
    executor.add_node(lidarCenterDataLoader);

    auto lidarRightDataLoader = std::make_shared<AtlasFusion::DataLoader::LidarDataLoader>(
            "LidarRightDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::LidarIdentifier::kRightLidar,
            AtlasFusion::Topics::kLidarRightDataLoader,
            nodeOptions
    );
    executor.add_node(lidarRightDataLoader);

    auto imuDataLoader = std::make_shared<AtlasFusion::DataLoader::ImuDataLoader>(
            "ImuDataLoader",
            datasetPath,
            nodeOptions
    );
    executor.add_node(imuDataLoader);

    auto gnssDataLoader = std::make_shared<AtlasFusion::DataLoader::GnssDataLoader>(
            "GnssDataLoader",
            datasetPath,
            nodeOptions
    );
    executor.add_node(gnssDataLoader);

    auto radarDataLoader = std::make_shared<AtlasFusion::DataLoader::RadarDataLoader>(
            "RadarDataLoader",
            datasetPath,
            AtlasFusion::Topics::kRadarTiDataLoader,
            nodeOptions
    );
    executor.add_node(radarDataLoader);

    auto dataLoaderController = std::make_shared<AtlasFusion::DataLoader::DataLoaderController>(
            "DataLoaderController",
            18,
            nodeOptions
    );
    executor.add_node(dataLoaderController);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}