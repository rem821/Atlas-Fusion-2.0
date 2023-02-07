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
#include "data_loaders/CameraDataLoader.h"
#include "Topics.h"
#include "data_loaders/DataLoaderController.h"


int main(int argc, char **argv) {
    std::cout << "Hello Atlas Fusion 2.0!" << std::endl;

    //std::string datasetPath = "/media/standa/174A20FD45B9BA09/BUD/3_1_1_2/";
    std::string datasetPath = "/home/standa/Desktop/BUD/3_1_3_3/";

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions().use_intra_process_comms(true);

    auto rgbCameraLSDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraLeftSideDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraLeftSide,
            AtlasFusion::Topics::kCameraLeftSideDataLoader,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraLSDataLoader);

    auto rgbCameraLFDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraLeftFrontDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraLeftFront,
            AtlasFusion::Topics::kCameraLeftFrontDataLoader,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraLFDataLoader);

    auto rgbCameraRFDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraRightFrontDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraRightFront,
            AtlasFusion::Topics::kCameraRightFrontDataLoader,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraRFDataLoader);

    auto rgbCameraRSDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraRightSideDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraRightSide,
            AtlasFusion::Topics::kCameraRightSideDataLoader,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraRSDataLoader);

    auto irCameraDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraIRDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraIr,
            AtlasFusion::Topics::kCameraIrDataLoader,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(irCameraDataLoader);

    auto dataLoaderController = std::make_shared<AtlasFusion::DataLoader::DataLoaderController>(
            "DataLoaderController",
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(dataLoaderController);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}