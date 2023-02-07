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

int main(int argc, char **argv) {
    std::cout << "Hello Atlas Fusion 2.0!" << std::endl;

    std::string datasetPath = "/media/standa/174A20FD45B9BA09/BUD/3_1_1_2/";
    rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions().use_intra_process_comms(true);

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto rgbCameraLSDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraLeftSideDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraLeftSide,
            AtlasFusion::Topics::kCameraLeftSide,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraLSDataLoader);

    auto rgbCameraLFDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraLeftFrontDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraLeftFront,
            AtlasFusion::Topics::kCameraLeftFront,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraLFDataLoader);

    auto rgbCameraRFDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraRightFrontDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraRightFront,
            AtlasFusion::Topics::kCameraRightFront,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraRFDataLoader);

    auto rgbCameraRSDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraRightSideDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraRightSide,
            AtlasFusion::Topics::kCameraRightSide,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(rgbCameraRSDataLoader);

    auto irCameraDataLoader = std::make_shared<AtlasFusion::DataLoader::CameraDataLoader>(
            "CameraIRDataLoader",
            datasetPath,
            AtlasFusion::DataLoader::CameraIdentifier::kCameraIr,
            AtlasFusion::Topics::kCameraIr,
            AtlasFusion::Topics::kDataLoaderSynchronization,
            nodeOptions
    );
    executor.add_node(irCameraDataLoader);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}