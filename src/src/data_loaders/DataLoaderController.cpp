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

#include "data_loaders/DataLoaderController.h"

namespace AtlasFusion::DataLoader {

    DataLoaderController::DataLoaderController(const std::string &name,
                                               const std::string &synchronizationTopic,
                                               const rclcpp::NodeOptions &options)
            : Node(name, options), latestTimestampPublished_(0) {

        // Publisher that publishes synchronization timestamps
        publisher_ = create_publisher<std_msgs::msg::UInt64>(synchronizationTopic, 1);

        // Init all subscribers
        initialize();
    }

    void DataLoaderController::onCameraData(const atlas_fusion_interfaces::msg::CameraData &msg) {
        std::cout << "Camera data of frame " << std::to_string(msg.camera_identifier) << " arrived" << std::endl;

        auto id = static_cast<CameraIdentifier>(msg.camera_identifier);
        if (cameraDataCache_[id] != nullptr) {
            throw std::runtime_error("CameraDataLoader misbehaving");
        }
        cameraDataCache_[id] = std::make_shared<atlas_fusion_interfaces::msg::CameraData>(msg);


        if (cameraDataCache_.size() == cameraSubscribers_.size()) {
            while(!cameraDataCache_.empty()) {
                auto it = std::min_element(
                        std::begin(cameraDataCache_), std::end(cameraDataCache_),
                        [](const auto &l, const auto &r) {
                            return l.second->timestamp < r.second->timestamp;
                        }
                );
                std_msgs::msg::UInt64 m;
                m.data = it->second->timestamp;
                publisher_->publish(m);

                cameraDataCache_.erase(it->first);
            }
            std::cout << "All sensors passed in order!" << std::endl;
        }
    }

    void DataLoaderController::initialize() {
        cameraSubscribers_[CameraIdentifier::kCameraLeftSide] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraLeftSideDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraLeftFront] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraLeftFrontDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraRightFront] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraRightFrontDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraRightSide] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraRightSideDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraIr] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraIrDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );
    }
}