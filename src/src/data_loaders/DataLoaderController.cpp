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

        using namespace std::chrono_literals;
        timer_ = create_wall_timer(200ms, std::bind(&DataLoaderController::onDataLoaderControllerTimer, this));

        // Init all subscribers
        initialize();
    }

    void DataLoaderController::onDataLoaderControllerTimer() {
        std::cout << "DataLoaderController: Retransmitting " << dataCache_.size() << " elements in order" << std::endl;
        while (!dataCache_.empty()) {
            auto min = std::min_element(
                    std::begin(dataCache_), std::end(dataCache_),
                    [](const auto &l, const auto &r) {
                        return l.second->timestamp < r.second->timestamp;
                    }
            );
            std_msgs::msg::UInt64 m;
            m.data = min->second->timestamp;
            std::cout << "Publishing synchronization timestamp: " << m.data << std::endl;
            publisher_->publish(m);
            if(latestTimestampPublished_ > m.data) {
                std::cout << "Desynchronization of: " << (latestTimestampPublished_ - m.data) / 1000000.0 << " ms!!!" << std::endl;
            }
            latestTimestampPublished_ = m.data;

            auto erase = std::find(dataCache_.begin(), dataCache_.end(), *min);
            if (erase != dataCache_.end()) dataCache_.erase(erase);
        }
        std::cout << "DataLoaderController: Data retransmission complete!" << std::endl;
    }

    void DataLoaderController::onCameraData(const atlas_fusion_interfaces::msg::CameraData &msg) {
        std::cout << "DataLoaderController: Camera data of frame " << std::to_string(msg.camera_identifier) << " arrived: ("
                  << &msg << ", " << std::to_string(this->get_clock()->now().nanoseconds()) << ")"
                  << std::endl;
        auto id = static_cast<CameraIdentifier>(msg.camera_identifier);

        dataCache_.emplace_back() = std::pair(id, std::make_shared<atlas_fusion_interfaces::msg::CameraData>(msg));
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