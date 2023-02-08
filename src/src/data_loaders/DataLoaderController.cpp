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

    void DataLoaderController::onDataLoaderControllerTimer() {
        // Keep this size the same as number of dataloaders
        if(dataCache_.size() < 8) return;
        std::cout << "DataLoaderController: Retransmitting " << dataCache_.size() << " elements in order" << std::endl;
        //while (!dataCache_.empty()) {
            auto min = std::min_element(
                    std::begin(dataCache_), std::end(dataCache_),
                    [](const auto &l, const auto &r) {
                        return getDataTimestamp(l) < getDataTimestamp(r);
                    }
            );
            std_msgs::msg::UInt64 m;
            m.data = getDataTimestamp(*min);
            std::cout << "Publishing synchronization timestamp: " << m.data << std::endl;
            publisher_->publish(m);
            if (latestTimestampPublished_ > m.data) {
                std::cout << "Desynchronization of: " << (latestTimestampPublished_ - m.data) / 1000000 << " ms!!!"
                          << std::endl;
            }
            latestTimestampPublished_ = m.data;

            auto erase = std::find(dataCache_.begin(), dataCache_.end(), *min);
            if (erase != dataCache_.end()) dataCache_.erase(erase);
        //}
        std::cout << "DataLoaderController: Data retransmission complete!" << std::endl;
    }

    void DataLoaderController::onCameraData(atlas_fusion_interfaces::msg::CameraData::UniquePtr msg) {
        std::cout << "DataLoaderController: Camera data of frame " << std::to_string(msg->camera_identifier)
                  << " arrived: ("
                  << msg.get() << ", " << std::to_string(this->get_clock()->now().nanoseconds()) << ")"
                  << std::endl;
        auto id = static_cast<CameraIdentifier>(msg->camera_identifier);

        dataCache_.emplace_back(id, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onLidarData(atlas_fusion_interfaces::msg::LidarData::UniquePtr msg) {
        std::cout << "DataLoaderController: Lidar data of frame " << std::to_string(msg->lidar_identifier)
                  << " arrived: ("
                  << msg.get() << ", " << std::to_string(this->get_clock()->now().nanoseconds()) << ")"
                  << std::endl;
        auto id = static_cast<LidarIdentifier>(msg->lidar_identifier);

        dataCache_.emplace_back(id, std::move(msg));
        onDataLoaderControllerTimer();
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


        lidarSubscribers_[LidarIdentifier::kLeftLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarLeftDataLoader,
                1,
                std::bind(&DataLoaderController::onLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[LidarIdentifier::kCenterLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarCenterDataLoader,
                1,
                std::bind(&DataLoaderController::onLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[LidarIdentifier::kRightLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarRightDataLoader,
                1,
                std::bind(&DataLoaderController::onLidarData, this, std::placeholders::_1)
        );
    }

    uint64_t DataLoaderController::getDataTimestamp(const std::pair<DataIdentifier, DataMsg> &d) {
        auto d_i = d.second.index();
        if (d_i == 0) {
            return std::get<atlas_fusion_interfaces::msg::CameraData::UniquePtr>(d.second)->timestamp;
        } else if (d_i == 1) {
            return std::get<atlas_fusion_interfaces::msg::LidarData::UniquePtr>(d.second)->timestamp;
        } else {
            throw std::runtime_error("Unexpected variant type when comparing timestamps!");
        }
    }
}