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

#include "lidar_processing/LidarAggregator.h"

namespace AtlasFusion::LocalMap {

    LidarAggregator::LidarAggregator(const std::string& name, const std::string &topic, const rclcpp::NodeOptions &options)
            : Node(name, options) {

        // Publisher that publishes aggregated lidar data
        // publisher_ = create_publisher<atlas_fusion_interfaces::msg::CameraData>(topic, 1);


        lidarSubscribers_[DataLoader::LidarIdentifier::kLeftLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarLeft,
                1,
                std::bind(&LidarAggregator::onLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[DataLoader::LidarIdentifier::kCenterLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarCenter,
                1,
                std::bind(&LidarAggregator::onLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[DataLoader::LidarIdentifier::kRightLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarRight,
                1,
                std::bind(&LidarAggregator::onLidarData, this, std::placeholders::_1)
        );
    }

    void LidarAggregator::onLidarData(atlas_fusion_interfaces::msg::LidarData::UniquePtr msg) {
        LOG_INFO("LidarAggregator: Lidar data of frame {} arrived: ({}, {})", msg->lidar_identifier, this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        auto lidarID = static_cast<DataLoader::LidarIdentifier>(msg->lidar_identifier);

        //dataCache_.emplace_back(id, std::move(msg));
        //onDataLoaderControllerTimer();
    }
}