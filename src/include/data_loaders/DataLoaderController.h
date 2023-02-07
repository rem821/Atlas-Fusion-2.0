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
#pragma once

#include <std_msgs/msg/u_int64.hpp>
#include <atlas_fusion_interfaces/msg/camera_data.hpp>
#include "rcpputils/endian.hpp"

#include "data_loaders/DataLoaderIdentifiers.h"
#include "data_loaders/RecordingConstants.h"
#include "util/CsvReader.h"

#include "Topics.h"


namespace AtlasFusion::DataLoader {

    class DataLoaderController : public rclcpp::Node {

    public:
        DataLoaderController(const std::string &name,
                             const std::string &synchronizationTopic,
                             const rclcpp::NodeOptions &options);

    private:
        void onCameraData(const atlas_fusion_interfaces::msg::CameraData &msg);

        void initialize();

        std::string datasetPath_;
        CameraIdentifier cameraIdentifier_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;

        std::map<CameraIdentifier, rclcpp::Subscription<atlas_fusion_interfaces::msg::CameraData>::SharedPtr> cameraSubscribers_;
        std::map<CameraIdentifier, atlas_fusion_interfaces::msg::CameraData::SharedPtr> cameraDataCache_;

        uint64_t latestTimestampPublished_;
    };
}

