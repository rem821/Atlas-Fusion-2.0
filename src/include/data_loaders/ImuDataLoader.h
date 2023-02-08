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
#include <atlas_fusion_interfaces/msg/lidar_data.hpp>
#include <utility>
#include "rcpputils/endian.hpp"

#include "data_loaders/DataLoaderIdentifiers.h"
#include "data_loaders/RecordingConstants.h"
#include "util/CsvReader.h"

namespace AtlasFusion::DataLoader {

    class ImuDataLoader : public rclcpp::Node {

        struct ImuFrame {

            /**
             * Constructor
             * @param ts Recording timestamp
             * @param iTs inner lidar's timestamp
             * @param pcPath point cloud file path
             */
            ImuFrame(uint64_t ts, uint64_t iTs, std::string pcPath)
                    : timestamp_(ts), innerTimestamp_(iTs), pointCloudPath_(std::move(pcPath)) {}

            uint64_t timestamp_;
            uint64_t innerTimestamp_;
            std::string pointCloudPath_;
        };

    public:
        ImuDataLoader(const std::string &name,
                         std::string datasetPath,
                         const ImuLoaderIdentifier &imuLoaderIdentifier,
                         const std::string &topic,
                         const std::string &synchronizationTopic,
                         const rclcpp::NodeOptions &options);

    private:
        void onDataLoaderTimer();

        void onSynchronizationTimestamp(const std_msgs::msg::UInt64 &msg);

        void initialize();

        bool isOnEnd() const;

        void clear();

        std::string datasetPath_;
        ImuLoaderIdentifier imuLoaderIdentifier_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::LidarData>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr timestampSubscription_;

        atlas_fusion_interfaces::msg::LidarData::UniquePtr dataFrame_;
        uint64_t latestTimestampPublished_;
        uint64_t synchronizationTimestamp_;

        std::vector<ImuFrame> data_;
        std::vector<ImuFrame>::iterator dataIt_;
        std::vector<ImuFrame>::iterator releaseIt_;
    };
}

