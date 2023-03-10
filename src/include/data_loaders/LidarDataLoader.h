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

#include <precompiled_headers/PCH.h>
#include <std_msgs/msg/u_int64.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace AtlasFusion::DataLoader {

    class LidarDataLoader : public rclcpp::Node {

        struct LidarFrame {

            /**
             * Constructor
             * @param ts Recording timestamp
             * @param iTs inner lidar's timestamp
             * @param pcPath point cloud file path
             */
            LidarFrame(uint64_t ts, uint64_t iTs, std::string pcPath)
                    : timestamp_(ts), innerTimestamp_(iTs), pointCloudPath_(std::move(pcPath)) {}

            uint64_t timestamp_;
            uint64_t innerTimestamp_;
            std::string pointCloudPath_;
        };

    public:
        LidarDataLoader(const std::string &name,
                         const LidarIdentifier &lidarIdentifier,
                         const std::string &topic,
                         const rclcpp::NodeOptions &options);

    private:
        void OnDataLoaderTimer();

        void OnSynchronizationTimestamp(const std_msgs::msg::UInt64 &msg);

        void Initialize();

        bool IsOnEnd() const;

        void Clear();

        LidarIdentifier lidarIdentifier_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr timestampSubscription_;

        sensor_msgs::msg::PointCloud2::UniquePtr dataFrame_;
        uint64_t latestTimestampPublished_;
        uint64_t synchronizationTimestamp_;

        std::vector<LidarFrame> data_;
        std::vector<LidarFrame>::iterator dataIt_;
        std::vector<LidarFrame>::iterator releaseIt_;
    };
}

