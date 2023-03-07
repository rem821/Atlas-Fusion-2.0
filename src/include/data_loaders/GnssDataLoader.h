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

namespace AtlasFusion::DataLoader {

    class GnssDataLoader : public rclcpp::Node {

    public:
        GnssDataLoader(const std::string &name, const rclcpp::NodeOptions &options);

    private:
        void onDataLoaderTimer();

        void onSynchronizationTimestamp(const std_msgs::msg::UInt64 &msg);

        void initialize();

        void loadGnssPositionData();
        void loadGnssTimeData();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::GnssPositionData>::SharedPtr positionPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::GnssTimeData>::SharedPtr timePublisher_;

        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr timestampSubscription_;

        atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr positionDataFrame_;
        atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr timeDataFrame_;

        uint64_t latestPositionTimestampPublished_;
        uint64_t latestTimeTimestampPublished_;

        uint64_t synchronizationTimestamp_;

        std::vector<atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr> positionData_;
        std::vector<atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr>::iterator positionDataIt_;

        std::vector<atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr> timeData_;
        std::vector<atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr>::iterator timeDataIt_;

    };
}