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

    class ImuDataLoader : public rclcpp::Node {

    public:
        ImuDataLoader(const std::string &name, std::string datasetPath, const rclcpp::NodeOptions &options);

    private:
        void onDataLoaderTimer();

        void onSynchronizationTimestamp(const std_msgs::msg::UInt64 &msg);

        void initialize();

        void loadImuDquatData();

        void loadImuGnssData();

        void loadImuImuData();

        void loadImuMagData();

        void loadImuPressureData();

        void loadImuTempData();

        void loadImuTimeData();

        void clear();

        std::string datasetPath_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuDquatData>::SharedPtr kDQuatPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuGnssData>::SharedPtr kGnssPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuImuData>::SharedPtr kImuPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuMagData>::SharedPtr kMagPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuPressureData>::SharedPtr kPressurePublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuTempData>::SharedPtr kTempPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuTimeData>::SharedPtr kTimePublisher_;

        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr timestampSubscription_;

        atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr kDQuatDataFrame_;
        atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr kGnssDataFrame_;
        atlas_fusion_interfaces::msg::ImuImuData::UniquePtr kImuDataFrame_;
        atlas_fusion_interfaces::msg::ImuMagData::UniquePtr kMagDataFrame_;
        atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr kPressureDataFrame_;
        atlas_fusion_interfaces::msg::ImuTempData::UniquePtr kTempDataFrame_;
        atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr kTimeDataFrame_;

        uint64_t latestDQuatTimestampPublished_;
        uint64_t latestGnssTimestampPublished_;
        uint64_t latestImuTimestampPublished_;
        uint64_t latestMagTimestampPublished_;
        uint64_t latestPressureTimestampPublished_;
        uint64_t latestTempTimestampPublished_;
        uint64_t latestTimeTimestampPublished_;

        uint64_t synchronizationTimestamp_;

        std::vector<atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr> kDQuatData_;
        std::vector<atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr>::iterator kDQuatDataIt_;

        std::vector<atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr> kGnssData_;
        std::vector<atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr>::iterator kGnssDataIt_;

        std::vector<atlas_fusion_interfaces::msg::ImuImuData::UniquePtr> kImuData_;
        std::vector<atlas_fusion_interfaces::msg::ImuImuData::UniquePtr>::iterator kImuDataIt_;

        std::vector<atlas_fusion_interfaces::msg::ImuMagData::UniquePtr> kMagData_;
        std::vector<atlas_fusion_interfaces::msg::ImuMagData::UniquePtr>::iterator kMagDataIt_;

        std::vector<atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr> kPressureData_;
        std::vector<atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr>::iterator kPressureDataIt_;

        std::vector<atlas_fusion_interfaces::msg::ImuTempData::UniquePtr> kTempData_;
        std::vector<atlas_fusion_interfaces::msg::ImuTempData::UniquePtr>::iterator kTempDataIt_;

        std::vector<atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr> kTimeData_;
        std::vector<atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr>::iterator kTimeDataIt_;
    };
}

