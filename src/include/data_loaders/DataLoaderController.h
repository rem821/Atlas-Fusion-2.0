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

    class DataLoaderController : public rclcpp::Node {
        using DataIdentifier = std::variant<CameraIdentifier, LidarIdentifier, ImuLoaderIdentifier, GnssLoaderIdentifier, RadarIdentifier>;
        using DataMsg = std::variant<atlas_fusion_interfaces::msg::CameraData::UniquePtr, atlas_fusion_interfaces::msg::LidarData::UniquePtr, atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr, atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr, atlas_fusion_interfaces::msg::ImuImuData::UniquePtr, atlas_fusion_interfaces::msg::ImuMagData::UniquePtr, atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr, atlas_fusion_interfaces::msg::ImuTempData::UniquePtr, atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr, atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr, atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr, atlas_fusion_interfaces::msg::RadarData::UniquePtr>;

    public:
        DataLoaderController(const std::string &name, const uint8_t noDataLoaders, const rclcpp::NodeOptions &options);

    private:
        void onDataLoaderControllerTimer();

        void onCameraData(atlas_fusion_interfaces::msg::CameraData::UniquePtr msg);

        void onLidarData(atlas_fusion_interfaces::msg::LidarData::UniquePtr msg);

        void onImuDquatData(atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr msg);
        void onImuGnssData(atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr msg);
        void onImuImuData(atlas_fusion_interfaces::msg::ImuImuData::UniquePtr msg);
        void onImuMagData(atlas_fusion_interfaces::msg::ImuMagData::UniquePtr msg);
        void onImuPressureData(atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr msg);
        void onImuTempData(atlas_fusion_interfaces::msg::ImuTempData::UniquePtr msg);
        void onImuTimeData(atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr msg);

        void onGnssPositionData(atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr msg);
        void onGnssTimeData(atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr msg);

        void onRadarData(atlas_fusion_interfaces::msg::RadarData::UniquePtr msg);

        void initializeSubscribers();
        void initializePublishers();

        static uint64_t getDataTimestamp(const std::pair<DataIdentifier, DataMsg> &d);
        void retransmitMsg(const std::pair<DataIdentifier, DataMsg> &d);

        std::string datasetPath_;
        uint8_t noDataLoaders_;        // Keep this size the same as number of dataloaders


        /* Data loader Subscribers */
        std::map<CameraIdentifier, rclcpp::Subscription<atlas_fusion_interfaces::msg::CameraData>::SharedPtr> cameraSubscribers_;
        std::map<LidarIdentifier, rclcpp::Subscription<atlas_fusion_interfaces::msg::LidarData>::SharedPtr> lidarSubscribers_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuDquatData>::SharedPtr imuDquatSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuGnssData>::SharedPtr imuGnssSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuImuData>::SharedPtr imuImuSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuMagData>::SharedPtr imuMagSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuPressureData>::SharedPtr imuPressureSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuTempData>::SharedPtr imuTempSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuTimeData>::SharedPtr imuTimeSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::GnssPositionData>::SharedPtr gnssPositionSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::GnssTimeData>::SharedPtr gnssTimeSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::RadarData>::SharedPtr radarSubscriber_;

        /* Synchronized data re-publishers */
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;

        std::map<CameraIdentifier, rclcpp::Publisher<atlas_fusion_interfaces::msg::CameraData>::SharedPtr> cameraPublishers_;
        std::map<LidarIdentifier, rclcpp::Publisher<atlas_fusion_interfaces::msg::LidarData>::SharedPtr> lidarPublishers_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuDquatData>::SharedPtr imuDquatPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuGnssData>::SharedPtr imuGnssPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuImuData>::SharedPtr imuImuPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuMagData>::SharedPtr imuMagPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuPressureData>::SharedPtr imuPressurePublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuTempData>::SharedPtr imuTempPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::ImuTimeData>::SharedPtr imuTimePublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::GnssPositionData>::SharedPtr gnssPositionPublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::GnssTimeData>::SharedPtr gnssTimePublisher_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::RadarData>::SharedPtr radarPublisher_;

        /* Data cache */
        std::vector<std::pair<DataIdentifier, DataMsg>> dataCache_;

        uint64_t latestTimestampPublished_;
    };
}

