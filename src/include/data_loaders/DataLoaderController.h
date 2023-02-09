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
#include <atlas_fusion_interfaces/msg/lidar_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_dquat_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_gnss_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_imu_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_mag_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_pressure_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_temp_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_time_data.hpp>
#include <any>
#include "rcpputils/endian.hpp"

#include "data_loaders/DataLoaderIdentifiers.h"
#include "data_loaders/RecordingConstants.h"
#include "util/CsvReader.h"

#include "Topics.h"

namespace AtlasFusion::DataLoader {

    class DataLoaderController : public rclcpp::Node {
        using DataIdentifier = std::variant<CameraIdentifier, LidarIdentifier, ImuLoaderIdentifier>;
        using DataMsg = std::variant<atlas_fusion_interfaces::msg::CameraData::UniquePtr, atlas_fusion_interfaces::msg::LidarData::UniquePtr, atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr, atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr, atlas_fusion_interfaces::msg::ImuImuData::UniquePtr, atlas_fusion_interfaces::msg::ImuMagData::UniquePtr, atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr, atlas_fusion_interfaces::msg::ImuTempData::UniquePtr, atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr>;

    public:
        DataLoaderController(const std::string &name,
                             const rclcpp::NodeOptions &options);

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

        void initialize();

        static uint64_t getDataTimestamp(const std::pair<DataIdentifier, DataMsg> &d);

        std::string datasetPath_;

        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        std::map<CameraIdentifier, rclcpp::Subscription<atlas_fusion_interfaces::msg::CameraData>::SharedPtr> cameraSubscribers_;
        std::map<LidarIdentifier, rclcpp::Subscription<atlas_fusion_interfaces::msg::LidarData>::SharedPtr> lidarSubscribers_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuDquatData>::SharedPtr imuDquatSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuGnssData>::SharedPtr imuGnssSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuImuData>::SharedPtr imuImuSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuMagData>::SharedPtr imuMagSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuPressureData>::SharedPtr imuPressureSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuTempData>::SharedPtr imuTempSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuTimeData>::SharedPtr imuTimeSubscriber_;

        std::vector<std::pair<DataIdentifier, DataMsg>> dataCache_;

        uint64_t latestTimestampPublished_;
    };
}

