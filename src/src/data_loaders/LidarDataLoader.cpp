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

#include "data_loaders/LidarDataLoader.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace AtlasFusion::DataLoader {

    LidarDataLoader::LidarDataLoader(const std::string& name, const LidarIdentifier& lidarIdentifier, const std::string& topic, const rclcpp::NodeOptions& options)
            : Node(name, options), lidarIdentifier_{lidarIdentifier}, latestTimestampPublished_(0), synchronizationTimestamp_(0) {

        // Publisher that publishes LidarData
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, 1);

        // Timestamp synchronization subscription to manage data loading speeds
        timestampSubscription_ = create_subscription<std_msgs::msg::UInt64>(
                Topics::kDataLoaderSynchronization,
                1,
                std::bind(&LidarDataLoader::OnSynchronizationTimestamp, this, std::placeholders::_1)
        );

        // Timer to control the polling frequency for publishing
        using namespace std::chrono_literals;
        timer_ = create_wall_timer(10ms, [this] { OnDataLoaderTimer(); });

        // Init lidar additional data
        Initialize();
    }

    void LidarDataLoader::OnDataLoaderTimer() {
        if (dataFrame_ != nullptr && latestTimestampPublished_ <= synchronizationTimestamp_) {
            latestTimestampPublished_ = STAMP_TO_NANOSEC(dataFrame_->header.stamp);

            LOG_TRACE("Lidar data of frame {} sent: ({}, {})", dataFrame_->lidar_identifier, this->get_clock()->now().nanoseconds(), HEX_ADDR(dataFrame_.get()));

            publisher_->publish(std::move(dataFrame_));
        }

        if (!IsOnEnd() && dataFrame_ == nullptr) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(dataIt_->pointCloudPath_, *scan) == -1) {
                throw std::runtime_error(fmt::format("Could not open pcd file: {}", dataIt_->pointCloudPath_));
            }

            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*scan, msg);
            msg.header.stamp.sec = NANOSEC_TO_STAMP_SEC(dataIt_->timestamp_);
            msg.header.stamp.nanosec = NANOSEC_TO_STAMP_NANOSEC(dataIt_->timestamp_);
            msg.header.frame_id = FrameTypeName(FrameTypeFromIdentifier(lidarIdentifier_));

            dataFrame_ = std::make_unique<sensor_msgs::msg::PointCloud2>(msg);
            dataIt_ = std::next(dataIt_, 1);
        }
    }

    void LidarDataLoader::OnSynchronizationTimestamp(const std_msgs::msg::UInt64& msg) {
        synchronizationTimestamp_ = msg.data;
    }

    void LidarDataLoader::Initialize() {
        std::string folder;
        switch (lidarIdentifier_) {
            case LidarIdentifier::kLeftLidar:
                folder = Folders::kLidarLeftFolder;
                break;
            case LidarIdentifier::kRightLidar:
                folder = Folders::kLidarRightFolder;
                break;
            case LidarIdentifier::kCenterLidar:
                folder = Folders::kLidarCenterFolder;
                break;
        }

        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + folder + Files::kTimestampFile);
        for (const auto& substrings: csvContent) {
            size_t timestamp = 0;
            size_t scan_no = 0;
            size_t lidar_timestamp = 0;

            if (substrings.size() == 2) {
                timestamp = std::stoll(substrings[0]);
                scan_no = std::stoll(substrings[1]);
            } else if (substrings.size() == 3) {
                timestamp = std::stoll(substrings[0]);
                scan_no = std::stoll(substrings[1]);
                lidar_timestamp = std::stoll(substrings[2]);
            } else {
                throw std::runtime_error("Unexpected length of lidar scan data");
            }

            std::stringstream scan_path;
            scan_path << datasetPath << folder << Files::kScanFile << std::setw(6) << std::setfill('0')
                      << scan_no << Files::kPcdExt;

            data_.emplace_back(timestamp, lidar_timestamp, scan_path.str());
        }
        dataIt_ = data_.begin();
        releaseIt_ = dataIt_;
    }

    bool LidarDataLoader::IsOnEnd() const {
        return dataIt_ >= data_.end();
    }

    void LidarDataLoader::Clear() {
        data_.clear();
        dataIt_ = data_.begin();
    }
}