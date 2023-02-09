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
#include "Topics.h"

namespace AtlasFusion::DataLoader {

    LidarDataLoader::LidarDataLoader(const std::string &name,
                                       std::string datasetPath,
                                       const LidarIdentifier &lidarIdentifier,
                                       const std::string &topic,
                                       const rclcpp::NodeOptions &options)
            : Node(name, options), datasetPath_{std::move(datasetPath)}, lidarIdentifier_{lidarIdentifier},
              latestTimestampPublished_(0), synchronizationTimestamp_(0) {

        // Publisher that publishes LidarData
        publisher_ = create_publisher<atlas_fusion_interfaces::msg::LidarData>(topic, 1);

        // Timestamp synchronization subscription to manage data loading speeds
        timestampSubscription_ = create_subscription<std_msgs::msg::UInt64>(
                Topics::kDataLoaderSynchronization,
                1,
                std::bind(&LidarDataLoader::onSynchronizationTimestamp, this, std::placeholders::_1)
        );

        // Timer to control the polling frequency for publishing
        using namespace std::chrono_literals;
        timer_ = create_wall_timer(50ms, [this] { onDataLoaderTimer(); });

        // Init lidar additional data
        initialize();
    }

    void LidarDataLoader::onDataLoaderTimer() {
        if (dataFrame_ != nullptr && latestTimestampPublished_ <= synchronizationTimestamp_) {
            latestTimestampPublished_ = dataFrame_->timestamp;

            std::cout << "Lidar data of frame " << std::to_string(dataFrame_->lidar_identifier) << " sent: ("
                      << dataFrame_.get() << ", " << std::to_string(this->get_clock()->now().nanoseconds()) << ")"
                      << std::endl;

            publisher_->publish(std::move(dataFrame_));
        }

        if (!isOnEnd() && dataFrame_ == nullptr) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(dataIt_->pointCloudPath_, *scan) == -1) {
                throw std::runtime_error(fmt::format("Could not open pcd file: {}", dataIt_->pointCloudPath_));
            }

            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*scan, msg);

            atlas_fusion_interfaces::msg::LidarData lidarData;
            lidarData.lidar_identifier = static_cast<int8_t>(lidarIdentifier_);
            lidarData.timestamp = dataIt_->timestamp_;
            lidarData.inner_timestamp = dataIt_->innerTimestamp_;
            lidarData.point_cloud = msg;

            dataFrame_ = std::make_unique<atlas_fusion_interfaces::msg::LidarData>(lidarData);
            dataIt_ = std::next(dataIt_, 1);
        }
    }

    void LidarDataLoader::onSynchronizationTimestamp(const std_msgs::msg::UInt64 &msg) {
        synchronizationTimestamp_ = msg.data;
    }

    void LidarDataLoader::initialize() {
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

        auto csvContent = CsvReader::readCsv(datasetPath_ + folder + Files::kTimestampFile);
        for (const auto &substrings: csvContent) {
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
            scan_path << datasetPath_ << folder << Files::kScanFile << std::setw(6) << std::setfill('0')
               << scan_no << Files::kPcdExt;

            data_.emplace_back(timestamp, lidar_timestamp, scan_path.str());
        }
        dataIt_ = data_.begin();
        releaseIt_ = dataIt_;
    }

    bool LidarDataLoader::isOnEnd() const {
        return dataIt_ >= data_.end();
    }

    void LidarDataLoader::clear() {
        data_.clear();
        dataIt_ = data_.begin();
    }
}