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

#include "data_loaders/RadarDataLoader.h"
#include "Topics.h"

namespace AtlasFusion::DataLoader {

    RadarDataLoader::RadarDataLoader(const std::string &name,
                                     std::string datasetPath,
                                     const std::string &topic,
                                     const rclcpp::NodeOptions &options)
            : Node(name, options), datasetPath_{std::move(datasetPath)}, latestTimestampPublished_(0), synchronizationTimestamp_(0) {

        // Publisher that publishes RadarData
        publisher_ = create_publisher<atlas_fusion_interfaces::msg::RadarData>(topic, 1);

        // Timestamp synchronization subscription to manage data loading speeds
        timestampSubscription_ = create_subscription<std_msgs::msg::UInt64>(
                Topics::kDataLoaderSynchronization,
                1,
                std::bind(&RadarDataLoader::onSynchronizationTimestamp, this, std::placeholders::_1)
        );

        // Timer to control the polling frequency for publishing
        using namespace std::chrono_literals;
        timer_ = create_wall_timer(50ms, [this] { onDataLoaderTimer(); });

        // Init radar additional data
        initialize();
    }

    void RadarDataLoader::onDataLoaderTimer() {
        if (dataFrame_ != nullptr && latestTimestampPublished_ <= synchronizationTimestamp_) {
            latestTimestampPublished_ = dataFrame_->timestamp;

            std::cout << "Radar data sent: ("
                      << dataFrame_.get() << ", " << std::to_string(this->get_clock()->now().nanoseconds()) << ")"
                      << std::endl;

            publisher_->publish(std::move(dataFrame_));
        }

        if (!isOnEnd() && dataFrame_ == nullptr) {
            std::vector<atlas_fusion_interfaces::msg::RadarDetection> detections;
            for (auto &det: dataIt_->radarDetections_) {
                geometry_msgs::msg::Point32 point;
                point.x = det.pose_.x();
                point.y = det.pose_.y();
                point.z = det.pose_.z();
                atlas_fusion_interfaces::msg::RadarDetection radarDetection;
                radarDetection.pose = point;
                radarDetection.velocity = det.velocity_;
                detections.emplace_back(radarDetection);
            }

            atlas_fusion_interfaces::msg::RadarData radarData;
            radarData.radar_identifier = 0;
            radarData.timestamp = dataIt_->timestamp_;
            radarData.no_detections = dataIt_->numberOfDetections_;
            radarData.detections = detections;

            dataFrame_ = std::make_unique<atlas_fusion_interfaces::msg::RadarData>(radarData);
            dataIt_ = std::next(dataIt_, 1);
        }
    }

    void RadarDataLoader::onSynchronizationTimestamp(const std_msgs::msg::UInt64 &msg) {
        synchronizationTimestamp_ = msg.data;
    }

    void RadarDataLoader::initialize() {
        auto csvContent = CsvReader::readCsv(datasetPath_ + Folders::kRadarTi + Files::kRadarTiScan);
        for (const auto &substrings: csvContent) {
            if (substrings.size() >= 2) {

                size_t timestamp = std::stoull(substrings.at(0));
                uint16_t numberOfDetections = std::stoull(substrings.at(1));

                std::vector<RadarDetection> detections;
                for (uint16_t i = 0; i < numberOfDetections; i++) {
                    size_t offset = 2 + (4 * i);
                    float x = std::stof(substrings.at(offset + 0));
                    float y = std::stof(substrings.at(offset + 1));
                    float z = std::stof(substrings.at(offset + 2));
                    float vel = std::stof(substrings.at(offset + 3));
                    detections.emplace_back(RadarDetection{{x, y, z}, vel});
                }
                data_.push_back({timestamp, numberOfDetections, detections});
            }
        }
        dataIt_ = data_.begin();
        releaseIt_ = dataIt_;
    }

    bool RadarDataLoader::isOnEnd() const {
        return dataIt_ >= data_.end();
    }

    void RadarDataLoader::clear() {
        data_.clear();
        dataIt_ = data_.begin();
    }
}