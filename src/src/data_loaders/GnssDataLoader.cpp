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

#include "data_loaders/GnssDataLoader.h"
#include <EntryPoint.h>

namespace AtlasFusion::DataLoader {

    GnssDataLoader::GnssDataLoader(const std::string& name, const rclcpp::NodeOptions& options)
            : Node(name, options), latestPositionTimestampPublished_(0), latestTimeTimestampPublished_(0), synchronizationTimestamp_(0) {

        // Publisher that publishes GnssData
        positionPublisher_ = create_publisher<atlas_fusion_interfaces::msg::GnssPositionData>(Topics::kGnssPositionDataLoader, 1);
        timePublisher_ = create_publisher<atlas_fusion_interfaces::msg::GnssTimeData>(Topics::kGnssTimeDataLoader, 1);

        // Timestamp synchronization subscription to manage data loading speeds
        timestampSubscription_ = create_subscription<std_msgs::msg::UInt64>(
                Topics::kDataLoaderSynchronization,
                1,
                std::bind(&GnssDataLoader::onSynchronizationTimestamp, this, std::placeholders::_1)
        );

        // Timer to control the polling frequency for publishing
        using namespace std::chrono_literals;
        timer_ = create_wall_timer(10ms, [this] { onDataLoaderTimer(); });

        // Init gnss additional data
        initialize();
    }

    void GnssDataLoader::onDataLoaderTimer() {
        // Position
        if (positionDataFrame_ != nullptr && latestPositionTimestampPublished_ <= synchronizationTimestamp_) {
            latestPositionTimestampPublished_ = positionDataFrame_->timestamp;

            LOG_TRACE("GNSS Position data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(positionDataFrame_.get()));

            positionPublisher_->publish(std::move(positionDataFrame_));
        }

        if (positionDataIt_ < positionData_.end() && positionDataFrame_ == nullptr) {

            positionDataFrame_ = std::move(*positionDataIt_);
            positionDataIt_ = std::next(positionDataIt_, 1);
        }


        // Time
        if (timeDataFrame_ != nullptr && latestTimeTimestampPublished_ <= synchronizationTimestamp_) {
            latestTimeTimestampPublished_ = timeDataFrame_->timestamp;

            LOG_TRACE("GNSS Time data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(timeDataFrame_.get()));

            timePublisher_->publish(std::move(timeDataFrame_));
        }

        if (timeDataIt_ < timeData_.end() && timeDataFrame_ == nullptr) {

            timeDataFrame_ = std::move(*timeDataIt_);
            timeDataIt_ = std::next(timeDataIt_, 1);
        }
    }

    void GnssDataLoader::onSynchronizationTimestamp(const std_msgs::msg::UInt64& msg) {
        synchronizationTimestamp_ = msg.data;
    }

    void GnssDataLoader::initialize() {
        loadGnssPositionData();
        loadGnssTimeData();
    }

    void GnssDataLoader::loadGnssPositionData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::readCsv(datasetPath + Folders::kGnssFolder + Files::kPoseFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 5) {
                atlas_fusion_interfaces::msg::GnssPositionData data;
                data.timestamp = std::stoll(substrings[0]);
                data.latitude = std::stod(substrings[1]);
                data.longitude = std::stod(substrings[2]);
                data.altitude = std::stod(substrings[3]);
                data.azimuth = std::stod(substrings[4]);

                positionData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::GnssPositionData>(data));
            } else {
                throw std::runtime_error("Unexpected length of gnss pose data");
            }
            positionDataIt_ = positionData_.begin();
        }
    }

    void GnssDataLoader::loadGnssTimeData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::readCsv(datasetPath + Folders::kGnssFolder + Files::kTimeFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 8) {
                atlas_fusion_interfaces::msg::GnssTimeData data;
                data.timestamp = std::stoll(substrings[0]);
                data.year = std::stoll(substrings[1]);
                data.month = std::stoll(substrings[2]);
                data.day = std::stoll(substrings[3]);
                data.hour = std::stoll(substrings[4]);
                data.minute = std::stoll(substrings[5]);
                data.sec = std::stoll(substrings[6]);
                data.nsec = std::stoll(substrings[7]);

                timeData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::GnssTimeData>(data));
            } else {
                throw std::runtime_error("Unexpected length of gnss time data");
            }
            timeDataIt_ = timeData_.begin();
        }
    }
}
