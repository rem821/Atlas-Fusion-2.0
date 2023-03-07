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

#include "data_loaders/ImuDataLoader.h"

namespace AtlasFusion::DataLoader {

    ImuDataLoader::ImuDataLoader(const std::string& name, const rclcpp::NodeOptions& options)
            : Node(name, options), latestDQuatTimestampPublished_(0), latestGnssTimestampPublished_(0), latestImuTimestampPublished_(0), latestMagTimestampPublished_(0),
              latestPressureTimestampPublished_(0), latestTempTimestampPublished_(0), latestTimeTimestampPublished_(0), synchronizationTimestamp_(0) {

        // Publisher that publishes ImuData
        kDQuatPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuDquatData>(Topics::kImuDquatDataLoader, 1);
        kGnssPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuGnssData>(Topics::kImuGnssDataLoader, 1);
        kImuPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuImuData>(Topics::kImuImuDataLoader, 1);
        kMagPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuMagData>(Topics::kImuMagDataLoader, 1);
        kPressurePublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuPressureData>(Topics::kImuPressureDataLoader, 1);
        kTempPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuTempData>(Topics::kImuTempDataLoader, 1);
        kTimePublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuTimeData>(Topics::kImuTimeDataLoader, 1);

        // Timestamp synchronization subscription to manage data loading speeds
        timestampSubscription_ = create_subscription<std_msgs::msg::UInt64>(
                Topics::kDataLoaderSynchronization,
                1,
                std::bind(&ImuDataLoader::OnSynchronizationTimestamp, this, std::placeholders::_1)
        );

        // Timer to control the polling frequency for publishing
        using namespace std::chrono_literals;
        timer_ = create_wall_timer(10ms, [this] { OnDataLoaderTimer(); });

        // Init imu additional data
        Initialize();
    }

    void ImuDataLoader::OnDataLoaderTimer() {
        // DQuat
        if (kDQuatDataFrame_ != nullptr && latestDQuatTimestampPublished_ <= synchronizationTimestamp_) {
            latestDQuatTimestampPublished_ = kDQuatDataFrame_->timestamp;

            LOG_TRACE("IMU DQuat data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(kDQuatDataFrame_.get()));

            kDQuatPublisher_->publish(std::move(kDQuatDataFrame_));
        }

        if (kDQuatDataIt_ < kDQuatData_.end() && kDQuatDataFrame_ == nullptr) {

            kDQuatDataFrame_ = std::move(*kDQuatDataIt_);
            kDQuatDataIt_ = std::next(kDQuatDataIt_, 1);
        }


        // Gnss
        if (kGnssDataFrame_ != nullptr && latestGnssTimestampPublished_ <= synchronizationTimestamp_) {
            latestGnssTimestampPublished_ = kGnssDataFrame_->timestamp;

            LOG_TRACE("IMU GNSS data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(kGnssDataFrame_.get()));

            kGnssPublisher_->publish(std::move(kGnssDataFrame_));
        }

        if (kGnssDataIt_ < kGnssData_.end() && kGnssDataFrame_ == nullptr) {

            kGnssDataFrame_ = std::move(*kGnssDataIt_);
            kGnssDataIt_ = std::next(kGnssDataIt_, 1);
        }


        // Imu
        if (kImuDataFrame_ != nullptr && latestImuTimestampPublished_ <= synchronizationTimestamp_) {
            latestImuTimestampPublished_ = kImuDataFrame_->timestamp;

            LOG_TRACE("IMU IMU data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(kImuDataFrame_.get()));

            kImuPublisher_->publish(std::move(kImuDataFrame_));
        }

        if (kImuDataIt_ < kImuData_.end() && kImuDataFrame_ == nullptr) {

            kImuDataFrame_ = std::move(*kImuDataIt_);
            kImuDataIt_ = std::next(kImuDataIt_, 1);
        }


        // Mag
        if (kMagDataFrame_ != nullptr && latestMagTimestampPublished_ <= synchronizationTimestamp_) {
            latestMagTimestampPublished_ = kMagDataFrame_->timestamp;

            LOG_TRACE("IMU Mag data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(kMagDataFrame_.get()));

            kMagPublisher_->publish(std::move(kMagDataFrame_));
        }

        if (kMagDataIt_ < kMagData_.end() && kMagDataFrame_ == nullptr) {

            kMagDataFrame_ = std::move(*kMagDataIt_);
            kMagDataIt_ = std::next(kMagDataIt_, 1);
        }


        // Pressure
        if (kPressureDataFrame_ != nullptr && latestPressureTimestampPublished_ <= synchronizationTimestamp_) {
            latestPressureTimestampPublished_ = kPressureDataFrame_->timestamp;

            LOG_TRACE("IMU Pressure data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(kPressureDataFrame_.get()));

            kPressurePublisher_->publish(std::move(kPressureDataFrame_));
        }

        if (kPressureDataIt_ < kPressureData_.end() && kPressureDataFrame_ == nullptr) {

            kPressureDataFrame_ = std::move(*kPressureDataIt_);
            kPressureDataIt_ = std::next(kPressureDataIt_, 1);
        }


        // Temp
        if (kTempDataFrame_ != nullptr && latestTempTimestampPublished_ <= synchronizationTimestamp_) {
            latestTempTimestampPublished_ = kTempDataFrame_->timestamp;

            LOG_TRACE("IMU Temp data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(kTempDataFrame_.get()));

            kTempPublisher_->publish(std::move(kTempDataFrame_));
        }

        if (kTempDataIt_ < kTempData_.end() && kTempDataFrame_ == nullptr) {

            kTempDataFrame_ = std::move(*kTempDataIt_);
            kTempDataIt_ = std::next(kTempDataIt_, 1);
        }


        // Time
        if (kTimeDataFrame_ != nullptr && latestTimeTimestampPublished_ <= synchronizationTimestamp_) {
            latestTimeTimestampPublished_ = kTimeDataFrame_->timestamp;

            LOG_TRACE("IMU Time data sent: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(kTimeDataFrame_.get()));

            kTimePublisher_->publish(std::move(kTimeDataFrame_));
        }

        if (kTimeDataIt_ < kTimeData_.end() && kTimeDataFrame_ == nullptr) {

            kTimeDataFrame_ = std::move(*kTimeDataIt_);
            kTimeDataIt_ = std::next(kTimeDataIt_, 1);
        }
    }

    void ImuDataLoader::OnSynchronizationTimestamp(const std_msgs::msg::UInt64& msg) {
        synchronizationTimestamp_ = msg.data;
    }

    void ImuDataLoader::Initialize() {
        LoadImuDquatData();
        LoadImuGnssData();
        LoadImuImuData();
        LoadImuMagData();
        LoadImuPressureData();
        LoadImuTempData();
        LoadImuTimeData();
    }

    void ImuDataLoader::LoadImuDquatData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + Folders::kImuFolder + Files::kDquatFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 5) {
                atlas_fusion_interfaces::msg::ImuDquatData data;
                data.timestamp = std::stoll(substrings[0]);
                data.quaternion.x = std::stod(substrings[1]);
                data.quaternion.x = std::stod(substrings[2]);
                data.quaternion.x = std::stod(substrings[3]);
                data.quaternion.x = std::stod(substrings[4]);

                kDQuatData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::ImuDquatData>(data));
            } else {
                throw std::runtime_error("Unexpected length of imu d_quat data");
            }
        }
        kDQuatDataIt_ = kDQuatData_.begin();
    }

    void ImuDataLoader::LoadImuGnssData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + Folders::kImuFolder + Files::kGnssFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 4) {
                atlas_fusion_interfaces::msg::ImuGnssData data;
                data.timestamp = std::stoll(substrings[0]);
                data.latitude = std::stod(substrings[1]);
                data.longitude = std::stod(substrings[2]);
                data.altitude = std::stod(substrings[3]);

                kGnssData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::ImuGnssData>(data));
            } else {
                throw std::runtime_error("Unexpected length of imu gnss data");
            }
        }
        kGnssDataIt_ = kGnssData_.begin();
    }

    void ImuDataLoader::LoadImuImuData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + Folders::kImuFolder + Files::kImuFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 11) {
                atlas_fusion_interfaces::msg::ImuImuData data;
                data.timestamp = std::stoll(substrings[0]);
                data.lin_acc.x = -std::stod(substrings[1]);
                data.lin_acc.y = -std::stod(substrings[2]);
                data.lin_acc.z = -std::stod(substrings[3]);
                data.ang_vel.x = std::stod(substrings[4]);
                data.ang_vel.y = std::stod(substrings[5]);
                data.ang_vel.z = std::stod(substrings[6]);
                data.orientation.x = std::stod(substrings[7]);
                data.orientation.y = std::stod(substrings[8]);
                data.orientation.z = std::stod(substrings[9]);
                data.orientation.w = std::stod(substrings[10]);
                kImuData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::ImuImuData>(data));
            } else {
                throw std::runtime_error("Unexpected length of imu gnss data");
            }
        }
        kImuDataIt_ = kImuData_.begin();
    }

    void ImuDataLoader::LoadImuMagData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + Folders::kImuFolder + Files::kMagFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 4) {
                atlas_fusion_interfaces::msg::ImuMagData data;
                data.timestamp = std::stoll(substrings[0]);
                data.mag.x = std::stod(substrings[1]);
                data.mag.y = std::stod(substrings[2]);
                data.mag.z = std::stod(substrings[3]);

                kMagData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::ImuMagData>(data));
            } else {
                throw std::runtime_error("Unexpected length of imu gnss data");
            }
        }
        kMagDataIt_ = kMagData_.begin();
    }

    void ImuDataLoader::LoadImuPressureData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + Folders::kImuFolder + Files::kPressureFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 2) {
                atlas_fusion_interfaces::msg::ImuPressureData data;
                data.timestamp = std::stoll(substrings[0]);
                data.pressure = std::stod(substrings[1]);

                kPressureData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::ImuPressureData>(data));
            } else {
                throw std::runtime_error("Unexpected length of imu gnss data");
            }
        }
        kPressureDataIt_ = kPressureData_.begin();
    }

    void ImuDataLoader::LoadImuTempData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + Folders::kImuFolder + Files::kTempFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 2) {
                atlas_fusion_interfaces::msg::ImuTempData data;
                data.timestamp = std::stoll(substrings[0]);
                data.temperature = std::stod(substrings[1]);

                kTempData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::ImuTempData>(data));
            } else {
                throw std::runtime_error("Unexpected length of imu gnss data");
            }
        }
        kTempDataIt_ = kTempData_.begin();
    }

    void ImuDataLoader::LoadImuTimeData() {
        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + Folders::kImuFolder + Files::kTimeFile);
        for (const auto& substrings: csvContent) {
            if (substrings.size() == 8) {
                atlas_fusion_interfaces::msg::ImuTimeData data;
                data.timestamp = std::stoll(substrings[0]);
                data.year = std::stod(substrings[1]);
                data.month = std::stod(substrings[2]);
                data.day = std::stod(substrings[3]);
                data.hour = std::stod(substrings[4]);
                data.minute = std::stod(substrings[5]);
                data.sec = std::stod(substrings[6]);
                data.nsec = std::stod(substrings[7]);

                kTimeData_.emplace_back(std::make_unique<atlas_fusion_interfaces::msg::ImuTimeData>(data));
            } else {
                throw std::runtime_error("Unexpected length of imu gnss data");
            }
        }
        kTimeDataIt_ = kTimeData_.begin();
    }


    void ImuDataLoader::Clear() {
        kDQuatData_.clear();
        kDQuatDataIt_ = kDQuatData_.begin();

        kGnssData_.clear();
        kGnssDataIt_ = kGnssData_.begin();

        kImuData_.clear();
        kImuDataIt_ = kImuData_.begin();

        kMagData_.clear();
        kMagDataIt_ = kMagData_.begin();

        kPressureData_.clear();
        kPressureDataIt_ = kPressureData_.begin();

        kTempData_.clear();
        kTempDataIt_ = kTempData_.begin();

        kTimeData_.clear();
        kTimeDataIt_ = kTimeData_.begin();
    }
}