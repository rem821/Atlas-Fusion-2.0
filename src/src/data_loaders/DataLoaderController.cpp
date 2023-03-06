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

#include "data_loaders/DataLoaderController.h"

namespace AtlasFusion::DataLoader {

    DataLoaderController::DataLoaderController(const std::string &name, const uint8_t noDataLoaders, const rclcpp::NodeOptions &options)
            : Node(name, options), noDataLoaders_(noDataLoaders), latestTimestampPublished_(0) {

        initializePublishers();
        initializeSubscribers();
    }

    void DataLoaderController::onDataLoaderControllerTimer() {
        if (dataCache_.size() < noDataLoaders_) return;

        // Find the index of the earliest msg in cache
        auto min = std::min_element(
                std::begin(dataCache_), std::end(dataCache_),
                [](const auto &l, const auto &r) {
                    return getDataTimestamp(l) < getDataTimestamp(r);
                }
        );
        auto max = std::max_element(
                std::begin(dataCache_), std::end(dataCache_),
                [](const auto &l, const auto &r) {
                    return getDataTimestamp(l) < getDataTimestamp(r);
                }
        );
        LOG_TRACE("DataLoader cache extent: {}ms", (getDataTimestamp(*max) - getDataTimestamp(*min)) / 1000000);
        retransmitMsg(*min);

        std_msgs::msg::UInt64 m;
        m.data = getDataTimestamp(*min);

        LOG_TRACE("Publishing synchronization timestamp: {}", m.data);
        publisher_->publish(m);
        if (latestTimestampPublished_ > m.data) {
            LOG_ERROR("Desynchronization of: {} ms!!!", (latestTimestampPublished_ - m.data) / 1000000);
        }
        latestTimestampPublished_ = m.data;

        auto erase = std::find(dataCache_.begin(), dataCache_.end(), *min);
        if (erase != dataCache_.end()) dataCache_.erase(erase);

    }

    void DataLoaderController::onCameraData(atlas_fusion_interfaces::msg::CameraData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: Camera data of frame {} arrived: ({}, {})", msg->camera_identifier, this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        auto id = static_cast<CameraIdentifier>(msg->camera_identifier);

        dataCache_.emplace_back(id, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onLidarData(atlas_fusion_interfaces::msg::LidarData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: Lidar data of frame {} arrived: ({}, {})", msg->lidar_identifier, this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        auto id = static_cast<LidarIdentifier>(msg->lidar_identifier);

        dataCache_.emplace_back(id, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onImuDquatData(atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: IMU DQuat data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(ImuLoaderIdentifier::kDQuat, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onImuGnssData(atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: IMU GNSS data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(ImuLoaderIdentifier::kGnss, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onImuImuData(atlas_fusion_interfaces::msg::ImuImuData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: IMU IMU data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(ImuLoaderIdentifier::kImu, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onImuMagData(atlas_fusion_interfaces::msg::ImuMagData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: IMU Mag data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(ImuLoaderIdentifier::kMag, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onImuPressureData(atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: IMU Pressure data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(ImuLoaderIdentifier::kPressure, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onImuTempData(atlas_fusion_interfaces::msg::ImuTempData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: IMU Temp data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(ImuLoaderIdentifier::kTemp, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onImuTimeData(atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: IMU Time data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(ImuLoaderIdentifier::kTime, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onGnssPositionData(atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: GNSS Pos data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(GnssLoaderIdentifier::kPose, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onGnssTimeData(atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: GNSS Time data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(GnssLoaderIdentifier::kTime, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::onRadarData(atlas_fusion_interfaces::msg::RadarData::UniquePtr msg) {
        LOG_TRACE("DataLoaderController: Radar data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        dataCache_.emplace_back(RadarIdentifier::kRadarTi, std::move(msg));
        onDataLoaderControllerTimer();
    }

    void DataLoaderController::initializeSubscribers() {
        cameraSubscribers_[CameraIdentifier::kCameraLeftSide] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraLeftSideDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraLeftFront] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraLeftFrontDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraRightFront] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraRightFrontDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraRightSide] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraRightSideDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[CameraIdentifier::kCameraIr] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraIrDataLoader,
                1,
                std::bind(&DataLoaderController::onCameraData, this, std::placeholders::_1)
        );


        lidarSubscribers_[LidarIdentifier::kLeftLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarLeftDataLoader,
                1,
                std::bind(&DataLoaderController::onLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[LidarIdentifier::kCenterLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarCenterDataLoader,
                1,
                std::bind(&DataLoaderController::onLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[LidarIdentifier::kRightLidar] = create_subscription<atlas_fusion_interfaces::msg::LidarData>(
                Topics::kLidarRightDataLoader,
                1,
                std::bind(&DataLoaderController::onLidarData, this, std::placeholders::_1)
        );


        imuDquatSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuDquatData>(
                Topics::kImuDquatDataLoader,
                1,
                std::bind(&DataLoaderController::onImuDquatData, this, std::placeholders::_1)
        );

        imuGnssSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuGnssData>(
                Topics::kImuGnssDataLoader,
                1,
                std::bind(&DataLoaderController::onImuGnssData, this, std::placeholders::_1)
        );

        imuImuSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuImuData>(
                Topics::kImuImuDataLoader,
                1,
                std::bind(&DataLoaderController::onImuImuData, this, std::placeholders::_1)
        );

        imuMagSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuMagData>(
                Topics::kImuMagDataLoader,
                1,
                std::bind(&DataLoaderController::onImuMagData, this, std::placeholders::_1)
        );

        imuPressureSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuPressureData>(
                Topics::kImuPressureDataLoader,
                1,
                std::bind(&DataLoaderController::onImuPressureData, this, std::placeholders::_1)
        );

        imuTempSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuTempData>(
                Topics::kImuTempDataLoader,
                1,
                std::bind(&DataLoaderController::onImuTempData, this, std::placeholders::_1)
        );

        imuTimeSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuTimeData>(
                Topics::kImuTimeDataLoader,
                1,
                std::bind(&DataLoaderController::onImuTimeData, this, std::placeholders::_1)
        );

        gnssPositionSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::GnssPositionData>(
                Topics::kGnssPositionDataLoader,
                1,
                std::bind(&DataLoaderController::onGnssPositionData, this, std::placeholders::_1)
        );

        gnssTimeSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::GnssTimeData>(
                Topics::kGnssTimeDataLoader,
                1,
                std::bind(&DataLoaderController::onGnssTimeData, this, std::placeholders::_1)
        );

        radarSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::RadarData>(
                Topics::kRadarTiDataLoader,
                1,
                std::bind(&DataLoaderController::onRadarData, this, std::placeholders::_1)
        );
    }

    void DataLoaderController::initializePublishers() {
        // Publisher that publishes synchronization timestamps
        publisher_ = create_publisher<std_msgs::msg::UInt64>(Topics::kDataLoaderSynchronization, 1);

        cameraPublishers_[CameraIdentifier::kCameraLeftSide] = create_publisher<atlas_fusion_interfaces::msg::CameraData>(Topics::kCameraLeftSide, 1);
        cameraPublishers_[CameraIdentifier::kCameraLeftFront] = create_publisher<atlas_fusion_interfaces::msg::CameraData>(Topics::kCameraLeftFront, 1);
        cameraPublishers_[CameraIdentifier::kCameraRightFront] = create_publisher<atlas_fusion_interfaces::msg::CameraData>(Topics::kCameraRightFront, 1);
        cameraPublishers_[CameraIdentifier::kCameraRightSide] = create_publisher<atlas_fusion_interfaces::msg::CameraData>(Topics::kCameraRightSide, 1);
        cameraPublishers_[CameraIdentifier::kCameraIr] = create_publisher<atlas_fusion_interfaces::msg::CameraData>(Topics::kCameraIr, 1);

        lidarPublishers_[LidarIdentifier::kLeftLidar] = create_publisher<atlas_fusion_interfaces::msg::LidarData>(Topics::kLidarLeft, 1);
        lidarPublishers_[LidarIdentifier::kCenterLidar] = create_publisher<atlas_fusion_interfaces::msg::LidarData>(Topics::kLidarCenter, 1);
        lidarPublishers_[LidarIdentifier::kRightLidar] = create_publisher<atlas_fusion_interfaces::msg::LidarData>(Topics::kLidarRight, 1);

        imuDquatPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuDquatData>(Topics::kImuDquat, 1);
        imuGnssPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuGnssData>(Topics::kImuGnss, 1);
        imuImuPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuImuData>(Topics::kImuImu, 1);
        imuMagPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuMagData>(Topics::kImuMag, 1);
        imuPressurePublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuPressureData>(Topics::kImuPressure, 1);
        imuTempPublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuTempData>(Topics::kImuTemp, 1);
        imuTimePublisher_ = create_publisher<atlas_fusion_interfaces::msg::ImuTimeData>(Topics::kImuTime, 1);

        gnssPositionPublisher_ = create_publisher<atlas_fusion_interfaces::msg::GnssPositionData>(Topics::kGnssPosition, 1);
        gnssTimePublisher_ = create_publisher<atlas_fusion_interfaces::msg::GnssTimeData>(Topics::kGnssTime, 1);

        radarPublisher_ = create_publisher<atlas_fusion_interfaces::msg::RadarData>(Topics::kRadarTi, 1);
    }

    uint64_t DataLoaderController::getDataTimestamp(const std::pair<DataIdentifier, DataMsg> &d) {
        auto d_i = d.second.index();
        if (d_i == 0) {
            return std::get<atlas_fusion_interfaces::msg::CameraData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 1) {
            return std::get<atlas_fusion_interfaces::msg::LidarData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 2) {
            return std::get<atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 3) {
            return std::get<atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 4) {
            return std::get<atlas_fusion_interfaces::msg::ImuImuData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 5) {
            return std::get<atlas_fusion_interfaces::msg::ImuMagData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 6) {
            return std::get<atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 7) {
            return std::get<atlas_fusion_interfaces::msg::ImuTempData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 8) {
            return std::get<atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 9) {
            return std::get<atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 10) {
            return std::get<atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr>(d.second)->timestamp;
        }
        if (d_i == 11) {
            return std::get<atlas_fusion_interfaces::msg::RadarData::UniquePtr>(d.second)->timestamp;
        }

        throw std::runtime_error("Unexpected variant type when comparing timestamps!");
    }

    void DataLoaderController::retransmitMsg(const std::pair<DataIdentifier, DataMsg> &d) {
        auto d_i = d.second.index();
        if (d_i == 0) {
            auto cameraIdentifier = std::get<CameraIdentifier>(d.first);
            auto &msg = std::get<atlas_fusion_interfaces::msg::CameraData::UniquePtr>(d.second);
            cameraPublishers_[cameraIdentifier]->publish(*msg);
            return;
        }
        if (d_i == 1) {
            auto lidarIdentifier = std::get<LidarIdentifier>(d.first);
            auto &msg = std::get<atlas_fusion_interfaces::msg::LidarData::UniquePtr>(d.second);
            lidarPublishers_[lidarIdentifier]->publish(*msg);
            return;
        }
        if (d_i == 2) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr>(d.second);
            imuDquatPublisher_->publish(*msg);
            return;
        }
        if (d_i == 3) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::ImuGnssData::UniquePtr>(d.second);
            imuGnssPublisher_->publish(*msg);
            return;
        }
        if (d_i == 4) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::ImuImuData ::UniquePtr>(d.second);
            imuImuPublisher_->publish(*msg);
            return;
        }
        if (d_i == 5) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::ImuMagData::UniquePtr>(d.second);
            imuMagPublisher_->publish(*msg);
            return;
        }
        if (d_i == 6) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::ImuPressureData::UniquePtr>(d.second);
            imuPressurePublisher_->publish(*msg);
            return;
        }
        if (d_i == 7) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::ImuTempData::UniquePtr>(d.second);
            imuTempPublisher_->publish(*msg);
            return;
        }
        if (d_i == 8) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::ImuTimeData::UniquePtr>(d.second);
            imuTimePublisher_->publish(*msg);
            return;
        }
        if (d_i == 9) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr>(d.second);
            gnssPositionPublisher_->publish(*msg);
            return;
        }
        if (d_i == 10) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::GnssTimeData::UniquePtr>(d.second);
            gnssTimePublisher_->publish(*msg);
            return;
        }
        if (d_i == 11) {
            auto &msg = std::get<atlas_fusion_interfaces::msg::RadarData::UniquePtr>(d.second);
            radarPublisher_->publish(*msg);
            return;
        }

        throw std::runtime_error("Unexpected variant type when comparing timestamps!");
    }
}