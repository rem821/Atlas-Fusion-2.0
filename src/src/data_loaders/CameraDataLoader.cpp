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

#include "data_loaders/CameraDataLoader.h"
#include "rcpputils/endian.hpp"

namespace AtlasFusion::DataLoader {

    CameraDataLoader::CameraDataLoader(const std::string& name, const CameraIdentifier& cameraIdentifier, const std::string& topic, const rclcpp::NodeOptions& options)
            : Node(name, options), cameraIdentifier_{cameraIdentifier}, latestTimestampPublished_(0), synchronizationTimestamp_(0) {

        // Publisher that publishes CameraData
        publisher_ = create_publisher<atlas_fusion_interfaces::msg::CameraData>(topic, 1);

        // Timestamp synchronization subscription to manage data loading speeds
        timestampSubscription_ = create_subscription<std_msgs::msg::UInt64>(
                Topics::kDataLoaderSynchronization,
                1,
                std::bind(&CameraDataLoader::OnSynchronizationTimestamp, this, std::placeholders::_1)
        );

        // Timer to control the polling frequency for publishing
        using namespace std::chrono_literals;
        timer_ = create_wall_timer(10ms, [this] { OnDataLoaderTimer(); });

        // Init camera additional data
        Initialize();
    }

    void CameraDataLoader::OnDataLoaderTimer() {
        if (dataFrame_ != nullptr && latestTimestampPublished_ <= synchronizationTimestamp_) {
            latestTimestampPublished_ = STAMP_TO_NANOSEC(dataFrame_->image.header.stamp);

            LOG_TRACE("Camera data of frame {} sent: ({}, 0x{})", dataFrame_->camera_identifier, this->get_clock()->now().nanoseconds(), HEX_ADDR(dataFrame_.get()));

            publisher_->publish(std::move(dataFrame_));
        }

        if (!IsOnEnd() && dataFrame_ == nullptr) {
            cv::Mat frame{};
            video_.read(frame);

            std_msgs::msg::Header header;
            header.stamp.sec = NANOSEC_TO_STAMP_SEC(dataIt_->timestamp_);
            header.stamp.nanosec = NANOSEC_TO_STAMP_NANOSEC(dataIt_->timestamp_);
            header.frame_id = FrameTypeName(FrameTypeFromIdentifier(cameraIdentifier_));

            atlas_fusion_interfaces::msg::CameraData cameraData;
            cameraData.image = ToCameraMsg(frame, header, cameraIdentifier_ == CameraIdentifier::kCameraIr ? "mono8" : "bgr8");
            cameraData.min_temperature = static_cast<float>(dataIt_->tempMin_);
            cameraData.max_temperature = static_cast<float>(dataIt_->tempMax_);
            cameraData.yolo_detections = dataIt_->detections_;

            dataFrame_ = std::make_unique<atlas_fusion_interfaces::msg::CameraData>(cameraData);
            dataIt_ = std::next(dataIt_, 1);
        }
    }

    void CameraDataLoader::OnSynchronizationTimestamp(const std_msgs::msg::UInt64& msg) {
        synchronizationTimestamp_ = msg.data;
    }

    void CameraDataLoader::Initialize() {
        std::string folder;
        switch (cameraIdentifier_) {
            case CameraIdentifier::kCameraLeftFront:
                folder = Folders::kCameraLeftFrontFolder;
                break;
            case CameraIdentifier::kCameraLeftSide:
                folder = Folders::kCameraLeftSideFolder;
                break;
            case CameraIdentifier::kCameraRightFront:
                folder = Folders::kCameraRightFrontFolder;
                break;
            case CameraIdentifier::kCameraRightSide:
                folder = Folders::kCameraRightSideFolder;
                break;
            case CameraIdentifier::kCameraIr:
                folder = Folders::kCameraIr;
                break;
        }

        std::string datasetPath = EntryPoint::GetContext().GetDatasetPath();
        auto csvContent = CsvReader::ReadCsv(datasetPath + folder + Files::kTimestampFile);
        for (const auto& substrings: csvContent) {
            switch (cameraIdentifier_) {
                case CameraIdentifier::kCameraLeftFront:
                case CameraIdentifier::kCameraLeftSide:
                case CameraIdentifier::kCameraRightFront:
                case CameraIdentifier::kCameraRightSide:
                    if (substrings.size() == 3) {
                        std::stringstream ss;
                        data_.emplace_back(std::stoull(substrings[0]), std::stol(substrings[1]),
                                           std::stoull(substrings[2]), 0.0, 0.0);
                    } else {
                        throw std::runtime_error("Unexpected length of camera RGB timestamp row");
                    }
                    break;

                case CameraIdentifier::kCameraIr:
                    if (substrings.size() == 4) {
                        std::stringstream ss;
                        data_.emplace_back(std::stoull(substrings[0]), std::stol(substrings[1]), 0,
                                           std::stod(substrings[2]), std::stod(substrings[3]));
                    } else {
                        throw std::runtime_error("Unexpected length of camera IR timestamp row");
                    }
                    break;
            }
        }

        auto videoPath = datasetPath + folder + Files::kVideoFile;
        video_.open(videoPath);
        if (!video_.isOpened()) {
            Clear();

            throw std::runtime_error(fmt::format("Unable to open video file: {}", videoPath));
        }

        if (static_cast<double>(data_.size()) != video_.get(cv::CAP_PROP_FRAME_COUNT)) {
            Clear();
            throw std::runtime_error(
                    fmt::format("Number of timestamps is not equal to number of frames: {} != {}",
                                data_.size(),
                                video_.get(cv::CAP_PROP_FRAME_COUNT))
            );
        }

        auto width = static_cast<int>(video_.get(cv::CAP_PROP_FRAME_WIDTH));
        auto height = static_cast<int>(video_.get(cv::CAP_PROP_FRAME_HEIGHT));
        imageWidthHeight_ = {width, height};

        std::string yoloFile = folder.replace(folder.find('/'), std::string("/").length(), ".txt");
        LoadYoloDetections(datasetPath + Folders::kYoloFolder + yoloFile);

        dataIt_ = data_.begin();
        releaseIt_ = dataIt_;
    }

    bool CameraDataLoader::IsOnEnd() const {
        return dataIt_ >= data_.end();
    }

    void CameraDataLoader::Clear() {
        video_.release();
        data_.clear();
        dataIt_ = data_.begin();
    }

    void CameraDataLoader::LoadYoloDetections(const std::string& path) {
        auto csvContent = CsvReader::ReadCsv(std::string(path));

        if (!csvContent.empty()) {
            for (const auto& detection: csvContent) {
                if (detection.size() != 7) {
                    throw std::runtime_error("Unexpected length of yolo detection in csv");
                }

                size_t frame_id = std::stoul(detection[0]);
                auto cx = std::stod(detection[1]) * imageWidthHeight_.first;
                auto cy = std::stod(detection[2]) * imageWidthHeight_.second;
                auto w = std::stod(detection[3]) * imageWidthHeight_.first;
                auto h = std::stod(detection[4]) * imageWidthHeight_.second;

                atlas_fusion_interfaces::msg::YoloDetection det;
                det.x1 = int(cx - w / 2);
                det.y1 = int(cy - h / 2);
                det.x2 = int(cx + w / 2);
                det.y2 = int(cy + h / 2);
                det.detection_confidence = std::stof(detection[5]);
                det.detection_class = static_cast<int16_t>(std::stoi(detection[6]));

                if (frame_id < data_.size()) {
                    data_.at(frame_id).detections_.emplace_back(det);
                }
            }
        }
    }

    sensor_msgs::msg::Image CameraDataLoader::ToCameraMsg(const cv::Mat& img,
                                                          const std_msgs::msg::Header& header,
                                                          const std::string& encoding) {
        sensor_msgs::msg::Image ros_image;
        ros_image.header = header;
        ros_image.height = img.rows;
        ros_image.width = img.cols;
        ros_image.encoding = encoding;
        ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        ros_image.step = img.cols * img.elemSize();
        size_t size = ros_image.step * img.rows;
        ros_image.data.resize(size);

        if (img.isContinuous()) {
            memcpy(reinterpret_cast<char*>(&ros_image.data[0]), img.data, size);
        } else {
            // Copy by row
            auto* ros_data_ptr = reinterpret_cast<uchar*>(&ros_image.data[0]);
            uchar* cv_data_ptr = img.data;
            for (int i = 0; i < img.rows; ++i) {
                memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
                ros_data_ptr += ros_image.step;
                cv_data_ptr += img.step;
            }
        }
        return ros_image;
    }

}