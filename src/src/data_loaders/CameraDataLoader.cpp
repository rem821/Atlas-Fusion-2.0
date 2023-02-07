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

namespace AtlasFusion::DataLoader {

    CameraDataLoader::CameraDataLoader(const std::string &name,
                                       const std::string &datasetPath,
                                       const CameraIdentifier &cameraIdentifier,
                                       const std::string &topic,
                                       const std::string &synchronizationTopic,
                                       const rclcpp::NodeOptions &options)
            : Node(name, options), datasetPath_{datasetPath}, cameraIdentifier_{cameraIdentifier},
              latestTimestampPublished_(0) {

        // Publisher that publishes Images
        publisher_ = create_publisher<atlas_fusion_interfaces::msg::RGBCameraData>(topic, 1);

        // Timestamp synchronization subscription to manage data loading speeds
        timestampSubscription_ = create_subscription<std_msgs::msg::UInt64>(
                synchronizationTopic,
                1,
                std::bind(&CameraDataLoader::synchronization_callback, this, std::placeholders::_1)
        );

        // Timer to control the polling frequency for publishing
        using namespace std::chrono_literals;
        timer_ = create_wall_timer(10ms, std::bind(&CameraDataLoader::timer_callback, this));

        // Init camera additional data
        initialize();
    }

    void CameraDataLoader::timer_callback() {
        //if (latestTimestampPublished_ <= synchronizationTimestamp_) return;

        if (!isOnEnd()) {
            cv::Mat frame{};
            video_.read(frame);

            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            header.frame_id = dataIt_->frameId_;

            atlas_fusion_interfaces::msg::RGBCameraData cameraData;
            cameraData.image = toCameraMsg(frame, header,
                                           cameraIdentifier_ == CameraIdentifier::kCameraIr ? "mono8" : "bgr8");
            cameraData.camera_identifier = std::to_string((int) cameraIdentifier_);
            cameraData.timestamp = dataIt_->timestamp_;
            cameraData.inner_timestamp = dataIt_->innerTimestamp_;

            publisher_->publish(cameraData);
            dataIt_ = std::next(dataIt_, 1);
        }
    }

    void CameraDataLoader::synchronization_callback(const std_msgs::msg::UInt64 &msg) {
        synchronizationTimestamp_ = msg.data;
    }

    void CameraDataLoader::initialize() {
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

        auto csvContent = CsvReader::readCsv(datasetPath_ + folder + Files::kTimestampFile);
        for (const auto &substrings: csvContent) {
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

        auto videoPath = datasetPath_ + folder + Files::kVideoFile;
        video_.open(videoPath);
        if (!video_.isOpened()) {
            clear();

            throw std::runtime_error(fmt::format("Unable to open video file: {}", videoPath));
        }

        if (data_.size() != video_.get(cv::CAP_PROP_FRAME_COUNT)) {
            clear();
            throw std::runtime_error(
                    fmt::format("Number of timestamps is not equal to number of frames: {} != {}",
                                data_.size(),
                                video_.get(cv::CAP_PROP_FRAME_COUNT))
            );
        }

        auto width = static_cast<int>(video_.get(cv::CAP_PROP_FRAME_WIDTH));
        auto height = static_cast<int>(video_.get(cv::CAP_PROP_FRAME_HEIGHT));
        imageWidthHeight_ = {width, height};

        /*
        std::string yoloFile = folder.replace(folder.find('/'), std::string("/").length(), ".txt");
        loadYoloDetections(datasetPath + Folders::kYoloFolder + yoloFile);
        */

        dataIt_ = data_.begin();
        releaseIt_ = dataIt_;
    }

    bool CameraDataLoader::isOnEnd() const {
        return dataIt_ >= data_.end();
    }

    void CameraDataLoader::clear() {
        video_.release();
        data_.clear();
        dataIt_ = data_.begin();
    }

    void CameraDataLoader::loadYoloDetections(const std::string &path) {
        auto csvContent = CsvReader::readCsv(std::string(path));

        if (!csvContent.empty()) {
            for (const auto &detection: csvContent) {
                if (detection.size() != 7) {
                    throw std::runtime_error("Unexpected length of yolo detection in csv");
                }

                size_t frame_id = std::stoul(detection[0]);
                auto cx = std::stod(detection[1]) * imageWidthHeight_.first;
                auto cy = std::stod(detection[2]) * imageWidthHeight_.second;
                auto w = std::stod(detection[3]) * imageWidthHeight_.first;
                auto h = std::stod(detection[4]) * imageWidthHeight_.second;
                if (frame_id < data_.size()) {
                    data_.at(frame_id).detections_.emplace_back(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2,
                                                                std::stof(detection[5]),
                                                                static_cast<DataModels::YoloDetectionClass>(std::stoi(
                                                                        detection[6])));
                }
            }
        }
    }

    sensor_msgs::msg::Image CameraDataLoader::toCameraMsg(const cv::Mat &img,
                                                          const std_msgs::msg::Header &header,
                                                          const std::string &encoding) {
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
            memcpy(reinterpret_cast<char *>(&ros_image.data[0]), img.data, size);
        } else {
            // Copy by row
            auto *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
            uchar *cv_data_ptr = img.data;
            for (int i = 0; i < img.rows; ++i) {
                memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
                ros_data_ptr += ros_image.step;
                cv_data_ptr += img.step;
            }
        }
        return ros_image;
    }

}