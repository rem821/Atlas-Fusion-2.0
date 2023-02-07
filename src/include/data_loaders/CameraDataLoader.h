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

#include <chrono>
#include <std_msgs/msg/u_int64.hpp>
#include <atlas_fusion_interfaces/msg/rgb_camera_data.hpp>
#include "rcpputils/endian.hpp"

#include "data_loaders/DataLoaderIdentifiers.h"
#include "data_loaders/RecordingConstants.h"
#include "util/CsvReader.h"

namespace AtlasFusion::DataLoader {

    class CameraDataLoader : public rclcpp::Node {

        /**
        * Simple structure that represents the frames in the video file, before the video frame is loaded.
        */
        struct CameraFrame {

            /**
             * Constructor
             * @param ts Recording timestamp
             * @param fId frame position in the video sequence
             * @param iTs inner camera's timestamp
             * @param tempMin minimal image temp (only for IR frames)
             * @param tempMax maximal image temp (only for IR frames)
             */
            CameraFrame(uint64_t ts, uint32_t fId, uint64_t iTs, double tempMin, double tempMax)
                    : timestamp_(ts), frameId_(fId), innerTimestamp_(iTs), tempMin_(tempMin), tempMax_(tempMax) {}

            uint64_t getTimestamp() { return timestamp_; };

            uint64_t timestamp_;
            uint32_t frameId_;
            uint64_t innerTimestamp_;
            double tempMin_;
            double tempMax_;
            std::vector<atlas_fusion_interfaces::msg::YoloDetection> detections_{};
        };

    public:
        CameraDataLoader(const std::string &name,
                         const std::string &datasetPath,
                         const CameraIdentifier &cameraIdentifier,
                         const std::string &topic,
                         const std::string &synchronizationTopic,
                         const rclcpp::NodeOptions &options);

    private:
        void timer_callback();

        void synchronization_callback(const std_msgs::msg::UInt64 &msg);

        void initialize();

        bool isOnEnd() const;

        void clear();

        void loadYoloDetections(const std::string& path);

        sensor_msgs::msg::Image toCameraMsg(const cv::Mat &img,
                                                              const std_msgs::msg::Header &header,
                                                              const std::string &encoding);

        std::string datasetPath_;
        CameraIdentifier cameraIdentifier_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::RGBCameraData>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr timestampSubscription_;
        uint64_t latestTimestampPublished_;
        uint64_t synchronizationTimestamp_;

        cv::VideoCapture video_;
        std::vector<CameraFrame> data_;
        std::vector<CameraFrame>::iterator dataIt_;
        std::vector<CameraFrame>::iterator releaseIt_;
        std::pair<int, int> imageWidthHeight_ = {-1, -1};
    };
}

