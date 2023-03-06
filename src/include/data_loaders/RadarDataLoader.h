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

#include <precompiled_headers/PCH.h>

namespace AtlasFusion::DataLoader {

    class RadarDataLoader : public rclcpp::Node {

        struct RadarDetection {
            rtl::Vector3f pose_;
            float velocity_;
        };

        struct RadarFrame {

            /**
             * Constructor
             * @param ts Recording timestamp
             * @param numberOfDetections number of detections
             * @param radarDetections vector of radarDetections
             */
            RadarFrame(uint64_t ts, uint16_t numberOfDetections, std::vector<RadarDetection> &radarDetections)
                    : timestamp_(ts), numberOfDetections_{numberOfDetections}, radarDetections_(std::move(radarDetections)) {}

            uint64_t timestamp_;
            uint16_t numberOfDetections_;
            std::vector<RadarDetection> radarDetections_;
        };

    public:
        RadarDataLoader(const std::string &name,
                        std::string datasetPath,
                        const std::string &topic,
                        const rclcpp::NodeOptions &options);

    private:
        void onDataLoaderTimer();

        void onSynchronizationTimestamp(const std_msgs::msg::UInt64 &msg);

        void initialize();

        bool isOnEnd() const;

        void clear();

        std::string datasetPath_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<atlas_fusion_interfaces::msg::RadarData>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr timestampSubscription_;

        atlas_fusion_interfaces::msg::RadarData::UniquePtr dataFrame_;
        uint64_t latestTimestampPublished_;
        uint64_t synchronizationTimestamp_;

        std::vector<RadarFrame> data_;
        std::vector<RadarFrame>::iterator dataIt_;
        std::vector<RadarFrame>::iterator releaseIt_;
    };
}

