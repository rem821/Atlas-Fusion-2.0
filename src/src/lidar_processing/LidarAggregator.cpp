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

#include <lidar_processing/LidarAggregator.h>
#include <algorithms/PointCloudExtrapolator.h>
#include <algorithms/LidarFilter.h>
#include <pcl_conversions/pcl_conversions.h>

namespace AtlasFusion::LocalMap {

    LidarAggregator::LidarAggregator(const std::string& name, const std::string& topic, const rclcpp::NodeOptions& options)
            : Node(name, options) {

        // Publisher that publishes aggregated lidar data
        aggregatedPointCloudPublisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, 1);


        lidarSubscribers_[DataLoader::LidarIdentifier::kLeftLidar] = create_subscription<sensor_msgs::msg::PointCloud2>(
                Topics::kLidarLeft,
                1,
                std::bind(&LidarAggregator::OnLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[DataLoader::LidarIdentifier::kCenterLidar] = create_subscription<sensor_msgs::msg::PointCloud2>(
                Topics::kLidarCenter,
                1,
                std::bind(&LidarAggregator::OnLidarData, this, std::placeholders::_1)
        );

        lidarSubscribers_[DataLoader::LidarIdentifier::kRightLidar] = create_subscription<sensor_msgs::msg::PointCloud2>(
                Topics::kLidarRight,
                1,
                std::bind(&LidarAggregator::OnLidarData, this, std::placeholders::_1)
        );

        clientNode_ = std::make_shared<rclcpp::Node>("PoseEstimationClient", get_node_options());
        positionEstimateClient_ = clientNode_->create_client<atlas_fusion_interfaces::srv::EstimatePositionInTime>("estimate_pose");
    }

    void LidarAggregator::OnLidarData(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        //Timer t("LidarAggregator->OnLidarData")
        LOG_TRACE("LidarAggregator: Lidar data of frame {} arrived: ({}, {})", msg->lidar_identifier, this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));
        const auto sensorFrame = NameToFrameType(msg->header.frame_id);
        const auto lidarTF = EntryPoint::GetContext().GetTransformationForFrame(sensorFrame);
        const auto lidarIdentifier = LidarIdentifierFromFrameType(sensorFrame);
        const auto timestamp = STAMP_TO_NANOSEC(msg->header.stamp);

        if (latestLidarScanTimestamp_[lidarIdentifier] > 0) {
            auto [poseBefore, poseNow] = EstimatePositionInTime(latestLidarScanTimestamp_[lidarIdentifier]);
            if (!poseBefore.GetPosition().hasNaN()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*msg.get(), *pc.get());
                Algorithms::LidarFilter::FillterNearObjects(pc);

                auto scanBatches = Algorithms::PointCloudExtrapolator::SplitPointCloudToBatches(pc, poseBefore, poseNow, lidarTF);
                pointCloudAggregator_.AddLidarScan(lidarIdentifier, scanBatches);
                pointCloudAggregator_.FilterOutBatches(timestamp);
                pointCloudAggregator_.AddPointCloudBatches(scanBatches);
                //LOG_INFO("Aggregated point cloud has {} points", pointCloudAggregator_.GetLatestScan()->width);

                sensor_msgs::msg::PointCloud2 out_msg;

                auto out_pc = pointCloudAggregator_.GetGlobalCoordinateAggregatedPointCloud();
                pcl::toROSMsg(*out_pc.get(), out_msg);

                out_msg.header.stamp = this->get_clock()->now();
                out_msg.header.frame_id = FrameTypeName(FrameType::kOrigin);
                aggregatedPointCloudPublisher_->publish(out_msg);
            }
        }

        //dataCache_.emplace_back(id, std::move(msg));

        latestLidarScanTimestamp_[lidarIdentifier] = timestamp;
    }

    std::pair<DataModels::LocalPosition, DataModels::LocalPosition> LidarAggregator::EstimatePositionInTime(uint64_t timestamp) {
        auto poseRequest = std::make_shared<atlas_fusion_interfaces::srv::EstimatePositionInTime::Request>();
        poseRequest->timestamp = timestamp;

        auto emptyPos = DataModels::LocalPosition(
                rtl::Vector3D<double>(NAN, NAN, NAN),
                rtl::Quaternion<double>(NAN, NAN, NAN, NAN),
                timestamp
        );

        while (!positionEstimateClient_->wait_for_service(std::chrono::milliseconds(10))) {
            if (!rclcpp::ok()) {
                LOG_WARN("Interrupted while waiting for the EstimatePositionInTime service");
                return {emptyPos, emptyPos};
            }
            LOG_WARN("Service not available... waiting");
        }

        auto future = positionEstimateClient_->async_send_request(poseRequest);
        if (rclcpp::spin_until_future_complete(clientNode_->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto r = future.get();
            auto poseBefore = DataModels::LocalPosition(
                    rtl::Vector3D<double>(r->position_before.x, r->position_before.y, r->position_before.z),
                    rtl::Quaternion<double>(r->orientation_before.w, r->orientation_before.x, r->orientation_before.y, r->orientation_before.z),
                    r->timestamp_before
            );
            auto poseNow = DataModels::LocalPosition(
                    rtl::Vector3D<double>(r->position_now.x, r->position_now.y, r->position_now.z),
                    rtl::Quaternion<double>(r->orientation_now.w, r->orientation_now.x, r->orientation_now.y, r->orientation_now.z),
                    r->timestamp_now
            );
            return {poseBefore, poseNow};
        } else {
            LOG_ERROR("Future fell through");
            return {emptyPos, emptyPos};
        }
    }
}