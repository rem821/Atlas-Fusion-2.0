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

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithms/Kalman1D.h>
#include <data_models/GlobalPosition.h>

#include <atlas_fusion_interfaces/msg/imu_imu_data.hpp>
#include <atlas_fusion_interfaces/msg/imu_dquat_data.hpp>
#include <atlas_fusion_interfaces/msg/gnss_position_data.hpp>
#include <atlas_fusion_interfaces/srv/estimate_position_in_time.hpp>

namespace AtlasFusion::LocalMap {

    class SelfModel : public rclcpp::Node {
    public:
        SelfModel(const std::string& name, const rclcpp::NodeOptions& options);

    private:
        /* Subscribers */
        void OnGnssPose(atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr msg);

        void OnImuImu(atlas_fusion_interfaces::msg::ImuImuData::UniquePtr msg);

        void OnImuDquat(atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr msg);

        rclcpp::Subscription<atlas_fusion_interfaces::msg::GnssPositionData>::SharedPtr gnssSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuImuData>::SharedPtr imuImuSubscriber_;
        rclcpp::Subscription<atlas_fusion_interfaces::msg::ImuDquatData>::SharedPtr imuDquatSubscriber_;

        /* Transforms */
        void PublishStaticTransforms();

        void UpdateOriginToRootTf();

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTransformBroadcaster_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> rootToOriginTransformBroadcaster_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr originTransformationPublisher_;

        /* Publishers */
        void OnPublishPoseAndTrajectory();

        visualization_msgs::msg::Marker GetSelfGlobalCube();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr selfGlobalPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr filteredTrajectoryPublisher_;

        /* Services */
        rclcpp::CallbackGroup::SharedPtr callbackGroup_;
        rclcpp::Service<atlas_fusion_interfaces::srv::EstimatePositionInTime>::SharedPtr positionEstimationService_;
        void EstimatePositionInTime(const std::shared_ptr<atlas_fusion_interfaces::srv::EstimatePositionInTime::Request>& request,
                               const std::shared_ptr<atlas_fusion_interfaces::srv::EstimatePositionInTime::Response>& response);

        [[nodiscard]] DataModels::LocalPosition GetPosition() const;

        std::deque<DataModels::LocalPosition> GetPositionHistory() const { return positionHistory_; };

        [[nodiscard]] double GetSpeedScalar() const;

        [[nodiscard]] rtl::Vector3D<double> GetSpeedVector() const;

        [[nodiscard]] double GetAvgAccScalar() const;

        [[nodiscard]] rtl::Vector3D<double> GetAvgAcceleration() const;

        [[nodiscard]] rtl::Quaternion<double> GetOrientation() const { return orientation_; }

        [[nodiscard]] double GetHeading() const;


        std::pair<double, float> ValidHeading(const atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr& data);

        std::pair<double, float> SpeedHeading();

        std::pair<double, float> FuseHeadings(const atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr& data);

        float EstimateSlerpFactor(float, float);

        void UpdateOrientation(double heading);

        [[nodiscard]] DataModels::GlobalPosition GnssPoseToRootFrame(const DataModels::GlobalPosition& gnssPose) const;

        [[nodiscard]] rtl::Vector3D<double> RemoveGravitationalForceFromLinAcc(const atlas_fusion_interfaces::msg::ImuImuData::UniquePtr& data) const;

        [[nodiscard]] uint64_t GetCurrentTime() const;

        [[nodiscard]] static double AzimuthToHeading(double azimuth) { return -azimuth * M_PI / 180.0f; }

        std::unique_ptr<Algorithms::Kalman1D> kalmanX_;
        std::unique_ptr<Algorithms::Kalman1D> kalmanY_;
        std::unique_ptr<Algorithms::Kalman1D> kalmanZ_;
        rtl::Quaternion<double> orientation_ = rtl::Quaternion<double>::identity();

        double headingDiff_ = 0;
        double attitudeDiff_ = 0;

        bool poseInitialized_ = false;
        bool orientationInitialized_ = false;

        DataModels::GlobalPosition anchorPose_{0, 0, 0, 0};
        std::deque<DataModels::LocalPosition> positionHistory_;
        std::deque<std::pair<rtl::Vector3D<double>, uint64_t >> accHistory_;

        int validHeadingCounter_ = 0;

        uint64_t lastGnssTimestamp_{};
        uint64_t lastImuTimestamp_{};
        uint64_t lastDquatTimestamp_{};
    };
}

