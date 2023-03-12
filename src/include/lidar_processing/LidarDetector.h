//
// Created by standa on 12.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <data_models/LidarDetection.h>

namespace AtlasFusion::LocalMap {

    class LidarDetector : public rclcpp::Node {
    public:
        LidarDetector(const std::string& name, const std::string &topic, const rclcpp::NodeOptions &options);

    private:
        void OnLidarData(sensor_msgs::msg::PointCloud2::UniquePtr msg);

        void OnSelfTransformation(geometry_msgs::msg::TransformStamped::UniquePtr msg);

        void PublishLidarDetections(const std::vector<std::shared_ptr<DataModels::LidarDetection>>& detections);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr aggregatedPointCloudSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr selfTransformationSubscriber_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidarDetectionsPublisher_;

        int maxLidarMarkers_;
        rtl::RigidTf3D<double> egoTf_;
    };
}