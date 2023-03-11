//
// Created by standa on 10.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <atlas_fusion_interfaces/msg/camera_data.hpp>
#include <atlas_fusion_interfaces/msg/yolo_detection.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <data_loaders/DataLoaderIdentifiers.h>
#include <data_models/FrustumDetection.h>
#include <algorithms/CameraProjector.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace AtlasFusion::LocalMap {

    class YoloDetector : public rclcpp::Node {

    public:
        YoloDetector(const std::string& name, const rclcpp::NodeOptions& options);

    private:
        void InitProjectors();

        void InitSubscribers();

        void InitPublishers();

        void OnCameraData(atlas_fusion_interfaces::msg::CameraData::UniquePtr msg);

        void OnNewLidarData(sensor_msgs::msg::PointCloud2::UniquePtr msg);

        void OnSelfTransformation(geometry_msgs::msg::TransformStamped::UniquePtr msg);

        void
        ProjectAllPointsIntoTheImage(FrameType frame, uint32_t imageWidth, uint32_t imageHeight, std::vector<cv::Point2f>& validPoints2D, std::vector<cv::Point3f>& validPoints3D);

        std::vector<uint32_t> GetPointsInsideDetectionIndices(const std::vector<cv::Point2f>& validPoints2D, atlas_fusion_interfaces::msg::YoloDetection detection);

        float GetMedianDepthOfPoints(const std::vector<cv::Point3f>& points, std::vector<uint32_t>& indices);

        rtl::Frustum3D<double> Get3DDetectionFrustum(FrameType frame, atlas_fusion_interfaces::msg::YoloDetection detection, float detectionDistance);

        void Publish3DFrustums(FrameType frame, std::vector<DataModels::FrustumDetection>& frustums);

        std::map<DataLoader::CameraIdentifier, rclcpp::Subscription<atlas_fusion_interfaces::msg::CameraData>::SharedPtr> cameraSubscribers_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr aggregatedPointCloudSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr selfTransformationSubscriber_;

        std::map<DataLoader::CameraIdentifier, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> frustumPublishers_;

        std::map<DataLoader::CameraIdentifier, std::unique_ptr<Algorithms::CameraProjector>> cameraProjectors_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr aggregatedPointCloud_;

        std::map<DataLoader::CameraIdentifier, int> maxYoloMarkers_;
        rtl::RigidTf3D<double> egoTf_;
    };
}