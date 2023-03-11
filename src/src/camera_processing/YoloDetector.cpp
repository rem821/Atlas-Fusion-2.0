//
// Created by standa on 10.3.23.
//
#include <camera_processing/YoloDetector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithms/PointCloudProcessor.h>
#include <data_models/YoloDetectionClass.h>
#include <data_models/LocalPosition.h>

namespace AtlasFusion::LocalMap {

    YoloDetector::YoloDetector(const std::string& name, const rclcpp::NodeOptions& options)
            : Node(name, options), aggregatedPointCloud_(new pcl::PointCloud<pcl::PointXYZ>) {

        InitProjectors();
        InitSubscribers();
        InitPublishers();
    }

    void YoloDetector::InitProjectors() {
        std::vector<DataLoader::CameraIdentifier> identifiers = {
                DataLoader::CameraIdentifier::kCameraLeftSide,
                DataLoader::CameraIdentifier::kCameraLeftFront,
                DataLoader::CameraIdentifier::kCameraRightFront,
                DataLoader::CameraIdentifier::kCameraRightSide,
                DataLoader::CameraIdentifier::kCameraIr
        };

        for (DataLoader::CameraIdentifier id: identifiers) {
            auto tf = EntryPoint::GetContext().GetTransformationForFrame(FrameTypeFromIdentifier(id));
            auto calib = EntryPoint::GetContext().CreateCameraCalibrationParams(id);
            cameraProjectors_[id] = std::make_unique<Algorithms::CameraProjector>(calib.GetMatIntrinsicParams(), calib.GetMatDistortionParams(), tf);
        }
    }

    void YoloDetector::InitSubscribers() {
        cameraSubscribers_[DataLoader::CameraIdentifier::kCameraLeftSide] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraLeftSideDataLoader,
                1,
                std::bind(&YoloDetector::OnCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[DataLoader::CameraIdentifier::kCameraLeftFront] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraLeftFrontDataLoader,
                1,
                std::bind(&YoloDetector::OnCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[DataLoader::CameraIdentifier::kCameraRightFront] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraRightFrontDataLoader,
                1,
                std::bind(&YoloDetector::OnCameraData, this, std::placeholders::_1)
        );

        cameraSubscribers_[DataLoader::CameraIdentifier::kCameraRightSide] = create_subscription<atlas_fusion_interfaces::msg::CameraData>(
                Topics::kCameraRightSideDataLoader,
                1,
                std::bind(&YoloDetector::OnCameraData, this, std::placeholders::_1)
        );

        aggregatedPointCloudSubscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                Topics::kLidarAggregatedGlobal,
                1,
                std::bind(&YoloDetector::OnNewLidarData, this, std::placeholders::_1)
        );

        selfTransformationSubscriber_ = create_subscription<geometry_msgs::msg::TransformStamped>(
                Topics::kSelfTransformation,
                1,
                std::bind(&YoloDetector::OnSelfTransformation, this, std::placeholders::_1)
        );
    }

    void YoloDetector::InitPublishers() {
        frustumPublishers_[DataLoader::CameraIdentifier::kCameraLeftSide] = create_publisher<visualization_msgs::msg::MarkerArray>(Topics::kCameraLeftSideYolo, 1);
        frustumPublishers_[DataLoader::CameraIdentifier::kCameraLeftFront] = create_publisher<visualization_msgs::msg::MarkerArray>(Topics::kCameraLeftFrontYolo, 1);
        frustumPublishers_[DataLoader::CameraIdentifier::kCameraRightFront] = create_publisher<visualization_msgs::msg::MarkerArray>(Topics::kCameraRightFrontYolo, 1);
        frustumPublishers_[DataLoader::CameraIdentifier::kCameraRightSide] = create_publisher<visualization_msgs::msg::MarkerArray>(Topics::kCameraRightSideYolo, 1);
    }

    void YoloDetector::OnCameraData(atlas_fusion_interfaces::msg::CameraData::UniquePtr msg) {
        if (aggregatedPointCloud_->empty()) {
            LOG_WARN("YoloDetector doesn't have point cloud to estimate detections frustums in 3D!");
            return;
        }

        if (msg->yolo_detections.empty()) { return; }

        auto frame = NameToFrameType(msg->image.header.frame_id);

        std::vector<cv::Point2f> valid2DPoints{};
        std::vector<cv::Point3f> valid3DPoints{};
        ProjectAllPointsIntoTheImage(frame, msg->image.width, msg->image.height, valid2DPoints, valid3DPoints);

        if (valid2DPoints.empty()) {
            LOG_WARN("All points {}, valid3D {}", aggregatedPointCloud_->size(), valid3DPoints.size());

            LOG_WARN("No valid 2D points to estimate detection frustums in 3D!");
        }

        std::vector<DataModels::FrustumDetection> frustumDetections{};
        for (auto& detection: msg->yolo_detections) {
            auto indices = GetPointsInsideDetectionIndices(valid2DPoints, detection);
            float detectionDistance = GetMedianDepthOfPoints(valid3DPoints, indices);
            if(detectionDistance < 0) break;
            LOG_INFO("YOLO detection from camera {} of class {} is being detected {}m from the vehicle with confidence of {}", msg->image.header.frame_id,
                     detection.detection_class, detectionDistance, detection.detection_confidence);
            frustumDetections.emplace_back(
                    Get3DDetectionFrustum(frame, detection, detectionDistance),
                    detection.detection_confidence,
                    static_cast<DataModels::YoloDetectionClass>(detection.detection_class)
            );
        }
        Publish3DFrustums(frame, frustumDetections);
    }

    void YoloDetector::OnNewLidarData(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        LOG_INFO("YoloDetector: Lidar Data");
        pcl::fromROSMsg(*msg.get(), *aggregatedPointCloud_.get());
    }

    void YoloDetector::OnSelfTransformation(geometry_msgs::msg::TransformStamped::UniquePtr msg) {
        rtl::Vector3D<double> position = rtl::Vector3D<double>(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
        rtl::Quaternion<double> orientation = rtl::Quaternion<double>(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
        auto lp = DataModels::LocalPosition(position, orientation, 0);
        egoTf_ = lp.ToTf().inverted();
    }

    void YoloDetector::ProjectAllPointsIntoTheImage(FrameType frame, uint32_t imageWidth, uint32_t imageHeight, std::vector<cv::Point2f>& validPoints2D,
                                                    std::vector<cv::Point3f>& validPoints3D) {
        auto agg_tf = Algorithms::PointCloudProcessor::TransformPointCloud(aggregatedPointCloud_, egoTf_);
        auto sensorCutoutPc = Algorithms::PointCloudProcessor::GetPointCloudCutoutForFrame(agg_tf, frame);
        LOG_WARN("Sensor cutout has length {}", sensorCutoutPc->size());

        auto transformedPc = Algorithms::PointCloudProcessor::TransformPointCloud(sensorCutoutPc, EntryPoint::GetContext().GetTransformationForFrame(frame).inverted());

        std::vector<cv::Point3f> points3D;
        std::vector<cv::Point2f> points2D;
        points3D.reserve(transformedPc->size());

        for (const auto& pnt: transformedPc->points) {
            if (pnt.z > -.5f) {
                points3D.emplace_back(pnt.x, pnt.y, pnt.z);
            }
        }

        cameraProjectors_[CameraIdentifierFromFrameType(frame)]->ProjectPoints(points3D, points2D, true);
        validPoints2D.reserve(points2D.size());
        validPoints3D.reserve(points3D.size());

        for (uint32_t i = 0; i < points3D.size(); i++) {
            const auto& p = points2D.at(i);
            if (p.y >= 0 && p.y < float(imageHeight) && p.x >= 0 && p.x < float(imageWidth)) {
                validPoints2D.push_back(points2D.at(i));
                validPoints3D.push_back(points3D.at(i));
            }
        }
    }

    std::vector<uint32_t>
    YoloDetector::GetPointsInsideDetectionIndices(const std::vector<cv::Point2f>& validPoints2D, const atlas_fusion_interfaces::msg::YoloDetection detection) {
        std::vector<uint32_t> output;

        for (uint32_t i = 0; i < validPoints2D.size(); i++) {
            const auto& p = validPoints2D.at(i);
            if (p.x > (float) detection.x1 && p.x<(float) detection.x2 && p.y>(float) detection.y1 && p.y < (float) detection.y2) {
                output.push_back(i);
            }
        }
        return output;
    }

    float YoloDetector::GetMedianDepthOfPoints(const std::vector<cv::Point3f>& points, std::vector<uint32_t>& indices) {
        std::vector<float> distances;
        distances.reserve(points.size());

        for (const auto& index: indices) {
            const auto& p = points.at(index);
            distances.push_back((float) std::sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2)));
        }

        if (!distances.empty()) {
            std::sort(distances.begin(), distances.end());
            auto midIndex = static_cast<size_t>(distances.size() / 2);
            return distances.at(midIndex);
        }
        return -1;
    }

    rtl::Frustum3D<double> YoloDetector::Get3DDetectionFrustum(const FrameType frame, const atlas_fusion_interfaces::msg::YoloDetection detection, float detectionDistance) {
        std::vector<cv::Point2f> points2D{
                cv::Point2f{(float) detection.x1, (float) detection.y1},
                cv::Point2f{(float) detection.x2, (float) detection.y1},
                cv::Point2f{(float) detection.x1, (float) detection.y2},
                cv::Point2f{(float) detection.x2, (float) detection.y2},
        };

        std::vector<cv::Point3f> points3D;
        cameraProjectors_[CameraIdentifierFromFrameType(frame)]->ReverseProjection(points2D, points3D, false);

        auto frustum = rtl::Frustum3D<double>(
                rtl::Vector3D<double>{0, 0, 0},
                rtl::Vector3D<double>{points3D[0].x * detectionDistance, points3D[0].y * detectionDistance, points3D[0].z * detectionDistance},
                rtl::Vector3D<double>{points3D[1].x * detectionDistance, points3D[1].y * detectionDistance, points3D[1].z * detectionDistance},
                rtl::Vector3D<double>{points3D[2].x * detectionDistance, points3D[2].y * detectionDistance, points3D[2].z * detectionDistance},
                rtl::Vector3D<double>{points3D[3].x * detectionDistance, points3D[3].y * detectionDistance, points3D[3].z * detectionDistance},
                1.0);
        auto tf = EntryPoint::GetContext().GetTransformationForFrame(frame);

        return frustum.transformed(tf);
    }

    void YoloDetector::Publish3DFrustums(FrameType frame, std::vector<DataModels::FrustumDetection>& detections) {
        visualization_msgs::msg::MarkerArray msg;
        auto time = get_clock()->now();

        int cnt = 0;
        for (const auto& detection: detections) {
            // Frustum
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = FrameTypeName(FrameType::kImu);
            marker.header.stamp = time;
            marker.id = cnt++;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.points = DataModels::FrustumDetection::GetGeometryVertices(detection.GetFrustum());

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color = GetColorByClass(detection.GetClass());
            marker.color.a = 0.2;
            msg.markers.push_back(marker);
        }

        for(int i = cnt ; i < maxYoloMarkers_[CameraIdentifierFromFrameType(frame)]; i++) {
            visualization_msgs::msg::Marker marker;
            marker.id = i;
            marker.header.frame_id = FrameTypeName(FrameType::kImu);
            marker.header.stamp = time;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.color.a = 0.0;
            msg.markers.push_back(marker);
        }

        frustumPublishers_[CameraIdentifierFromFrameType(frame)]->publish(msg);
        maxYoloMarkers_[CameraIdentifierFromFrameType(frame)] = std::max(maxYoloMarkers_[CameraIdentifierFromFrameType(frame)], cnt);
    }
}