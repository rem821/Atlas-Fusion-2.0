//
// Created by standa on 10.3.23.
//
#include <camera_processing/YoloDetector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithms/PointCloudProcessor.h>

namespace AtlasFusion::LocalMap {

    YoloDetector::YoloDetector(const std::string& name, const std::string& topic, const rclcpp::NodeOptions& options) : Node(name, options),
                                                                                                                        aggregatedPointCloud_(new pcl::PointCloud<pcl::PointXYZ>) {

        InitProjectors();
        InitSubscribers();
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
            LOG_WARN("No valid 2D points to estimate detection frustums in 3D!");
        }

        for (auto& detection: msg->yolo_detections) {
            auto indices = GetPointsInsideDetectionIndices(valid2DPoints, detection);
            float distance = GetMedianDepthOfPoints(valid3DPoints, indices);
            LOG_INFO("YOLO detection from camera {} of class {} is being detected {}m from the vehicle with confidence of {}", msg->image.header.frame_id,
                     detection.detection_class, distance, detection.detection_confidence);
        }
    }

    void YoloDetector::OnNewLidarData(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        LOG_INFO("YoloDetector: Lidar Data");
        pcl::fromROSMsg(*msg.get(), *aggregatedPointCloud_.get());
    }

    void YoloDetector::ProjectAllPointsIntoTheImage(FrameType frame, uint32_t imageWidth, uint32_t imageHeight, std::vector<cv::Point2f>& validPoints2D,
                                                    std::vector<cv::Point3f>& validPoints3D) {
        auto sensorCutoutPc = Algorithms::PointCloudProcessor::GetPointCloudCutoutForFrame(aggregatedPointCloud_, frame);
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
}