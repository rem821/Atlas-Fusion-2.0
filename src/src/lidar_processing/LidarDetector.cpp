//
// Created by standa on 12.3.23.
//
#include <lidar_processing/LidarDetector.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <algorithms/PointCloudProcessor.h>
#include <data_models/LocalPosition.h>


namespace AtlasFusion::LocalMap {

    LidarDetector::LidarDetector(const std::string& name, const std::string& topic, const rclcpp::NodeOptions& options) : rclcpp::Node(name, options) {

        lidarDetectionsPublisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(topic, 1);

        aggregatedPointCloudSubscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                Topics::kLidarAggregatedGlobal,
                1,
                std::bind(&LidarDetector::OnLidarData, this, std::placeholders::_1)
        );

        selfTransformationSubscriber_ = create_subscription<geometry_msgs::msg::TransformStamped>(
                Topics::kSelfTransformation,
                1,
                std::bind(&LidarDetector::OnSelfTransformation, this, std::placeholders::_1)
        );
    }

    void LidarDetector::OnLidarData(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        Timer t("LidarDetector->OnLidarData");

        if (msg->data.empty()) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg.get(), *pc.get());

        // Prepare point cloud
        auto ego_pc = Algorithms::PointCloudProcessor::TransformPointCloud(pc, egoTf_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr detection_pc(new pcl::PointCloud<pcl::PointXYZ>());
        detection_pc = Algorithms::PointCloudProcessor::GetPointCloudCutout(ego_pc, {{-40.f, -5.f, -.95f},
                                                                                     {40.f,  12.f, 1.2f}});
        auto detection_pc_downsampled = Algorithms::PointCloudProcessor::DownsamplePointCloud(detection_pc);

        std::vector<pcl::PointIndices> cluster_indices;
        {
            //Timer t("EuclideanClusterExtraction");
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.50); // 20cm
            ec.setMinClusterSize(20);
            ec.setMaxClusterSize(100000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(detection_pc_downsampled);
            ec.extract(cluster_indices);
        }
        if (cluster_indices.empty()) return;

        std::deque<std::shared_ptr<DataModels::LidarDetection>> detections;
        detections.resize(cluster_indices.size());

        std::vector<std::future<void>> outputFutures{};
        outputFutures.resize(cluster_indices.size());

        {
            //Timer t("Object detection");
            for (size_t i = 0; i < cluster_indices.size(); i++) {
                outputFutures[i] = EntryPoint::GetContext().GetThreadPool().submit([&, i]() {
                    auto it = cluster_indices[i];
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                    cloud_cluster->points.reserve(cluster_indices.size());
                    cloud_cluster->width = cluster_indices.size();
                    cloud_cluster->height = 1;
                    for (const auto& pit: it.indices) {
                        cloud_cluster->points.emplace_back(pc->points[pit]);
                    }

                    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> featureExtractor;
                    featureExtractor.setInputCloud(cloud_cluster);
                    featureExtractor.compute();

                    std::vector<float> eccentricity;
                    pcl::PointXYZ minPointOBB, maxPointOBB, positionOBB;
                    Eigen::Matrix3f rotationalMatrixOBB;

                    featureExtractor.getOBB(minPointOBB, maxPointOBB, positionOBB, rotationalMatrixOBB);

                    auto euler = rotationalMatrixOBB.eulerAngles(0, 1, 2);
                    Eigen::Quaternionf q(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
                    rtl::Quaterniond quat(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));

                    LOG_WARN("Cluster {} min point: {} {} {}, max point: {} {} {}, position: {} {} {}",
                             i,
                             minPointOBB.x, minPointOBB.y, minPointOBB.z,
                             maxPointOBB.x, maxPointOBB.y, maxPointOBB.z,
                             positionOBB.x, positionOBB.y, positionOBB.z
                    );

                    detections[i] = std::make_shared<DataModels::LidarDetection>(
                            rtl::BoundingBox3d{
                                    {positionOBB.x + minPointOBB.x, positionOBB.y + minPointOBB.y, positionOBB.z + minPointOBB.z},
                                    {positionOBB.x + maxPointOBB.x, positionOBB.y + maxPointOBB.y, positionOBB.z + maxPointOBB.z}
                            },
                            quat,
                            i
                    );
                });
            }

            for (auto& outputFuture: outputFutures) {
                outputFuture.wait();
            }
        }

        PublishLidarDetections({detections.begin(), detections.end()});
    }

    void LidarDetector::OnSelfTransformation(geometry_msgs::msg::TransformStamped::UniquePtr msg) {
        rtl::Vector3D<double> position = rtl::Vector3D<double>(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
        rtl::Quaternion<double> orientation = rtl::Quaternion<double>(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
        auto lp = DataModels::LocalPosition(position, orientation, 0);
        egoTf_ = lp.ToTf().inverted();
    }

    void LidarDetector::PublishLidarDetections(const std::vector<std::shared_ptr<DataModels::LidarDetection>>& detections) {
        visualization_msgs::msg::MarkerArray output;

        auto timestamp = get_clock()->now();
        int cnt = 0;
        for (const auto& detection: detections) {
            double dx = detection->GetBoundingBox().max().getElement(0) - detection->GetBoundingBox().min().getElement(0);
            double dy = detection->GetBoundingBox().max().getElement(1) - detection->GetBoundingBox().min().getElement(1);
            double dz = detection->GetBoundingBox().max().getElement(2) - detection->GetBoundingBox().min().getElement(2);

            double cx = (detection->GetBoundingBox().max().getElement(0) + detection->GetBoundingBox().min().getElement(0)) / 2;
            double cy = (detection->GetBoundingBox().max().getElement(1) + detection->GetBoundingBox().min().getElement(1)) / 2;
            double cz = (detection->GetBoundingBox().max().getElement(2) + detection->GetBoundingBox().min().getElement(2)) / 2;

            // Bounding Box
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = FrameTypeName(FrameType::kImu);
            marker.header.stamp = timestamp;
            marker.id = cnt++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;

            marker.pose.orientation.x = detection->GetOrientation().x();
            marker.pose.orientation.y = detection->GetOrientation().y();
            marker.pose.orientation.z = detection->GetOrientation().z();
            marker.pose.orientation.w = detection->GetOrientation().w();

            marker.scale.x = dx;
            marker.scale.y = dy;
            marker.scale.z = dz;

            marker.color.a = 0.5;
            marker.color.r = 0.7;
            marker.color.g = 0.7;
            marker.color.b = 0.7;

            output.markers.push_back(marker);
        }

        for (int i = cnt; i < maxLidarMarkers_; i++) {
            visualization_msgs::msg::Marker marker;
            marker.id = i;
            marker.header.frame_id = FrameTypeName(FrameType::kImu);
            marker.header.stamp = timestamp;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.color.a = 0.0;
            output.markers.push_back(marker);
        }

        maxLidarMarkers_ = std::max(maxLidarMarkers_, cnt);
        lidarDetectionsPublisher_->publish(output);
    }
}