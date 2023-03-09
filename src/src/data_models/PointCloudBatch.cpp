//
// Created by standa on 8.3.23.
//
#include <data_models/PointCloudBatch.h>

namespace AtlasFusion::DataModels {


    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBatch::GetPoints() const {
        return points_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBatch::GetPointsInGlobalCoordinates() const {
        return Algorithms::PointCloudProcessor::TransformPointCloud(points_, globalTf_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBatch::GetTransformedPointsWithAnotherTF(rtl::RigidTf3D<double>& tf) const {
        return Algorithms::PointCloudProcessor::TransformPointCloud(points_, tf(globalTf_));
    }
}