//
// Created by standa on 8.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace AtlasFusion::Algorithms {

    class PointCloudProcessor {

    public:

        enum Axis {
            X, Y, Z
        };

        PointCloudProcessor() = default;

        ~PointCloudProcessor() = default;


        static pcl::PointCloud<pcl::PointXYZ>::Ptr DownsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

        static pcl::PointCloud<pcl::PointXYZ>::Ptr TransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const rtl::RigidTf3D<double>& tf);

        static pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointCloudCutout(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const rtl::BoundingBox3f& boundingBox);

        static pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointCloudCutoutForFrame(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const FrameType& frame);

        void SortPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const Axis& axis, bool ascending = true);

        void SortPointCloudByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, bool ascending = true);

    private:

        static bool CompareXAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);

        static bool CompareYAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);

        static bool CompareZAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);

        static bool CompareXDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);

        static bool CompareYDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);

        static bool CompareZDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);

        static bool CompareDistAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);

        static bool CompareDistDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r);
    };

}