//
// Created by standa on 8.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <algorithms/PointCloudProcessor.h>

namespace AtlasFusion::DataModels {

    class PointCloudBatch {

    public:

        PointCloudBatch() = delete;

        explicit PointCloudBatch(uint64_t ts, pcl::PointCloud<pcl::PointXYZ>::Ptr points, FrameType frame, const rtl::RigidTf3D<double> &globalTf)
                : timestamp_{ts}, points_{std::move(points)}, referenceFrame_{frame}, globalTf_{globalTf} {}

        [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr GetPoints() const;

        [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointsInGlobalCoordinates() const;

        pcl::PointCloud<pcl::PointXYZ>::Ptr GetTransformedPointsWithAnotherTF(rtl::RigidTf3D<double> &tf) const;

        [[nodiscard]] uint64_t GetTimestamp() const { return timestamp_; };

        [[nodiscard]] size_t GetPointsSize() const { return points_->size(); };

        [[nodiscard]] FrameType GetFrame() const { return referenceFrame_; };

        [[nodiscard]] rtl::RigidTf3D<double> GetGlobalTf() const { return globalTf_; };

    private:

        uint64_t timestamp_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr points_;
        FrameType referenceFrame_;
        rtl::RigidTf3D<double> globalTf_;
    };

}