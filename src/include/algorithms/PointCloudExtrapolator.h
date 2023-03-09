//
// Created by standa on 8.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <data_models/PointCloudBatch.h>
#include <data_models/LocalPosition.h>

namespace AtlasFusion::Algorithms {

    class PointCloudExtrapolator {

    public:
        static std::vector<std::shared_ptr<DataModels::PointCloudBatch>> SplitPointCloudToBatches(
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan,
                DataModels::LocalPosition startPose,
                DataModels::LocalPosition endPose,
                const rtl::RigidTf3D<double> &sensorOffset);

    };

}
