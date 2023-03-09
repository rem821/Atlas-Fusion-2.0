//
// Created by standa on 9.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <data_models/PointCloudBatch.h>

namespace AtlasFusion::Algorithms {

    class PointCloudAggregator {

    public:

        explicit PointCloudAggregator() : aggregatedPoints_{new pcl::PointCloud<pcl::PointXYZ>} {
            aggregationTime_ = EntryPoint::GetContext().GetConfigService().GetFloatValue({"lidar_aggregator", "aggregation_time"});
        }


        void AddLidarScan(const DataLoader::LidarIdentifier& lidarIdentifier,
                          const std::vector<std::shared_ptr<DataModels::PointCloudBatch>>& scanBatches);


        pcl::PointCloud<pcl::PointXYZ>::ConstPtr GetLatestScan();

        pcl::PointCloud<pcl::PointXYZ>::ConstPtr GetLatestScanEgoCentric(const rtl::RigidTf3D<double>& egoTf);

        pcl::PointCloud<pcl::PointXYZ>::ConstPtr GetLatestScanCutout(const rtl::RigidTf3D<double>& egoTf, const FrameType& frame);


        void AddPointCloudBatches(const std::vector<std::shared_ptr<DataModels::PointCloudBatch>>& batches);

        void FilterOutBatches(uint64_t currentTime);

        pcl::PointCloud<pcl::PointXYZ>::ConstPtr GetGlobalCoordinateAggregatedPointCloud();

    private:


        pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetPointCloudFromBatches(const std::vector<std::shared_ptr<DataModels::PointCloudBatch>>& batches);

        /** Lidar scan aggregation section */
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> lidarLeftBatches_{}, lidarCenterBatches_{}, lidarRightBatches_{};
        pcl::PointCloud<pcl::PointXYZ>::Ptr latestScanPoints_{}, latestScanEgoPoints_{};

        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> latestScanCutouts_{};

        bool latestScanValid_ = false;
        bool latestScanEgoValid_ = false;

        /** Short term aggregation section */
        double aggregationTime_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr aggregatedPoints_{};
        pcl::PointCloud<pcl::PointXYZ>::Ptr aggregatedPointsDownsampled_{};

        bool aggregatedDownsampledPointsValid_ = false;

        // Holds timestamp (first) and number of points (second) of all batches added in order
        std::deque<std::pair<uint64_t, long>> batchInfo_;
    };
}