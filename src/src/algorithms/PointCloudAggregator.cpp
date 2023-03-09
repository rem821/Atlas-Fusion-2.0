//
// Created by standa on 9.3.23.
//
#include <algorithms/PointCloudAggregator.h>

namespace AtlasFusion::Algorithms {

    void PointCloudAggregator::AddLidarScan(const DataLoader::LidarIdentifier& lidarIdentifier,
                                            const std::vector<std::shared_ptr<DataModels::PointCloudBatch>>& scanBatches) {
        //Timer t("Add lidar scan");
        switch (lidarIdentifier) {
            case DataLoader::LidarIdentifier::kLeftLidar: {
                lidarLeftBatches_ = scanBatches;
                break;
            }
            case DataLoader::LidarIdentifier::kCenterLidar: {
                lidarCenterBatches_ = scanBatches;
                break;
            }
            case DataLoader::LidarIdentifier::kRightLidar: {
                lidarRightBatches_ = scanBatches;
                break;
            }
        }
        latestScanCutouts_.clear();
        latestScanValid_ = false;
        latestScanEgoValid_ = false;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudAggregator::GetLatestScan() {
        //Timer t("Get latest scan");

        if (latestScanValid_) return latestScanPoints_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
        scan->height = 1;
        if (!lidarLeftBatches_.empty()) {
            auto left = GetPointCloudFromBatches(lidarLeftBatches_);
            pcl::concatenate(*scan, *left, *scan);
        }
        if (!lidarCenterBatches_.empty()) {
            auto center = GetPointCloudFromBatches(lidarCenterBatches_);
            pcl::concatenate(*scan, *center, *scan);
        }
        if (!lidarRightBatches_.empty()) {
            auto right = GetPointCloudFromBatches(lidarRightBatches_);
            pcl::concatenate(*scan, *right, *scan);
        }
        scan->width = scan->points.size();
        latestScanPoints_ = scan->makeShared();
        latestScanValid_ = true;

        return latestScanPoints_;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr
    PointCloudAggregator::GetLatestScanEgoCentric(const rtl::RigidTf3D<double>& egoTf) {
        //Timer t("Get ego centric point cloud");

        if (latestScanEgoValid_) return latestScanEgoPoints_;

        latestScanEgoPoints_ = PointCloudProcessor::TransformPointCloud(GetLatestScan(), egoTf);
        latestScanEgoValid_ = true;

        return latestScanEgoPoints_;
    }


    pcl::PointCloud<pcl::PointXYZ>::ConstPtr
    PointCloudAggregator::GetLatestScanCutout(const rtl::RigidTf3D<double>& egoTf, const FrameType& frame) {
        //Timer t("Get latest scan cutout for frame: " + FrameTypeName(frame), 0);
        if (!latestScanValid_) GetLatestScanEgoCentric(egoTf);

        std::string f = FrameTypeName(frame);
        if (latestScanCutouts_.count(f) == 0) {
            latestScanCutouts_[f] = PointCloudProcessor::GetPointCloudCutoutForFrame(latestScanEgoPoints_, frame);
        }

        return latestScanCutouts_[f];
    }


    /** Short term aggregation section */

    void PointCloudAggregator::AddPointCloudBatches(
            const std::vector<std::shared_ptr<DataModels::PointCloudBatch>>& batches) {
        //Timer t("Add lidar batches");

        auto downsampledBatches = std::vector<std::shared_ptr<DataModels::PointCloudBatch>>();
        for (const auto& batch: batches) {
            auto points = PointCloudProcessor::DownsamplePointCloud(batch->GetPoints());
            downsampledBatches.emplace_back(
                    std::make_shared<DataModels::PointCloudBatch>(
                            DataModels::PointCloudBatch(batch->GetTimestamp(),
                                                        points,
                                                        batch->GetFrame(),
                                                        batch->GetGlobalTf()))
            );
            batchInfo_.emplace_back(batch->GetTimestamp(), points->size());
        }

        auto points = GetPointCloudFromBatches(downsampledBatches);
        pcl::concatenate(*aggregatedPoints_, *points, *aggregatedPoints_);

        aggregatedDownsampledPointsValid_ = false;
    }

    void PointCloudAggregator::FilterOutBatches(uint64_t currentTime) {
        //Timer t("Filter out lidar aggregated batches");
        if (batchInfo_.empty()) return;

        size_t pointsToDelete = 0;
        while (!batchInfo_.empty()) {
            auto timestamp = batchInfo_.front().first;
            auto noOfPoints = batchInfo_.front().second;

            auto timeDiff = static_cast<double>(currentTime - timestamp) * 1e-9;
            if (timeDiff < aggregationTime_) break;

            pointsToDelete += noOfPoints;
            batchInfo_.pop_front();
        }
        if (pointsToDelete == 0) return;
        if (pointsToDelete > aggregatedPoints_->size()) {
            aggregatedPoints_->points.clear();
            return;
        }

        aggregatedPoints_->points.erase(aggregatedPoints_->points.begin(),
                                        aggregatedPoints_->points.begin() + static_cast<long>(pointsToDelete));
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudAggregator::GetGlobalCoordinateAggregatedPointCloud() {
        //Timer t("Get aggregated point cloud");
        if (aggregatedDownsampledPointsValid_) return aggregatedPointsDownsampled_;

        aggregatedPointsDownsampled_ = PointCloudProcessor::DownsamplePointCloud(aggregatedPoints_);
        aggregatedDownsampledPointsValid_ = true;

        return aggregatedPointsDownsampled_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudAggregator::GetPointCloudFromBatches(
            const std::vector<std::shared_ptr<DataModels::PointCloudBatch>>& batches) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

        uint32_t totalPoints = 0;
        for (const auto& batch: batches) {

            auto batchPc = batch->GetPointsInGlobalCoordinates();
            totalPoints += batchPc->width;
            pcl::concatenate(*pc, *batchPc, *pc);
        }

        if (totalPoints != pc->size()) {
            throw std::runtime_error("Mismatch during batches to point cloud conversion!");
        }

        return pc;
    }
}