//
// Created by standa on 8.3.23.
//
#include <algorithms/PointCloudExtrapolator.h>

namespace AtlasFusion::Algorithms {

    std::vector<std::shared_ptr<DataModels::PointCloudBatch>> PointCloudExtrapolator::SplitPointCloudToBatches(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan,
            DataModels::LocalPosition startPose,
            DataModels::LocalPosition endPose,
            const rtl::RigidTf3D<double>& sensorOffsetTf) {
        //Timer t("Split point cloud into batches");

        auto noOfBatches = EntryPoint::GetContext().GetConfigService().GetUInt32Value({"lidar_aggregator", "no_of_batches_per_scan"});

        std::deque<std::shared_ptr<DataModels::PointCloudBatch>> output;
        output.resize(noOfBatches);


        uint32_t singleBatchSize = std::ceil(scan->width / noOfBatches);
        uint64_t timeOffset = startPose.GetTimestamp();
        auto poseDiff = endPose - startPose;
        uint64_t duration = poseDiff.GetTimestamp();

        std::vector<std::future<uint32_t>> outputFutures;
        outputFutures.resize(noOfBatches);

        for (uint32_t i = 0; i < noOfBatches; i++) {
            outputFutures[i] = EntryPoint::GetContext().GetThreadPool().submit([&, i]() {
                double ratio = (double) i / noOfBatches;
                auto pose = DataModels::LocalPosition{
                        {poseDiff.GetPosition().x() * (ratio), poseDiff.GetPosition().y() * (ratio), poseDiff.GetPosition().z() * (ratio)},
                        {poseDiff.GetOrientation().slerp(rtl::Quaternion<double>::identity(), (float) (1 - ratio))},
                        uint64_t(duration * ratio)
                };

                rtl::RigidTf3D<double> movementCompensationTF{pose.GetOrientation(), pose.GetPosition()};
                uint64_t ts = timeOffset + static_cast<uint64_t>(ratio * duration);

                pcl::PointCloud<pcl::PointXYZ>::Ptr batch(new pcl::PointCloud<pcl::PointXYZ>);
                batch->reserve(singleBatchSize);

                // Filter selected part of the input point cloud
                uint32_t start = i * singleBatchSize;
                uint32_t end = start + singleBatchSize;
                if (i == noOfBatches - 1) end = scan->points.size();
                batch->width = end - start;

                std::copy(scan->begin() + start, scan->begin() + end, back_inserter(batch->points));

                //pointCloudProcessor_.sortPointCloud(batch, PointCloudProcessor::Axis::Z, false);
                auto globalTf = endPose.ToTf()(poseDiff.ToTf().inverted()(movementCompensationTF(sensorOffsetTf)));
                output[i] = std::make_shared<DataModels::PointCloudBatch>(ts, batch, FrameType::kOrigin, globalTf);
                return batch->width;
            });
        }

        uint32_t points = 0;
        for (auto& outputFuture: outputFutures) {
            outputFuture.wait();
            points += outputFuture.get();
        }

        if (scan->size() != points) {
            throw std::runtime_error("Mismatch during splitting point cloud into batches!");
        }

        return {output.begin(), output.end()};
    }

}