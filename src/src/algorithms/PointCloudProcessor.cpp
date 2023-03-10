//
// Created by standa on 8.3.23.
//
#include <algorithms/PointCloudProcessor.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

namespace AtlasFusion::Algorithms {


    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::DownsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
        //Timer t("Downsampling points");
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        auto leafSize = EntryPoint::GetContext().GetConfigService().GetFloatValue({"lidar_aggregator", "leaf_size"});

        pcl::VoxelGrid<pcl::PointXYZ> downsampler;
        downsampler.setInputCloud(input);
        downsampler.setLeafSize(leafSize, leafSize, leafSize);
        downsampler.filter(*output);

        return output;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudProcessor::TransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const rtl::RigidTf3D<double>& tf) {
        //TODO: This function is really slow concatenating point clouds doesn't really have an alternative that I know of.
        //Timer t("Transform point cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (input->points.empty()) return output;
        output->reserve(input->points.size());

        auto threads = EntryPoint::GetContext().GetThreadPool().get_thread_count();
        std::vector<std::future<pcl::PointCloud<pcl::PointXYZ>::Ptr>> outputFutures;
        outputFutures.resize(threads);
        size_t batchSize = std::ceil(input->points.size() / threads);

        if (batchSize == 0) return output;

        auto rotMat = tf.rotMat();
        Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf(0, 0) = static_cast<float>(rotMat(0, 0));
        pcl_tf(1, 0) = static_cast<float>(rotMat(1, 0));
        pcl_tf(2, 0) = static_cast<float>(rotMat(2, 0));
        pcl_tf(0, 1) = static_cast<float>(rotMat(0, 1));
        pcl_tf(1, 1) = static_cast<float>(rotMat(1, 1));
        pcl_tf(2, 1) = static_cast<float>(rotMat(2, 1));
        pcl_tf(0, 2) = static_cast<float>(rotMat(0, 2));
        pcl_tf(1, 2) = static_cast<float>(rotMat(1, 2));
        pcl_tf(2, 2) = static_cast<float>(rotMat(2, 2));
        pcl_tf.translation() << float(tf.trVecX()), float(tf.trVecY()), float(tf.trVecZ());

        for (uint32_t i = 0; i < threads; i++) {
            outputFutures[i] = EntryPoint::GetContext().GetThreadPool().submit([&input, pcl_tf, i, batchSize, threads] {
                pcl::PointCloud<pcl::PointXYZ>::Ptr outBatch(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr inBatch(new pcl::PointCloud<pcl::PointXYZ>);

                // Filter selected part of the input point cloud
                uint32_t start = i * batchSize;
                uint32_t end = start + batchSize;
                if (i == threads - 1) end = input->points.size();

                inBatch->reserve(end - start);
                inBatch->width = end - start;
                inBatch->height = 1;
                outBatch->width = end - start;
                outBatch->height = 1;

                std::copy(input->begin() + start, input->begin() + end, back_inserter(inBatch->points));

                pcl::transformPointCloud(*inBatch, *outBatch, pcl_tf);
                return outBatch;
            });
        }

        for (auto& outputFuture: outputFutures) {
            outputFuture.wait();
            pcl::concatenate(*output, *outputFuture.get(), *output);
        }

        if (input->size() != output->size()) {
            throw std::runtime_error("Mismatch during the point cloud transformation!");
        }

        return output;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudProcessor::GetPointCloudCutout(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const rtl::BoundingBox3f& boundingBox) {
        //Timer t("Get point cloud cutout");

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        output->reserve(input->size());

        for (const auto& point: input->points) {
            if (point.x > boundingBox.min().getElement(0) && point.x < boundingBox.max().getElement(0) &&
                point.y > boundingBox.min().getElement(1) && point.y < boundingBox.max().getElement(1) &&
                point.z > boundingBox.min().getElement(2) && point.z < boundingBox.max().getElement(2)) {
                output->push_back(point);
            }
        }
        return output;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudProcessor::GetPointCloudCutoutForFrame(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const FrameType& frame) {
        //Timer t("Get point cloud cutout for frame");

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        output->reserve(input->size());

        float max = std::numeric_limits<float>::max();
        float min = std::numeric_limits<float>::lowest();

        switch (frame) {
            case FrameType::kCameraIr:
            case FrameType::kCameraLeftFront:
            case FrameType::kCameraRightFront: {
                output = GetPointCloudCutout(input, {{0.f, -20.f, min},
                                                     {max, 20.f,  max}});
                break;
            }
            case FrameType::kCameraLeftSide: {
                output = GetPointCloudCutout(input, {{-5.f, 0.f, min},
                                                     {20.f, max, max}});
                break;
            }
            case FrameType::kCameraRightSide: {
                output = GetPointCloudCutout(input, {{-5.f, min, min},
                                                     {20.f, 0.f, max}});
                break;
            }
            default:
                break;
        }

        return output;
    }

    void PointCloudProcessor::SortPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const Axis& axis, bool ascending) {
        auto timerName = "Sorting point cloud of length: " + std::to_string(input->width)
                         + " by " + std::string(axis == X ? "X" : axis == Y ? "Y" : "Z") + " axis";
        //Timer t(timerName);

        switch (axis) {
            case X:
                std::stable_sort(input->points.begin(), input->points.end(), ascending ? &CompareXAsc : &CompareXDesc);
                break;
            case Y:
                std::stable_sort(input->points.begin(), input->points.end(), ascending ? &CompareYAsc : &CompareYDesc);
                break;
            case Z:
                std::stable_sort(input->points.begin(), input->points.end(), ascending ? &CompareZAsc : &CompareZDesc);
                break;
        }
    }

    void PointCloudProcessor::SortPointCloudByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, bool ascending) {
        //Timer t("Sorting point cloud of length: " + std::to_string(input->width) + " by distance");

        std::stable_sort(input->points.begin(), input->points.end(), ascending ? &CompareDistAsc : &CompareDistDesc);
    }


    bool PointCloudProcessor::CompareXAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return l.x < r.x;
    }

    bool PointCloudProcessor::CompareYAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return l.y < r.y;
    }

    bool PointCloudProcessor::CompareZAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return l.z < r.z;
    }

    bool PointCloudProcessor::CompareXDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return !CompareXAsc(l,r);
    }

    bool PointCloudProcessor::CompareYDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return !CompareYAsc(l,r);
    }

    bool PointCloudProcessor::CompareZDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return !CompareZAsc(l,r);
    }

    bool PointCloudProcessor::CompareDistAsc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return (std::abs(l.x) + std::abs(l.y) + std::abs(l.z)) < (std::abs(r.x) + std::abs(r.y) + std::abs(r.z));
    }

    bool PointCloudProcessor::CompareDistDesc(const pcl::PointXYZ& l, const pcl::PointXYZ& r) {
        return !CompareZAsc(l, r);
    }
}