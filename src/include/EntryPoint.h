//
// Created by standa on 7.3.23.
//
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <ConfigService.h>
#include <BS_thread_pool.hpp>
#include <data_models/CameraCalibrationParams.h>

namespace AtlasFusion {

    class EntryPoint {
    public:
        EntryPoint(std::string datasetPath);

        ~EntryPoint() = default;

        inline static EntryPoint& GetContext() { return *context_; }

        std::string GetDatasetPath() const { return datasetPath_; }

        const FrameType& GetRootFrameType() const { return tfTree_->GetRootFrameType(); }

        std::vector<FrameType> GetFrameTypes() const { return tfTree_->GetFrameTypes(); }

        rtl::RigidTf3D<double> GetTransformationForFrame(FrameType frameType) const { return tfTree_->GetTransformationForFrame(frameType); }

        rtl::Vector3D<double> TransformPointFromFrameToFrame(const rtl::Vector3D<double>& srcPoint, const FrameType& source,
                                                             const FrameType& destination) const { return tfTree_->TransformPointFromFrameToFrame(srcPoint, source, destination); }

        ConfigService& GetConfigService() const { return *configService_; }

        BS::thread_pool& GetThreadPool() const { return *threadPool_; }

        DataModels::CameraCalibrationParams CreateCameraCalibrationParams(DataLoader::CameraIdentifier cameraIdentifier);

    private:
        void InitTFTree();

        void InitROS();

        void InitNodes();

        rtl::RigidTf3D<double> GetTFFrameFromConfig(ConfigService& service, const FrameType& type);

        TFTree BuildTFTree(FrameType rootFrame, const std::vector<FrameType>& frames, const std::string& tfFilePath);

        std::unique_ptr<ConfigService> configService_;
        std::unique_ptr<TFTree> tfTree_;
        std::string datasetPath_;

        /* ROS */
        rclcpp::executors::MultiThreadedExecutor rosExecutor_;
        rclcpp::NodeOptions nodeOptions_;

        std::map<std::string, rclcpp::Node::SharedPtr> nodes_;

        /* Thread pool */
        std::unique_ptr<BS::thread_pool> threadPool_;

        static EntryPoint* context_;
    };
}
