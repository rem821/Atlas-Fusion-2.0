//
// Created by standa on 7.3.23.
//
#include <EntryPoint.h>
#include <TFTree.h>
#include <Topics.h>
#include <data_loaders/DataLoaderController.h>
#include <data_loaders/CameraDataLoader.h>
#include <data_loaders/LidarDataLoader.h>
#include <data_loaders/ImuDataLoader.h>
#include <data_loaders/GnssDataLoader.h>
#include <data_loaders/RadarDataLoader.h>
#include <position_processing/SelfModel.h>
#include <lidar_processing/LidarAggregator.h>

namespace AtlasFusion {
    EntryPoint *EntryPoint::context_ = nullptr;

    EntryPoint::EntryPoint(std::string datasetPath) {
        context_ = this;
        Log::Init();

        configService_ = std::make_unique<ConfigService>(datasetPath);
        datasetPath_ = configService_->GetStringValue({"data_folder"});
        threadPool_ = std::make_unique<BS::thread_pool>(std::thread::hardware_concurrency() - 1);

        InitTFTree();
        InitROS();
    }

    void EntryPoint::InitTFTree() {
        FrameType rootFrame = FrameType::kImu;
        std::vector<FrameType> childFrames = {
                FrameType::kGnssAntennaFront,
                FrameType::kGnssAntennaRear,
                FrameType::kLidarLeft,
                FrameType::kLidarRight,
                FrameType::kLidarCenter,
                FrameType::kRadarTi,
                FrameType::kCameraLeftFront,
                FrameType::kCameraLeftSide,
                FrameType::kCameraRightFront,
                FrameType::kCameraRightSide,
                FrameType::kCameraIr
        };

        auto calibFolder = configService_->GetStringValue({"calibrations_folder"});
        tfTree_ = std::make_unique<TFTree>(
                BuildTFTree(
                        rootFrame,
                        childFrames,
                        std::string(calibFolder + "frames.yaml")
                )
        );
    }

    void EntryPoint::InitROS() {
        nodeOptions_ = rclcpp::NodeOptions().use_intra_process_comms(true);

        InitNodes();

        rosExecutor_.spin();
    }

    void EntryPoint::InitNodes() {
        /* Data Loaders */
        std::string cameraLeftSideDataLoader = "CameraLeftSideDataLoader";
        nodes_[cameraLeftSideDataLoader] = std::make_shared<DataLoader::CameraDataLoader>(
                cameraLeftSideDataLoader,
                DataLoader::CameraIdentifier::kCameraLeftSide,
                Topics::kCameraLeftSideDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[cameraLeftSideDataLoader]);

        std::string cameraLeftFrontDataLoader = "CameraLeftFrontDataLoader";
        nodes_[cameraLeftFrontDataLoader] = std::make_shared<DataLoader::CameraDataLoader>(
                cameraLeftFrontDataLoader,
                DataLoader::CameraIdentifier::kCameraLeftFront,
                Topics::kCameraLeftFrontDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[cameraLeftFrontDataLoader]);

        std::string cameraRightFrontDataLoader = "CameraRightFrontDataLoader";
        nodes_[cameraRightFrontDataLoader] = std::make_shared<DataLoader::CameraDataLoader>(
                cameraRightFrontDataLoader,
                DataLoader::CameraIdentifier::kCameraRightFront,
                Topics::kCameraRightFrontDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[cameraRightFrontDataLoader]);

        std::string cameraRightSideDataLoader = "CameraRightSideDataLoader";
        nodes_[cameraRightSideDataLoader] = std::make_shared<DataLoader::CameraDataLoader>(
                cameraRightSideDataLoader,
                DataLoader::CameraIdentifier::kCameraRightSide,
                Topics::kCameraRightSideDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[cameraRightSideDataLoader]);

        std::string cameraIrDataLoader = "CameraIrDataLoader";
        nodes_[cameraIrDataLoader] = std::make_shared<DataLoader::CameraDataLoader>(
                cameraIrDataLoader,
                DataLoader::CameraIdentifier::kCameraIr,
                Topics::kCameraIrDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[cameraIrDataLoader]);

        std::string lidarLeftDataLoader = "LidarLeftDataLoader";
        nodes_[lidarLeftDataLoader] = std::make_shared<DataLoader::LidarDataLoader>(
                lidarLeftDataLoader,
                DataLoader::LidarIdentifier::kLeftLidar,
                Topics::kLidarLeftDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[lidarLeftDataLoader]);

        std::string lidarCenterDataLoader = "LidarCenterDataLoader";
        nodes_[lidarCenterDataLoader] = std::make_shared<DataLoader::LidarDataLoader>(
                lidarCenterDataLoader,
                DataLoader::LidarIdentifier::kCenterLidar,
                Topics::kLidarCenterDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[lidarCenterDataLoader]);

        std::string lidarRightDataLoader = "LidarRightDataLoader";
        nodes_[lidarRightDataLoader] = std::make_shared<DataLoader::LidarDataLoader>(
                lidarRightDataLoader,
                DataLoader::LidarIdentifier::kRightLidar,
                Topics::kLidarRightDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[lidarRightDataLoader]);

        std::string imuDataLoader = "ImuDataLoader";
        nodes_[imuDataLoader] = std::make_shared<DataLoader::ImuDataLoader>(
                imuDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[imuDataLoader]);

        std::string gnssDataLoader = "GnssDataLoader";
        nodes_[gnssDataLoader] = std::make_shared<DataLoader::GnssDataLoader>(
                gnssDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[gnssDataLoader]);

        std::string radarDataLoader = "RadarDataLoader";
        nodes_[radarDataLoader] = std::make_shared<DataLoader::RadarDataLoader>(
                radarDataLoader,
                Topics::kRadarTiDataLoader,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[radarDataLoader]);


        /* Data Loader Controller */
        std::string dataLoaderController = "DataLoaderController";
        nodes_[dataLoaderController] = std::make_shared<DataLoader::DataLoaderController>(
                dataLoaderController,
                18,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[dataLoaderController]);


        /* Self model */
        std::string selfModel = "SelfModel";
        nodes_[selfModel] = std::make_shared<LocalMap::SelfModel>(
                selfModel,
                rclcpp::NodeOptions().use_intra_process_comms(false)
        );
        rosExecutor_.add_node(nodes_[selfModel]);


        /* Lidar Aggregator */
        std::string lidarAggregator = "LidarAggregator";
        nodes_[lidarAggregator] = std::make_shared<LocalMap::LidarAggregator>(
                lidarAggregator,
                Topics::kLidarAggregatedGlobal,
                nodeOptions_
        );
        rosExecutor_.add_node(nodes_[lidarAggregator]);
    }

    rtl::RigidTf3D<double> EntryPoint::GetTFFrameFromConfig(ConfigService &service, const FrameType &type) {
        auto translation = service.GetVector3DValue<double>({FrameTypeName(type), "trans"});
        auto rotation = service.GetQuaternionValue<double>({FrameTypeName(type), "rot"});
        rtl::RigidTf3D<double> frame{rotation, translation};
        return frame;
    }

    TFTree EntryPoint::BuildTFTree(FrameType rootFrame, const std::vector<FrameType> &frames, const std::string &tfFilePath) {
        ConfigService TFConfigService(tfFilePath);
        TFTree tfTree(rootFrame);
        for (const auto &frameType: frames) {
            auto frame = GetTFFrameFromConfig(TFConfigService, frameType);
            tfTree.AddFrame(frame, frameType);
        }
        return tfTree;
    }
}