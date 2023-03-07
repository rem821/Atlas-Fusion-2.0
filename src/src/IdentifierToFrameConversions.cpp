//
// Created by standa on 21.12.22.
//

#include <IdentifierToFrameConversions.h>

namespace AtlasFusion {

    std::string FrameTypeName(const FrameType& frame) {
        switch(frame) {
            case FrameType::kOrigin: return "origin";
            case FrameType::kLidarLeft: return "lidar_left";
            case FrameType::kLidarRight: return "lidar_right";
            case FrameType::kLidarCenter: return "lidar_center";
            case FrameType::kRadarTi: return "radar_ti";
            case FrameType::kImu: return "imu";
            case FrameType::kCameraLeftFront: return "camera_left_front";
            case FrameType::kCameraLeftSide: return "camera_left_side";
            case FrameType::kCameraRightFront: return "camera_right_front";
            case FrameType::kCameraRightSide: return "camera_right_side";
            case FrameType::kCameraIr: return "camera_ir";
            case FrameType::kGnssAntennaFront: return "gnss_front";
            case FrameType::kGnssAntennaRear: return "gnss_rear";
            default: throw std::runtime_error("Unknown frame type name!");
        }
    }

    FrameType FrameTypeFromIdentifier(const DataLoader::CameraIdentifier &identifier) {
        switch (identifier) {
            case DataLoader::CameraIdentifier::kCameraLeftFront:
                return FrameType::kCameraLeftFront;
            case DataLoader::CameraIdentifier::kCameraLeftSide:
                return FrameType::kCameraLeftSide;
            case DataLoader::CameraIdentifier::kCameraRightFront:
                return FrameType::kCameraRightFront;
            case DataLoader::CameraIdentifier::kCameraRightSide:
                return FrameType::kCameraRightSide;
            case DataLoader::CameraIdentifier::kCameraIr:
                return FrameType::kCameraIr;
            default:
                throw std::runtime_error("Unable to convert between camera identifier and frame type!");
        }
    }

    FrameType FrameTypeFromIdentifier(const DataLoader::LidarIdentifier &identifier) {
        switch (identifier) {
            case DataLoader::LidarIdentifier::kLeftLidar:
                return FrameType::kLidarLeft;
            case DataLoader::LidarIdentifier::kCenterLidar:
                return FrameType::kLidarCenter;
            case DataLoader::LidarIdentifier::kRightLidar:
                return FrameType::kLidarRight;
            default:
                throw std::runtime_error("Unable to convert between lidar identifier and frame type!");
        }
    }
}