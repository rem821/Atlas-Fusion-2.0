//
// Created by standa on 21.12.22.
//

#include <IdentifierToFrameConversions.h>

namespace AtlasFusion {

    std::string frameTypeName(const FrameType& frame) {
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
}