//
// Created by standa on 6.3.23.
//
#pragma once

namespace AtlasFusion {

    enum class FrameType {
        kOrigin,
        kLidarLeft,
        kLidarRight,
        kLidarCenter,
        kRadarTi,
        kImu,
        kCameraLeftFront,
        kCameraLeftSide,
        kCameraRightFront,
        kCameraRightSide,
        kCameraIr,
        kGnssAntennaFront,
        kGnssAntennaRear,
    };
}