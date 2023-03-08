/*
 * Copyright 2023 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

namespace AtlasFusion::Topics {

        const std::string kCameraLeftFrontDataLoader = "/atlasfusion/data_loader/cameras/camera_left_front/camera";
        const std::string kCameraLeftSideDataLoader = "/atlasfusion/data_loader/cameras/camera_left_side/camera";
        const std::string kCameraRightFrontDataLoader = "/atlasfusion/data_loader/cameras/camera_right_front/camera";
        const std::string kCameraRightSideDataLoader = "/atlasfusion/data_loader/cameras/camera_right_side/camera";
        const std::string kCameraIrDataLoader = "/atlasfusion/data_loader/cameras/camera_ir/camera";
        const std::string kLidarLeftDataLoader = "/atlasfusion/data_loader/lidar_left/points";
        const std::string kLidarRightDataLoader = "/atlasfusion/data_loader/lidar_right/points";
        const std::string kLidarCenterDataLoader = "/atlasfusion/data_loader/lidar_center/points";
        const std::string kImuDquatDataLoader = "/atlasfusion/data_loader/imu/dquat";
        const std::string kImuGnssDataLoader = "/atlasfusion/data_loader/imu/gnss";
        const std::string kImuImuDataLoader = "/atlasfusion/data_loader/imu/imu";
        const std::string kImuMagDataLoader = "/atlasfusion/data_loader/imu/mag";
        const std::string kImuPressureDataLoader = "/atlasfusion/data_loader/imu/pressure";
        const std::string kImuTempDataLoader = "/atlasfusion/data_loader/imu/temp";
        const std::string kImuTimeDataLoader = "/atlasfusion/data_loader/imu/time";
        const std::string kGnssPositionDataLoader = "/atlasfusion/data_loader/gnss/position";
        const std::string kGnssTimeDataLoader = "/atlasfusion/data_loader/gnss/time";
        const std::string kRadarTiDataLoader = "/atlasfusion/data_loader/radar_ti";

        const std::string kDataLoaderSynchronization = "/atlasfusion/data_loader/synchronization";

        const std::string kCameraLeftFront = "/atlasfusion/data/camera_left_front/camera";
        const std::string kCameraLeftSide = "/atlasfusion/data/camera_left_side/camera";
        const std::string kCameraRightFront = "/atlasfusion/data/camera_right_front/camera";
        const std::string kCameraRightSide = "/atlasfusion/data/camera_right_side/camera";
        const std::string kCameraIr = "/atlasfusion/data/camera_ir/camera";

        const std::string kCameraLeftFrontImage = "/atlasfusion/data/camera_left_front/camera/image";
        const std::string kCameraLeftSideImage = "/atlasfusion/data/camera_left_side/camera/image";
        const std::string kCameraRightFrontImage = "/atlasfusion/data/camera_right_front/camera/image";
        const std::string kCameraRightSideImage = "/atlasfusion/data/camera_right_side/camera/image";
        const std::string kCameraIrImage = "/atlasfusion/data/camera_ir/camera/image";


        const std::string kLidarLeft = "/atlasfusion/data/lidar_left/points";
        const std::string kLidarRight = "/atlasfusion/data/lidar_right/points";
        const std::string kLidarCenter = "/atlasfusion/data/lidar_center/points";
        const std::string kImuDquat = "/atlasfusion/data/imu/dquat";
        const std::string kImuGnss = "/atlasfusion/data/imu/gnss";
        const std::string kImuImu = "/atlasfusion/data/imu/imu";
        const std::string kImuMag = "/atlasfusion/data/imu/mag";
        const std::string kImuPressure = "/atlasfusion/data/imu/pressure";
        const std::string kImuTemp = "/atlasfusion/data/imu/temp";
        const std::string kImuTime = "/atlasfusion/data/imu/time";
        const std::string kGnssPosition = "/atlasfusion/data/gnss/position";
        const std::string kGnssTime = "/atlasfusion/data/gnss/time";
        const std::string kRadarTi = "/atlasfusion/data/radar_ti";

        const std::string kRawTrajectory = "/atlasfusion/local_map/trajectory/raw";
        const std::string kFilteredTrajectory = "/atlasfusion/local_map/trajectory/filtered";
        const std::string kImuGpsTrajectory = "/atlasfusion/local_map/trajectory/imu_gps";

        const std::string kSelfGlobal = "/atlasfusion/local_map/self/global";
        const std::string kSelfEgo = "/atlasfusion/local_map/self/ego";

        /*

        const std::string kLidarLeftStatus = "/atlasfusion/local_map/lidar_left/status";
        const std::string kLidarRightStatus = "/atlasfusion/local_map/lidar_right/status";
        const std::string kLidarCenterStatus = "/atlasfusion/local_map/lidar_center/status";
        const std::string kLidarLeftStatusString = "/atlasfusion/local_map/lidar_left/string";
        const std::string kLidarRightStatusString = "/atlasfusion/local_map/lidar_right/string";
        const std::string kLidarCenterStatusString = "/atlasfusion/local_map/lidar_center/string";
        const std::string kImuTopic = "/atlasfusion/local_map/imu/imu";
        const std::string kImuAvgTopic = "/atlasfusion/local_map/imu/imu_avg";
        const std::string kGnssTopic = "/atlasfusion/local_map/gnss/pose_text";

        const std::string kCameraLeftFront = "/atlasfusion/local_map/cameras/camera_left_front/camera";
        const std::string kCameraLeftSide = "/atlasfusion/local_map/cameras/camera_left_side/camera";
        const std::string kCameraRightFront = "/atlasfusion/local_map/cameras/camera_right_front/camera";
        const std::string kCameraRightSide = "/atlasfusion/local_map/cameras/camera_right_side/camera";
        const std::string kCameraIr = "/atlasfusion/local_map/cameras/camera_ir/camera";

        const std::string kCameraLeftFrontInfo = "/atlasfusion/local_map/cameras/camera_left_front/camera_info";
        const std::string kCameraLeftSideInfo = "/atlasfusion/local_map/cameras/camera_left_side/camera_info";
        const std::string kCameraRightFrontInfo = "/atlasfusion/local_map/cameras/camera_right_front/camera_info";
        const std::string kCameraRightSideInfo = "/atlasfusion/local_map/cameras/camera_right_side/camera_info";
        const std::string kCameraIrInfo = "/atlasfusion/local_map/cameras/camera_ir/camera_info";

        const std::string kCameraLeftFrontStatus = "/atlasfusion/local_map/cameras/camera_left_front/status";
        const std::string kCameraLeftSideStatus = "/atlasfusion/local_map/cameras/camera_left_side/status";
        const std::string kCameraRightFrontStatus = "/atlasfusion/local_map/cameras/camera_right_front/status";
        const std::string kCameraRightSideStatus = "/atlasfusion/local_map/cameras/camera_right_side/status";
        const std::string kCameraIrStatus = "/atlasfusion/local_map/cameras/camera_ir/status";
        const std::string kCameraLeftFrontStatusString = "/atlasfusion/local_map/cameras/camera_left_front/string";
        const std::string kCameraLeftSideStatusString = "/atlasfusion/local_map/cameras/camera_left_side/string";
        const std::string kCameraRightFrontStatusString = "/atlasfusion/local_map/cameras/camera_right_front/string";
        const std::string kCameraRightSideStatusString = "/atlasfusion/local_map/cameras/camera_right_side/string";
        const std::string kCameraIrStatusString = "/atlasfusion/local_map/cameras/camera_ir/string";

        const std::string kYoloFrustumDetections = "/atlasfusion/local_map/yolo/frustums";
        const std::string kYoloFusedFrustumDetections = "/atlasfusion/local_map/yolo/fused_frustums";


        const std::string kLidarAggregatedGlobal = "/atlasfusion/local_map/lidar/aggregated/global";
        const std::string kLidarAggregatedEgo = "/atlasfusion/local_map/lidar/aggregated/ego";

        const std::string kLidarLaser = "/atlasfusion/local_map/lidar/laser";
        const std::string kGlobalPointCloud = "/atlasfusion/local_map/lidar/global";
        const std::string kCutoutPointcloud = "/atlasfusion/local_map/lidar/cutout";

        const std::string kLidarApproximation = "/atlasfusion/local_map/lidar/approximations";
        const std::string kLidarApproximationRoad = "/atlasfusion/local_map/lidar/approximations_road";

        const std::string kLidarDetections = "/atlasfusion/local_map/lidar_detections";

        const std::string kRadarTiObjects = "/atlasfusion/local_map/radar_ti_data";

        const std::string kTelemetryText = "/atlasfusion/local_map/telemetry/text";
        const std::string kSpeedTopic = "/atlasfusion/local_map/speed";
        const std::string kEnvironmentalModel = "/atlasfusion/local_map/environmental_model";
         */

    }