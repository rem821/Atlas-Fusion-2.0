//
// Created by standa on 12.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <data_models/YoloDetectionClass.h>

namespace AtlasFusion::DataModels {

    class LidarDetection {

    public:

        LidarDetection(const rtl::BoundingBox3d &box, const rtl::Quaterniond &quat, size_t id, YoloDetectionClass cls = YoloDetectionClass::kUnknown, uint32_t ttl = 10)
                : box_{box}, quat_{quat}, id_{id}, ttl_{ttl}, cls_{cls} {}


        [[nodiscard]] rtl::BoundingBox3d GetBoundingBox() const { return box_; }

        [[nodiscard]] rtl::Quaterniond GetOrientation() const { return quat_; }

        [[nodiscard]] size_t GetID() const { return id_; }

        [[nodiscard]] size_t GetTTL() const { return ttl_; };

        [[nodiscard]] YoloDetectionClass GetDetectionClass() const { return cls_; };

        void SetDetectionClass(const YoloDetectionClass& detectionClass) { cls_ = detectionClass; };
    private:

        rtl::BoundingBox3d box_;
        rtl::Quaterniond quat_;
        size_t id_;
        uint32_t ttl_;
        YoloDetectionClass cls_;
    };
}