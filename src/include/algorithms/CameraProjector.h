//
// Created by standa on 10.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <opencv2/opencv.hpp>

namespace AtlasFusion::Algorithms {

    class CameraProjector {

    public:

        CameraProjector(cv::Mat intrinsic, cv::Mat distortion, rtl::RigidTf3D<double>& tf) : intrinsic_{std::move(intrinsic)}, distortion_{std::move(distortion)} {
            auto rotMat = tf.rotQuaternion().rotMat();

            tvec_ = (cv::Mat_<float>(3, 1) << 0, 0, 0);
            rvec_ = (cv::Mat_<float>(3, 1) << 0, 0, 0);
        }

        void ProjectPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dest, bool useDist = true);

        void ReverseProjection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest, bool useDist = true);

    private:

        const cv::Mat intrinsic_;
        const cv::Mat distortion_;
        cv::Mat rvec_;
        cv::Mat tvec_;
    };
}
