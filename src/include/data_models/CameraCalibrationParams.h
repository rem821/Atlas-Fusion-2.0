//
// Created by standa on 10.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <opencv2/opencv.hpp>

namespace AtlasFusion::DataModels {

    class CameraCalibrationParams {

    public:
        CameraCalibrationParams() {};

        CameraCalibrationParams(size_t width, size_t height, std::vector<std::vector<double>>& intrinsic, std::vector<double>& distortion)
                : width_{width}, height_{height}, intrinsic_{intrinsic}, distortion_{distortion} {
        }

        size_t GetWidth() const { return width_; };

        size_t GetHeight() const { return height_; };

        std::vector<std::vector<double>> GetIntrinsicParams() const { return intrinsic_; };

        std::vector<double> GetDistortionParams() const { return distortion_; };

        cv::Mat GetMatIntrinsicParams() {
            cv::Mat mat = (
                    cv::Mat_<double>(3, 3)
                            <<
                            intrinsic_[0][0], intrinsic_[0][1], intrinsic_[0][2],
                            intrinsic_[1][0], intrinsic_[1][1], intrinsic_[1][2],
                            intrinsic_[2][0], intrinsic_[2][1], intrinsic_[2][2]
            );
            return mat;
        };

        cv::Mat GetMatDistortionParams() {
            cv::Mat mat = (cv::Mat_<double>(1, 5) << distortion_[0], distortion_[1], distortion_[2], distortion_[3], distortion_[4]);
            return mat;
        };

    private:
        size_t width_{};
        size_t height_{};
        std::vector<std::vector<double>> intrinsic_{};
        std::vector<double> distortion_{};
    };
}