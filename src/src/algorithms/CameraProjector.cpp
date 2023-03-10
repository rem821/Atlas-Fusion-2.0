//
// Created by standa on 10.3.23.
//
#include <algorithms/CameraProjector.h>

namespace AtlasFusion::Algorithms {

    void CameraProjector::ProjectPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dest, bool useDist) {
        if (src.empty()) return;
        dest.reserve(src.size());

        if (useDist) {
            cv::projectPoints(src, rvec_, tvec_, intrinsic_, distortion_, dest);
        } else {
            cv::projectPoints(src, rvec_, tvec_, intrinsic_, {}, dest);
        }
    }


    void CameraProjector::ReverseProjection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest, bool useDist) {
        std::vector<cv::Point2f> undistorted;
        if (useDist) {
            cv::undistortPoints(src, undistorted, intrinsic_, distortion_);
        } else {
            cv::undistortPoints(src, undistorted, intrinsic_, {});
        }

        for (const auto& p: undistorted) {
            auto denominator = static_cast<float>(std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + 1));
            cv::Point3f direction = {p.x / denominator, p.y / denominator, 1 / denominator};
            dest.push_back(direction);
        }
    }

}