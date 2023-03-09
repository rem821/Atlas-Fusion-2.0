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

#include "position_processing/SelfModel.h"
#include <algorithms/QuaternionConversions.h>

namespace AtlasFusion::LocalMap {

    SelfModel::SelfModel(const std::string& name, const rclcpp::NodeOptions& options) : Node(name, options) {

        auto processingNoise = EntryPoint::GetContext().GetConfigService().GetFloatValue({"self_model", "kalman_process_noise"});
        auto observationNoise = EntryPoint::GetContext().GetConfigService().GetFloatValue({"self_model", "kalman_observation_noise"});

        kalmanX_ = std::make_unique<Algorithms::Kalman1D>(processingNoise, observationNoise);
        kalmanY_ = std::make_unique<Algorithms::Kalman1D>(processingNoise, observationNoise);
        kalmanZ_ = std::make_unique<Algorithms::Kalman1D>(processingNoise, observationNoise);

        // Publisher that publishes position data
        staticTransformBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        rootToOriginTransformBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        selfGlobalPublisher_ = create_publisher<visualization_msgs::msg::Marker>(Topics::kSelfGlobal, 1);
        filteredTrajectoryPublisher_ = create_publisher<visualization_msgs::msg::Marker>(Topics::kFilteredTrajectory, 1);

        positionEstimationService_ = create_service<atlas_fusion_interfaces::srv::EstimatePositionInTime>(
                "estimate_pose",
                std::bind(&SelfModel::EstimatePositionInTime, this, std::placeholders::_1, std::placeholders::_2)
        );

        using namespace std::chrono_literals;
        timer_ = create_wall_timer(20ms, [this] { OnPublishPoseAndTrajectory(); });

        gnssSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::GnssPositionData>(
                Topics::kGnssPosition,
                1,
                std::bind(&SelfModel::OnGnssPose, this, std::placeholders::_1)
        );

        imuImuSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuImuData>(
                Topics::kImuImu,
                1,
                std::bind(&SelfModel::OnImuImu, this, std::placeholders::_1)
        );

        imuDquatSubscriber_ = create_subscription<atlas_fusion_interfaces::msg::ImuDquatData>(
                Topics::kImuDquat,
                1,
                std::bind(&SelfModel::OnImuDquat, this, std::placeholders::_1)
        );

        PublishStaticTransforms();
    }


    void SelfModel::OnGnssPose(atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr msg) {
        LOG_TRACE("SelfModel: GNSS Pose data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        auto gnssPose = DataModels::GlobalPosition{msg->latitude, msg->longitude, msg->altitude, 0};
        auto imuPose = GnssPoseToRootFrame(gnssPose);

        if (!poseInitialized_) {
            anchorPose_ = imuPose;
            poseInitialized_ = true;

            lastImuTimestamp_ = msg->timestamp;
            lastDquatTimestamp_ = msg->timestamp;
        }

        auto heading = FuseHeadings(msg);
        UpdateOrientation(heading.first);
        orientationInitialized_ = true;

        while (positionHistory_.size() >= 100) {
            positionHistory_.pop_front();
        }

        auto offset = DataModels::GlobalPosition::GetOffsetBetweenCoords(anchorPose_, imuPose);
        cv::Mat measurementX = (cv::Mat_<double>(2, 1) << offset.x(), 0);
        cv::Mat measurementY = (cv::Mat_<double>(2, 1) << offset.y(), 0);
        cv::Mat measurementZ = (cv::Mat_<double>(2, 1) << offset.z(), 0);
        kalmanX_->Correct(measurementX);
        kalmanY_->Correct(measurementY);
        kalmanZ_->Correct(measurementZ);

        DataModels::LocalPosition position{{kalmanX_->GetPosition(), kalmanY_->GetPosition(), kalmanZ_->GetPosition()}, orientation_, msg->timestamp};
        positionHistory_.push_back(position);

        lastGnssTimestamp_ = msg->timestamp;
    }

    void SelfModel::OnImuImu(atlas_fusion_interfaces::msg::ImuImuData::UniquePtr msg) {
        LOG_TRACE("SelfModel: IMU Imu data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        if (poseInitialized_) {
            auto linAccNoGrav = RemoveGravitationalForceFromLinAcc(msg);
            auto orientationTF = rtl::RigidTf3D<double>{orientation_, {0.0, 0.0, 0.0}};
            auto rotatedLinAccNoGrav = orientationTF(linAccNoGrav);

            auto dt = float(msg->timestamp - lastImuTimestamp_) * 1e-9;
            kalmanX_->Predict(dt, rotatedLinAccNoGrav.x());
            kalmanY_->Predict(dt, rotatedLinAccNoGrav.y());
            kalmanZ_->Predict(dt, rotatedLinAccNoGrav.z());

            accHistory_.emplace_back(linAccNoGrav, msg->timestamp);

            if (float(msg->timestamp - accHistory_.front().second) * 1e-9 > 1.0) {
                accHistory_.pop_front();

            }
        }

        if (orientationInitialized_) {
            double r, p, y;
            Algorithms::QuaternionToRPY(rtl::Quaternion{msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z}, r, p, y);
            rtl::Quaternion<double> modifiedImuOrientation = Algorithms::RPYToQuaternion(r, p, GetHeading());
            orientation_ = orientation_.slerp(modifiedImuOrientation, 0.001);
        }

        lastImuTimestamp_ = msg->timestamp;
    }

    void SelfModel::OnImuDquat(atlas_fusion_interfaces::msg::ImuDquatData::UniquePtr msg) {
        LOG_TRACE("SelfModel: IMU Dquat data arrived: ({}, {})", this->get_clock()->now().nanoseconds(), HEX_ADDR(msg.get()));

        auto quat = rtl::Quaternion<double>{msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z};

        double rollDiff_;
        Algorithms::QuaternionToRPY(quat, rollDiff_, attitudeDiff_, headingDiff_);
        orientation_ = orientation_ * quat;
        lastDquatTimestamp_ = msg->timestamp;
    }


    void SelfModel::PublishStaticTransforms() {
        std::vector<geometry_msgs::msg::TransformStamped> msgVector;
        for (const auto& frameType: EntryPoint::GetContext().GetFrameTypes()) {
            auto tf = EntryPoint::GetContext().GetTransformationForFrame(frameType);
            geometry_msgs::msg::TransformStamped msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = FrameTypeName(EntryPoint::GetContext().GetRootFrameType());
            msg.child_frame_id = FrameTypeName(frameType);
            msg.transform.translation.x = tf.trVecX();
            msg.transform.translation.y = tf.trVecY();
            msg.transform.translation.z = tf.trVecZ();
            msg.transform.rotation.x = tf.rotQuaternion().x();
            msg.transform.rotation.y = tf.rotQuaternion().y();
            msg.transform.rotation.z = tf.rotQuaternion().z();
            msg.transform.rotation.w = tf.rotQuaternion().w();
            msgVector.push_back(msg);
        }
        staticTransformBroadcaster_->sendTransform(msgVector);
    }

    void SelfModel::UpdateOriginToRootTf() {
        auto tf = rtl::RigidTf3D<double>{GetPosition().GetOrientation(), GetPosition().GetPosition()};
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = FrameTypeName(FrameType::kOrigin);
        tf_msg.child_frame_id = FrameTypeName(EntryPoint::GetContext().GetRootFrameType());
        tf_msg.transform.translation.x = tf.trVecX();
        tf_msg.transform.translation.y = tf.trVecY();
        tf_msg.transform.translation.z = tf.trVecZ();
        tf_msg.transform.rotation.x = tf.rotQuaternion().x();
        tf_msg.transform.rotation.y = tf.rotQuaternion().y();
        tf_msg.transform.rotation.z = tf.rotQuaternion().z();
        tf_msg.transform.rotation.w = tf.rotQuaternion().w();
        rootToOriginTransformBroadcaster_->sendTransform(tf_msg);
    }

    void SelfModel::OnPublishPoseAndTrajectory() {
        UpdateOriginToRootTf();

        visualization_msgs::msg::Marker msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = FrameTypeName(FrameType::kOrigin);

        msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        msg.color.r = 0.0f;
        msg.color.g = 1.0f;
        msg.color.b = 0.0f;
        msg.color.a = 1.0f;

        msg.scale.x = 0.02;

        for (const DataModels::LocalPosition& pos: positionHistory_) {
            auto p = pos.GetPosition();

            geometry_msgs::msg::Point point;
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();

            msg.points.push_back(point);
        }
        filteredTrajectoryPublisher_->publish(msg);
        selfGlobalPublisher_->publish(GetSelfGlobalCube());
    }

    visualization_msgs::msg::Marker SelfModel::GetSelfGlobalCube() {
        visualization_msgs::msg::Marker cube;
        cube.header.frame_id = FrameTypeName(FrameType::kImu);
        cube.header.stamp = this->get_clock()->now();
        cube.id = 0;
        cube.type = visualization_msgs::msg::Marker::CUBE;
        cube.action = visualization_msgs::msg::Marker::ADD;
        cube.pose.position.x = 0;
        cube.pose.position.y = 0;
        cube.pose.position.z = -0.75;
        cube.pose.orientation.x = 0.0;
        cube.pose.orientation.y = 0.0;
        cube.pose.orientation.z = 0.0;
        cube.pose.orientation.w = 1.0;
        cube.scale.x = 4.5;
        cube.scale.y = 2.5;
        cube.scale.z = 1.5;

        cube.color.a = 0.5;
        cube.color.r = 0.0;
        cube.color.g = 1.0;
        cube.color.b = 0.0;
        return cube;
    }

    void SelfModel::EstimatePositionInTime(const std::shared_ptr<atlas_fusion_interfaces::srv::EstimatePositionInTime::Request> request,
                                           std::shared_ptr<atlas_fusion_interfaces::srv::EstimatePositionInTime::Response> response) {
        for (long i = long(positionHistory_.size()) - 1; i >= 0; i--) {
            if (positionHistory_.at(i).GetTimestamp() < request->timestamp) {
                if (i == 0) {
                    auto pos = positionHistory_.at(i).GetPosition();
                    auto ori = positionHistory_.at(i).GetOrientation();

                    geometry_msgs::msg::Vector3 posBefore;
                    posBefore.x = pos.x();
                    posBefore.y = pos.y();
                    posBefore.z = pos.z();

                    geometry_msgs::msg::Quaternion oriBefore;
                    oriBefore.x = ori.x();
                    oriBefore.y = ori.y();
                    oriBefore.z = ori.z();
                    oriBefore.w = ori.w();

                    pos = positionHistory_.back().GetPosition();
                    ori = positionHistory_.back().GetOrientation();

                    geometry_msgs::msg::Vector3 posNow;
                    posNow.x = pos.x();
                    posNow.y = pos.y();
                    posNow.z = pos.z();

                    geometry_msgs::msg::Quaternion oriNow;
                    oriNow.x = ori.x();
                    oriNow.y = ori.y();
                    oriNow.z = ori.z();
                    oriNow.w = ori.w();

                    response->position_before = posBefore;
                    response->orientation_before = oriBefore;
                    response->timestamp_before = positionHistory_.at(i).GetTimestamp();
                    response->position_now = posNow;
                    response->orientation_now = oriNow;
                    response->timestamp_now = positionHistory_.back().GetTimestamp();
                    return;
                } else {
                    auto poseBefore = &positionHistory_.at(std::min(static_cast<size_t>(i), positionHistory_.size() - 2));
                    auto poseAfter = &positionHistory_.at(std::min(static_cast<size_t>(i + 1), positionHistory_.size() - 1));

                    auto numerator = static_cast<float>(poseAfter->GetTimestamp() - request->timestamp);
                    auto denominator = static_cast<float>(poseAfter->GetTimestamp() - poseBefore->GetTimestamp());
                    float ratio = numerator / denominator;

                    geometry_msgs::msg::Vector3 interpolatedPose;
                    interpolatedPose.x = poseBefore->GetPosition().x() + (poseAfter->GetPosition().x() - poseBefore->GetPosition().x()) * ratio;
                    interpolatedPose.y = poseBefore->GetPosition().y() + (poseAfter->GetPosition().y() - poseBefore->GetPosition().y()) * ratio;
                    interpolatedPose.z = poseBefore->GetPosition().z() + (poseAfter->GetPosition().z() - poseBefore->GetPosition().z()) * ratio;

                    auto newOri = poseBefore->GetOrientation().slerp(poseAfter->GetOrientation(), ratio);
                    geometry_msgs::msg::Quaternion newOrientation;
                    newOrientation.x = newOri.x();
                    newOrientation.y = newOri.y();
                    newOrientation.z = newOri.z();
                    newOrientation.w = newOri.w();

                    uint64_t duration = poseAfter->GetTimestamp() - poseBefore->GetTimestamp();
                    uint64_t ts = poseBefore->GetTimestamp() + static_cast<uint64_t>(duration * ratio);

                    auto pos = positionHistory_.back().GetPosition();
                    auto ori = positionHistory_.back().GetOrientation();

                    geometry_msgs::msg::Vector3 posNow;
                    posNow.x = pos.x();
                    posNow.y = pos.y();
                    posNow.z = pos.z();

                    geometry_msgs::msg::Quaternion oriNow;
                    oriNow.x = ori.x();
                    oriNow.y = ori.y();
                    oriNow.z = ori.z();
                    oriNow.w = ori.w();

                    response->position_before = interpolatedPose;
                    response->orientation_before = newOrientation;
                    response->timestamp_before = ts;
                    response->position_now = posNow;
                    response->orientation_now = oriNow;
                    response->timestamp_now = positionHistory_.back().GetTimestamp();
                    return;
                }
            }
        }

        LOG_WARN("Unable to estimate position in time! Missing time point in history.");
        response->position_before = geometry_msgs::msg::Vector3();
        response->orientation_before = geometry_msgs::msg::Quaternion();
        response->timestamp_before = request->timestamp;
        response->position_now = geometry_msgs::msg::Vector3();
        response->orientation_now = geometry_msgs::msg::Quaternion();
        response->timestamp_now = request->timestamp;
    }

    DataModels::LocalPosition SelfModel::GetPosition() const {
        if (positionHistory_.empty()) {
            return DataModels::LocalPosition{{}, {}, 0};
        }
        return positionHistory_.back();
    }

    double SelfModel::GetSpeedScalar() const {
        return std::sqrt(std::pow(kalmanX_->GetVelocity(), 2) + std::pow(kalmanY_->GetVelocity(), 2) + std::pow(kalmanZ_->GetVelocity(), 2));
    }

    rtl::Vector3D<double> SelfModel::GetSpeedVector() const {
        auto speed = rtl::Vector3D<double>{kalmanX_->GetVelocity(), kalmanY_->GetVelocity(), kalmanZ_->GetVelocity()};
        auto tf = rtl::RigidTf3D<double>{orientation_, {0.0, 0.0, 0.0}};
        return tf.inverted()(speed);
    }

    double SelfModel::GetAvgAccScalar() const {
        auto acc = GetAvgAcceleration();
        return std::sqrt(std::pow(acc.x(), 2) + std::pow(acc.y(), 2) + std::pow(acc.z(), 2));
    }

    rtl::Vector3D<double> SelfModel::GetAvgAcceleration() const {
        rtl::Vector3D<double> sum;
        for (const auto& acc: accHistory_) {
            sum += acc.first;
        }
        return sum / accHistory_.size();
    }

    double SelfModel::GetHeading() const {
        double r, p, y;
        Algorithms::QuaternionToRPY(orientation_, r, p, y);
        return y;
    }


    std::pair<double, float> SelfModel::ValidHeading(const atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr& data) {
        if (data->azimuth != 0) {
            validHeadingCounter_++;
        } else {
            validHeadingCounter_ = 0;
        }
        float validityFactor = std::min(static_cast<float>( 1 / (1 + std::exp(-0.1 * (validHeadingCounter_ - 50.0)))), 0.9f);
        return {AzimuthToHeading(data->azimuth), validityFactor};
    }


    std::pair<double, float> SelfModel::SpeedHeading() {
        double heading = atan2(kalmanY_->GetVelocity(), kalmanX_->GetVelocity());
        double speed = GetSpeedScalar();
        auto validityFactor = static_cast<float>( 1 / (1 + std::exp(-1.0 * (speed - 5.0))));
        return {heading, validityFactor};
    }


    std::pair<double, float> SelfModel::FuseHeadings(const atlas_fusion_interfaces::msg::GnssPositionData::UniquePtr& data) {

        auto gnssHeading = ValidHeading(data);
        auto spdHeading = SpeedHeading();

        auto gnssQuaternion = Algorithms::RPYToQuaternion(0, 0, gnssHeading.first);
        auto speedQuaternion = Algorithms::RPYToQuaternion(0, 0, spdHeading.first);

        auto slerpFactor = EstimateSlerpFactor(gnssHeading.second, spdHeading.second);
        auto fused = gnssQuaternion.slerp(speedQuaternion, 1 - slerpFactor);

        double r, p, y;
        Algorithms::QuaternionToRPY(fused, r, p, y);
        return {y, std::max(gnssHeading.second, spdHeading.second)};
    }


    float SelfModel::EstimateSlerpFactor(float a, float b) {
        a += 0.001;
        b += 0.001;
        return a / (a + b);
    }

    void SelfModel::UpdateOrientation(double heading) {
        double r, p, y;
        Algorithms::QuaternionToRPY(orientation_, r, p, y);
        orientation_ = Algorithms::RPYToQuaternion(r, p, heading);
    }

    DataModels::GlobalPosition SelfModel::GnssPoseToRootFrame(const DataModels::GlobalPosition& gnssPose) const {
        auto frameOffset = EntryPoint::GetContext().TransformPointFromFrameToFrame({}, FrameType::kGnssAntennaRear, FrameType::kImu);

        auto gnssOffset = DataModels::LocalPosition{frameOffset, rtl::Quaternion<double>::identity(), GetCurrentTime()};
        return DataModels::GlobalPosition::localPoseToGlobalPose(gnssOffset, gnssPose);
    }

    rtl::Vector3D<double> SelfModel::RemoveGravitationalForceFromLinAcc(const atlas_fusion_interfaces::msg::ImuImuData::UniquePtr& data) const {
        auto orientation = rtl::Quaternion<double>{data->orientation.w, data->orientation.x, data->orientation.y, data->orientation.z};
        auto acc = rtl::Vector3d{data->lin_acc.x, data->lin_acc.y, data->lin_acc.z};
        auto tf = rtl::RigidTf3D<double>{orientation, {0, 0, 0}};
        auto grav = tf.inverted()(rtl::Vector3D<double>{0.0, 0.0, 9.81});
        rtl::Vector3D<double> diff = acc - grav;
        return diff;
    }

    uint64_t SelfModel::GetCurrentTime() const {
        return std::max(std::max(lastGnssTimestamp_, lastImuTimestamp_), lastDquatTimestamp_);
    }
}