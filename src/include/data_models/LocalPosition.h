//
// Created by standa on 7.3.23.
//
#pragma once

#include <rtl/Transformation.h>
#include <rtl/Core.h>

namespace AtlasFusion::DataModels {

    /**
     * Local Position represents metric position and orientation in the 3D space
     */
    class LocalPosition {

    public:

        /**
         * Constructor
         * @param position 3D metric position
         * @param orientation Quaternion orientation
         * @param timestamp timestamp, the position has been estimated
         */
        explicit LocalPosition(const rtl::Vector3D<double>& position, const rtl::Quaternion<double>& orientation, uint64_t timestamp)
                : position_(position), orientation_(orientation), timestamp_{timestamp} {

        }

        /**
         * 3D vector position getter
         * @return 3D metric position
         */
        [[nodiscard]] rtl::Vector3D<double> GetPosition() const { return position_; };

        /**
         * 3D space orientation represented by quaternion getter
         * @return quaternion represented orientation
         */
        [[nodiscard]] rtl::Quaternion<double> GetOrientation() const { return orientation_; };

        /**
         * set X component of 3D position
         * @param x axis position
         */
        void SetX(double x) { position_.setX(x); };

        /**
         * set Y component of 3D position
         * @param y axis position
         */
        void SetY(double y) { position_.setY(y); };

        /**
         * set Z component of 3D position
         * @param z axis position
         */
        void SetZ(double z) { position_.setZ(z); };

        /**
         * Setter for full 3D position
         * @param pose 3D metric position
         */
        void SetPosition(const rtl::Vector3D<double>& pose) { position_ = pose; };

        /**
         * Setter for orientation
         * @param orientation quaternion represented 3D orientation
         */
        void SetOrientation(const rtl::Quaternion<double>& orientation) { orientation_ = orientation; };

        /**
         * Converts 3D orientation nad translation into the 3D transformation
         * @return
         */
        [[nodiscard]] rtl::RigidTf3D<double> ToTf() const;

        /**
         * Nanosecond timestamp getter
         * @return timesamp of the position estimation time
         */
        [[nodiscard]] uint64_t GetTimestamp() const { return timestamp_; };

        /**
         * Operator combines two local poses into one. Translation are added and quatermions are multiplied
         * @param other second position
         * @return Returns newly created local position combined by this and the given one
         */
        LocalPosition operator+(LocalPosition &other);

        /**
         * Inverse operator to operator+. A + B - B = A
         * @param other B component
         * @return Result of substracting two local poses
         */
        LocalPosition operator-(LocalPosition &other);

        /**
         * Method generates short string form of the Local Position
         * @return string expression of the local pose
         */
        std::string ToString();

    private:

        rtl::Vector3D<double> position_;
        rtl::Quaternion<double> orientation_;
        uint64_t timestamp_;
    };

}

