//
// Created by standa on 6.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>

namespace AtlasFusion {

    /**
     * Transformation Tree holds and allows simple searching in the transformation graph between the sensor frames, and
     * the world origin. Currently TF Tree supports only the root frame and 1 level of child-nodes and transformations
     * between them.
     */
    class TFTree {
    public:

        /**
         * Constructor
         * @param rootFrameName the central frame name
         */
        TFTree(FrameType rootFrameType) : rootFrameType_(rootFrameType) {}

        /**
         * Methods allows to add new child-frame under the root frame level
         * @param tf new child transformations
         * @param name new frame name
         */
        void addFrame(const rtl::RigidTf3D<double> &tf, const FrameType &frameType);

        /**
         * Method returns transformation between the root frame the the child frame
         * @param frameType child frame type
         * @return child transformation
         */
        rtl::RigidTf3D<double> getTransformationForFrame(const FrameType &frameType);

        /**
         * Method returns the vector of all child frame names.
         * @return all child frame names
         */
        std::vector<FrameType> getFrameTypes() const { return frameTypes_; };

        /**
         * Getter for root frame name
         * @return root frame name
         */
        const FrameType &getRootFrameType() const { return rootFrameType_; };

        /**
         * Method estimates transformation between two child frames.
         * @param srcPoint 3D point in the source coordinate systems (frame)
         * @param source source frame type
         * @param destination destination frame type
         * @return returns the point transformed from the original coordinate system to the new one.
         */
        rtl::Vector3D<double> transformPointFromFrameToFrame(const rtl::Vector3D<double> &, const FrameType &source, const FrameType &destination);

    protected:

        FrameType rootFrameType_;
        std::vector<FrameType> frameTypes_{};
        std::unordered_map<FrameType, rtl::RigidTf3D<double>> frameMap_{};

        const std::unordered_map<FrameType, rtl::RigidTf3D<double>> &getTree() { return frameMap_; };
    };
}