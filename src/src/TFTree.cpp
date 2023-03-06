//
// Created by standa on 6.3.23.
//
#include <TFTree.h>

namespace AtlasFusion {

    void TFTree::addFrame(const rtl::RigidTf3D<double>& tf, const FrameType& type) {

        if (frameMap_.find(type) != frameMap_.end()) {
            LOG_WARN("Unable to insert {} frame to TFTree. Frame already exists.", frameTypeName(type));
            return;
        }

        frameMap_[type] = tf;
        frameTypes_.emplace_back(type);
    }


    rtl::RigidTf3D<double> TFTree::getTransformationForFrame(const FrameType& frameType) {
        if(frameType == rootFrameType_) {
            return rtl::RigidTf3D<double>{rtl::Quaternion<double>::identity(), {0.0, 0.0, 0.0}};
        }
        return frameMap_.at(frameType);
    }


    rtl::Vector3D<double> TFTree::transformPointFromFrameToFrame(const rtl::Vector3D<double>& srcPoint, const FrameType& source, const FrameType& destination) {
        auto tfs = getTransformationForFrame(source);
        auto tfd = getTransformationForFrame(destination);

        auto interResult = tfs(srcPoint);
        return tfd.inverted()(interResult);

    }

}
