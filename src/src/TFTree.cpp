//
// Created by standa on 6.3.23.
//
#include <TFTree.h>

namespace AtlasFusion {

    void TFTree::AddFrame(const rtl::RigidTf3D<double>& tf, const FrameType& frameType) {

        if (frameMap_.find(frameType) != frameMap_.end()) {
            LOG_WARN("Unable to insert {} frame to TFTree. Frame already exists.", FrameTypeName(frameType));
            return;
        }

        frameMap_[frameType] = tf;
        frameTypes_.emplace_back(frameType);
    }


    rtl::RigidTf3D<double> TFTree::GetTransformationForFrame(const FrameType& frameType) const {
        if(frameType == rootFrameType_) {
            return rtl::RigidTf3D<double>{rtl::Quaternion<double>::identity(), {0.0, 0.0, 0.0}};
        }
        return frameMap_.at(frameType);
    }


    rtl::Vector3D<double> TFTree::TransformPointFromFrameToFrame(const rtl::Vector3D<double>& srcPoint, const FrameType& source, const FrameType& destination) const {
        auto tfs = GetTransformationForFrame(source);
        auto tfd = GetTransformationForFrame(destination);

        auto interResult = tfs(srcPoint);
        return tfd.inverted()(interResult);
    }

}
