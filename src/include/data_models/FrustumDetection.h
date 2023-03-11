//
// Created by standa on 11.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <data_models/YoloDetectionClass.h>
#include <geometry_msgs/msg/point.hpp>

namespace AtlasFusion::DataModels {

    class FrustumDetection {

    public:

        explicit FrustumDetection(const rtl::Frustum3D<double>& frustum, float detConfidence, YoloDetectionClass cls)
                : frustum_{frustum}, detConfidence_{detConfidence}, cls_{cls} {}

        [[nodiscard]] rtl::Frustum3D<double> GetFrustum() const { return frustum_; };

        [[nodiscard]] float GetDetectionConfidence() const { return detConfidence_; };

        [[nodiscard]] YoloDetectionClass GetClass() const { return cls_; };

        static std::vector<geometry_msgs::msg::Point> GetGeometryVertices(const rtl::Frustum3D<double>& f) {
            geometry_msgs::msg::Point ntl;
            geometry_msgs::msg::Point ntr;
            geometry_msgs::msg::Point nbl;
            geometry_msgs::msg::Point nbr;

            geometry_msgs::msg::Point ftl;
            geometry_msgs::msg::Point ftr;
            geometry_msgs::msg::Point fbl;
            geometry_msgs::msg::Point fbr;

            ntl.x = f.getNearTopLeft().x();
            ntl.y = f.getNearTopLeft().y();
            ntl.z = f.getNearTopLeft().z();

            ntr.x = f.getNearTopRight().x();
            ntr.y = f.getNearTopRight().y();
            ntr.z = f.getNearTopRight().z();

            nbl.x = f.getNearBottomLeft().x();
            nbl.y = f.getNearBottomLeft().y();
            nbl.z = f.getNearBottomLeft().z();

            nbr.x = f.getNearBottomRight().x();
            nbr.y = f.getNearBottomRight().y();
            nbr.z = f.getNearBottomRight().z();

            ftl.x = f.getFarTopLeft().x();
            ftl.y = f.getFarTopLeft().y();
            ftl.z = f.getFarTopLeft().z();

            ftr.x = f.getFarTopRight().x();
            ftr.y = f.getFarTopRight().y();
            ftr.z = f.getFarTopRight().z();

            fbl.x = f.getFarBottomLeft().x();
            fbl.y = f.getFarBottomLeft().y();
            fbl.z = f.getFarBottomLeft().z();

            fbr.x = f.getFarBottomRight().x();
            fbr.y = f.getFarBottomRight().y();
            fbr.z = f.getFarBottomRight().z();

            return {nbl, nbr,
                    nbr, ntr,
                    ntr, ntl,
                    ntl, nbl,
                    fbl, fbr,
                    fbr, ftr,
                    ftr, ftl,
                    ftl, fbl,
                    ntl, ftl,
                    ntr, ftr,
                    nbl, fbl,
                    nbr, fbr};
        };

    private:

        const rtl::Frustum3D<double> frustum_;
        float detConfidence_;
        YoloDetectionClass cls_;
    };
}