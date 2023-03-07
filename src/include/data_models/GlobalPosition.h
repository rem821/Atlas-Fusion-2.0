//
// Created by standa on 7.3.23.
//
#pragma once

#include <data_models/LocalPosition.h>

namespace AtlasFusion::DataModels {

    /**
     * Global Position represents WGS84 global position
     */
    class GlobalPosition {

    public:

        /**
         * Constructor
         * @param lat WGS84 latitude
         * @param lon WGS84 longitude
         * @param alt WGS84 altitude
         * @param azim azimuth
         */
        GlobalPosition(double lat, double lon, double alt, double azim)
        : position_{lat, lon, alt}
        , azim_(azim) {

        }

        /**
         * Latitude getter
         * @return latitude
         */
        double GetLatitude() { return position_.x(); };

        /**
         * Logitude getter
         * @return longitude
         */
        double GetLongitude() { return position_.y(); };

        /**
         * Altitude getter
         * @return altitude
         */
        double GetAltitude() { return position_.z(); };

        /**
         * Azimuth getter
         * @return azimuth
         */
        double GetAzimuth() { return azim_; };

        /**
         * Converts degrees to radians
         * @param deg angle in degrees
         * @return angle in radians
         */
        static inline double DegToRad(double deg) {
            return deg * 3.1415 / 180;
        }

        /**
         * Method calculates metric distance between two WGS84 coordinates
         * @param coord1 first WGS84 coordinate
         * @param coord2 second WGS84 coordinate
         * @return metric distance between coords
         */
        static double GetDistanceBetweenCoords(GlobalPosition& coord1, GlobalPosition& coord2){

            const double R = 6371000;

            double lat_1 = DegToRad(coord1.GetLatitude());
            double lat_2 = DegToRad(coord2.GetLatitude());

            double lon_1 = DegToRad(coord1.GetLongitude());
            double lon_2 = DegToRad(coord2.GetLongitude());

            double delta_lat = lat_2 - lat_1;
            double delta_lon = lon_2 - lon_1;

            double a = std::sin(delta_lat/2) * std::sin(delta_lat/2) + std::cos(lat_1) * std::cos(lat_2) * std::sin(delta_lon/2) * std::sin(delta_lon/2);
            double c = 2* std::atan2(std::sqrt(a), std::sqrt(1-a));
            double d = R * c;
            return d;
        }


        /**
         * Returns the 2D vector of matrix distances between two coordinates. Positive direction to north and east.
         * @param coord1 Reference coord
         * @param coord2 Vector coord
         * @return (x, y, 0) distance between two WGS84 coordinates
         */
        static rtl::Vector3D<double> GetOffsetBetweenCoords(GlobalPosition& coord1, GlobalPosition& coord2){

            rtl::Vector3D<double> offset{0,0,0};

            double mid_lat = (coord1.GetLatitude() + coord2.GetLatitude())/2;
            double mid_lon = (coord1.GetLongitude() + coord2.GetLongitude())/2;

            auto c1 = GlobalPosition(coord1.GetLatitude(), mid_lon, 0, 0);
            auto c2 = GlobalPosition(coord2.GetLatitude(), mid_lon, 0, 0);
            offset.setX(GetDistanceBetweenCoords(c1, c2));

            c1 = GlobalPosition(mid_lat, coord1.GetLongitude(), 0, 0);
            c2 = GlobalPosition(mid_lat, coord2.GetLongitude(), 0, 0);
            offset.setY(GetDistanceBetweenCoords(c1, c2));

            if (coord1.GetLatitude() > coord2.GetLatitude()) {
                offset.setX(-offset.x());
            }
            if (coord1.GetLongitude() < coord2.GetLongitude()) {
                offset.setY(-offset.y());
            }
            offset.setZ(coord2.GetAltitude() - coord1.GetAltitude());

            return offset;

        }

        /**
         * Converts relative local metric position referenced to WGS84 global anchor into the nwe WGS84 position.
         * @param localOffset local position that will be converted
         * @param globalOrigin WGS position of the local origin
         * @return WGS84 position of the input local offset
         */
        static GlobalPosition localPoseToGlobalPose(const LocalPosition& localOffset, GlobalPosition globalOrigin) {

            const double step = 0.001f;
            auto g1 = GlobalPosition(globalOrigin.GetLatitude() + step, globalOrigin.GetLongitude() + step, 0, 0);
            auto mili_deg_distance = GetOffsetBetweenCoords(globalOrigin, g1);

            double lat_step = (localOffset.GetPosition().y() * step) / mili_deg_distance.y();
            double lon_step = (localOffset.GetPosition().x() * step) / mili_deg_distance.x();

            GlobalPosition output{globalOrigin.GetLatitude() + lat_step, globalOrigin.GetLongitude() + lon_step, globalOrigin.GetAltitude() + localOffset.GetPosition().z(), 0};
            return output;
        }


    private:

        rtl::Vector3D<double> position_;
        double azim_;

    };

}