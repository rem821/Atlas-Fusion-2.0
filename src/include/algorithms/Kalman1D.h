//
// Created by standa on 7.3.23.
//
#pragma once

#include <opencv2/video/tracking.hpp>
#include <iostream>

namespace AtlasFusion::Algorithms {

    /**
     * Wrapper around the OpenCV Kalman filter implementations. Models position and velocity in 1D space
     */

    class Kalman1D {

    public:

        /**
         * Constructor
         * @param processNoise the noise of the modeled process
         * @param observationNoise the noise of the measured data
         */
        Kalman1D(double processNoise, double observationNoise)
        : processNoise_(processNoise)
        , observationNoise_(observationNoise) {
            states_ = (cv::Mat_<double>(2, 1) <<
                    0,
                    0);
            covariance_ = (cv::Mat_<double>(2, 2) <<
                    1, 0,
                    0, 1);
        }

        /**
         * Prediction phase of the Kalman Filter
         * @param dt time period from last update
         * @param u action input (acceleration in given axis)
         */
        void Predict(double dt, double u);

        /**
         * Corection phase of the Kalman Filter
         * @param measurement measurements of the inner states
         */
        void Correct(cv::Mat measurement);

        /**
         * Getter for a position inner state
         * @return modeled position
         */
        double GetPosition() const;

        /**
         * Getter for a velocity inner state
         * @return modeles velocity
         */
        double GetVelocity() const;

        /**
         * Setter for a modeled position
         * @param position position to be forced into the model
         */
        void SetPosition(double position);

        /**
         * Setter for a modeled velocity
         * @param velocity velocity to be forced into the model
         */
        void SetVelocity(double velocity);

    private:

        cv::Mat states_;
        cv::Mat covariance_;

        double processNoise_;
        double observationNoise_;

        static void PrintMatrix(cv::Mat mat, std::string name);
        static cv::Mat GetTransitionMatrix(double dt);  // Matrix A
        static cv::Mat GetControlMatrix(double dt) ;    // Matrix B
        static cv::Mat GetMeasurementMatrix() ;         // Matrix H
        static cv::Mat GetCovarianceObservationNoiseMatrix(double cov);     // Matrix R
        static cv::Mat GetCovarianceProcessNoiseMatrix(double sigma, double dt);    // Matrix Q
    };
}
