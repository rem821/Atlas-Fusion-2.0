/*
 * Copyright 2020 Brno University of Technology
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

#include "data_models/YoloDetectionClass.h"

namespace AtlasFusion::DataModels {

    ReducedYoloDetectionClasses GetReducedDetectionClass(const YoloDetectionClass& cls) {

        auto detectionClass = ReducedYoloDetectionClasses::kOther;
        switch(cls) {
            case YoloDetectionClass::kPerson:
                detectionClass = ReducedYoloDetectionClasses::kPedestrian; break;
            case YoloDetectionClass::kBicycle:
            case YoloDetectionClass::kMotorbike:
                detectionClass = ReducedYoloDetectionClasses::kBike; break;
            case YoloDetectionClass::kCar:
            case YoloDetectionClass::kBus:
            case YoloDetectionClass::kTrain:
            case YoloDetectionClass::kTruck:
            case YoloDetectionClass::kBoat:
                detectionClass = ReducedYoloDetectionClasses::kVehicle; break;
            case YoloDetectionClass::kBird:
            case YoloDetectionClass::kCat:
            case YoloDetectionClass::kDog:
            case YoloDetectionClass::kHorse:
            case YoloDetectionClass::kCow:
            case YoloDetectionClass::kSheep:
            case YoloDetectionClass::kElephant:
            case YoloDetectionClass::kBear:
            case YoloDetectionClass::kZebra:
            case YoloDetectionClass::kGiraffe:
                detectionClass = ReducedYoloDetectionClasses::kAnimal; break;
            case YoloDetectionClass::kTrafficLight:
            case YoloDetectionClass::kStopSign:
                detectionClass = ReducedYoloDetectionClasses::kTraffic; break;
            default:
                break;
        }
        return detectionClass;
    }

    std_msgs::msg::ColorRGBA GetColorByClass(const YoloDetectionClass cls) {
        std_msgs::msg::ColorRGBA output;
        output.a = 1.0;

        switch (GetReducedDetectionClass(cls)) {
            case DataModels::ReducedYoloDetectionClasses::kPedestrian:  // red
                output.r = 1.0;
                output.g = 0.0;
                output.b = 0.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kBike:    // purple
                output.r = 1.0;
                output.g = 0.0;
                output.b = 1.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kVehicle: // blue
                output.r = 0.0;
                output.g = 0.0;
                output.b = 1.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kAnimal:  // green
                output.r = 0.0;
                output.g = 1.0;
                output.b = 0.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kTraffic:  // aqua
                output.r = 0.0;
                output.g = 1.0;
                output.b = 1.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kOther:   //yellow
                output.r = 1.0;
                output.g = 1.0;
                output.b = 0.0;
                break;
        }

        return output;
    }

}