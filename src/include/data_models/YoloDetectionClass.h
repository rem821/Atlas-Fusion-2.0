//
// Created by standa on 7.3.23.
//
#pragma once

namespace AtlasFusion::DataModels {

    /**
     * Full list of COCO dataset labels used for the video-yolo inference.
     */
    enum class YoloDetectionClass {
        kUnknown = -1,
        kPerson = 0,
        kBicycle = 1,
        kCar = 2,
        kMotorbike = 3,
        kAeroplane = 4,
        kBus = 5,
        kTrain = 6,
        kTruck = 7,
        kBoat = 8,
        kTrafficLight = 9,
        kFireHydrant = 10,
        kStopSign = 11,
        kParkingMeter = 12,
        kBench = 13,
        kBird = 14,
        kCat = 15,
        kDog = 16,
        kHorse = 17,
        kSheep = 18,
        kCow = 19,
        kElephant = 20,
        kBear = 21,
        kZebra = 22,
        kGiraffe = 23,
        kBackpack = 24,
        kUmbrella = 25,
        kHandbag = 26,
        kTie = 27,
        kSuitcase = 28,
        kFrisbee = 29,
        kSkis = 30,
        kSnowboard = 31,
        kSportsBall = 32,
        kKite = 33,
        kBaseballBat = 34,
        kBaseballGlove = 35,
        kSkateboard = 36,
        kSurfboard = 37,
        kTennisRacket = 38,
        kBottle = 39,
        kWineGlass = 40,
        kCup = 41,
        kFork = 42,
        kKnife = 43,
        kSpoon = 44,
        kBowl = 45,
        kBanana = 46,
        kApple = 47,
        kSandwich = 48,
        kOrange = 49,
        kBroccoli = 50,
        kCarrot = 51,
        kHotDog = 52,
        kPizza = 53,
        kDonut = 54,
        kCake = 55,
        kChair = 56,
        kSofa = 57,
        kPottedplant = 58,
        kBed = 59,
        kDiningtable = 60,
        kToilet = 61,
        kTvmonitor = 62,
        kLaptop = 63,
        kMouse = 64,
        kRemote = 65,
        kKeyboard = 66,
        kCellPhone = 67,
        kMicrowave = 68,
        kOven = 69,
        kToaster = 70,
        kSink = 71,
        kRefrigerator = 72,
        kBook = 73,
        kClock = 74,
        kVase = 75,
        kScissors = 76,
        kTeddyBear = 77,
        kHairDrier = 78,
        kToothbrush = 79,
    };

    /**
     * Simplified YOLO detection classes. Reduces the number of classes which are used in this mapping system.
     */
    enum class ReducedYoloDetectionClasses {
        kPedestrian = 0,
        kBike = 1,
        kVehicle = 2,
        kAnimal = 3,
        kTraffic = 4,
        kOther = 5,
    };

    ReducedYoloDetectionClasses getReducedDetectionClass(const YoloDetectionClass& cls);
}