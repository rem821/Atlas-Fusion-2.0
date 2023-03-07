//
// Created by standa on 6.3.23.
//
#pragma once

#include <FrameTypes.h>
#include <data_loaders/DataLoaderIdentifiers.h>

namespace AtlasFusion {

    std::string FrameTypeName(const FrameType& frame);
    FrameType NameToFrameType(const std::string& name);
    DataLoader::CameraIdentifier CameraIdentifierFromFrameType(const FrameType& frame);
    FrameType FrameTypeFromIdentifier(const DataLoader::CameraIdentifier &identifier);
    DataLoader::LidarIdentifier LidarIdentifierFromFrameType(const FrameType& frame);
    FrameType FrameTypeFromIdentifier(const DataLoader::LidarIdentifier &identifier);
}