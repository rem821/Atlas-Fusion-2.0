//
// Created by standa on 6.3.23.
//
#pragma once

#include <precompiled_headers/PCH.h>
#include <yaml-cpp/yaml.h>

#include <utility>

namespace AtlasFusion {

    /**
     *  Config Service is a class used to read out the startup configuration from the yaml file.
     */

    class ConfigService {

    private:


    public:

        explicit ConfigService(std::string configPath) : confPath_(std::move(configPath)) {

        };

        /**
         * Method extracts the uint16_t value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return uint16_t value that is corresponding to the given keys
         */
        uint16_t GetUInt16Value(const std::vector<std::string>& keys) { return GetNode(keys).as<uint16_t>(); }

        /**
         * Method extracts the uint32_t value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return uint32_t value that is corresponding to the given keys
         */
        uint32_t GetUInt32Value(const std::vector<std::string>& keys) { return GetNode(keys).as<uint32_t>(); }

        /**
         * Method extracts the int32_t value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return int32_t value that is corresponding to the given keys
         */
        int32_t GetInt32Value(const std::vector<std::string>& keys) { return GetNode(keys).as<int32_t>(); }

        /**
         * Method extracts the float value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return float value that is corresponding to the given keys
         */
        float GetFloatValue(const std::vector<std::string>& keys) { return GetNode(keys).as<float>(); }

        /**
         * Method extracts the double value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return double value that is corresponding to the given keys
         */
        double GetDoubleValue(const std::vector<std::string>& keys) { return GetNode(keys).as<double>(); }

        /**
         * Method extracts the string value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return string value that is corresponding to the given keys
         */
        std::string GetStringValue(const std::vector<std::string>& keys) { return GetNode(keys).as<std::string>(); }

        /**
         * Method extracts the bool value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return bool value that is corresponding to the given keys
         */
        bool GetBoolValue(const std::vector<std::string>& keys) {
            const auto value = GetStringValue(keys);
            if (value == "True" || value == "true" || value == "1") {
                return true;
            }
            return false;
        }

        /**
         * Method extracts the the array of values, represented as a std::vector. Type is defined via the template.
         * @param keys Vector of keys that are used to access the values.
         * @return vector of values that is corresponding to the given keys
         */
        template<typename T>
        std::vector<T> GetArrayValue(const std::vector<std::string>& keys) {
            auto node = GetNode(keys).as<std::vector<T> >();
            return node;
        }

        /**
         * Method extracts the 4-element quaternion that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the quaternion.
         * @return instance of the rtl::Quaternion that is corresponding to the given keys
         */
        template<typename T>
        rtl::Quaternion<T> GetQuaternionValue(const std::vector<std::string>& keys) {
            auto arr = GetArrayValue<T>(keys);
            return rtl::Quaternion<T>{arr[3], arr[0], arr[1], arr[2]};
        }

        /**
         * Method extracts the vector of 3 values that correspond to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return instance of rhe rtl::Vector3D that is corresponding to the given keys
         */
        template<typename T>
        rtl::Vector3D<T> GetVector3DValue(const std::vector<std::string>& keys) {
            auto arr = GetArrayValue<T>(keys);
            return rtl::Vector3D<T>{arr[0], arr[1], arr[2]};
        }

        /**
         * Method extracts the matrix that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return the matrix of data types defined by the template, represented as a std::vec of std:vecs
         */
        template<typename T>
        std::vector<std::vector<T>> GetMatValue(const std::vector<std::string>& keys) {
            auto arr = GetArrayValue<T>(keys);
            size_t size = size_t(std::sqrt(arr.size()));

            std::vector<std::vector<T>> output;
            if (size == std::sqrt(arr.size())) {
                for (size_t i = 0; i < size; i++) {
                    std::vector<T> vec;
                    for (size_t j = 0; j < size; j++) {
                        vec.push_back(arr[j + i * size]);
                    }
                    output.push_back(vec);
                }
            }
            return output;
        }


    protected:

        const std::string confPath_;

        YAML::Node GetNode(const std::vector<std::string>& keys) {
            YAML::Node config = YAML::LoadFile(confPath_);
            for (const auto& key: keys) {
                if (config[key]) {
                    config = config[key];
                } else {
                    LOG_ERROR("Unable to read {} from config file!", key);
                    throw std::runtime_error("Config service error!");
                }
            }
            return config;
        }
    };
}
