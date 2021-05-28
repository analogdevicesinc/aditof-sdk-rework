/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "connections/usb/usb_utils.h"
#include "usb_buffer.pb.h"
#include "utils.h"

using namespace std;
using namespace aditof;

std::vector<std::pair<std::string, unsigned int>>
UsbUtils::getStorageNamesAndIds(const std::vector<std::string> &tokens) {
    std::vector<std::pair<std::string, unsigned int>> v;

    for (const auto &t : tokens) {
        vector<string> keyValueStr;
        Utils::splitIntoTokens(t, '=', keyValueStr);
        if (keyValueStr[0] == "STORAGE_NAME") {
            std::pair<std::string, unsigned int> pair;
            pair.first = keyValueStr[1];
            v.emplace_back(pair);
        } else if (keyValueStr[0] == "STORAGE_ID") {
            v.back().second = std::stoi(keyValueStr[1]);
        }
    }

    return v;
}

std::vector<std::pair<std::string, unsigned int>>
UsbUtils::getTemperatureSensorNamesAndIds(
    const std::vector<std::string> &tokens) {
    std::vector<std::pair<std::string, unsigned int>> v;

    for (const auto &t : tokens) {
        vector<string> keyValueStr;
        Utils::splitIntoTokens(t, '=', keyValueStr);
        if (keyValueStr[0] == "TEMP_SENSOR_NAME") {
            std::pair<std::string, unsigned int> pair;
            pair.first = keyValueStr[1];
            v.emplace_back(pair);
        } else if (keyValueStr[0] == "TEMP_SENSOR_ID") {
            v.back().second = std::stoi(keyValueStr[1]);
        }
    }

    return v;
}

aditof::Status UsbUtils::convertDepthSensorTypes(
    std::vector<aditof::DepthSensorFrameType> &depthSensorFrameTypes,
    const std::string &availableDepthSensorsFrameTypesBlob) {
    using namespace google::protobuf::io;

    usb_payload::DepthSensorFrameTypeVector depthSensorFrameTypesPayload;
    if (!depthSensorFrameTypesPayload.ParseFromString(
            availableDepthSensorsFrameTypesBlob)) {
        return aditof::Status::INVALID_ARGUMENT;
    }

    depthSensorFrameTypes.clear();

    for (const auto &depthSensorFrameTypePayload :
         depthSensorFrameTypesPayload.depthsensorframetypes()) {
        aditof::DepthSensorFrameType depthSensorFrameType;

        depthSensorFrameType.type = depthSensorFrameTypePayload.type();
        depthSensorFrameType.width = depthSensorFrameTypePayload.width();
        depthSensorFrameType.height = depthSensorFrameTypePayload.height();

        for (const auto &depthSensorFrameContentPayload :
             depthSensorFrameTypePayload.depthsensorframecontent()) {
            DepthSensorFrameContent depthSensorFrameContent;

            depthSensorFrameContent.width =
                depthSensorFrameContentPayload.width();
            depthSensorFrameContent.height =
                depthSensorFrameContentPayload.height();
            depthSensorFrameContent.type =
                depthSensorFrameContentPayload.type();
            depthSensorFrameType.content.push_back(depthSensorFrameContent);
        }

        depthSensorFrameTypes.push_back(depthSensorFrameType);
    }

    return aditof::Status::OK;
}

aditof::Status UsbUtils::convertDepthSensorFrameTypeToSerializedProtobuf(
    const aditof::DepthSensorFrameType &depthSensorFrameType,
    std::string &depthSensorFrameTypeBlob) {
    using namespace google::protobuf::io;

    usb_payload::DepthSensorFrameType depthSensorFrameTypePayload;
    depthSensorFrameTypePayload.set_type(depthSensorFrameType.type);
    depthSensorFrameTypePayload.set_width(depthSensorFrameType.width);
    depthSensorFrameTypePayload.set_height(depthSensorFrameType.height);

    for (const aditof::DepthSensorFrameContent &depthSensorFrameContent :
         depthSensorFrameType.content) {
        usb_payload::DepthSensorFrameContent *depthSensorFrameContentPayload =
            depthSensorFrameTypePayload.add_depthsensorframecontent();
        depthSensorFrameContentPayload->set_type(depthSensorFrameContent.type);
        depthSensorFrameContentPayload->set_width(
            depthSensorFrameContent.width);
        depthSensorFrameContentPayload->set_height(
            depthSensorFrameContent.height);
    }

    depthSensorFrameTypePayload.SerializeToString(&depthSensorFrameTypeBlob);

    return aditof::Status::OK;
}
