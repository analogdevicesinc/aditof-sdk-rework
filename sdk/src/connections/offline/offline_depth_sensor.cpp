#include "offline_depth_sensor.h"
#include <fstream>
#include <iostream>
#include <cstring>

OfflineDepthSensor::OfflineDepthSensor(std::string path) : m_path(path) {}

OfflineDepthSensor::~OfflineDepthSensor() = default;

aditof::Status OfflineDepthSensor::getFrame(uint16_t *buffer) {
    std::ifstream ifs(m_path.c_str());
    std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    for (int i = 0; i < content.size(); i += 2) {
        buffer[i / 2] = *(uint16_t*)&content[i];
    }
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::open() {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::start() {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::stop() {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::getAvailableFrameTypes(
        std::vector<aditof::DepthSensorFrameType> &types) {
    aditof::DepthSensorFrameType frameType;
    frameType.type = "depth_ir";
    frameType.width = 1024;
    frameType.height = 1024;
    types.clear();
    types.emplace_back(frameType);
    return aditof::Status::OK;
}

aditof::Status
OfflineDepthSensor::setFrameType(const aditof::DepthSensorFrameType &type) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::program(const uint8_t *firmware,
                       size_t size) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::readAfeRegisters(const uint16_t *address,
                                        uint16_t *data,
                                size_t length) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::writeAfeRegisters(const uint16_t *address,
                                         const uint16_t *data,
                                                     size_t length) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineDepthSensor::getDetails(aditof::SensorDetails &details) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::getHandle(void **handle) {
    return aditof::Status::UNAVAILABLE;
}
