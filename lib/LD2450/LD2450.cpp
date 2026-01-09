/*
 * LD2450.cpp - Implementation for Hi-Link LD2450 radar module
 * Lightweight parser tailored for LightTrack VISION sketch
 */

#include "LD2450.h"
#include <math.h>

LD2450::LD2450() : _serial(nullptr), _targetCount(SENSOR_MAX_TARGETS), _bufferIndex(0) {
    for (uint8_t i = 0; i < SENSOR_MAX_TARGETS; i++) {
        _targets[i] = {i, 0, 0, 0, 0, 0, false};
    }
}

void LD2450::begin(Stream &stream) {
    _serial = &stream;
    _serial->setTimeout(10);
}

void LD2450::begin(HardwareSerial &serial, int8_t rxPin, int8_t txPin) {
    if (rxPin >= 0 && txPin >= 0) {
        serial.begin(SERIAL_SPEED, SERIAL_8N1, rxPin, txPin);
    } else {
        serial.begin(SERIAL_SPEED);
    }
    begin(static_cast<Stream&>(serial));
}

int LD2450::read() {
    if (!_serial) return 0;

    int validTargets = 0;

    while (_serial->available()) {
        uint8_t byte = _serial->read();
        _buffer[_bufferIndex++] = byte;

        if (_bufferIndex >= SERIAL_BUFFER_SIZE) {
            _bufferIndex = 0;
        }

        // Check for frame end: 0x55 0xCC
        if (_bufferIndex >= 2 &&
            _buffer[_bufferIndex - 2] == 0x55 &&
            _buffer[_bufferIndex - 1] == 0xCC) {
            parseFrame();
            _bufferIndex = 0;
        }
    }

    for (uint8_t i = 0; i < _targetCount; i++) {
        if (_targets[i].valid) validTargets++;
    }

    return validTargets;
}

void LD2450::parseFrame() {
    // Frame format: AA FF 03 00 [target1: 8 bytes] [target2: 8 bytes] [target3: 8 bytes] 55 CC
    // Total frame: 4 header + 24 target data + 2 footer = 30 bytes

    // Find header
    int headerPos = -1;
    for (uint16_t i = 0; i + 3 < _bufferIndex; i++) {
        if (_buffer[i] == 0xAA && _buffer[i+1] == 0xFF &&
            _buffer[i+2] == 0x03 && _buffer[i+3] == 0x00) {
            headerPos = i;
            break;
        }
    }

    if (headerPos < 0) return;

    // Parse up to 3 targets (8 bytes each)
    for (uint8_t t = 0; t < SENSOR_MAX_TARGETS && t < _targetCount; t++) {
        int offset = headerPos + 4 + (t * 8);

        if (offset + 8 > _bufferIndex) break;

        // X coordinate (little-endian, bit 15 is sign)
        int16_t x = (int16_t)(_buffer[offset] | (_buffer[offset + 1] << 8));
        if (x & 0x8000) x = -(x & 0x7FFF);

        // Y coordinate (little-endian, bit 15 is sign)
        int16_t y = (int16_t)(_buffer[offset + 2] | (_buffer[offset + 3] << 8));
        if (y & 0x8000) y = -(y & 0x7FFF);

        // Speed (little-endian, bit 15 is sign)
        int16_t speed = (int16_t)(_buffer[offset + 4] | (_buffer[offset + 5] << 8));
        if (speed & 0x8000) speed = -(speed & 0x7FFF);

        // Resolution
        uint16_t resolution = _buffer[offset + 6] | (_buffer[offset + 7] << 8);

        // Calculate distance using Euclidean distance
        uint16_t distance = (uint16_t)sqrt((float)(x * x) + (float)(y * y));

        // Target is valid if resolution > 0
        _targets[t].id = t;
        _targets[t].x = x;
        _targets[t].y = y;
        _targets[t].speed = speed;
        _targets[t].resolution = resolution;
        _targets[t].distance = distance;
        _targets[t].valid = (resolution > 0);
    }
}

String LD2450::waitForSensorMessage(int retryCount) {
    int count = 0;
    while (count < retryCount || retryCount < 0) {
        if (read() > 0) {
            return ProcessSerialDataIntoRadarData();
        }
        delay(1);
        count++;
    }
    return "";
}

String LD2450::ProcessSerialDataIntoRadarData() {
    String result = "";
    for (uint8_t i = 0; i < _targetCount; i++) {
        if (_targets[i].valid) {
            if (result.length() > 0) result += " ";
            result += "T" + String(i) + ":";
            result += "x=" + String(_targets[i].x) + ",";
            result += "y=" + String(_targets[i].y) + ",";
            result += "d=" + String(_targets[i].distance) + ",";
            result += "s=" + String(_targets[i].speed);
        }
    }
    return result;
}

LD2450::RadarTarget LD2450::getTarget(uint8_t index) {
    if (index < SENSOR_MAX_TARGETS) {
        return _targets[index];
    }
    return {0, 0, 0, 0, 0, 0, false};
}

void LD2450::setNumberOfTargets(uint8_t count) {
    _targetCount = min(count, (uint8_t)SENSOR_MAX_TARGETS);
}

uint8_t LD2450::getSensorSupportedTargetCount() {
    return _targetCount;
}
