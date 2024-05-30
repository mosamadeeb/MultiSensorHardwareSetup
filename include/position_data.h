#ifndef MULTISENSORARDUINO_POSITION_DATA_H
#define MULTISENSORARDUINO_POSITION_DATA_H

#include "Arduino.h"

// Uncomment this line to send all available sensor data for MPU
#define USE_ALL_SENSORS

enum struct SensorType {
    None = 0,
    Acc = 1,
    Gyro = 2,
    Quat = 4,
    Euler = 8,
};

inline SensorType operator|(SensorType a, SensorType b)
{
    return static_cast<SensorType>(static_cast<int>(a) | static_cast<int>(b));
}

inline SensorType operator&(SensorType a, SensorType b)
{
    return static_cast<SensorType>(static_cast<int>(a) & static_cast<int>(b));
}

#define xstr(s) str(s)
#define str(s) #s

SensorType getSensorsForMPU(int index) {
#ifdef USE_ALL_SENSORS
    return SensorType::Acc | SensorType::Gyro | SensorType::Quat | SensorType::Euler;
#else
    if (strcmp(xstr(DEVICE_NAME), "L_ARM_ESP32") == 0) {
        switch (index) {
            case 0:
                // LUA
                return SensorType::Acc;
            case 1:
                // BACK
                return SensorType::Acc | SensorType::Gyro | SensorType::Quat;
            case 2:
                // LH
                return SensorType::Acc;
        }
    } else if (strcmp(xstr(DEVICE_NAME), "R_ARM_ESP32") == 0) {
        switch (index) {
            case 0:
                // RUA
                return SensorType::Acc;
            case 1:
                // RWR
                return SensorType::Acc;
            case 2:
                // HIP
                return SensorType::Acc;
        }
    } else if (strcmp(xstr(DEVICE_NAME), "L_LEG_NRF52") == 0) {
        // L-SHOE
        return SensorType::Euler;
    } else if (strcmp(xstr(DEVICE_NAME), "R_LEG_NRF52") == 0) {
        switch (index) {
            case 0:
                // RKN
                return SensorType::Acc;
            case 1:
                // R-SHOE
                return SensorType::Gyro;
        }
    }

    return SensorType::None;
#endif
}

#endif //MULTISENSORARDUINO_POSITION_DATA_H
