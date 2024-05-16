#ifndef MULTISENSORARDUINO_CALIBRATION_DATA_H
#define MULTISENSORARDUINO_CALIBRATION_DATA_H

#include "MPU6050_6Axis_MotionApps20.h"
#include "QMC5883LCompass.h"

void calibrateMPU(MPU6050& mpu, int index) {
#if DEVICE_NAME == ESP32_LEFT_ARM
    if (index == 0) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    } else if (index == 1) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    } else if (index == 2) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    }
#elif DEVICE_NAME == ESP32_RIGHT_ARM
    if (index == 0) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    } else if (index == 1) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    } else if (index == 2) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    }
#elif DEVICE_NAME == NRF52_LEFT_LEG
    if (index == 0) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    } else if (index == 1) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    }
#elif DEVICE_NAME == NRF52_RIGHT_LEG
    if (index == 0) {
        mpu.setXAccelOffset(-1011);
        mpu.setYAccelOffset(-1011);
        mpu.setZAccelOffset(1040);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    }
#endif
}

void calibrateQMC(QMC5883LCompass& qmc, int index) {
#if DEVICE_NAME == ESP32_LEFT_ARM
    if (index == 0) {
        qmc.setCalibrationOffsets(0, 0, 0);
        qmc.setCalibrationScales(1, 1, 1);
    } else if (index == 1) {
        qmc.setCalibrationOffsets(0, 0, 0);
        qmc.setCalibrationScales(1, 1, 1);
    }
#elif DEVICE_NAME == ESP32_RIGHT_ARM
    if (index == 0) {
        qmc.setCalibrationOffsets(0, 0, 0);
        qmc.setCalibrationScales(1, 1, 1);
    } else if (index == 1) {
        qmc.setCalibrationOffsets(0, 0, 0);
        qmc.setCalibrationScales(1, 1, 1);
    }
#endif
}

#endif //MULTISENSORARDUINO_CALIBRATION_DATA_H
