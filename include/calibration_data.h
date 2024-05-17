#ifndef MULTISENSORARDUINO_CALIBRATION_DATA_H
#define MULTISENSORARDUINO_CALIBRATION_DATA_H

#include "MPU6050_6Axis_MotionApps20.h"
#include "QMC5883LCompass.h"

#define xstr(s) str(s)
#define str(s) #s

void calibrateMPU(MPU6050& mpu, int index) {
    if (strcmp(xstr(DEVICE_NAME), "L_ARM_ESP32") == 0) {
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
    } else if (strcmp(xstr(DEVICE_NAME), "R_ARM_ESP32") == 0) {
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
    } else if (strcmp(xstr(DEVICE_NAME), "L_LEG_NRF52") == 0) {
        if (index == 0) {
            mpu.setXAccelOffset(0);
            mpu.setYAccelOffset(0);
            mpu.setZAccelOffset(0);
            mpu.setXGyroOffset(0);
            mpu.setYGyroOffset(0);
            mpu.setZGyroOffset(0);
        }
    } else if (strcmp(xstr(DEVICE_NAME), "R_LEG_NRF52") == 0) {
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
    }
}

void calibrateQMC(QMC5883LCompass& qmc, int index) {
    if (strcmp(xstr(DEVICE_NAME), "L_ARM_ESP32") == 0) {
        if (index == 0) {
            qmc.setCalibrationOffsets(0, 0, 0);
            qmc.setCalibrationScales(1, 1, 1);
        } else if (index == 1) {
            qmc.setCalibrationOffsets(0, 0, 0);
            qmc.setCalibrationScales(1, 1, 1);
        }
    } else if (strcmp(xstr(DEVICE_NAME), "R_ARM_ESP32") == 0) {
        if (index == 0) {
            qmc.setCalibrationOffsets(0, 0, 0);
            qmc.setCalibrationScales(1, 1, 1);
        } else if (index == 1) {
            qmc.setCalibrationOffsets(0, 0, 0);
            qmc.setCalibrationScales(1, 1, 1);
        }

    }
}

#endif //MULTISENSORARDUINO_CALIBRATION_DATA_H
