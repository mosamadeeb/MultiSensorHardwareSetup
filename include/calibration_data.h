#ifndef MULTISENSORARDUINO_CALIBRATION_DATA_H
#define MULTISENSORARDUINO_CALIBRATION_DATA_H

#include "MPU6050_6Axis_MotionApps20.h"
#include "QMC5883LCompass.h"

#define xstr(s) str(s)
#define str(s) #s

void calibrateMPU(MPU6050& mpu, int index) {
    if (strcmp(xstr(DEVICE_NAME), "L_ARM_ESP32") == 0) {
        if (index == 0) {
            mpu.setXAccelOffset(-4022);
            mpu.setYAccelOffset(-11);
            mpu.setZAccelOffset(1066);
            mpu.setXGyroOffset(-12);
            mpu.setYGyroOffset(-14);
            mpu.setZGyroOffset(-25);
        } else if (index == 1) {
            mpu.setXAccelOffset(-1663);
            mpu.setYAccelOffset(154);
            mpu.setZAccelOffset(1254);
            mpu.setXGyroOffset(-681);
            mpu.setYGyroOffset(77);
            mpu.setZGyroOffset(-69);
        } else if (index == 2) {
            mpu.setXAccelOffset(-1988);
            mpu.setYAccelOffset(-270);
            mpu.setZAccelOffset(1358);
            mpu.setXGyroOffset(53);
            mpu.setYGyroOffset(-13);
            mpu.setZGyroOffset(-187);
        }
    } else if (strcmp(xstr(DEVICE_NAME), "R_ARM_ESP32") == 0) {
        if (index == 0) {
            mpu.setXAccelOffset(-993);
            mpu.setYAccelOffset(33);
            mpu.setZAccelOffset(1053);
            mpu.setXGyroOffset(77);
            mpu.setYGyroOffset(50);
            mpu.setZGyroOffset(-13);
        } else if (index == 1) {
            mpu.setXAccelOffset(-5197);
            mpu.setYAccelOffset(-444);
            mpu.setZAccelOffset(1565);
            mpu.setXGyroOffset(5);
            mpu.setYGyroOffset(61);
            mpu.setZGyroOffset(39);
        } else if (index == 2) {
            mpu.setXAccelOffset(-796);
            mpu.setYAccelOffset(-1633);
            mpu.setZAccelOffset(419);
            mpu.setXGyroOffset(122);
            mpu.setYGyroOffset(-13);
            mpu.setZGyroOffset(7);
        }
    } else if (strcmp(xstr(DEVICE_NAME), "L_LEG_NRF52") == 0) {
        if (index == 0) {
            mpu.setXAccelOffset(-1607);
            mpu.setYAccelOffset(275);
            mpu.setZAccelOffset(1154);
            mpu.setXGyroOffset(487);
            mpu.setYGyroOffset(-531);
            mpu.setZGyroOffset(-151);
        }
    } else if (strcmp(xstr(DEVICE_NAME), "R_LEG_NRF52") == 0) {
        if (index == 0) {
            mpu.setXAccelOffset(-602);
            mpu.setYAccelOffset(1570);
            mpu.setZAccelOffset(929);
            mpu.setXGyroOffset(-53);
            mpu.setYGyroOffset(-19);
            mpu.setZGyroOffset(-135);
        } else if (index == 1) {
            mpu.setXAccelOffset(-2624);
            mpu.setYAccelOffset(2117);
            mpu.setZAccelOffset(983);
            mpu.setXGyroOffset(1921);
            mpu.setYGyroOffset(-8667);
            mpu.setZGyroOffset(-262);
        }
    }
}

void calibrateQMC(QMC5883LCompass& qmc, int index) {
    if (strcmp(xstr(DEVICE_NAME), "L_ARM_ESP32") == 0) {
        if (index == 0) {
            qmc.setCalibrationOffsets(63.00, -141.00, -743.00);
            qmc.setCalibrationScales(5.01, 2.24, 0.42);
        } else if (index == 1) {
            qmc.setCalibrationOffsets(-256.00, -640.00, -763.00);
            qmc.setCalibrationScales(2.16, 0.86, 0.72);
        }
    } else if (strcmp(xstr(DEVICE_NAME), "R_ARM_ESP32") == 0) {
        if (index == 0) {
            qmc.setCalibrationOffsets(3.00, 820.00, -503.00);
            qmc.setCalibrationScales(2.09, 0.64, 1.04);
        } else if (index == 1) {
            qmc.setCalibrationOffsets(-473.00, -267.00, -646.00);
            qmc.setCalibrationScales(0.98, 1.73, 0.72);
        }

    }
}

#endif //MULTISENSORARDUINO_CALIBRATION_DATA_H
