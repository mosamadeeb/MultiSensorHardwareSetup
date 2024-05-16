#ifndef MULTISENSORARDUINO_FILTEREDMPU_H
#define MULTISENSORARDUINO_FILTEREDMPU_H

#include "MPU6050_6Axis_MotionApps20.h"
#include "TrivialKalmanVector.h"

struct FilteredMPU {
    MPU6050 mpu;
    TrivialKalmanVector<float> acc;
    TrivialKalmanVector<float> gyro;
    TrivialKalmanVector<float> euler;
    TrivialKalmanVector<float> quat;
};

#endif //MULTISENSORARDUINO_FILTEREDMPU_H
