#ifndef MULTISENSORARDUINO_TRIVIALKALMANVECTOR_H
#define MULTISENSORARDUINO_TRIVIALKALMANVECTOR_H

#include <vector>

#include "MPU6050_6Axis_MotionApps20.h"
#include "TrivialKalmanFilter.h"

template <typename D = float> class TrivialKalmanVector {
public:
    TrivialKalmanVector(int count, const D Rk, const D Qk) {
        for (int i = 0; i < count; ++i) {
            filters.push_back(TrivialKalmanFilter<D>(Rk, Qk));
        }
    }

    TrivialKalmanFilter<D>& operator[](int index) {
        return filters[index];
    }

    std::vector<D> update(const std::vector<D>& zk) {
        std::vector<D> result;
        for (int i = 0; i < filters.size(); ++i) {
            result.push_back(filters[i].update(zk[i]));
        }

        return result;
    }

    std::vector<D> get() const {
        std::vector<D> result;
        for (int i = 0; i < filters.size(); ++i) {
            result.push_back(filters[i].get());
        }

        return result;
    }

    void reset() {
        for (int i = 0; i < filters.size(); ++i) {
            filters[i].reset();
        }
    }
private:
    std::vector<TrivialKalmanFilter<D>> filters;
};


#endif //MULTISENSORARDUINO_TRIVIALKALMANVECTOR_H
