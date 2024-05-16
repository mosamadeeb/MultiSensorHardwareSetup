#ifndef MULTISENSORARDUINO_FILTEREDQMC_H
#define MULTISENSORARDUINO_FILTEREDQMC_H

#include "QMC5883LCompass.h"
#include "TrivialKalmanVector.h"

struct FilteredQMC {
    QMC5883LCompass qmc;
    TrivialKalmanVector<float> mag;
};

#endif //MULTISENSORARDUINO_FILTEREDQMC_H
