/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "calibration_itof.h"

uint16_t *CalibrationItof::buildCalibrationCache(float gain, float offset,
                                               int16_t maxPixelValue) {
    uint16_t *cache = new uint16_t[maxPixelValue];
    for (int16_t current = 0; current < maxPixelValue; ++current) {
        cache[current] =
            static_cast<int16_t>(static_cast<float>(current) * gain + offset);
    }
    return cache;
}

void CalibrationItof::calibrateFrame(uint16_t *calibrationData, uint16_t *frame,
                                   unsigned int width, unsigned int height) {
#if 0
    //	uint16_t *cache = calibrationData;

    unsigned int size = width * height / 2;

    uint16_t *end = frame + (size - size % 8);
    uint16_t *framePtr = frame;

    for (; framePtr < end; framePtr += 8) {
        *framePtr = *(*framePtr);
        *(framePtr + 1) = *(*(framePtr + 1));
        *(framePtr + 2) = *( *(framePtr + 2));
        *(framePtr + 3) = *( *(framePtr + 3));
        *(framePtr + 4) = *(*(framePtr + 4));
        *(framePtr + 5) = *( *(framePtr + 5));
        *(framePtr + 6) = *( *(framePtr + 6));
        *(framePtr + 7) = *( *(framePtr + 7));
    }

    end += (size % 8);

    for (; framePtr < end; framePtr++) {
        *framePtr = *(cache + *framePtr);
    }
#endif
}
