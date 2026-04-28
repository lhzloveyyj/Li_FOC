#ifndef __SMO_OBSERVER_H
#define __SMO_OBSERVER_H

#include "at32f403a_407.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rs;
    float ls;
    float ts;
    float k_slide;
    float e_lpf_alpha;
    float speed_lpf_alpha;
} SmoObserverConfig;

typedef struct {
    SmoObserverConfig cfg;

    float iAlphaHat;
    float iBetaHat;
    float zAlpha;
    float zBeta;
    float eAlpha;
    float eBeta;

    float angle;
    float lastAngle;
    float speed;
    float speedRaw;
    uint8_t valid;
} SmoObserver;

extern SmoObserver g_smoObserver;

void SMO_Init(SmoObserver *smo, const SmoObserverConfig *cfg);
void SMO_Reset(SmoObserver *smo);
void SMO_Update(SmoObserver *smo, float uAlpha, float uBeta, float iAlpha, float iBeta);

#ifdef __cplusplus
}
#endif

#endif
