//
// Created by root on 18-10-29.
//

#ifndef BUNDLESLAM_INITIALSFM_H
#define BUNDLESLAM_INITIALSFM_H
#include "CameraBuffer.h"
#include "FeatureTrack.h"
class InitialSFM{
public:
    InitialSFM(){};
    ~InitialSFM(){};
    bool tryReConstruct(FeaturesVector &features,CameraBuffer&poses);
};
#endif //BUNDLESLAM_INITIALSFM_H
