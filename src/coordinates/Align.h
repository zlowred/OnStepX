// -----------------------------------------------------------------------------------
// GEOMETRIC ALIGN FOR ALT/AZM AND EQ MOUNTS
//
// by Howard Dutton
//
// Copyright (C) 2012 to 2021 Howard Dutton
//

#pragma once

#include <Arduino.h>
#include "../../Constants.h"
#include "../../Config.h"
#include "../../ConfigX.h"
#include "../HAL/HAL.h"
#include "../pinmaps/Models.h"
#include "../debug/Debug.h"

#include "../commands/ProcessCmds.h"

#if defined(ALIGN_MAX_STARS)
  #if ALIGN_MAX_STARS > 9 || ALIGN_MAX_STARS < 3
    #error "ALIGN_MAX_STARS must be 3 to 9"
  #endif
#else
  #if defined(HAL_FAST_PROCESSOR)
    #define ALIGN_MAX_STARS 9
  #else
    #define ALIGN_MAX_STARS 6
  #endif
#endif

// -----------------------------------------------------------------------------------
// ADVANCED GEOMETRIC ALIGN FOR EQUATORIAL MOUNTS (GOTO ASSIST)

enum PierSide: uint8_t {PIER_SIDE_NONE, PIER_SIDE_EAST, PIER_SIDE_WEST};

typedef struct Coordinate {
  double r;
  double h;
  double d;
  double a;
  double z;
  PierSide pierSide;
} Coordinate;

typedef struct AlignCoordinate {
  float ax1;
  float ax2;
  int side;
} AlignCoordinate;

#define AlignModelSize 32
typedef struct AlignModel {
  float ax1Cor;
  float ax2Cor;
  float altCor;
  float azmCor;
  float doCor;
  float pdCor;
  float dfCor;
  float tfCor;
} AlignModel;

class GeoAlign
{
  public:
    // prepare goto assist for operation, also clears the alignment model;
    void init(uint8_t mountType, float latitude);

    // reads the last saved alignment model from NV
    void modelRead();
    // writes the alignment model to NV
    void modelWrite();
    // clear the alignment model
    void modelClear();
    // reports if ready for operation
    bool modelReady();

    // add a star to an alignment model
    // thisStar: 1 for 1st star, 2 for 2nd star, etc. up to numberStars (at which point the mount model is calculated)
    // numberStars: total number of stars for this align (1 to 9)
    // actual: equatorial or horizon coordinate (depending on the mount type) for where the star should be (in mount coordinates)
    // mount:  equatorial or horizon coordinate (depending on the mount type) for where the star should is (in mount coordinates)
    CommandError addStar(int thisStar, int numberStars, Coordinate *actual, Coordinate *mount);
    
    // convert equatorial (h,d) or horizon (a,z) coordinate from observed place to mount
    void observedPlaceToMount(Coordinate *coord);
    // convert equatorial (h,d) or horizon (a,z) coordinate from mount to observed place
    void mountToObservedPlace(Coordinate *coord);

    void autoModel(int n);

    AlignCoordinate mount[ALIGN_MAX_STARS];
    AlignCoordinate actual[ALIGN_MAX_STARS];
    AlignCoordinate delta[ALIGN_MAX_STARS];
    AlignModel model;

  private:
    void correct(float ha, float dec, float pierSide, float sf, float _deo, float _pd, float _pz, float _pe, float _da, float _ff, float _tf, float *h1, float *d1);
    void doSearch(float sf, int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8, int p9);

    float backInRads2(float angle);
    
    bool modelIsReady;
    uint8_t mountType;
    float lat, cosLat, sinLat;

    long num, l;
    long Ff, Df;
    float best_deo, best_pd, best_pz, best_pe, best_ohw, best_odw, best_ohe, best_ode, best_tf, best_df, best_ff;
    float avg_ha, avg_dec;
    float dist, sumd, rms;
    float best_dist;
    float ohe, ode, ohw, odw, dh;
    float sum1;
    float max_dist;

    uint8_t autoModelTask = 0;
};
