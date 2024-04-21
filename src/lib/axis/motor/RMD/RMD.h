// -----------------------------------------------------------------------------------
// axis rmd servo motor
#pragma once
#include "../../../../Common.h"

// RMD DRIVER MODEL
#ifndef RMD_DRIVER_FIRST
  #define RMD_DRIVER_FIRST       400
  #define RMD                    400
  #define RMD_DRIVER_LAST        400
#endif

#ifdef RMD_MOTOR_PRESENT

#include "../Motor.h"
#include "../../../convert/Convert.h"

// RMD update rate default 1Hz
#ifndef RMD_UPDATE_MS
  #define RMD_UPDATE_MS   100
#endif
#ifndef RMD_MAX_SPEED_NULTIPLIER
  #define RMD_MAX_SPEED_NULTIPLIER   1.01
#endif


typedef struct RMDDriverSettings {
  int16_t model;
  int8_t  status;
} RMDDriverSettings;

class RMDMotor : public Motor {
  public:
    // constructor
    RMDMotor(uint8_t axisNumber, const RMDDriverSettings *Settings, bool useFastHardwareTimers = true);

    // sets up the motor
    bool init();

    // set driver reverse state
    void setReverse(int8_t state);

    // get driver type code
    inline char getParameterTypeCode() { return 'R'; }  // codes used so far are S(tep/dir), T(mc), P(id), and O(Drive)

    // set driver parameters
    void setParameters(float param1, float param2, float param3, float param4, float param5, float param6);

    // validate driver parameters
    bool validateParameters(float param1, float param2, float param3, float param4, float param5, float param6);

    // sets motor enable on/off (if possible)
    void enable(bool value);

    // set instrument coordinate, in steps
    void setInstrumentCoordinateSteps(long value);

    // get the associated driver status
    DriverStatus getDriverStatus();

    // resets motor and target angular position in steps, also zeros backlash and index 
    void resetPositionSteps(long value);

    // get tracking mode steps per slewing mode step
    inline int getStepsPerStepSlewing() { return 64; }

    // get movement frequency in steps per second
    float getFrequencySteps();

    // set frequency (+/-) in steps per second negative frequencies move reverse in direction (0 stops motion)
    void setFrequencySteps(float frequency);

    // set slewing state (hint that we are about to slew or are done slewing)
    void setSlewing(bool state);

    // updates PID and sets servo motor power/direction
    void poll();

    // sets dir as required and moves coord toward target at setFrequencySteps() rate
    void move();

  private:

    unsigned long lastSetPositionTime = 0;
    uint8_t rmdMonitorHandle = 0;
    uint8_t taskHandle = 0;

    double reductionRatio = 0;
    double stepsPerDegree = 0;

    int  stepSize = 1;                  // step size
    volatile int  homeSteps = 1;        // step count for microstep sequence between home positions (driver indexer)
    volatile bool takeStep = false;     // should we take a step

    float currentFrequency = 0.0F;      // last frequency set 
    float lastFrequency = 0.0F;         // last frequency requested
    unsigned long lastPeriod = 0;       // last timer period (in sub-micros)

    volatile int absStep = 1;           // absolute step size (unsigned)

    void (*callback)() = NULL;

    bool useFastHardwareTimers = true;

    bool isSlewing = false;

    DriverStatus status = { false, {false, false}, {false, false}, false, false, false, false };
};

#endif