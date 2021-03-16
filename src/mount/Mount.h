//--------------------------------------------------------------------------------------------------
// telescope mount control
#pragma once
#include <Arduino.h>
#include "../../Constants.h"
#include "../../Config.h"
#include "../../ConfigX.h"
#include "../HAL/HAL.h"
#include "../pinmaps/Models.h"

#if AXIS1_DRIVER_MODEL != OFF && AXIS2_DRIVER_MODEL != OFF

#if AXIS1_STEPS_PER_WORMROT == 0
  #define AXIS1_PEC OFF
#else
  #define AXIS1_PEC ON
#endif

#include "../debug/Debug.h"
#include "../commands/ProcessCmds.h"
#include "../coordinates/Transform.h"
#include "../motion/StepDrivers.h"
#include "../motion/Axis.h"

enum MeridianFlip: uint8_t     {MF_NEVER, MF_ALWAYS};
enum RateCompensation: uint8_t {RC_NONE, RC_REFR_RA, RC_REFR_BOTH, RC_FULL_RA, RC_FULL_BOTH};
enum TrackingState: uint8_t    {TS_NONE, TS_SIDEREAL};
enum GotoState: uint8_t        {GS_NONE, GS_GOTO};
enum GotoStage: uint8_t        {GG_NONE, GG_WAYPOINT, GG_DESTINATION, GG_READY_ABORT, GG_ABORT};
enum GotoType: uint8_t         {GT_NONE, GT_HOME, GT_PARK};
enum PierSideSelect: uint8_t   {PSS_NONE, PSS_EAST, PSS_WEST, PSS_BEST, PSS_EAST_ONLY, PSS_WEST_ONLY, PSS_SAME_ONLY};
enum GuideState: uint8_t       {GU_NONE, GU_GUIDE, GU_PULSE_GUIDE};
enum GuideRateSelect: uint8_t  {GR_QUARTER, GR_HALF, GR_1X, GR_2X, GR_4X, GR_8X, GR_20X, GR_48X, GR_HALF_MAX, GR_MAX, GR_CUSTOM};
enum GuideAction: uint8_t      {GA_NONE, GA_BREAK, GA_FORWARD, GA_REVERSE};
enum ParkState: uint8_t        {PS_NONE, PS_UNPARKED, PS_PARKING, PS_PARKED, PS_PARK_FAILED};
enum PecState: uint8_t         {PEC_NONE, PEC_READY_PLAY, PEC_PLAY, PEC_READY_RECORD, PEC_RECORD};
enum CoordReturn: uint8_t      {CR_MOUNT, CR_MOUNT_EQU, CR_MOUNT_ALT, CR_MOUNT_HOR};

#pragma pack(1)
typedef struct AltitudeLimits {
  float min;
  float max;
} AltitudeLimits;

#define LimitsSize 16
typedef struct Limits {
  AltitudeLimits altitude;
  float pastMeridianE;
  float pastMeridianW;
} Limits;

typedef struct Backlash {
  float axis1;
  float axis2;
} Backlash;

#define MiscSize 14
typedef struct Misc {
  bool syncToEncodersOnly:1;
  bool meridianFlipAuto  :1;
  bool meridianFlipPause :1;
  bool buzzer            :1;
  float usPerStepCurrent;
  GuideRateSelect pulseGuideRateSelect;
  Backlash backlash;
} Misc;

#define PecSize 6
typedef struct Pec {
  bool recorded:1;
  PecState state;
  long wormRotationSteps;
} Pec;

typedef struct ParkPosition {
  float h;
  float d;
  PierSide pierSide;
} ParkPosition;

#define ParkSize 15
typedef struct Park {
  ParkPosition position;
  bool         saved:1;
  ParkState    state;
  long         wormSensePositionSteps;
} Park;
#pragma pack()

typedef struct AlignState {
  uint8_t currentStar:1;
  uint8_t lastStar:1;
} AlignState;

typedef struct MinMaxError {
  uint8_t min:1;
  uint8_t max:1;
} MinMaxError;

typedef struct MerdianError {
  uint8_t east:1;
  uint8_t west:1;
} MerdianError;

typedef struct LimitError {
  MinMaxError axis1;
  MinMaxError axis2;
} LimitError;

typedef struct MountError {
  MinMaxError   altitude;
  LimitError    limit;
  LimitError    limitSense;
  MerdianError  meridian;
  uint8_t       parkFailed: 1;
  uint8_t       motorFault: 1;
  uint8_t       initWeather: 1;
  uint8_t       initSite: 1;
  uint8_t       initNV: 1;
} MountError;



class Mount {
  public:
    void init();
    void pecInit();

    // handle mount commands
    bool command(char *reply, char *command, char *parameter, bool *supressFrame, bool *numericReply, CommandError *commandError);
    bool commandGoto(char *reply, char *command, char *parameter, bool *supressFrame, bool *numericReply, CommandError *commandError);
    bool commandGuide(char *reply, char *command, char *parameter, bool *supressFrame, bool *numericReply, CommandError *commandError);
    bool commandLimit(char *reply, char *command, char *parameter, bool *supressFrame, bool *numericReply, CommandError *commandError);
    bool commandPec(char *reply, char *command, char *parameter, bool *supressFrame, bool *numericReply, CommandError *commandError);

    // update the home position
    void updateHomePosition();

    // reset all mount related errors
    void errorReset();

    // check for any mount related error
    bool errorAny();
    
    // return OnStep general error code
    uint8_t errorNumber();

    // monitor goto operation
    void gotoPoll();

    // monitor guide operation
    void guidePoll();

    // monitor pec operation
    void pecPoll();

    // monitor limits
    void limitPoll();

    // calculate tracking rates for alt/azm, refraction, and pointing model in the background
    void trackPoll();

    Transform  transform;

    Axis axis1;
    Axis axis2;

    MountError error;

  private:
    // general status checks ahead of sync or goto
    CommandError validateGoto();
    // target coordinate checks ahead of sync or goto
    CommandError validateGotoCoords(Coordinate *coords);
    // set goto or sync target
    CommandError setMountTarget(Coordinate *coords, PierSideSelect pierSideSelect, bool native = true);
    // sync. to equatorial coordinates
    CommandError syncEqu(Coordinate *coords, PierSideSelect pierSideSelect, bool native = true);
    // goto equatorial coordinates
    CommandError gotoEqu(Coordinate *coords, PierSideSelect pierSideSelect, bool native = true);
    // set any waypoints required for a goto
    void gotoWaypoint();
    // stop any presently active goto
    void gotoStop();

    // reset mount at home
    CommandError resetHome();

    // change tracking state
    void setTrackingState(TrackingState state);
    // Alternate tracking rate calculation method
    float ztr(float a);

    // update where we are pointing *now*
    // CR_MOUNT for Horizon or Equatorial mount coordinates, depending on mode
    // CR_MOUNT_EQU for Equatorial mount coordinates, depending on mode
    // CR_MOUNT_ALT for altitude (a) and Horizon or Equatorial mount coordinates, depending on mode
    // CR_MOUNT_HOR for Horizon mount coordinates, depending on mode
    void updatePosition(CoordReturn coordReturn);

    // update tracking rates
    void updateTrackingRates();
    // update acceleration rates for goto and guiding
    void updateAccelerationRates();
    // estimate average microseconds per step lower limit
    float usPerStepLowerLimit();

    bool alignActive();
    CommandError alignAddStar();

    // return guide rate (sidereal x) for guide rate selection
    float guideRateSelectToRate(GuideRateSelect guideRateSelect, uint8_t axis = 1);
    // valid guide on Axis1
    bool guideValidAxis1(GuideAction guideAction);
    // valid guide on Axis2
    bool guideValidAxis2(GuideAction guideAction);
    // start guide on Axis1
    CommandError guideStartAxis1(GuideAction guideAction, GuideRateSelect guideRateSelect, unsigned long guideTimeLimit);
    // start guide on Axis2
    CommandError guideStartAxis2(GuideAction guideAction, GuideRateSelect guideRateSelect, unsigned long guideTimeLimit);
    // stop guide on Axis1, use GA_BREAK to stop in either direction or specifiy the direction to be stopped GA_FORWARD or GA_REVERSE
    void guideStopAxis1(GuideAction stopDirection);
    // stop guide on Axis2, use GA_BREAK to stop in either direction or specifiy the direction to be stopped GA_FORWARD or GA_REVERSE
    void guideStopAxis2(GuideAction stopDirection);
    // check for spiral guide using both axes
    bool guideIsSpiral();

    #if AXIS1_PEC == ON
      // disable PEC
      void pecDisable();
      // PEC low pass and linear regression filters
      void pecCleanup();
    #endif

    // read in the park information
    void parkInit();
    // goto park position
    CommandError parkGoto();
    // once parked save park state
    void parkFinish();
    // sets a park position
    CommandError parkSet();
    // finds nearest valid park position (for microstepping up to 256x)
    void parkNearest();
    // returns a parked telescope to operation
    CommandError parkRestore(bool withTrackingOn);

    // read in the limit information, start limit monitor
    void limitInit();
    void limitBroken();
    void limitStop(GuideAction stopDirection);
    void limitStopAxis1(GuideAction stopDirection);
    void limitStopAxis2(GuideAction stopDirection);

    const double radsPerCentisecond  = (degToRad(15.0/3600.0)/100.0)*SIDEREAL_RATIO;
    
    // current position in Mount coordinates (Observed Place with no corrections except index offset)
    // coordinates are either Horizon (a, z) or Equatorial (h, d) depending on the mountType
    Coordinate current;
    TrackingState trackingState         = TS_NONE;
    #if TRACK_REFRACTION_RATE_DEFAULT == ON
      RateCompensation rateCompensation = RC_REFR_RA;
    #else
      RateCompensation rateCompensation = RC_NONE;
    #endif
    float trackingRate                  = 1.0F;
    float trackingRateAxis1             = 0.0F;
    float trackingRateAxis2             = 0.0F;
    float stepsPerSiderealSecondAxis1   = 0.0F;
    double stepsPerCentisecondAxis1     = 0.0F;
    double stepsPerCentisecondAxis2     = 0.0F;

    // align
    AlignState alignState = {0, 0};

    // goto
    PierSideSelect preferredPierSide    = (PierSideSelect)PIER_SIDE_PREFERRED_DEFAULT;
    MeridianFlip meridianFlip           = MF_ALWAYS;
    Coordinate gotoTarget;
    Coordinate start, destination, target;
    GotoState  gotoState                = GS_NONE;
    GotoStage  gotoStage                = GG_NONE;
    GotoState  gotoStateAbort           = GS_NONE;
    GotoState  gotoStateLast            = GS_NONE;
    uint8_t    gotoTaskHandle           = 0;
    float      usPerStepDefault         = 64.0F;
    float      usPerStepBase            = 128.0F;
    float      radsPerSecondCurrent;

    // limits
    bool       limitsEnabled            = false;
    Limits limits = { { degToRad(-10), degToRad(85) }, degToRad(15), degToRad(15) };

    // homing
    Coordinate home;
    bool       atHome                   = true;
    bool       waitingHome              = false;
    bool       waitingHomeContinue      = false;
  
    // guiding
    float guideRateAxis1                = 0.0F;
    float guideRateAxis2                = 0.0F;
    float customGuideRateAxis1          = 0.0F;
    float customGuideRateAxis2          = 0.0F;
    GuideState      guideState          = GU_NONE;
    GuideRateSelect guideRateSelect     = GR_20X;
    GuideRateSelect guideRateSelectAxis1= GR_20X;
    GuideRateSelect guideRateSelectAxis2= GR_20X;
    GuideAction     guideActionAxis1    = GA_NONE;
    GuideAction     guideActionAxis2    = GA_NONE;
    unsigned long guideFinishTimeAxis1  = 0;
    unsigned long guideFinishTimeAxis2  = 0;

    // pec
    long      pecBufferSize             = 0;
    float     pecRateAxis1              = 0;
    #if AXIS1_PEC == ON
      uint8_t  pecMonitorHandle         = 0;
      Pec pec = {false, PEC_NONE, 0};
      bool     pecIndexSensedSinceLast  = false;
      int      pecAnalogValue           = 0;

      bool     pecFirstRecording        = false;
      long     pecRecordStopTime        = 0;
      float    accPecGuideAxis1         = 0.0F;
      long     pecIndex                 = 0;
      int      pecValue                 = 0;
      long     wormRotationSteps        = 0;
      long     lastWormRotationSteps    = -1;
      long     wormSenseSteps           = 0;
      bool     wormSenseAgain           = false;
      uint32_t wormPeriodStartCs        = 0;
      long     wormRotationSeconds      = 0;
      bool     pecBufferStart           = false;
      int8_t*  pecBuffer;
    #endif

    // park
    Park park = {{0, 0, PIER_SIDE_NONE}, false, PS_NONE, 0};

    // misc. settings stored in NV
    Misc misc = {false, false, false, false, 64.0, GR_20X};

    float timerRateRatio = 1.0;

};

#endif
