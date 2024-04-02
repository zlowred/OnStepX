// -----------------------------------------------------------------------------------
// axis rmd servo motor

#include "RMD.h"

#ifdef RMD_MOTOR_PRESENT

#include "../../../tasks/OnTask.h"

RMDMotor *rmdMotorInstance[2];
IRAM_ATTR void moveRMDMotorAxis1() { rmdMotorInstance[0]->move(); }
IRAM_ATTR void moveRMDMotorAxis2() { rmdMotorInstance[1]->move(); }

// constructor
RMDMotor::RMDMotor(uint8_t axisNumber, const RMDDriverSettings *Settings, bool useFastHardwareTimers) {
  V("constructor(");V(axisNumber);V(", ");V("<settings>");V(", ");V(useFastHardwareTimers);VL(")");
  if (axisNumber < 1 || axisNumber > 2) return;

  driverType = RMDRIVER;
  strcpy(axisPrefix, "MSG: RMD _, ");
  axisPrefix[9] = '0' + this->axisNumber;
  this->axisNumber = axisNumber;


  if (axisNumber > 2) useFastHardwareTimers = false;
  this->useFastHardwareTimers = useFastHardwareTimers;

  // attach the function pointers to the callbacks
  rmdMotorInstance[this->axisNumber - 1] = this;
  switch (this->axisNumber) {
    case 1: callback = moveRMDMotorAxis1; break;
    case 2: callback = moveRMDMotorAxis2; break;
  }
}

bool RMDMotor::init() {
  V(axisPrefix);VL("init()");
  if (axisNumber < 1 || axisNumber > 2) return false;

  enable(false);

  // start the motor timer
  V(axisPrefix);
  VF("start task to move motor... ");
  char timerName[] = "Target_";
  timerName[6] = '0' + axisNumber;
  taskHandle = tasks.add(0, 0, true, 0, callback, timerName);
  if (taskHandle) {
    V("success");
    if (useFastHardwareTimers && !tasks.requestHardwareTimer(taskHandle, 0)) { VLF(" (no hardware timer!)"); } else { VLF(""); }
  } else {
    VLF("FAILED!");
    return false;
  }

  return true;
}

// set driver reverse state
void RMDMotor::setReverse(int8_t state) {
  V(axisPrefix);V("setReverse(");V(state);VL(")");
  if (state == ON) {
    VF("WRN: RMD"); V(axisNumber); VF(", ");
    VLF("axis reversal must be accomplished with hardware or RMD setup!");
  }
}

// set driver parameters
void RMDMotor::setParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
  V(axisPrefix);V("setParameters(");V(param1);V(", ");V(param2);V(", ");V(param3);V(", ");V(param4);V(", ");V(param5);V(", ");V(param6);VL(")");
  UNUSED(param1); // general purpose settings defined in Extended.config.h and stored in NV, they can be modified at runtime
  UNUSED(param2);
  UNUSED(param3);
  UNUSED(param4);
  UNUSED(param5);
  UNUSED(param6);
  setSlewing(isSlewing);
}

// validate driver parameters
bool RMDMotor::validateParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
    V(axisPrefix);V("validateParameters(");V(param1);V(", ");V(param2);V(", ");V(param3);V(", ");V(param4);V(", ");V(param5);V(", ");V(param6);VL(")");
  UNUSED(param1);
  UNUSED(param2);
  UNUSED(param3);
  UNUSED(param4);
  UNUSED(param5);
  UNUSED(param6);
  return true;
}

// sets motor enable on/off (if possible)
void RMDMotor::enable(bool state) {
  V(axisPrefix); V("driver powered ");
  if (state) { VLF("up"); } else { VLF("down"); } 
  
//   int requestedState = AXIS_STATE_IDLE;
//   if (state) requestedState = AXIS_STATE_CLOSED_LOOP_CONTROL;
  
  V(axisPrefix); VF("closed loop control - ");
  if (state) { VLF("Active"); } else { VLF("Idle"); }

  enabled = state;
}

void RMDMotor::setInstrumentCoordinateSteps(long value) {
  V(axisPrefix);V("setInstrumentCoordinateSteps(");V(value);VL(")");  
  Motor::setInstrumentCoordinateSteps(value);
}

// get the associated driver status
DriverStatus RMDMotor::getDriverStatus() {
    V(axisPrefix);VL("getDriverStatus()");
  return status;
}

// resets motor and target angular position in steps, also zeros backlash and index
void RMDMotor::resetPositionSteps(long value) {
    V(axisPrefix);V("resetPositionSteps(");V(value);VL(")");  
  // this is where the initial odrive position in "steps" is brought into agreement with the motor position in steps
  // not sure on this... but code below ignores (value,) gets the odrive position convert to steps and resets the motor
  // there (as the odrive encoders are absolute.)

  long oPosition;
//   if (axisNumber - 1 == 0) oPosition = o_position0;
//   if (axisNumber - 1 == 1) oPosition = o_position1;

  // get ODrive position in fractionial Turns
//   #if ODRIVE_COMM_MODE == OD_UART
//     oPosition = _oDriveDriver->GetPosition(axisNumber - 1)*TWO_PI*stepsPerMeasure; // axis1/2 are in steps per radian
//   #elif ODRIVE_COMM_MODE == OD_CAN
//     oPosition = _oDriveDriver->GetPosition(axisNumber - 1)*TWO_PI*stepsPerMeasure; // axis1/2 are in steps per radian
//   #endif

    oPosition = 0;

  noInterrupts();
  motorSteps    = oPosition;
  targetSteps   = motorSteps;

    // but what if the odrive encoders are incremental?  how to tell the odrive what its angular position is?
    // here thinking we'll ignore it... sync OnStepX there and let the offset handle it
    indexSteps  = value - motorSteps;

  backlashSteps = 0;
  interrupts();
}

// set frequency (+/-) in steps per second negative frequencies move reverse in direction (0 stops motion)
void RMDMotor::setFrequencySteps(float frequency) {
    V(axisPrefix);V("setFrequencySteps(");V(frequency);VL(")");  
  // negative frequency, convert to positive and reverse the direction
  int dir = 0;
  if (frequency > 0.0F) dir = 1; else if (frequency < 0.0F) { frequency = -frequency; dir = -1; }

  // if in backlash override the frequency
  if (inBacklash)
    frequency = backlashFrequency;

  if (frequency != currentFrequency) {
    lastFrequency = frequency;

    // if slewing has a larger step size divide the frequency to account for it
    if (lastFrequency <= backlashFrequency * 2.0F) stepSize = 1; else { if (!inBacklash) stepSize = 64; }
    frequency /= stepSize;

    // timer period in microseconds
    float period = 1000000.0F / frequency;

    // range is 0 to 134 seconds/step
    if (!isnan(period) && period <= 130000000.0F) {
      period *= 16.0F;
      lastPeriod = (unsigned long)lroundf(period);
    } else {
      lastPeriod = 0;
      frequency = 0.0F;
      dir = 0;
    }

    currentFrequency = frequency;

    // change the motor rate/direction
    noInterrupts();
    step = 0;
    interrupts();
    tasks.setPeriodSubMicros(taskHandle, lastPeriod);
  }

  noInterrupts();
  step = dir * stepSize;
  absStep = abs(step);
  interrupts();
}

float RMDMotor::getFrequencySteps() {
    V(axisPrefix);VL("getFrequencySteps()");
  if (lastPeriod == 0) return 0;
  return (16000000.0F / lastPeriod) * absStep;
}

// set slewing state (hint that we are about to slew or are done slewing)
void RMDMotor::setSlewing(bool state) {
    V(axisPrefix);V("setSlewing(");V(state);VL(")");  
  isSlewing = state;
}

// updates PID and sets odrive position
void RMDMotor::poll() {
  if ((long)(millis() - lastSetPositionTime) < RMD_UPDATE_MS) return;
  lastSetPositionTime = millis();
    V(axisPrefix);VL("poll()");

  noInterrupts();
  long target = motorSteps + backlashSteps;
  interrupts();

  UNUSED(target);
//   #if ODRIVE_COMM_MODE == OD_UART
//     setPosition(axisNumber -1, target/(TWO_PI*stepsPerMeasure));
//   #elif ODRIVE_COMM_MODE == OD_CAN
//     _oDriveDriver->SetPosition(axisNumber -1, target/(TWO_PI*stepsPerMeasure));
//   #endif
}

// sets dir as required and moves coord toward target at setFrequencySteps() rate
IRAM_ATTR void RMDMotor::move() {
    V(axisPrefix);VL("move()");
  if (sync && !inBacklash) targetSteps += step;

  if (motorSteps > targetSteps) {
    if (backlashSteps > 0) {
      backlashSteps -= absStep;
      inBacklash = backlashSteps > 0;
    } else {
      motorSteps -= absStep;
      inBacklash = false;
    }
  } else

  if (motorSteps < targetSteps || inBacklash) {
    if (backlashSteps < backlashAmountSteps) {
      backlashSteps += absStep;
      inBacklash = backlashSteps < backlashAmountSteps;
    } else {
      motorSteps += absStep;
      inBacklash = false;
    }
  }
}

#endif