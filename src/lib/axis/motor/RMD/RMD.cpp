// -----------------------------------------------------------------------------------
// axis RMD servo motor

#include "RMD.h"
#include "CAN.h"

#ifdef RMD_MOTOR_PRESENT

#include "../../../tasks/OnTask.h"

RMDMotor *rmdMotorInstance[2];
IRAM_ATTR void moveRMDMotorAxis1() { rmdMotorInstance[0]->move(); }
IRAM_ATTR void moveRMDMotorAxis2() { rmdMotorInstance[1]->move(); }

RmdCan rmdCan;

// constructor
RMDMotor::RMDMotor(uint8_t axisNumber, const RMDDriverSettings *Settings, bool useFastHardwareTimers) {
  if (axisNumber < 1 || axisNumber > 2) return;

  driverType = RMDRIVER;
  strcpy(axisPrefix, "MSG: RMD _, ");
  axisPrefix[9] = '0' + axisNumber;
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
  if (axisNumber < 1 || axisNumber > 2) return false;

  if (!rmdCan.init()) {
    V(axisPrefix);VL("Failed to init CAN bus!");
  }

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
  if (state == ON) {
    VF("WRN: RMD"); V(axisNumber); VF(", ");
    VLF("axis reversal must be accomplished with hardware or RMD setup!");
  }
}

// set driver parameters
void RMDMotor::setParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
  stepsPerDegree = param1 / 360 * TWO_PI;
  reductionRatio = param2;

  UNUSED(param3);
  UNUSED(param4);
  UNUSED(param5);
  UNUSED(param6);
  setSlewing(isSlewing);
}

// validate driver parameters
bool RMDMotor::validateParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
  if (param1 < 1000) {
    V(axisPrefix);
    VL("Steps per degree appear way too low");
    return false;
  }

  if (param2 < 1) {
    V(axisPrefix);
    VL("Reduction ratio seems incorrect");
    return false;
  }

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
  
  if (state) {
    rmdCan.enable(axisNumber);
  } else {
    rmdCan.disable(axisNumber);
  }
  
  V(axisPrefix); VF("closed loop control - ");
  if (state) { VLF("Active"); } else { VLF("Idle"); }

  enabled = state;
}

void RMDMotor::setInstrumentCoordinateSteps(long value) {
  Motor::setInstrumentCoordinateSteps(value);
}

// get the associated driver status
DriverStatus RMDMotor::getDriverStatus() {
  return status;
}

// resets motor and target angular position in steps, also zeros backlash and index
void RMDMotor::resetPositionSteps(long value) {
  // this is where the initial odrive position in "steps" is brought into agreement with the motor position in steps
  // not sure on this... but code below ignores (value,) gets the odrive position convert to steps and resets the motor
  // there (as the odrive encoders are absolute.)

  long oPosition;

  oPosition = rmdCan.getPosition(axisNumber) * stepsPerDegree / reductionRatio; // TODO adjust to steps

  noInterrupts();
  motorSteps    = oPosition;
  targetSteps   = motorSteps;

  indexSteps  = value - motorSteps;

  backlashSteps = 0;
  interrupts();
}

// set frequency (+/-) in steps per second negative frequencies move reverse in direction (0 stops motion)
void RMDMotor::setFrequencySteps(float frequency) {
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
  if (lastPeriod == 0) return 0;
  return (16000000.0F / lastPeriod) * absStep;
}

// set slewing state (hint that we are about to slew or are done slewing)
void RMDMotor::setSlewing(bool state) {
  isSlewing = state;
}

// updates PID and sets odrive position
void RMDMotor::poll() {
  if ((long)(millis() - lastSetPositionTime) < RMD_UPDATE_MS) return;
  lastSetPositionTime = millis();

  noInterrupts();
  long target = motorSteps + backlashSteps;
  interrupts();

  double rpm = getFrequencySteps() * 60. / stepsPerDegree / 360. * reductionRatio * RMD_MAX_SPEED_NULTIPLIER;

  int arcSeconds = target / stepsPerDegree * 3600.;
  V(axisPrefix);V("Set pos: ");V(arcSeconds/3600);V("º");V((arcSeconds/60)%60);V("'");V(arcSeconds%60);V("\" (x");V(reductionRatio);V(") @ ");V(rpm);V("RPM");
  if (axisNumber == 1) V(" "); else VL("");

  rmdCan.setPosition(axisNumber, arcSeconds / 36. * reductionRatio, rpm * 360. / 60.); 
}

// sets dir as required and moves coord toward target at setFrequencySteps() rate
IRAM_ATTR void RMDMotor::move() {
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