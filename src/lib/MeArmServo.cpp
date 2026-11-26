#include "MeArmServo.h"

MeArmServo::MeArmServo(){
  currentAngle = 0;
  readyTime = 0;
  positionKnown = false;
  _moving = false;
  msPerDegree = MEARM_SMOOTH_MS_PER_DEGREE;
  lastPulse = -1;
}

void MeArmServo::begin(int pin, int min, int max, int offset){
  _servo.attach(pin, MEARM_SERVO_MIN_US, MEARM_SERVO_MAX_US);
  readyTime = 0;
  maxAngle = max;
  minAngle = min;
  offsetAngle = offset;
}

float MeArmServo::clampAngle(float angle){
  if(angle < minAngle) return minAngle;
  if(angle > maxAngle) return maxAngle;
  return angle;
}

// Write as microseconds rather than whole degrees so the slow ends of an
// S-curve move in small steps instead of visible 1 degree jumps.
void MeArmServo::writeAngle(float angle){
  currentAngle = angle;
  int pulse = MEARM_SERVO_MIN_US + (int)(angle * (MEARM_SERVO_MAX_US - MEARM_SERVO_MIN_US) / 180.0f + 0.5f);
  if(pulse != lastPulse){
    _servo.writeMicroseconds(pulse);
    lastPulse = pulse;
  }
}

void MeArmServo::moveToCentre(){
  setCurrentAngle(minAngle + (maxAngle - minAngle)/2);
}

// Immediate move. Cancels any smooth move in progress.
void MeArmServo::setCurrentAngle(float angle){
  float diff = positionKnown ? fabsf(angle - currentAngle) : 180;
  _moving = false;
  positionKnown = true;
  writeAngle(clampAngle(angle));
  readyTime = millis() + (unsigned long)(diff * 2.5); //approx 2.5ms / degree
}

float MeArmServo::getCurrentAngle(){
  return currentAngle - offsetAngle;
}

void MeArmServo::moveToAngle(float angle){
  setCurrentAngle(angle + offsetAngle);
}

void MeArmServo::setSpeed(float ms){
  if(ms > 0) msPerDegree = ms;
}

unsigned long MeArmServo::smoothDuration(float angle){
  float distance = fabsf(clampAngle(angle + offsetAngle) - currentAngle);
  unsigned long ms = (unsigned long)(distance * msPerDegree);
  return ms < MEARM_SMOOTH_MIN_MS ? MEARM_SMOOTH_MIN_MS : ms;
}

// Move from the current position to `angle` along an S-curve: speed ramps up
// from zero, peaks halfway, and ramps back down to zero at the target.
void MeArmServo::smoothMoveTo(float angle, unsigned long durationMs){
  float target = clampAngle(angle + offsetAngle);
  if(!positionKnown){
    // Nothing to ease from on the first move after power-up
    setCurrentAngle(target);
    return;
  }
  if(fabsf(target - currentAngle) < 0.05f){
    _moving = false;
    return;
  }
  if(durationMs == 0) durationMs = smoothDuration(angle);
  startAngle = currentAngle;   // starts from wherever it is now, even mid-move
  targetAngle = target;
  moveStart = millis();
  moveDuration = durationMs;
  _moving = true;
}

void MeArmServo::update(){
  if(!_moving) return;
  unsigned long elapsed = millis() - moveStart;
  if(elapsed >= moveDuration){
    _moving = false;
    writeAngle(targetAngle);
    readyTime = millis() + 20; // let the servo settle on the final position
    return;
  }
  float t = (float)elapsed / moveDuration;
  // Quintic smootherstep: 6t^5 - 15t^4 + 10t^3. Velocity and acceleration
  // are both zero at the start and end, so there is no jerk on the arm.
  float s = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
  writeAngle(startAngle + (targetAngle - startAngle) * s);
}

void MeArmServo::stop(){
  _moving = false;
}

bool MeArmServo::moving(){
  return _moving;
}

void MeArmServo::moveByDegrees(float angle){
  setCurrentAngle(currentAngle + angle);
}

void MeArmServo::moveByPercent(float percent){
  setCurrentAngle(currentAngle + (((maxAngle - minAngle) / 100.0 ) * percent));
}

bool MeArmServo::ready(){
  return !_moving && millis() >= readyTime;
}
