#include "MeArmServo.h"

MeArmServo::MeArmServo(){
  currentAngle = 0;
  readyTime = 0;
  positionKnown = false;
  _moving = false;
  msPerDegree = MEARM_SMOOTH_MS_PER_DEGREE;
  lastPulse = -1;
  _pin = -1;
  _minUs = MEARM_SERVO_MIN_US;
  _maxUs = MEARM_SERVO_MAX_US;
  _attached = false;
  lastActive = 0;
  idleTimeout = MEARM_SERVO_IDLE_TIMEOUT_MS;
  offsetAngle = 0;
  reversed = false;
  minAngle = 0;
  maxAngle = 180;
  hardMin = 0;
  hardMax = 180;
  limitsSet = false;
}

void MeArmServo::begin(int pin, int min, int max, int offset, int minUs, int maxUs){
  _pin = pin;
  _minUs = minUs;
  _maxUs = maxUs;
  _servo.attach(pin, minUs, maxUs);
  _attached = true;
  lastActive = millis();
  readyTime = 0;
  offsetAngle = offset;
  // min/max are the joint's hard limits: nothing can move it beyond them
  if(min > max){ int t = min; min = max; max = t; }
  hardMin = min < 0 ? 0 : min;
  hardMax = max > 180 ? 180 : max;
  if(limitsSet){
    // setLimits() was called before begin(): keep those limits
    applyLimits(toServo(limitMin), toServo(limitMax));
  }else{
    applyLimits(min, max);
  }
}

// Commanded angles (what the move commands use) are relative to the offset:
//   normal:   commanded = servo angle - offset
//   reversed: commanded = offset - servo angle
float MeArmServo::toServo(float commanded){
  return reversed ? offsetAngle - commanded : commanded + offsetAngle;
}

float MeArmServo::toCommanded(float servo){
  return reversed ? offsetAngle - servo : servo - offsetAngle;
}

void MeArmServo::setReversed(bool r){
  reversed = r;
}

// rawMin/rawMax include the offset, i.e. they are servo angles 0..180.
// They're cut back to the hard limits from begin().
void MeArmServo::applyLimits(float rawMin, float rawMax){
  if(rawMin > rawMax){ float t = rawMin; rawMin = rawMax; rawMax = t; }
  if(rawMin < hardMin) rawMin = hardMin;
  if(rawMax > hardMax) rawMax = hardMax;
  if(rawMin > rawMax) rawMin = rawMax;   // both were outside on the same side
  minAngle = rawMin;
  maxAngle = rawMax;
}

void MeArmServo::setLimits(float minA, float maxA){
  limitsSet = true;
  limitMin = minA;
  limitMax = maxA;
  applyLimits(toServo(minA), toServo(maxA));
  // If the joint is now outside its limits, ease it back inside
  if(positionKnown){
    if(_moving) targetAngle = clampAngle(targetAngle);
    float inside = clampAngle(currentAngle);
    if(inside != currentAngle) smoothMoveTo(toCommanded(inside));
  }
}

float MeArmServo::getMinAngle(){
  return reversed ? toCommanded(maxAngle) : toCommanded(minAngle);
}

float MeArmServo::getMaxAngle(){
  return reversed ? toCommanded(minAngle) : toCommanded(maxAngle);
}

void MeArmServo::resetLimits(){
  setLimits(getTravelMin(), getTravelMax());
}

float MeArmServo::getTravelMin(){
  return reversed ? toCommanded(hardMax) : toCommanded(hardMin);
}

float MeArmServo::getTravelMax(){
  return reversed ? toCommanded(hardMin) : toCommanded(hardMax);
}

float MeArmServo::clampAngle(float angle){
  if(angle < minAngle) return minAngle;
  if(angle > maxAngle) return maxAngle;
  return angle;
}

void MeArmServo::setPulseRange(int minUs, int maxUs){
  _minUs = minUs;
  _maxUs = maxUs;
  if(_pin >= 0 && _attached) _servo.attach(_pin, minUs, maxUs); // updates the Servo's own clamp
  lastPulse = -1;
  if(positionKnown) writeAngle(currentAngle);
}

// Write as microseconds rather than whole degrees so the slow ends of an
// S-curve move in small steps instead of visible 1 degree jumps.
void MeArmServo::writeAngle(float angle){
  currentAngle = angle;
  int pulse = _minUs + (int)(angle * (_maxUs - _minUs) / 180.0f + 0.5f);
  lastActive = millis();
  if(!_attached){
    // Timed out: start the pulses again, straight at the new position
    _servo.attach(_pin, _minUs, _maxUs, pulse);
    _attached = true;
    lastPulse = pulse;
    return;
  }
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
  return toCommanded(currentAngle);
}

void MeArmServo::moveToAngle(float angle){
  setCurrentAngle(toServo(angle));
}

void MeArmServo::setSpeed(float ms){
  if(ms > 0) msPerDegree = ms;
}

unsigned long MeArmServo::smoothDuration(float angle){
  float distance = fabsf(clampAngle(toServo(angle)) - currentAngle);
  unsigned long ms = (unsigned long)(distance * msPerDegree);
  return ms < MEARM_SMOOTH_MIN_MS ? MEARM_SMOOTH_MIN_MS : ms;
}

// Move from the current position to `angle` along an S-curve: speed ramps up
// from zero, peaks halfway, and ramps back down to zero at the target.
void MeArmServo::smoothMoveTo(float angle, unsigned long durationMs){
  float target = clampAngle(toServo(angle));
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

void MeArmServo::setIdleTimeout(unsigned long ms){
  idleTimeout = ms;
  lastActive = millis();
}

bool MeArmServo::attached(){
  return _attached;
}

void MeArmServo::update(){
  if(!_moving){
    // Idle timeout: stop pulsing a servo that hasn't been moved for a while.
    // Note detach() blocks for one 20ms servo frame on ESP8266.
    if(_attached && idleTimeout && _pin >= 0 && millis() - lastActive >= idleTimeout){
      _servo.detach();
      _attached = false;
    }
    return;
  }
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
