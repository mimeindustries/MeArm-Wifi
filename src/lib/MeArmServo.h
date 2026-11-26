#ifndef MeArmServo_h
#define MeArmServo_h

#include <Servo.h>

// Pulse range for 0..180 degrees. ESP8266 core 3.x defaults attach(pin) to
// 1000-2000us (about 110 degrees of travel on SG90/MG90S-type servos), so
// set it explicitly. 500-2500us is the usual full range for these servos:
// roughly 11us per degree, centred on 1500us. If a joint still falls short,
// or buzzes against its end stop, adjust it per servo with setPulseRange().
#ifndef MEARM_SERVO_MIN_US
#define MEARM_SERVO_MIN_US 500
#endif
#ifndef MEARM_SERVO_MAX_US
#define MEARM_SERVO_MAX_US 2500
#endif

// Default speed for smooth moves when no duration is given. This is the
// average speed; an S-curve peaks at ~1.9x this in the middle of the move.
#define MEARM_SMOOTH_MS_PER_DEGREE 8
#define MEARM_SMOOTH_MIN_MS 150

// Stop sending pulses to a servo that hasn't moved for this long, so it
// stops buzzing and drawing current. It starts again on the next move.
// 0 disables the timeout.
#ifndef MEARM_SERVO_IDLE_TIMEOUT_MS
#define MEARM_SERVO_IDLE_TIMEOUT_MS 5000
#endif

class MeArmServo{
  public:
    MeArmServo();
    void begin(int pin, int minAngle, int maxAngle, int offset,
               int minUs = MEARM_SERVO_MIN_US, int maxUs = MEARM_SERVO_MAX_US);
    void setPulseRange(int minUs, int maxUs);         // pulse widths for 0 and 180 degrees
    void setReversed(bool reversed);                 // commanded angles run the other way:
                                                     // commanded = offset - servo angle
    void setLimits(float minAngle, float maxAngle);  // joint limits, in the same angles as moveToAngle(),
                                                     // kept within the hard limits from begin()
    void resetLimits();                              // back to the hard limits from begin()
    float getMinAngle();
    float getMaxAngle();
    float getTravelMin();   // the hard limits from begin(), in the same angles:
    float getTravelMax();   // setLimits() can narrow the joint within these, never beyond
    void moveToCentre();
    void moveToAngle(float);                         // jump straight to the angle
    void smoothMoveTo(float angle, unsigned long durationMs = 0); // S-curve move; 0 = auto duration
    unsigned long smoothDuration(float angle);       // auto duration for a move to this angle
    void moveByDegrees(float);
    void moveByPercent(float);
    void stop();                                     // stop a smooth move where it is
    void update();                                   // call every loop to run smooth moves
    void setSpeed(float msPerDegree);
    void setIdleTimeout(unsigned long ms);           // 0 = never time out
    bool attached();                                 // false while timed out
    bool ready();
    bool moving();
    float getCurrentAngle();
  private:
    Servo _servo;
    float currentAngle;
    float maxAngle;
    float minAngle;
    int offsetAngle;
    bool reversed;
    float toServo(float commanded);    // commanded angle -> servo angle (0..180)
    float toCommanded(float servo);    // servo angle -> commanded angle
    float hardMin, hardMax;  // servo angles 0..180 from begin(): the most the joint may ever move
    bool limitsSet;          // setLimits() called, possibly before begin()
    float limitMin, limitMax; // the limits given to setLimits()
    void applyLimits(float rawMin, float rawMax);
    unsigned long readyTime;
    bool positionKnown;
    // smooth move state
    bool _moving;
    float startAngle;
    float targetAngle;
    unsigned long moveStart;
    unsigned long moveDuration;
    float msPerDegree;
    int lastPulse;
    int _pin;
    int _minUs;
    int _maxUs;
    // idle timeout state
    bool _attached;
    unsigned long lastActive;
    unsigned long idleTimeout;
    void wake();
    float clampAngle(float);
    void writeAngle(float);
    void setCurrentAngle(float);
};
    
#endif
