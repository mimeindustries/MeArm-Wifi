#ifndef MeArmServo_h
#define MeArmServo_h

#include <Servo.h>

// Pulse range for 0..180 degrees. ESP8266 core 2.x used 544-2400us for
// attach(pin); core 3.x changed the default to 1000-2000us, which cuts the
// MeArm's travel, so set it explicitly.
#define MEARM_SERVO_MIN_US 544
#define MEARM_SERVO_MAX_US 2400

// Default speed for smooth moves when no duration is given. This is the
// average speed; an S-curve peaks at ~1.9x this in the middle of the move.
#define MEARM_SMOOTH_MS_PER_DEGREE 8
#define MEARM_SMOOTH_MIN_MS 150

class MeArmServo{
  public:
    MeArmServo();
    void begin(int, int, int, int);
    void moveToCentre();
    void moveToAngle(float);                         // jump straight to the angle
    void smoothMoveTo(float angle, unsigned long durationMs = 0); // S-curve move; 0 = auto duration
    unsigned long smoothDuration(float angle);       // auto duration for a move to this angle
    void moveByDegrees(float);
    void moveByPercent(float);
    void stop();                                     // stop a smooth move where it is
    void update();                                   // call every loop to run smooth moves
    void setSpeed(float msPerDegree);
    bool ready();
    bool moving();
    float getCurrentAngle();
  private:
    Servo _servo;
    float currentAngle;
    int maxAngle;
    int minAngle;
    int offsetAngle;
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
    float clampAngle(float);
    void writeAngle(float);
    void setCurrentAngle(float);
};
    
#endif
