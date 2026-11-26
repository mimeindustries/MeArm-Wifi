#ifndef __MeArm_Marceau_h__
#define __MeArm_Marceau_h__

#include "lib/MeArmServo.h"
#include "lib/pcf8591.h"
#include "lib/Discovery.h"
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <Marceau.h>

#define MEARM_DEFAULT_BASE_PIN 16
#define MEARM_DEFAULT_RIGHT_PIN 12
#define MEARM_DEFAULT_LEFT_PIN 14
#define MEARM_DEFAULT_GRIP_PIN 13

#define I2C_DATA 0u
#define I2C_CLOCK 2u
#define PCF8591_ADDRESS B1001000

#define JOYSTICK_THRESHOLD 0.3f

#define BASE_SERVO 0
#define RIGHT_SERVO 1
#define GRIP_SERVO 2
#define LEFT_SERVO 3

#define VERSION "2.0.0"

// Joint limits saved in Marceau's device settings area. Raise the version
// if the layout of MeArmSavedLimits changes.
#define MEARM_LIMITS_VERSION 1
struct MeArmSavedLimits {
  float limits[4][2];   // base, right, left, grip: {min, max}
};

static void _moveJointsTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _openGrip(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _closeGrip(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _moveBaseTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _moveRightTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _moveLeftTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _moveGripTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _getServoState(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _version(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _empty(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _stop(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _setLimits(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _getLimits(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
static void _resetLimits(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);

#ifdef ESP8266
static void _updateFirmware(ArduinoJson::JsonObject &inJson, ArduinoJson::JsonObject &outJson);
static void _updateUI(ArduinoJson::JsonObject &inJson, ArduinoJson::JsonObject &outJson);
#endif //ESP8266

class MeArm {
  public:
    MeArm();
    void begin();
    void loop();
    void setServoPins(uint8_t, uint8_t, uint8_t, uint8_t);
    static MeArm getMeArmInstance();
    MeArmServo base;
    MeArmServo right;
    MeArmServo left;
    MeArmServo grip;
    void getServoState(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output);
    static MeArm * mainInstance;
    void smoothMoveJointsTo(float base, float right, float left, float grip, unsigned long durationMs = 0);
    void stop();
    void getLimits(ArduinoJson::JsonObject &output);
    bool saveLimits();      // store the current joint limits in EEPROM
    bool loadLimits();      // apply saved limits; false if none are saved
    void resetLimits();     // back to the default limits, and save those
    void updateFirmware();
    void updateHandler();
    void updateUI();
    boolean _updateFWflag = false;
    boolean _updateUIflag = false;
  private:
    long adcNextSample = 0;
    int basePin;
    int rightPin;
    int leftPin;
    int gripPin;
    long nextDiscovery = 0;
    void joystickControl();
    void checkDone();
    void setupServos();
    void moveServoByPercent(uint8_t servo, float percent);
    void generateAPName(char * name);
    void sendDiscovery();
};
    
#endif