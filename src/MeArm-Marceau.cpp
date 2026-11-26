#include "MeArm-Marceau.h"

PCF8591 adc(I2C_DATA, I2C_CLOCK, PCF8591_ADDRESS);
// Number of MeArm commands registered below (Marceau adds its own on top)
Marceau<17> marcel;

MeArm *MeArm::mainInstance;

MeArm::MeArm(){
  mainInstance = this;
  setServoPins(MEARM_DEFAULT_BASE_PIN, MEARM_DEFAULT_RIGHT_PIN, MEARM_DEFAULT_LEFT_PIN, MEARM_DEFAULT_GRIP_PIN);
  //Command name    Handler function   Returns immediately?
  marcel.addCmd("moveJointsTo",     _moveJointsTo,    false);
  marcel.addCmd("moveBaseTo",       _moveBaseTo,      false);
  marcel.addCmd("moveRightTo",      _moveRightTo,      false);
  marcel.addCmd("moveLeftTo",       _moveLeftTo,      false);
  marcel.addCmd("moveGripTo",       _moveGripTo,      false);
  marcel.addCmd("openGrip",         _openGrip,        false);
  marcel.addCmd("closeGrip",        _closeGrip,       false);
  marcel.addCmd("getServoState",    _getServoState,    true);
  marcel.addCmd("version",          _version,          true);
  marcel.addCmd("pause",            _empty,            true);
  marcel.addCmd("resume",           _empty,            true);
  marcel.addCmd("stop",             _stop,             true);
  marcel.addCmd("setLimits",        _setLimits,        true);
  marcel.addCmd("getLimits",        _getLimits,        true);
  marcel.addCmd("resetLimits",      _resetLimits,      true);
#ifdef ESP8266
  marcel.addCmd("updateFirmware",   _updateFirmware,   true);
  marcel.addCmd("updateUI",         _updateUI,         true);
#endif //ESP8266
}

void MeArm::generateAPName(char * name){
  uint8_t mac[6];
  WiFi.softAPmacAddress(mac);
  sprintf(name, "MeArm-%02X%02X", mac[4], mac[5]);
}

void MeArm::begin(){
  static char defaultAPName[11];
  generateAPName(defaultAPName);
  setupServos();
  Serial.begin(230400);
  marcel.enableSerial(Serial);
  marcel.setDefaultAPName(defaultAPName);
  marcel.setHostname("local.mearm.com");
  marcel.enableWifi();
  marcel.reserveDeviceSettings(sizeof(MeArmSavedLimits) + 3);
  marcel.begin();
  loadLimits();   // saved limits, if any, replace the defaults
  base.moveToCentre();
  right.moveToCentre();
  left.moveToCentre();
  grip.moveToCentre();
}

void MeArm::loop(){
  marcel.loop();
  base.update();
  right.update();
  left.update();
  grip.update();
  joystickControl();
  sendDiscovery();
  checkDone();
  updateHandler();
}

void MeArm::joystickControl(){
  uint8_t rawValues[4];
  bool changed = false;
  float scaledValue;
  if(adcNextSample < millis()){
    adcNextSample = millis() + 20;
    adc.readSensors(rawValues);
    for(uint8_t i=0; i< 4; i++){
      scaledValue = (rawValues[i] - 128) / 128.0;
      if(scaledValue > JOYSTICK_THRESHOLD || scaledValue < -JOYSTICK_THRESHOLD){
        // remove the threshold
        scaledValue -= (scaledValue < 0 ? -1 : 1) * JOYSTICK_THRESHOLD;
        //move the servo
        moveServoByPercent(i, scaledValue * 2);
        changed = true;
      }
    }
  }
  if(changed){
    StaticJsonBuffer<500> outBuffer;
    JsonObject& output = outBuffer.createObject();
    JsonObject& msg = output.createNestedObject("msg");
    msg["base"] = base.getCurrentAngle();
    msg["right"] = right.getCurrentAngle();
    msg["left"] = left.getCurrentAngle();
    msg["grip"] = grip.getCurrentAngle();
    marcel.notify("servoChange", output);
  }
}

void MeArm::checkDone(){
  if(base.ready() && right.ready() && left.ready() && grip.ready()){
    marcel.cmdComplete();
  }
}

void MeArm::sendDiscovery(){
  if(nextDiscovery < millis()){
    if(marcel.wifi.online){
      send_discovery_request((uint32_t)WiFi.localIP(), marcel.settings.ap_ssid, "MeArm WiFi");
      nextDiscovery = millis() + 30000;
    }else{
      nextDiscovery = millis() + 1000;
    }
  }
}

void MeArm::moveServoByPercent(uint8_t servo, float percent){
  switch(servo){
    case BASE_SERVO:
      base.moveByPercent(percent);
      break;
    case RIGHT_SERVO:
      right.moveByPercent(percent);
      break;
    case LEFT_SERVO:
      left.moveByPercent(percent);
      break;
    case GRIP_SERVO:
      grip.moveByPercent(percent);
      break;
  }
}

void MeArm::setServoPins(uint8_t _basePin, uint8_t _rightPin, uint8_t _leftPin, uint8_t _gripPin){
  basePin = _basePin;
  rightPin = _rightPin;
  leftPin = _leftPin;
  gripPin = _gripPin;
}

// Smooth move several joints at once. All joints finish together: the
// duration is set by the joint with the furthest to go (or durationMs if
// given). Pass NAN for a joint to leave it where it is.
void MeArm::smoothMoveJointsTo(float baseAngle, float rightAngle, float leftAngle, float gripAngle, unsigned long durationMs){
  if(durationMs == 0){
    if(!isnan(baseAngle))  durationMs = max(durationMs, base.smoothDuration(baseAngle));
    if(!isnan(rightAngle)) durationMs = max(durationMs, right.smoothDuration(rightAngle));
    if(!isnan(leftAngle)) durationMs = max(durationMs, left.smoothDuration(leftAngle));
    if(!isnan(gripAngle))  durationMs = max(durationMs, grip.smoothDuration(gripAngle));
  }
  if(!isnan(baseAngle))  base.smoothMoveTo(baseAngle, durationMs);
  if(!isnan(rightAngle)) right.smoothMoveTo(rightAngle, durationMs);
  if(!isnan(leftAngle)) left.smoothMoveTo(leftAngle, durationMs);
  if(!isnan(gripAngle))  grip.smoothMoveTo(gripAngle, durationMs);
}

void MeArm::stop(){
  base.stop();
  right.stop();
  left.stop();
  grip.stop();
}

void MeArm::getLimits(ArduinoJson::JsonObject &output){
  JsonObject& msg = output.createNestedObject("msg");
  MeArmServo *servos[] = {&base, &right, &left, &grip};
  const char *names[] = {"base", "right", "left", "grip"};
  for(uint8_t i = 0; i < 4; i++){
    JsonObject& j = msg.createNestedObject(names[i]);
    j["min"] = servos[i]->getMinAngle();
    j["max"] = servos[i]->getMaxAngle();
    j["travelMin"] = servos[i]->getTravelMin();
    j["travelMax"] = servos[i]->getTravelMax();
  }
}

bool MeArm::saveLimits(){
  MeArmSavedLimits saved;
  MeArmServo *servos[] = {&base, &right, &left, &grip};
  for(uint8_t i = 0; i < 4; i++){
    saved.limits[i][0] = servos[i]->getMinAngle();
    saved.limits[i][1] = servos[i]->getMaxAngle();
  }
  return marcel.saveDeviceSettings(&saved, sizeof(saved), MEARM_LIMITS_VERSION);
}

bool MeArm::loadLimits(){
  MeArmSavedLimits saved;
  if(!marcel.loadDeviceSettings(&saved, sizeof(saved), MEARM_LIMITS_VERSION)) return false;
  MeArmServo *servos[] = {&base, &right, &left, &grip};
  for(uint8_t i = 0; i < 4; i++){
    float mn = saved.limits[i][0], mx = saved.limits[i][1];
    if(isnan(mn) || isnan(mx)) continue;   // damaged entry: keep the default
    servos[i]->setLimits(mn, mx);
  }
  return true;
}

void MeArm::resetLimits(){
  MeArmServo *servos[] = {&base, &right, &left, &grip};
  for(uint8_t i = 0; i < 4; i++){
    servos[i]->resetLimits();   // the hard limits from setupServos()
  }
  saveLimits();
}

// Each joint: begin(pin, min, max, offset)
//   min, max  servo angles (0-180, i.e. 500-2500us) the joint may never go
//             beyond. They're also the default limits: setLimits, the UI's
//             Adjust limits and saved limits can only narrow the joint within
//             them, and resetLimits / Defaults go back to them.
//   offset    the servo angle the joint calls 0. Commands use angles relative
//             to it: commanded angle = servo angle - offset.
// setReversed(true) before begin() makes a joint's commanded angles run the
// other way: commanded angle = offset - servo angle.
void MeArm::setupServos(){
  base.begin(basePin, 0, 180, 90);
  // Reversed: servo 180 (parallel to the ground) is 0, servo 30 is 150
  right.setReversed(true);
  right.begin(rightPin, 30, 180, 180);
  left.begin(leftPin, 20, 135, 20);
  grip.begin(gripPin, 0, 180, 0);
}

void MeArm::getServoState(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  JsonObject& msg = output.createNestedObject("msg");
  msg["base"] = base.getCurrentAngle();
  msg["right"] = right.getCurrentAngle();
  msg["left"] = left.getCurrentAngle();
  msg["grip"] = grip.getCurrentAngle();
}

#ifdef ESP8266
void MeArm::updateFirmware(){
  if(marcel.wifi.online){
    WiFiClient client;
    ESPhttpUpdate.rebootOnUpdate(true);
    if(ESPhttpUpdate.update(client, "http://downloads.mearm.com/MeArm/WiFi/v2/firmware-latest.bin", "") != HTTP_UPDATE_OK){
      Serial.println(ESPhttpUpdate.getLastErrorString());
    }
  }
}

void MeArm::updateUI(){
  if(marcel.wifi.online){
    WiFiClient client;
    ESPhttpUpdate.rebootOnUpdate(false);
    if(ESPhttpUpdate.updateFS(client, "http://downloads.mearm.com/MeArm/WiFi/v2/ui-latest.bin", "") != HTTP_UPDATE_OK){
      Serial.println(ESPhttpUpdate.getLastErrorString());
    }
  }
}

void MeArm::updateHandler(){
  if(_updateFWflag){
    _updateFWflag = false;
    updateFirmware();
  }
  if(_updateUIflag){
    _updateUIflag = false;
    updateUI();
  }
}
#endif //ESP8266

// arg: {"base":90, "right":45, "left":60, "grip":0, "duration":1000}
// Every key is optional; joints that are left out don't move. Without
// "duration" the time comes from the joint with the furthest to travel.
static float _jointArg(ArduinoJson::JsonObject &arg, const char *key){
  return arg.containsKey(key) ? arg[key].as<float>() : NAN;
}

static void _moveJointsTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  if(!input["arg"].is<ArduinoJson::JsonObject&>()) return;
  ArduinoJson::JsonObject &arg = input["arg"].asObject();
  unsigned long duration = arg.containsKey("duration") ? arg["duration"].as<unsigned long>() : 0;
  MeArm::mainInstance->smoothMoveJointsTo(
    _jointArg(arg, "base"), _jointArg(arg, "right"),
    _jointArg(arg, "left"), _jointArg(arg, "grip"), duration);
}

// Open and close go to the grip's current limits, so they follow setupServos()
// and anything set with setLimits / the UI's Adjust limits: open is the low
// end, close the high end.
static void _openGrip(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->grip.smoothMoveTo(MeArm::mainInstance->grip.getMinAngle());
}

static void _closeGrip(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->grip.smoothMoveTo(MeArm::mainInstance->grip.getMaxAngle());
}

static void _moveBaseTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->base.smoothMoveTo(atoi(input["arg"].asString()));
}

static void _moveRightTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->right.smoothMoveTo(atoi(input["arg"].asString()));
}

static void _moveLeftTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->left.smoothMoveTo(atoi(input["arg"].asString()));
}

static void _moveGripTo(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->grip.smoothMoveTo(atoi(input["arg"].asString()));
}

static void _getServoState(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->getServoState(input, output);
}

static void _version(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  output["msg"] = VERSION;
}

static void _empty(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
}

static void _stop(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->stop();
}

// arg: {"base":{"min":-80,"max":80}, "grip":{"max":70}}
// Any joint, and either end, can be left out to keep its current limit.
// The limits are saved to EEPROM unless the arg has "save":false (used by the
// UI while the end points are being adjusted, until they're confirmed).
// Replies with all the limits now in force (as getLimits).
static void _setLimits(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm *m = MeArm::mainInstance;
  if(input["arg"].is<ArduinoJson::JsonObject&>()){
    ArduinoJson::JsonObject &arg = input["arg"].asObject();
    MeArmServo *servos[] = {&m->base, &m->right, &m->left, &m->grip};
    const char *names[] = {"base", "right", "left", "grip"};
    for(uint8_t i = 0; i < 4; i++){
      if(!arg[names[i]].is<ArduinoJson::JsonObject&>()) continue;
      ArduinoJson::JsonObject &lim = arg[names[i]].asObject();
      float mn = lim.containsKey("min") ? lim["min"].as<float>() : servos[i]->getMinAngle();
      float mx = lim.containsKey("max") ? lim["max"].as<float>() : servos[i]->getMaxAngle();
      servos[i]->setLimits(mn, mx);
    }
    bool save = arg.containsKey("save") ? arg["save"].as<bool>() : true;
    if(save) m->saveLimits();
  }
  m->getLimits(output);
}

static void _getLimits(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->getLimits(output);
}

static void _resetLimits(ArduinoJson::JsonObject &input, ArduinoJson::JsonObject &output){
  MeArm::mainInstance->resetLimits();
  MeArm::mainInstance->getLimits(output);
}

#ifdef ESP8266
static void _updateFirmware(ArduinoJson::JsonObject &inJson, ArduinoJson::JsonObject &outJson){
  MeArm::mainInstance->_updateFWflag = true;
}
static void _updateUI(ArduinoJson::JsonObject &inJson, ArduinoJson::JsonObject &outJson){
  MeArm::mainInstance->_updateUIflag = true;
}
#endif //ESP8266
