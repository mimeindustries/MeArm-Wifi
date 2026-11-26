MeArm WiFi
==========

Firmware library for the **MeArm WiFi** board: an ESP8266 that drives the four MeArm servos and serves a
browser-based programming environment over its own WiFi network. You connect a phone, tablet or laptop to the
arm's WiFi, open the page it serves, and control or program the arm from there. No app or install is needed.

It is built on [Marceau](https://github.com/MeArm/marceau-artiemax), which provides the WiFi access point, the
web server, the WebSocket connection and the JSON command interpreter.

> Looking for the code for the original MeArm Brains board? See
> [mearm-brains-arduino](https://github.com/mimeindustries/mearm-brains-arduino).

## What it does

- **WiFi access point.** On boot the board creates an open network named `MeArm-XXXX`, where `XXXX` comes from
  the board's MAC address. A password can be set with
  `setConfig` (`ap_pass`). The UI is at `http://192.168.4.1/`.
- **Web UI.** The main page (`index.html`, CSS and JS) is compiled into the firmware from `web.h`. The
  programming apps are stored in the board's flash filesystem and served under `/a/`:
  - Blockly (drag-and-drop blocks)
  - JavaScript
  - Python
  - Snap!
- **Control API.** JSON commands arrive over a WebSocket on port **8899** or over serial at **230400 baud**:
  - `moveJointsTo`, `moveBaseTo`, `moveRightTo`, `moveLeftTo`, `moveGripTo`
  - `openGrip`, `closeGrip`, `getServoState`, `version`, `pause`, `resume`, `stop`, `setLimits`, `getLimits`, `resetLimits`
  - Marceau's built-in commands: `ping`, `uptime`, `freeHeap`, `getConfig`, `setConfig`, `resetConfig`,
    `startWifiScan`

  Servo moves report back to the client when they finish.
- **Joystick control.** The two on-board joysticks are read through a PCF8591 ADC (I²C on GPIO0/GPIO2). They
  move the servos directly, and each move is sent to connected clients as a `servoChange` notification.
- **Discovery.** When the board is connected to a network as a client, it announces its IP address and name to
  `local.mearm.com` every 30 seconds.
- **Over-the-air updates.** The `updateFirmware` and `updateUI` commands download new firmware or a new
  filesystem image (see the notes below).

### Default servo pins

| Servo | GPIO |
|-------|------|
| Base  | 16   |
| Right | 12   |
| Left  | 14   |
| Grip  | 13   |

Change them with `meArm.setServoPins(base, right, left, grip)` before `meArm.begin()`.

### Servo calibration

Angles from 0° to 180° map onto pulse widths of **500–2500 µs**, about 11 µs per degree, centred on 1500 µs.
This is the usual full range for the SG90/MG90S-type servos on the MeArm. Servos vary, so:

- **A joint falls short of where it should go:** widen its range, e.g. to 450–2550 µs.
- **A joint buzzes or strains at the end of its travel:** narrow its range, e.g. to 600–2400 µs.

Set the range per servo after `meArm.begin()`:

```cpp
meArm.begin();
meArm.grip.setPulseRange(600, 2400);
```

Or change the default for every servo by defining `MEARM_SERVO_MIN_US` / `MEARM_SERVO_MAX_US` in
`lib/MeArmServo.h`.

### Joint limits

Each joint has a range it won't move past. Moves, `moveJointsTo` and the joysticks are all clamped to it.
The limits use the same angles as the move commands.

**Hard limits and offsets are set in `setupServos()`** in `src/MeArm-Marceau.cpp`, with
`begin(pin, min, max, offset)`:

- **`min`, `max`:** servo angles (0–180°, i.e. 500–2500 µs) the joint may never go beyond. Limits set any other
  way can only narrow the joint within them. They're also the defaults that `resetLimits` and the UI's Defaults
  button go back to.
- **`offset`:** the servo angle the joint calls 0°. Commands use angles relative to it: commanded angle = servo
  angle − offset. With `base.begin(basePin, 0, 180, 90)`, servo angles 0–180° are commanded as −90° to 90°.
- **Reversed joints:** call `setReversed(true)` before `begin()` to make a joint's commanded angles run the other
  way: commanded angle = offset − servo angle. The Right joint uses this, so that servo 180° (the arm parallel
  to the ground) is 0° and servo 30° is 150°:

  ```cpp
  right.setReversed(true);
  right.begin(rightPin, 30, 180, 180);   // commanded 0° to 150°
  ```

- **To choose an offset,** move the joint to the position you want to call 0° (e.g. with the UI's sliders),
  add the joint's current offset to the angle shown, and use that as the new offset.
- **Changing an offset changes what every saved program does,** because the same commanded angle now means a
  different servo angle. Limits saved on the arm are stored in commanded angles too, so press Defaults (or send
  `resetLimits`) after changing offsets or hard limits.

**From a client,** use `setLimits`. Any joint, and either end, can be left out to keep its current value. The
reply lists all the limits now in force. `getLimits` returns them without changing anything. Both replies
also give each joint's `travelMin` / `travelMax`: its hard limits from `setupServos()`, in commanded angles,
which is how far a limit can be set. Add `"save":false` to `setLimits` to try limits out without saving them.

```json
{"cmd":"setLimits", "arg":{"base":{"min":-80, "max":80}, "grip":{"max":70}}}
```

**From a sketch,** call `meArm.base.setLimits(-80, 80)`. It works before or after `meArm.begin()`. Read the limits
back with `getMinAngle()` / `getMaxAngle()`.

- If a joint is outside its new limits, it eases back inside them.
- A reversed pair is swapped, and values beyond the hard limits are cut back to them.
- **Saved across restarts.** `setLimits` saves the limits to EEPROM, and they're loaded again at boot.
  `resetLimits` puts back the hard limits from `setupServos()` and saves those. Saved limits are cut back to
  the hard limits when they're loaded, so a narrower `setupServos()` range always wins.
- **From a sketch,** `setLimits()` doesn't save on its own: call `meArm.saveLimits()` afterwards to keep the
  change. Saved limits are loaded inside `meArm.begin()`, so they replace any limits set before it. Limits set
  after `begin()` override the saved ones until the next restart.
- **Where they're stored:** Marceau's device settings area in EEPROM, after Marceau's own settings. Resetting
  Marceau's config (`resetConfig`) doesn't clear them.
- **`openGrip` / `closeGrip`** move the grip to its current minimum / maximum limit, so they follow the grip's
  limits.
- **In the UI,** the joint sliders use the current limits, and the Adjust limits panel lets you drag each end
  within the hard limits.

### Servo idle timeout

A servo that hasn't moved for **5 seconds** stops receiving pulses (it is detached), so it stops buzzing and
drawing current. The next move starts the pulses again: at the position the servo was holding, then on to the
new one. While a servo is timed out, nothing holds it, so a joint carrying load may sag a little.

- **Change it per servo:** `meArm.left.setIdleTimeout(10000);`
- **Turn it off:** `setIdleTimeout(0)`
- **Change the default for every servo:** define `MEARM_SERVO_IDLE_TIMEOUT_MS` in `lib/MeArmServo.h`.

### Smooth moves

The move commands (`moveBaseTo`, `moveRightTo`, `moveLeftTo`, `moveGripTo`, `openGrip`, `closeGrip` and
`moveJointsTo`) don't jump straight to the new angle. They move from the current position along an **S-curve**
(quintic "smootherstep"): speed starts at zero, peaks halfway, and drops back to zero at the target, so the arm
starts and stops without a jerk. A move given while another is running starts from wherever the arm is at that
moment. The command reports complete when the move finishes.

- **Speed.** With no duration given, a move takes 8 ms per degree (`MEARM_SMOOTH_MS_PER_DEGREE` in
  `lib/MeArmServo.h`), and at least 150 ms. Peak speed is about 1.9 times the average. Change it per servo
  with `meArm.base.setSpeed(msPerDegree)`.
- **`moveJointsTo`** moves several joints together, and they all arrive at the same time:

  ```json
  {"cmd":"moveJointsTo", "arg":{"base":90, "right":45, "left":60, "grip":0, "duration":1000}}
  ```

  Every key is optional. Joints you leave out stay where they are. Without `duration`, the joint with the
  furthest to go sets the time.
- **`stop`** halts a smooth move where it is.
- **Joysticks** still move the servos directly. Using them cancels any smooth move that's running.
- **From a sketch,** call `meArm.base.smoothMoveTo(angle, durationMs)` or
  `meArm.smoothMoveJointsTo(base, right, left, grip, durationMs)`. Use `NAN` for a joint you don't want to
  move, and `0` for an automatic duration. `moveToAngle()` still jumps straight to the angle. `meArm.loop()`
  must run often, because it steps the moves.

## Requirements

- **Arduino IDE 2.x**
- **ESP8266 board package 3.x**: in Boards Manager, install "esp8266 by ESP8266 Community". Tested with 3.1.2.
- **Libraries** (install with Library Manager, or use the copies in this repo's `libraries/` folder):
  - **Marceau** 0.1.0 or later, with LittleFS on ESP8266 and the device settings area
    (`reserveDeviceSettings()`), which stores the joint limits
  - **ESP Async WebServer** by ESP32Async (3.6.0 or later)
  - **ESP Async TCP** by ESP32Async (2.0.0 or later). This is the ESP8266 TCP library. Don't use the older
    `me-no-dev` version.
  - **Async TCP** by ESP32Async. Marceau lists it as a dependency; it is only used on ESP32.

## Setup

### 1. Build and upload the firmware

1. Open the example: **File → Examples → MeArm → MeArm-WiFi**. Then **File → Save As** to a normal sketch
   folder, so the sketch has its `data` folder next to it.
2. Select your ESP8266 board and port.
3. Set **Tools → Flash Size** to a layout with a filesystem partition big enough for the apps (about 860 KB),
   e.g. `4MB (FS:2MB OTA:~1019KB)`.
4. Upload as normal.

### 2. Upload the app files to the filesystem (LittleFS)

The Blockly, JavaScript, Python and Snap! apps live in the sketch's `data` folder. They go onto the board's
flash in a separate upload. Without this step the UI still loads, but no apps appear and `/a/apps.json`
returns 404.

1. Install the LittleFS upload plugin (one time only):
   - Download the `.vsix` from
     [arduino-littlefs-upload releases](https://github.com/earlephilhower/arduino-littlefs-upload/releases).
   - Put it in `C:\Users\<you>\.arduinoIDE\plugins\` (on Mac or Linux, `~/.arduinoIDE/plugins/`). Create the
     folder if it doesn't exist.
   - Restart the IDE.
2. Close the Serial Monitor, because it holds the port.
3. Press **Ctrl+Shift+P** (Cmd+Shift+P on Mac) and run **"Upload LittleFS to Pico/ESP8266/ESP32"**.

Use the **same Flash Size setting** for the sketch and the filesystem upload, or the filesystem won't mount.
You only need to repeat this step when the files in `data/` change.

### 3. Check it

Open the Serial Monitor at 230400 baud and reset the board. Marceau prints:

```
[[[File system info]]]
Mounted:          yes
Total space:      ... bytes
Total space used: ... bytes
```

`Mounted: NO` or a used size near 0 means the data upload is missing, or was made with a different Flash Size.
Join the `MeArm-XXXX` network and open `http://192.168.4.1/`. The apps should be listed, and
`http://192.168.4.1/a/apps.json` should load.

## Building the UI files

The example sketch already contains a built UI, so you only need this when you change the UI. The UI is
built from [mime-app-libs](https://github.com/mimeindustries/mime-app-libs), which needs
[Node.js](https://nodejs.org/) and npm.

```bash
git clone https://github.com/mimeindustries/mime-app-libs
```

```bash
cd mime-app-libs
```

```bash
npm install
```

```bash
npm run build:mearm-wifi
```

The output goes to `dist/mearm-wifi/`:

- **`web.h`** holds the main page, favicon, `base.css/js` and the built-in UI as gzipped PROGMEM arrays.
  Copy it over the sketch's `web.h`. It is compiled into the firmware, so rebuild and upload the sketch.
- **`data/`** holds the apps (Blockly, JavaScript, Python, Snap!), the code editor, `apps.json` and media.
  Copy it over the sketch's `data/` folder, then upload it again with the LittleFS plugin (step 2 above).

You can also write straight into the sketch folder. Existing files are overwritten and other files are left
alone:

```bash
npm run build:mearm-wifi -- --out <path-to>/MeArm-WiFi
```

The MeArm-specific parts (`index.html`, `apps.json`, and each app's start-up code) are in `src/mearm-wifi/`
in mime-app-libs. Which source files go into each output file is set at the top of `tools/build-ui.js`.

For an over-the-air UI update, build `ui-latest.bin` from the new `data/` folder with `mklittlefs` (see
[Known issues](#known-issues)).

## Updates and fixes (2026)

These changes bring the library up to date with ESP8266 core 3.x and the current Marceau.

### Marceau

- **Switched to the ESP32Async libraries.** The web server and TCP code now come from ESP32Async's
  ESP Async WebServer, ESP Async TCP and Async TCP, instead of the bundled `me-no-dev` copies.
- **ESP8266 build.** It needs the **ESP Async TCP** library. Without it the build fails with
  `ESPAsyncTCP.h: No such file or directory`.
- **LittleFS on ESP8266.** Marceau used to mount SPIFFS. The IDE 2.x upload plugin only builds LittleFS images,
  and SPIFFS is deprecated in core 3.x, so the apps were never found. It now mounts LittleFS with auto-format
  turned off, so a failed mount can't erase the flash. The serial log reports whether the mount succeeded.
  ESP32 builds still use SPIFFS.

### MeArm-Wifi-Lib

- **Discovery rewritten.** The old discovery code used `espconn.h` from the legacy ESP8266 SDK. It clashes with
  lwIP2 in core 3.x (`conflicting declaration 'typedef void (* dns_found_callback)…'`). Discovery now uses
  ESPAsyncTCP's `AsyncClient` and stays non-blocking. It skips a send while the previous one is still running,
  and drops a request that has been stuck for 20 seconds.
- **OTA calls updated for core 3.x.** `ESPhttpUpdate.update(url)` and `updateSpiffs(url)` were removed from the
  core. The library now calls `update(client, url)` and `updateFS(client, url)`, and the update files moved to
  `downloads.mearm.com/MeArm/WiFi/v2/`.
- **Header guard fixed.** `__MeArm-Marceau_h__` isn't a valid macro name because of the hyphen. It is now
  `__MeArm_Marceau_h__`.
- **Joints renamed: upper → left, lower → right.** The commands are `moveLeftTo` / `moveRightTo` (were `moveUpperTo` / `moveLowerTo`). The
  JSON keys in `moveJointsTo`, `getServoState` and `servoChange` are `left` / `right`, and in a sketch
  they are `meArm.left` / `meArm.right`. The old names are gone, so the UI must be built from a matching
  [mime-app-libs](https://github.com/mimeindustries/mime-app-libs).
- **Smooth S-curve moves.** The move commands now ease in and out instead of jumping (see
  [Smooth moves](#smooth-moves)). `moveJointsTo` was an empty stub; it now moves several joints at once.
  `stop` now halts motion.
- **Servo range restored on core 3.x.** Core 3.x changed `Servo::attach(pin)` to map 0–180° onto
  1000–2000 µs, where core 2.x used 544–2400 µs. That cut the arm's travel to about 110°. The servos are now
  attached with 500–2500 µs, the usual full range for SG90/MG90S-type servos, and positions are written in
  microseconds for finer steps.
- **Servo idle timeout.** Servos stop being driven after 5 seconds without a move, and start again on the
  next one (see [Servo idle timeout](#servo-idle-timeout)).
- **Settable joint limits.** Set them with `setLimits` / `getLimits` or `setLimits()` in a sketch (see
  [Joint limits](#joint-limits)). They're saved in EEPROM, and `resetLimits` restores the defaults.
  Marceau's command table was raised to 17 to make room.
- **PCF8591 warning fixed.** `Wire.requestFrom()` was ambiguous on core 3.x. The arguments are now cast to
  `uint8_t`.

## Known issues

- **The arm only runs its own access point.** Joining an existing WiFi network is currently commented out in
  Marceau's `MarceauWifi::setupWifi()`, so discovery via `local.mearm.com` doesn't run.
- **Over-the-air update files.** `updateFirmware` and `updateUI` now fetch
  `http://downloads.mearm.com/MeArm/WiFi/v2/firmware-latest.bin` and `.../v2/ui-latest.bin`. Build
  `ui-latest.bin` as a **LittleFS** image, not SPIFFS, or the board won't be able to read the filesystem after
  an update:

  ```bash
  mklittlefs -c data -p 256 -b 8192 -s 2072576 ui-latest.bin
  ```

  `2072576` is the FS size for the 4MB / FS:2MB layout. `mklittlefs` comes with the ESP8266 core under
  `Arduino15/packages/esp8266/tools/mklittlefs/`. The plugin's output panel shows the exact command and sizes it
  used. The firmware file is the `.bin` from **Sketch → Export Compiled Binary**.
- **Duplicate `Servo.h` warning.** "Multiple libraries were found for Servo.h" is harmless. The ESP8266 core's
  Servo library is the correct one.
