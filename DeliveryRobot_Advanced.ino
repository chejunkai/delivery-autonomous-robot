/*
 * =====================================================================
 *  Dual-Mode Autonomous Delivery Robot - ADVANCED CHALLENGE (Figure 2)
 *  Arduino MEGA 2560
 * =====================================================================
 *  Hardware
 *    - Arduino Mega 2560
 *    - 2x IR sensor (LEFT / RIGHT, just outside the line edges)
 *    - 1x HW-511 tracking sensor (CENTRE, on the line)
 *    - TCS3200 colour sensor (right side, facing the floor -> colour papers)
 *    - L298N + 2 DC motors
 *    - ESP-01 (ESP8266) on 5V adapter -> WiFi phone app (ESP01_App_Advanced.ino) on Serial1
 *    - 2x MG90S servo: ARM (raise / lower) + GRIPPER (grab / release)
 *    - 0.96" OLED SSD1306 I2C, KY-032 obstacle sensor
 *    - HW-496 sound sensor -> DOUBLE CLAP starts auto mode at base 1
 *    - DFPlayer Mini + 8 ohm 1.5 W oval speaker -> voice
 *
 *  Mission (all automatic)
 *    base 1 --(double clap / AUTO: "Auto mode")--> top line to the right
 *      dash line: "Slow down"; colour papers on the right:
 *        colour 1 -> BALL number     ("Red. Ball number one" ...)
 *        colour 2 -> STATION number  ("Blue. Station number two" ...)
 *        colour 3 -> BASE number     ("Red. Base one" / "Blue. Base two")
 *      corner -> right side down (passes stations) -> corner -> bottom line
 *      -> ball N branch: grab -> back to bottom line -> corner -> right side up
 *      -> station S branch: put the ball on the ring -> back
 *      -> base 1 (up, top line) or base 2 (down, bottom line) -> "Mission complete"
 *    Red = 1, Blue = 2, Green = 3 (ball / station).  Red = base 1, Blue = base 2.
 *    Every junction: the robot slows down.
 *
 *  Phone app (WiFi "AirostRobot" / robot1234, http://192.168.4.1) or Serial Monitor (115200):
 *    A auto/resume  M manual  X stop->manual  Z restart mission
 *    F B L R S drive   u/d arm up/down  o/c gripper open/close  h arm home  G grab  P unload
 *    + / - speed   V / v volume   ? status   K colour raw   I line sensors   O obstacle/sound
 *    C<slot><colour> set a colour by hand, e.g. C1R  C2B  C3R   (slot 1..3, colour R/B/G)
 *
 *  Libraries: "Adafruit SSD1306" (+ Adafruit GFX + BusIO) from the Library Manager. Servo is built in.
 *  Board: Arduino Mega or Mega 2560
 * =====================================================================
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== PIN MAP (matches DeliveryRobot_Advanced.fzz) ==
const uint8_t PIN_OBSTACLE  = 2;    // KY-032 OUT (LOW = obstacle)
const uint8_t PIN_SOUND     = 3;    // HW-496 OUT (interrupt pin)
const uint8_t PIN_ENB = 6;          // L298N ENB (PWM) right motor - remove the jumper
const uint8_t PIN_IN4 = 7;
const uint8_t PIN_IN3 = 8;
const uint8_t PIN_IN2 = 9;
const uint8_t PIN_IN1 = 10;
const uint8_t PIN_ENA = 11;         // L298N ENA (PWM) left motor - remove the jumper
// D14 TX3 -> 1k -> DFPlayer RX,  D15 RX3 <- DFPlayer TX
const uint8_t PIN_DF_BUSY   = 16;   // DFPlayer BUSY (LOW while playing)
// D18 TX1 -> ESP adapter RX,  D19 RX1 <- ESP adapter TX,  D20 SDA / D21 SCL -> OLED
const uint8_t PIN_TCS_OUT = 23;
const uint8_t PIN_TCS_S2  = 25;
const uint8_t PIN_TCS_S3  = 27;     // TCS S0 -> 5V, S1 -> GND, LED pin open
const uint8_t PIN_IR_LEFT  = 31;    // IR sensor LEFT
const uint8_t PIN_TRACK_C  = 33;    // HW-511 tracking sensor CENTRE
const uint8_t PIN_IR_RIGHT = 35;    // IR sensor RIGHT
const uint8_t PIN_SERVO_ARM  = 45;
const uint8_t PIN_SERVO_GRIP = 47;

#define ESP  Serial1
#define DFP  Serial3

// ===================== TUNING ======================================
const uint8_t IR_BLACK       = HIGH;  // IR module output on BLACK (LED off). Check with 'I'.
const uint8_t TRACK_BLACK    = HIGH;  // HW-511 output on BLACK. Check with 'I'.
const uint8_t OBSTACLE_LEVEL = LOW;   // KY-032 output when an obstacle is seen
const bool LEFT_MOTOR_REVERSED  = false;
const bool RIGHT_MOTOR_REVERSED = false;

int       baseSpeed    = 140;  // normal line speed (0..255), +/- from the app
const int DASH_SPEED   = 95;   // on the dash line (colour papers are read here)
const int JUNC_SPEED   = 90;   // at every junction
const int BRANCH_SPEED = 95;   // inside a ball / station branch
const int CREEP_SPEED  = 85;   // short straight moves (advance, enter base ...)
const int TURN_FAST    = 150;  // line correction: outer wheel
const int TURN_BACK    = 70;   // line correction: inner wheel (backwards)
const int SPIN_SPEED   = 135;  // pivot turns
const int MANUAL_SPEED = 170;
const int MANUAL_TURN  = 150;
const int JUNC_BIAS_PCT = 80;  // inner wheel % while one side + centre see black
uint8_t   volume       = 24;   // DFPlayer 0..30

// Timing (ms) - tune on the real map
const unsigned long JUNC_CONFIRM_MS     = 35;   // side+centre black this long = junction / corner
const unsigned long JUNC_PASS_MS        = 120;  // centre-only this long after it = junction passed
const unsigned long JUNC_SLOW_MS        = 500;  // keep JUNC_SPEED this long after a junction
const unsigned long CORNER_WINDOW_MS    = 900;  // line ends this soon after a side = corner
const unsigned long T_CONFIRM_MS        = 30;   // L+C+R black this long = T / bar / base edge
const unsigned long GAP_GRACE_MS        = 250;  // all white shorter than this = dash gap, go straight
const unsigned long LOST_MS             = 1500; // all white longer than this = line lost
const unsigned long GAP_MIN_MS          = 15, GAP_MAX_MS = 220; // one dash gap (1 cm)
const uint8_t       DASH_GAPS_NEEDED    = 2;    // gaps in a row -> dash line
const unsigned long DASH_WINDOW_MS      = 900;
const unsigned long DASH_EXIT_MS        = 1300; // no gap this long -> dash line finished
const unsigned long IGNORE_AFTER_TURN_MS = 300;
const unsigned long TURN_ADVANCE_MS     = 260;  // junction seen -> wheels over the junction
const unsigned long T_ADVANCE_MS        = 300;  // T seen (back on main line) -> wheels over it
const unsigned long CORNER_ADVANCE_MS   = 120;  // corner seen -> wheels over the corner
const unsigned long TURN_MIN_MS         = 250, TURN_MAX_MS = 2500;
const unsigned long UTURN_BACK_MS       = 350, UTURN_MIN_MS = 700, UTURN_MAX_MS = 4500;
const unsigned long SPIN_BRAKE_MS       = 40;
const unsigned long LEAVE_BASE_MAX_MS   = 4000;
const unsigned long BALL_BRANCH_MIN_MS  = 400;  // ignore the KY-032 this long after entering
const unsigned long BALL_BRANCH_MAX_MS  = 3000; // safety stop in the ball branch
const long          GRAB_ADJUST_MS      = 0;    // + forward / - backward before grabbing
const unsigned long STATION_FORWARD_MS  = 250;  // past the station bar, over the ring
const unsigned long BASE_ENTER_MS       = 700;  // base edge -> middle of the base
const unsigned long DRIVE_TIMEOUT_MS    = 600;  // manual: stop if the app stops sending
const unsigned long COLOUR_SCAN_MS      = 25;
const uint8_t       COLOUR_CONFIRM      = 3;    // same colour N reads in a row = paper
const uint8_t       FLOOR_CONFIRM       = 3;    // floor N reads in a row = paper finished
const unsigned long OBSTACLE_CONFIRM_MS = 40, OBSTACLE_CLEAR_MS = 600, OBSTACLE_TIMEOUT_MS = 15000;

const bool BALL_STOP_BY_KY032 = true;            // stop in front of the ball when the KY-032 sees it
const bool OBSTACLE_IGNORE_WHILE_CARRYING = false; // set true if the KY-032 sees the held ball

// Double clap (HW-496). Turn the module potentiometer to LOW sensitivity first.
const bool          CLAP_START      = true;
const uint8_t       CLAP_MIN_EDGES  = 4;    // output edges in one 20 ms window = loud sound
const unsigned long CLAP_MAX_MS     = 250;  // longer loud sound = not a clap (talking, music)
const unsigned long CLAP_GAP_MIN_MS = 150, CLAP_GAP_MAX_MS = 900; // time between the 2 claps

// ===================== SERVO POSITIONS ==============================
const int ARM_UP = 40, ARM_DOWN = 130, ARM_MIN = 20, ARM_MAX = 160;
const int GRIP_OPEN = 30, GRIP_CLOSE = 105, SERVO_STEP = 5;

// ===================== VOICE (SD card folder 01) ====================
enum Voice : uint8_t {
  V_AUTO = 1, V_SLOW = 2,
  V_BALL_RED = 3, V_BALL_BLUE = 4, V_BALL_GREEN = 5,
  V_STN_RED = 6, V_STN_BLUE = 7, V_STN_GREEN = 8,
  V_BASE_RED = 9, V_BASE_BLUE = 10,
  V_TO_MANUAL = 11, V_TO_AUTO = 12, V_COLOUR_FAIL = 13, V_OBSTACLE = 14, V_COMPLETE = 15
};

// ===================== COLOURS =====================================
enum Colour : uint8_t { C_NONE = 0, C_RED, C_BLUE, C_GREEN, C_FLOOR, C_BLACK };
const char* const COLOUR_NAME[] = {"-", "RED", "BLUE", "GREEN", "FLOOR", "BLACK"};
struct ColourRef { Colour c; int r, g, b; };
ColourRef colourRefs[] = {          // put the sensor over each paper, send 'K', copy the numbers
  { C_RED,    40, 110,  90 },
  { C_GREEN,  90,  60,  80 },
  { C_BLUE,  100,  80,  45 },
  { C_FLOOR,  30,  32,  28 },       // the map floor between the papers
  { C_BLACK, 200, 210, 170 },
};
const uint8_t NUM_REFS = sizeof(colourRefs) / sizeof(colourRefs[0]);
const long COLOUR_MAX_DIST = 70;

// ===================== STATE =======================================
enum Mode : uint8_t { MODE_MANUAL = 0, MODE_AUTO };
enum Phase : uint8_t {
  PH_READY = 0, PH_LEAVE_BASE, PH_TOP_EAST, PH_RIGHT_SOUTH, PH_FIND_BALL, PH_BALL_BRANCH, PH_GRAB,
  PH_BALL_RETURN, PH_BOTTOM_EAST, PH_FIND_STATION, PH_STATION_BRANCH, PH_UNLOAD, PH_STATION_RETURN,
  PH_TO_BASE, PH_DONE
};
const char* const PHASE_NAME[] = {
  "READY", "LEAVE BASE1", "READ COLOUR", "RIGHT SIDE", "FIND BALL", "TO BALL", "GRAB",
  "BALL RETURN", "BOTTOM LINE", "FIND STATION", "TO STATION", "UNLOAD", "STN RETURN",
  "TO BASE", "DONE"
};

enum LineEvt : uint8_t { EV_NONE, EV_JUNC_L, EV_JUNC_R, EV_PASS_L, EV_PASS_R, EV_CORNER_L, EV_CORNER_R, EV_T, EV_LOST };

Mode    mode = MODE_MANUAL;
Phase   phase = PH_READY;
Colour  colours[3] = {C_NONE, C_NONE, C_NONE};  // colour 1 / 2 / 3
uint8_t coloursFound = 0;
uint8_t juncCount = 0;       // junctions passed in the current phase
uint8_t baseLeg = 0;         // PH_TO_BASE: 0 = to the corner, 1 = last line to the base
bool    carrying = false;
bool    abortAuto = false, firstAuto = true, obstacleNow = false;
unsigned long pausedTotalMs = 0, lastDriveCmdMs = 0, lastStatusMs = 0, ignoreUntil = 0;

Servo servoArm, servoGrip;
int posArm = ARM_UP, posGrip = GRIP_OPEN;

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false, oledDirty = true;
char oledMsg[22] = "Ready", oledMsg2[22] = "";
unsigned long oledLast = 0;

void serviceComms();
void enterManual(const char *why);
void followerReset();

uint8_t numOf(Colour c) { return c == C_RED ? 1 : c == C_BLUE ? 2 : c == C_GREEN ? 3 : 0; }
uint8_t ballTarget()    { return numOf(colours[0]); }
uint8_t stationTarget() { return numOf(colours[1]); }
uint8_t baseTarget()    { return colours[2] == C_BLUE ? 2 : colours[2] == C_RED ? 1 : 0; }

// ===================== DFPLAYER =====================================
void dfCommand(uint8_t cmd, uint16_t param) {
  uint8_t b[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, (uint8_t)(param >> 8), (uint8_t)param, 0, 0, 0xEF};
  int16_t sum = 0;
  for (uint8_t i = 1; i < 7; i++) sum -= b[i];
  b[7] = (uint8_t)(sum >> 8); b[8] = (uint8_t)sum;
  DFP.write(b, 10);
}
uint8_t voiceQ[12]; uint8_t qHead = 0, qTail = 0;
unsigned long voiceStartMs = 0;
void say(uint8_t track) {
  uint8_t next = (qTail + 1) % sizeof(voiceQ);
  if (next != qHead) { voiceQ[qTail] = track; qTail = next; }
}
bool voicePlaying() { return millis() - voiceStartMs < 300 || digitalRead(PIN_DF_BUSY) == LOW; }
bool voiceBusy()    { return qHead != qTail || voicePlaying(); }
void serviceVoice() {
  if (qHead == qTail || voicePlaying()) return;
  dfCommand(0x0F, (1 << 8) | voiceQ[qHead]);   // folder 01, file nnn
  qHead = (qHead + 1) % sizeof(voiceQ);
  voiceStartMs = millis();
}
void voiceWaitAll(unsigned long maxMs = 6000) {
  unsigned long t0 = millis();
  while (voiceBusy() && millis() - t0 < maxMs) serviceComms();
}

// ===================== OLED ========================================
void oledStatus(const char *l1, const char *l2 = "") {
  strncpy(oledMsg, l1, sizeof(oledMsg) - 1);   oledMsg[sizeof(oledMsg) - 1] = 0;
  strncpy(oledMsg2, l2, sizeof(oledMsg2) - 1); oledMsg2[sizeof(oledMsg2) - 1] = 0;
  oledDirty = true;
  Serial.print(F("[")); Serial.print(PHASE_NAME[phase]); Serial.print(F("] "));
  Serial.print(l1); Serial.print(' '); Serial.println(l2);
}
bool sL, sC, sR;   // last line sensor reading (for OLED / app)
void oledRefresh(bool force = false) {
  if (!oledOk || (!oledDirty && !force)) return;
  unsigned long gap = (mode == MODE_AUTO) ? 300 : 150;   // a redraw blocks ~25 ms
  if (!force && millis() - oledLast < gap) return;
  oledLast = millis(); oledDirty = false;
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK); oled.setTextSize(1);
  oled.setCursor(2, 2);  oled.print(mode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
  oled.setCursor(50, 2); oled.print(PHASE_NAME[phase]);
  oled.setTextColor(SSD1306_WHITE);
  const char *what[3] = {"Ball", "Stn ", "Base"};
  for (uint8_t i = 0; i < 3; i++) {
    oled.setCursor(0, 13 + i * 9);
    oled.print(i + 1); oled.print(':'); oled.print(COLOUR_NAME[colours[i]]);
    oled.setCursor(48, 13 + i * 9); oled.print(what[i]); oled.print(' ');
    uint8_t n = i == 0 ? ballTarget() : i == 1 ? stationTarget() : baseTarget();
    if (n) oled.print(n); else oled.print('-');
  }
  oled.setCursor(92, 13); oled.print(F("J:")); oled.print(juncCount);
  oled.setCursor(92, 22); oled.print(sL ? '#' : '.'); oled.print(sC ? '#' : '.'); oled.print(sR ? '#' : '.');
  if (obstacleNow) { oled.setCursor(92, 31); oled.print(F("OBST")); }
  oled.drawFastHLine(0, 41, 128, SSD1306_WHITE);
  oled.setCursor(0, 44); oled.print(oledMsg);
  oled.setCursor(0, 54); oled.print(oledMsg2);
  oled.display();
}

// ===================== MOTORS ======================================
void setOneMotor(uint8_t en, uint8_t inA, uint8_t inB, int speed, bool reversed) {
  if (reversed) speed = -speed;
  speed = constrain(speed, -255, 255);
  digitalWrite(inA, speed > 0 ? HIGH : LOW);
  digitalWrite(inB, speed < 0 ? HIGH : LOW);
  analogWrite(en, abs(speed));
}
void drive(int left, int right) {
  setOneMotor(PIN_ENA, PIN_IN1, PIN_IN2, left,  LEFT_MOTOR_REVERSED);
  setOneMotor(PIN_ENB, PIN_IN3, PIN_IN4, right, RIGHT_MOTOR_REVERSED);
}
void stopMotors() { drive(0, 0); }

// ===================== SENSORS =====================================
void readLine() {
  sL = digitalRead(PIN_IR_LEFT)  == IR_BLACK;
  sC = digitalRead(PIN_TRACK_C)  == TRACK_BLACK;
  sR = digitalRead(PIN_IR_RIGHT) == IR_BLACK;
}
bool obstacleRaw() { return digitalRead(PIN_OBSTACLE) == OBSTACLE_LEVEL; }
bool obstacleDetected() {
  static unsigned long since = 0;
  if (!obstacleRaw()) { since = 0; return false; }
  if (!since) since = millis();
  return millis() - since >= OBSTACLE_CONFIRM_MS;
}

// ---- HW-496 double clap: count output edges in 20 ms windows ----
volatile uint16_t soundEdges = 0;
void soundIsr() { soundEdges++; }
uint16_t lastWindowEdges = 0;
bool doubleClapDetected() {
  static unsigned long winStart = 0, loudStart = 0, quietSince = 0, firstClapAt = 0;
  static bool inSound = false, valid = false;
  unsigned long now = millis();
  if (now - winStart < 20) return false;
  winStart = now;
  noInterrupts(); uint16_t e = soundEdges; soundEdges = 0; interrupts();
  lastWindowEdges = e;
  if (voiceBusy()) { inSound = false; firstClapAt = 0; return false; }   // our own speaker
  bool loud = e >= CLAP_MIN_EDGES;
  if (loud) {
    quietSince = 0;
    if (!inSound) { inSound = true; loudStart = now; valid = true; }
    else if (now - loudStart > CLAP_MAX_MS) valid = false;           // too long: not a clap
    return false;
  }
  if (!inSound) { if (firstClapAt && now - firstClapAt > CLAP_GAP_MAX_MS) firstClapAt = 0; return false; }
  if (!quietSince) quietSince = now;
  if (now - quietSince < 60) return false;                          // wait for the echo to end
  inSound = false;
  if (!valid) { firstClapAt = 0; return false; }
  unsigned long gap = loudStart - firstClapAt;
  if (firstClapAt && gap >= CLAP_GAP_MIN_MS && gap <= CLAP_GAP_MAX_MS) { firstClapAt = 0; return true; }
  firstClapAt = loudStart;
  Serial.println(F("clap 1"));
  return false;
}

// ---- TCS3200 ----
unsigned int readFilter(bool s2, bool s3) {
  digitalWrite(PIN_TCS_S2, s2); digitalWrite(PIN_TCS_S3, s3);
  delayMicroseconds(200);
  unsigned long sum = 0;
  for (uint8_t i = 0; i < 4; i++) { unsigned long p = pulseIn(PIN_TCS_OUT, LOW, 3000UL); sum += p ? p : 2000; }
  return sum / 4;
}
void readColourRaw(int &r, int &g, int &b) { r = readFilter(LOW, LOW); g = readFilter(HIGH, HIGH); b = readFilter(LOW, HIGH); }
Colour classify(int r, int g, int b, bool redBlueOnly = false) {
  long bestD = 0x7FFFFFFF; Colour best = C_NONE;
  for (uint8_t i = 0; i < NUM_REFS; i++) {
    Colour c = colourRefs[i].c;
    if (redBlueOnly && c != C_RED && c != C_BLUE) continue;
    long dr = r - colourRefs[i].r, dg = g - colourRefs[i].g, db = b - colourRefs[i].b;
    long d = dr * dr + dg * dg + db * db;
    if (d < bestD) { bestD = d; best = c; }
  }
  if (redBlueOnly) return best;
  return bestD > COLOUR_MAX_DIST * COLOUR_MAX_DIST ? C_NONE : best;
}
bool isPaper(Colour c) { return c == C_RED || c == C_BLUE || c == C_GREEN; }

// ===================== SERVOS ======================================
void servoMoveSmooth(Servo &s, int &pos, int target, uint8_t stepDelayMs = 12) {
  while (pos != target) { pos += (target > pos) ? 1 : -1; s.write(pos); delay(stepDelayMs); }
}
void nudgeServo(Servo &s, int &pos, int delta, int lo, int hi) { pos = constrain(pos + delta, lo, hi); s.write(pos); }
void armHome() { servoMoveSmooth(servoArm, posArm, ARM_UP); }
void grabMacro() {
  stopMotors(); oledStatus("Grabbing ball...", ""); oledRefresh(true);
  servoMoveSmooth(servoGrip, posGrip, GRIP_OPEN);
  servoMoveSmooth(servoArm,  posArm,  ARM_DOWN);   delay(200);
  servoMoveSmooth(servoGrip, posGrip, GRIP_CLOSE); delay(250);
  servoMoveSmooth(servoArm,  posArm,  ARM_UP);
  carrying = true; oledStatus("Ball grabbed", "");
}
void unloadMacro() {
  stopMotors(); oledStatus("Unloading ball...", ""); oledRefresh(true);
  servoMoveSmooth(servoArm,  posArm,  ARM_DOWN);   delay(200);
  servoMoveSmooth(servoGrip, posGrip, GRIP_OPEN);  delay(250);
  servoMoveSmooth(servoArm,  posArm,  ARM_UP);
  carrying = false; oledStatus("Ball on the ring", "");
}

// ===================== STATUS / MODES ==============================
// "S:mode|phase|c1|c2|c3|junctions|LCR|obstacle|speed|volume|message"   (read by the phone app)
void sendStatus(bool toUsb = false) {
  char buf[112];
  snprintf(buf, sizeof(buf), "S:%s|%s|%s|%s|%s|%u|%c%c%c|%u|%d|%u|%s %s",
           mode == MODE_AUTO ? "AUTO" : "MANUAL", PHASE_NAME[phase],
           COLOUR_NAME[colours[0]], COLOUR_NAME[colours[1]], COLOUR_NAME[colours[2]], juncCount,
           sL ? '1' : '0', sC ? '1' : '0', sR ? '1' : '0', obstacleRaw() ? 1 : 0,
           baseSpeed, volume, oledMsg, oledMsg2);
  ESP.println(buf);
  if (toUsb) Serial.println(buf);
}
void enterManual(const char *why) {
  stopMotors();
  bool wasAuto = (mode == MODE_AUTO);
  mode = MODE_MANUAL; abortAuto = true;
  if (wasAuto) say(V_TO_MANUAL);
  oledStatus("Manual mode", why);
  sendStatus();
}
void setPhase(Phase p) { phase = p; juncCount = 0; oledDirty = true; }
void enterAuto() {
  stopMotors();
  if (phase == PH_DONE) { oledStatus("Mission done", "Z = restart"); return; }
  if (phase >= PH_RIGHT_SOUTH && coloursFound < 3) { oledStatus("Set colours first", "C1x C2x C3x"); return; }
  if (phase == PH_READY) setPhase(PH_LEAVE_BASE);
  mode = MODE_AUTO; abortAuto = false;
  say(firstAuto ? V_AUTO : V_TO_AUTO);
  oledStatus(firstAuto ? "Auto mode" : "Switch to auto", "");
  firstAuto = false;
  followerReset();
  sendStatus();
}
void restartMission() {
  stopMotors();
  mode = MODE_MANUAL; setPhase(PH_READY); firstAuto = true; carrying = false;
  for (uint8_t i = 0; i < 3; i++) colours[i] = C_NONE;
  coloursFound = 0; baseLeg = 0;
  armHome(); servoMoveSmooth(servoGrip, posGrip, GRIP_OPEN);
  oledStatus("Ready at BASE 1", CLAP_START ? "Double clap / AUTO" : "Press AUTO");
  sendStatus();
}
void announceColour(uint8_t slot) {      // slot 0..2
  Colour c = colours[slot];
  static const uint8_t V[3][3] = { {V_BALL_RED, V_BALL_BLUE, V_BALL_GREEN},
                                   {V_STN_RED,  V_STN_BLUE,  V_STN_GREEN},
                                   {V_BASE_RED, V_BASE_BLUE, V_BASE_BLUE} };
  uint8_t n = numOf(c);
  if (n) say(V[slot][n - 1]);
  char l1[22], l2[22];
  snprintf(l1, sizeof(l1), "Colour %u: %s", slot + 1, COLOUR_NAME[c]);
  const char *what[3] = {"Ball", "Station", "Base"};
  snprintf(l2, sizeof(l2), "-> %s %u", what[slot], slot == 2 ? baseTarget() : n);
  oledStatus(l1, l2);
}
void setColourByHand(uint8_t slot, char k) {
  Colour c = (k == 'R' || k == 'r') ? C_RED : (k == 'B' || k == 'b') ? C_BLUE : (k == 'G' || k == 'g') ? C_GREEN : C_NONE;
  if (slot > 2 || c == C_NONE || (slot == 2 && c == C_GREEN)) return;   // colour 3 = red / blue only
  colours[slot] = c;
  coloursFound = 0;
  for (uint8_t i = 0; i < 3; i++) if (colours[i] != C_NONE) coloursFound = i + 1; else break;
  announceColour(slot);
  sendStatus();
}

// ===================== COMMANDS ====================================
void manualDrive(int l, int r) {
  if (mode != MODE_MANUAL) return;
  if (l > 0 && r > 0 && obstacleRaw()) { stopMotors(); oledStatus("OBSTACLE ahead!", "Forward blocked"); return; }
  drive(l, r); lastDriveCmdMs = millis();
}
void printSensors() {
  readLine(); int r, g, b; readColourRaw(r, g, b);
  Serial.print(F("Line L=")); Serial.print(sL); Serial.print(F(" C=")); Serial.print(sC);
  Serial.print(F(" R=")); Serial.print(sR); Serial.print(F("   (1 = black)  TCS R=")); Serial.print(r);
  Serial.print(F(" G=")); Serial.print(g); Serial.print(F(" B=")); Serial.print(b);
  Serial.print(F(" -> ")); Serial.println(COLOUR_NAME[classify(r, g, b)]);
}
void handleCommand(char c) {
  switch (c) {
    case 'A': enterAuto(); break;
    case 'M': enterManual("by user"); break;
    case 'X': enterManual("STOP pressed"); break;
    case 'Z': restartMission(); break;
    case 'F': manualDrive( MANUAL_SPEED,  MANUAL_SPEED); break;
    case 'B': manualDrive(-MANUAL_SPEED, -MANUAL_SPEED); break;
    case 'L': manualDrive(-MANUAL_TURN,   MANUAL_TURN);  break;
    case 'R': manualDrive( MANUAL_TURN,  -MANUAL_TURN);  break;
    case 'S': if (mode == MODE_MANUAL) stopMotors(); break;
    case 'u': if (mode == MODE_MANUAL) { nudgeServo(servoArm, posArm, -SERVO_STEP, ARM_MIN, ARM_MAX); oledStatus("Arm up", ""); } break;
    case 'd': if (mode == MODE_MANUAL) { nudgeServo(servoArm, posArm, +SERVO_STEP, ARM_MIN, ARM_MAX); oledStatus("Arm down", ""); } break;
    case 'o': if (mode == MODE_MANUAL) { servoMoveSmooth(servoGrip, posGrip, GRIP_OPEN);  oledStatus("Gripper open", ""); } break;
    case 'c': if (mode == MODE_MANUAL) { servoMoveSmooth(servoGrip, posGrip, GRIP_CLOSE); oledStatus("Gripper close", ""); } break;
    case 'h': if (mode == MODE_MANUAL) armHome(); break;
    case 'G': if (mode == MODE_MANUAL) grabMacro(); break;
    case 'P': if (mode == MODE_MANUAL) unloadMacro(); break;
    case '+': baseSpeed = min(baseSpeed + 10, 255); oledStatus("Speed", String(baseSpeed).c_str()); break;
    case '-': baseSpeed = max(baseSpeed - 10, 80);  oledStatus("Speed", String(baseSpeed).c_str()); break;
    case 'V': volume = min(volume + 2, 30); dfCommand(0x06, volume); break;
    case 'v': volume = max(volume - 2, 0);  dfCommand(0x06, volume); break;
    case '?': sendStatus(true); break;
    case 'K': case 'I': printSensors(); break;
    case 'O': Serial.print(F("Obstacle=")); Serial.print(obstacleRaw());
              Serial.print(F("  sound edges/20ms=")); Serial.println(lastWindowEdges); break;
    default: return;
  }
  sendStatus();
}
// Parser shared by the ESP link ("#<cmd>") and the USB Serial Monitor.
// "C" takes 2 more characters: slot 1..3 + colour R/B/G.
struct CmdParser {
  uint8_t cNeed = 0; char slot = 0;
  void feed(char ch) {
    if (cNeed == 2) { slot = ch; cNeed = 1; return; }
    if (cNeed == 1) { cNeed = 0; if (slot >= '1' && slot <= '3') setColourByHand(slot - '1', ch); return; }
    if (ch == 'C') { cNeed = 2; return; }
    handleCommand(ch);
  }
};
CmdParser espParser, usbParser;

void serviceComms() {
  static bool espArmed = false;
  while (ESP.available()) {
    char ch = (char)ESP.read();
    if (ch == '#') { espArmed = true; espParser.cNeed = 0; }
    else if (espArmed) { espParser.feed(ch); if (!espParser.cNeed) espArmed = false; }
  }
  while (Serial.available()) { char ch = (char)Serial.read(); if (ch > ' ') usbParser.feed(ch); }
  if (mode == MODE_MANUAL && lastDriveCmdMs && millis() - lastDriveCmdMs > DRIVE_TIMEOUT_MS) { stopMotors(); lastDriveCmdMs = 0; }
  bool clap = doubleClapDetected();
  if (CLAP_START && clap && phase == PH_READY && mode == MODE_MANUAL) { Serial.println(F("Double clap -> start")); enterAuto(); }
  serviceVoice();
  oledRefresh();
  if (millis() - lastStatusMs > 700) { lastStatusMs = millis(); if (mode == MODE_MANUAL) readLine(); sendStatus(); }
}

// ===================== AUTO HELPERS ================================
bool obstacleIgnored() {
  if (phase == PH_BALL_BRANCH || phase == PH_GRAB || phase == PH_STATION_BRANCH || phase == PH_UNLOAD) return true;
  return OBSTACLE_IGNORE_WHILE_CARRYING && carrying;
}
bool obstacleGuard() {
  if (obstacleIgnored() || !obstacleDetected()) { obstacleNow = false; return true; }
  stopMotors(); obstacleNow = true;
  say(V_OBSTACLE); oledStatus("OBSTACLE! Stopped", "Waiting for clear");
  unsigned long t0 = millis(), clearSince = 0;
  while (true) {
    serviceComms();
    if (abortAuto || mode != MODE_AUTO) { obstacleNow = false; return false; }
    if (obstacleRaw()) clearSince = 0;
    else if (!clearSince) clearSince = millis();
    else if (millis() - clearSince > OBSTACLE_CLEAR_MS) break;
    if (millis() - t0 > OBSTACLE_TIMEOUT_MS) { obstacleNow = false; enterManual("Obstacle not clear"); return false; }
  }
  pausedTotalMs += millis() - t0;
  obstacleNow = false; oledStatus("Path clear", "Resuming");
  return true;
}
bool autoTick() {
  serviceComms();
  if (abortAuto || mode != MODE_AUTO) { stopMotors(); return false; }
  return obstacleGuard();
}
unsigned long driveClock() { return millis() - pausedTotalMs; }   // obstacle stops not counted
bool driveFor(int left, int right, unsigned long ms) {
  unsigned long t0 = driveClock();
  while (driveClock() - t0 < ms) { if (!autoTick()) return false; drive(left, right); }
  return true;
}

// ===================== LINE FOLLOWER (L / C / R) ===================
//  Normal: C black, L/R white.  Side + C black = junction or corner on that side.
//  Line ends right after a side = CORNER.  C stays black after it = junction PASSED.
//  L+C+R black = T (branch end bar, main line, base edge).  Short all-white = dash gap.

struct Follower {
  int8_t side = 0, pending = 0, lastSteer = 0;
  unsigned long sideSince = 0, pendingAt = 0, centreSince = 0, tSince = 0, whiteSince = 0;
  unsigned long gapStart = 0, lastGapAt = 0, slowUntil = 0;
  uint8_t gaps = 0;
  bool dash = false, dashStarted = false;
};
Follower fol;
void followerReset() { fol = Follower(); ignoreUntil = millis() + IGNORE_AFTER_TURN_MS; }

int cruiseSpeed() {
  int sp = (phase == PH_BALL_BRANCH || phase == PH_STATION_BRANCH) ? BRANCH_SPEED : baseSpeed;
  if (fol.dash) sp = min(sp, DASH_SPEED);
  if (millis() < fol.slowUntil) sp = min(sp, JUNC_SPEED);
  return sp;
}
LineEvt followTick() {
  readLine();
  unsigned long now = millis();
  bool armed = now >= ignoreUntil;
  int sp = cruiseSpeed();

  // dash line: count short white gaps of the centre sensor
  if (sC && fol.gapStart) {
    unsigned long g = now - fol.gapStart; fol.gapStart = 0;
    if (g >= GAP_MIN_MS && g <= GAP_MAX_MS) {
      fol.gaps = (now - fol.lastGapAt < DASH_WINDOW_MS) ? fol.gaps + 1 : 1;
      fol.lastGapAt = now;
      if (!fol.dash && fol.gaps >= DASH_GAPS_NEEDED) { fol.dash = true; fol.dashStarted = true; }
    }
  }
  if (fol.dash && now - fol.lastGapAt > DASH_EXIT_MS) fol.dash = false;

  if (sL && sC && sR) {                                  // T / bar / cross
    fol.whiteSince = 0; fol.side = 0;
    if (!fol.tSince) fol.tSince = now;
    fol.slowUntil = now + JUNC_SLOW_MS;
    drive(JUNC_SPEED, JUNC_SPEED);
    if (armed && now - fol.tSince >= T_CONFIRM_MS) { fol.tSince = 0; fol.pending = 0; return EV_T; }
    return EV_NONE;
  }
  fol.tSince = 0;

  if (sC && (sL != sR)) {                                // centre + one side
    int8_t sd = sL ? -1 : 1;
    if (fol.side != sd) { fol.side = sd; fol.sideSince = now; }
    fol.whiteSince = 0; fol.centreSince = 0;
    fol.slowUntil = now + JUNC_SLOW_MS;                  // junction -> slow down
    int in = (long)JUNC_SPEED * JUNC_BIAS_PCT / 100;
    drive(sd < 0 ? in : JUNC_SPEED, sd < 0 ? JUNC_SPEED : in);
    fol.lastSteer = sd;
    if (armed && !fol.pending && now - fol.sideSince >= JUNC_CONFIRM_MS) {
      fol.pending = sd; fol.pendingAt = now;
      return sd < 0 ? EV_JUNC_L : EV_JUNC_R;
    }
    return EV_NONE;
  }
  fol.side = 0;

  if (sC) {                                              // on the line
    fol.whiteSince = 0;
    drive(sp, sp);
    if (fol.pending) {
      if (!fol.centreSince) fol.centreSince = now;
      if (now - fol.centreSince >= JUNC_PASS_MS) {
        int8_t p = fol.pending; fol.pending = 0; fol.centreSince = 0;
        return p < 0 ? EV_PASS_L : EV_PASS_R;
      }
    }
    return EV_NONE;
  }
  fol.centreSince = 0;

  if (fol.pending) {                                     // line ended after a side = corner
    int8_t p = fol.pending; fol.pending = 0;
    if (now - fol.pendingAt < CORNER_WINDOW_MS) return p < 0 ? EV_CORNER_L : EV_CORNER_R;
  }
  if (sL) { fol.whiteSince = 0; fol.lastSteer = -1; drive(-TURN_BACK, TURN_FAST); return EV_NONE; }
  if (sR) { fol.whiteSince = 0; fol.lastSteer =  1; drive(TURN_FAST, -TURN_BACK); return EV_NONE; }

  if (!fol.whiteSince) { fol.whiteSince = now; fol.gapStart = now; }   // all white
  unsigned long w = now - fol.whiteSince;
  if (w < GAP_GRACE_MS) drive(sp, sp);                   // dash gap: keep straight
  else if (w < LOST_MS) {                                // search towards the last correction
    if (fol.lastSteer < 0)      drive(-TURN_BACK, TURN_FAST);
    else if (fol.lastSteer > 0) drive(TURN_FAST, -TURN_BACK);
    else                        drive(CREEP_SPEED, CREEP_SPEED);
  } else { stopMotors(); return EV_LOST; }
  return EV_NONE;
}
bool dashJustStarted() { if (!fol.dashStarted) return false; fol.dashStarted = false; return true; }

// Pivot (dir -1 = left, +1 = right) until the CENTRE sensor finds a line again.
bool spinToLine(int8_t dir, unsigned long minMs, unsigned long maxMs) {
  readLine();
  bool leftLine = !sC;
  unsigned long t0 = driveClock();
  while (true) {
    if (!autoTick()) return false;
    drive(dir * SPIN_SPEED, -dir * SPIN_SPEED);
    readLine();
    unsigned long el = driveClock() - t0;
    if (!sC) leftLine = true;
    if (el >= minMs && leftLine && sC) break;
    if (el > maxMs) { enterManual("Turn: no line found"); return false; }
  }
  drive(-dir * SPIN_SPEED, dir * SPIN_SPEED); delay(SPIN_BRAKE_MS);   // short brake
  stopMotors();
  followerReset();
  return true;
}
bool turnAt(int8_t dir, unsigned long advanceMs) {
  if (!driveFor(CREEP_SPEED, CREEP_SPEED, advanceMs)) return false;
  return spinToLine(dir, TURN_MIN_MS, TURN_MAX_MS);
}
bool turnCorner(int8_t dir) {
  if (!driveFor(CREEP_SPEED, CREEP_SPEED, CORNER_ADVANCE_MS)) return false;
  return spinToLine(dir, 80, TURN_MAX_MS);
}
bool uTurn(unsigned long backMs) {
  if (!driveFor(-CREEP_SPEED, -CREEP_SPEED, backMs)) return false;
  return spinToLine(+1, UTURN_MIN_MS, UTURN_MAX_MS);
}
void juncMsg(const char *what) {
  char l2[22]; snprintf(l2, sizeof(l2), "%s %u passed", what, juncCount);
  oledStatus("Junction: slow down", l2);
}
// common handling: returns false when the phase must stop
bool commonEvent(LineEvt e) {
  if (e == EV_LOST) { enterManual("Line lost"); return false; }
  return true;
}

// ===================== MISSION PHASES ==============================
void phaseLeaveBase() {
  armHome(); servoMoveSmooth(servoGrip, posGrip, GRIP_OPEN); carrying = false;
  oledStatus("Leaving base 1", "");
  unsigned long t0 = driveClock(), onLineSince = 0;
  bool sawEdge = false;
  while (true) {                       // straight out until the base edge is crossed
    if (!autoTick()) return;
    readLine();
    drive(CREEP_SPEED, CREEP_SPEED);
    if (sL && sC && sR) sawEdge = true;
    if (sC && !sL && !sR) {
      if (!onLineSince) onLineSince = millis();
      if (sawEdge || millis() - onLineSince > 300) break;
    } else onLineSince = 0;
    if (driveClock() - t0 > LEAVE_BASE_MAX_MS) { enterManual("Base 1 exit: no line"); return; }
  }
  followerReset();
  setPhase(PH_TOP_EAST);
}

// Top line to the right: dash line + colour papers 1, 2, 3 on the right side
void phaseTopEast() {
  oledStatus("Reading colours", "");
  Colour cand = C_NONE; uint8_t candN = 0, floorN = 0;
  bool onPaper = false;
  unsigned long lastScan = 0;
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (dashJustStarted()) { say(V_SLOW); oledStatus("Dash line:", "SLOW DOWN"); }

    if (coloursFound < 3 && millis() - lastScan >= COLOUR_SCAN_MS) {
      lastScan = millis();
      int r, g, b; readColourRaw(r, g, b);
      Colour c = classify(r, g, b);
      if (!onPaper) {
        if (isPaper(c)) {
          if (c == cand) candN++; else { cand = c; candN = 1; }
          if (candN >= COLOUR_CONFIRM) {
            if (coloursFound == 2 && c == C_GREEN) c = classify(r, g, b, true);  // colour 3: red / blue
            colours[coloursFound] = c;
            announceColour(coloursFound);
            coloursFound++;
            onPaper = true; floorN = 0; candN = 0;
            sendStatus();
          }
        } else candN = 0;
      } else {
        if (!isPaper(c)) { if (++floorN >= FLOOR_CONFIRM) onPaper = false; }
        else floorN = 0;
      }
    }

    if (e == EV_CORNER_R || e == EV_CORNER_L) {
      if (!turnCorner(e == EV_CORNER_R ? 1 : -1)) return;
      setPhase(PH_RIGHT_SOUTH);
      if (coloursFound < 3) {
        say(V_COLOUR_FAIL);
        char l2[22]; snprintf(l2, sizeof(l2), "found %u/3 colours", coloursFound);
        oledStatus("Colour detect FAIL", l2);
        voiceWaitAll();
        enterManual("Set colours in app");
      }
      return;
    }
    if (e == EV_T) { enterManual("Unexpected T (top)"); return; }
  }
}

// Down the right side (stations are on the left), corner -> bottom line
void phaseRightSouth() {
  oledStatus("Right side down", "to the ball line");
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (e == EV_PASS_L || e == EV_PASS_R) { juncCount++; juncMsg("Station"); }
    if (e == EV_CORNER_R || e == EV_CORNER_L) {
      if (!turnCorner(e == EV_CORNER_R ? 1 : -1)) return;
      setPhase(PH_FIND_BALL); return;
    }
    if (e == EV_T) { enterManual("Unexpected T (right)"); return; }
  }
}

// Bottom line to the left: ball 1, 2, 3 branches on the left
void phaseFindBall() {
  char l1[22]; snprintf(l1, sizeof(l1), "Going to ball %u", ballTarget());
  oledStatus(l1, "");
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (e == EV_JUNC_L && juncCount + 1 == ballTarget()) {
      juncCount++;
      oledStatus("Junction: slow down", "Turn to the ball");
      if (!turnAt(-1, TURN_ADVANCE_MS)) return;
      setPhase(PH_BALL_BRANCH); return;
    }
    if (e == EV_PASS_L) { juncCount++; juncMsg("Ball"); }
    if (e == EV_T || e == EV_CORNER_L || e == EV_CORNER_R) { enterManual("Ball branch missed"); return; }
  }
}

void phaseBallBranch() {
  oledStatus("To the ball", "");
  unsigned long t0 = driveClock();
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (e == EV_T || e == EV_LOST) break;                                    // end bar
    unsigned long el = driveClock() - t0;
    if (BALL_STOP_BY_KY032 && el > BALL_BRANCH_MIN_MS && obstacleDetected()) break;  // ball in front
    if (el > BALL_BRANCH_MAX_MS) break;
  }
  stopMotors();
  if (GRAB_ADJUST_MS > 0 && !driveFor(CREEP_SPEED, CREEP_SPEED, GRAB_ADJUST_MS)) return;
  if (GRAB_ADJUST_MS < 0 && !driveFor(-CREEP_SPEED, -CREEP_SPEED, -GRAB_ADJUST_MS)) return;
  stopMotors();
  setPhase(PH_GRAB);
}

void phaseGrab() {
  if (!carrying) grabMacro();
  if (!uTurn(UTURN_BACK_MS)) return;
  setPhase(PH_BALL_RETURN);
}

// back up the ball branch -> bottom line (T) -> turn RIGHT (towards the stations)
void phaseBallReturn() {
  oledStatus("Ball grabbed", "Back to main line");
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (e == EV_T) { if (!turnAt(1, T_ADVANCE_MS)) return; break; }
    if (e == EV_CORNER_L || e == EV_CORNER_R) { if (!turnCorner(1)) return; break; }
  }
  setPhase(PH_BOTTOM_EAST);
}

// bottom line to the right (passes the other ball branches), corner -> right side up
void phaseBottomEast() {
  oledStatus("To the stations", "");
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (e == EV_PASS_L || e == EV_PASS_R) { juncCount++; juncMsg("Ball"); }
    if (e == EV_CORNER_L || e == EV_CORNER_R) {
      if (!turnCorner(e == EV_CORNER_L ? -1 : 1)) return;
      setPhase(PH_FIND_STATION); return;
    }
    if (e == EV_T) { enterManual("Unexpected T (bottom)"); return; }
  }
}

// right side up: station 3, 2, 1 branches on the right
void phaseFindStation() {
  uint8_t target = 4 - stationTarget();          // station 3 is the first junction going up
  char l1[22]; snprintf(l1, sizeof(l1), "Going to station %u", stationTarget());
  oledStatus(l1, "");
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (e == EV_JUNC_R && juncCount + 1 == target) {
      juncCount++;
      oledStatus("Junction: slow down", "Turn to station");
      if (!turnAt(1, TURN_ADVANCE_MS)) return;
      setPhase(PH_STATION_BRANCH); return;
    }
    if (e == EV_PASS_R) { juncCount++; juncMsg("Station"); }
    if (e == EV_T || e == EV_CORNER_L || e == EV_CORNER_R) { enterManual("Station missed"); return; }
  }
}

void phaseStationBranch() {
  oledStatus("To the station", "");
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (e == EV_T || e == EV_LOST) break;        // station bar
  }
  stopMotors();
  setPhase(PH_UNLOAD);
}

void phaseUnload() {
  if (carrying) {
    if (!driveFor(CREEP_SPEED, CREEP_SPEED, STATION_FORWARD_MS)) return;   // over the ring
    stopMotors();
    unloadMacro();
  }
  if (!uTurn(UTURN_BACK_MS + STATION_FORWARD_MS)) return;
  setPhase(PH_STATION_RETURN);
}

// back to the right side line (T) -> base 1 = turn RIGHT (up), base 2 = turn LEFT (down)
void phaseStationReturn() {
  oledStatus("Ball delivered", baseTarget() == 1 ? "Going to base 1" : "Going to base 2");
  int8_t dir = baseTarget() == 1 ? 1 : -1;
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (e == EV_T) { if (!turnAt(dir, T_ADVANCE_MS)) return; break; }
    if (e == EV_CORNER_L || e == EV_CORNER_R) { if (!turnCorner(dir)) return; break; }
  }
  setPhase(PH_TO_BASE); baseLeg = 0;
}

void phaseToBase() {
  char l1[22]; snprintf(l1, sizeof(l1), "Going to base %u", baseTarget());
  oledStatus(l1, baseLeg ? "last line" : "");
  while (true) {
    if (!autoTick()) return;
    LineEvt e = followTick();
    if (!commonEvent(e)) return;
    if (dashJustStarted()) { say(V_SLOW); oledStatus("Dash line:", "SLOW DOWN"); }
    if (e == EV_PASS_L || e == EV_PASS_R) { juncCount++; juncMsg(baseLeg ? "Ball" : "Station"); }
    if ((e == EV_CORNER_L || e == EV_CORNER_R) && baseLeg == 0) {
      if (!turnCorner(e == EV_CORNER_L ? -1 : 1)) return;
      baseLeg = 1; juncCount = 0;
      oledStatus(l1, "last line");
    }
    if (e == EV_T) {
      if (baseLeg == 0) { enterManual("Unexpected T (base)"); return; }
      break;                                       // edge of the base square
    }
  }
  if (!driveFor(CREEP_SPEED, CREEP_SPEED, BASE_ENTER_MS)) return;
  stopMotors();
  setPhase(PH_DONE);
  say(V_COMPLETE);
  char l2[22]; snprintf(l2, sizeof(l2), "Reached base %u", baseTarget());
  oledStatus("MISSION COMPLETE", l2);
  mode = MODE_MANUAL;                              // finished: no "switch to manual" voice
  sendStatus();
}

void runAuto() {
  switch (phase) {
    case PH_READY:          setPhase(PH_LEAVE_BASE);  break;
    case PH_LEAVE_BASE:     phaseLeaveBase();         break;
    case PH_TOP_EAST:       phaseTopEast();           break;
    case PH_RIGHT_SOUTH:    phaseRightSouth();        break;
    case PH_FIND_BALL:      phaseFindBall();          break;
    case PH_BALL_BRANCH:    phaseBallBranch();        break;
    case PH_GRAB:           phaseGrab();              break;
    case PH_BALL_RETURN:    phaseBallReturn();        break;
    case PH_BOTTOM_EAST:    phaseBottomEast();        break;
    case PH_FIND_STATION:   phaseFindStation();       break;
    case PH_STATION_BRANCH: phaseStationBranch();     break;
    case PH_UNLOAD:         phaseUnload();            break;
    case PH_STATION_RETURN: phaseStationReturn();     break;
    case PH_TO_BASE:        phaseToBase();            break;
    case PH_DONE:           stopMotors(); mode = MODE_MANUAL; break;
  }
}

// ===================== SETUP / LOOP ================================
void setup() {
  Serial.begin(115200);
  ESP.begin(115200);                  // must match Serial.begin() in ESP01_App_Advanced.ino
  DFP.begin(9600);

  pinMode(PIN_IR_LEFT, INPUT); pinMode(PIN_IR_RIGHT, INPUT); pinMode(PIN_TRACK_C, INPUT);
  pinMode(PIN_OBSTACLE, INPUT); pinMode(PIN_SOUND, INPUT);
  pinMode(PIN_DF_BUSY, INPUT_PULLUP);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  stopMotors();
  pinMode(PIN_TCS_OUT, INPUT); pinMode(PIN_TCS_S2, OUTPUT); pinMode(PIN_TCS_S3, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SOUND), soundIsr, CHANGE);

  servoArm.attach(PIN_SERVO_ARM);   servoArm.write(posArm);
  servoGrip.attach(PIN_SERVO_GRIP); servoGrip.write(posGrip);

  Wire.begin(); Wire.setClock(400000);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D);   // try 0x3D if the screen stays black
  if (!oledOk) Serial.println(F("OLED not found"));

  delay(600);                          // DFPlayer boot
  dfCommand(0x09, 2); delay(200);      // source = SD card
  dfCommand(0x06, volume);

  restartMission();
  oledRefresh(true);
  Serial.println(F("Delivery Robot (Advanced) ready. Double clap or send A to start."));
}

void loop() {
  serviceComms();
  if (mode == MODE_AUTO) { abortAuto = false; runAuto(); }
}
