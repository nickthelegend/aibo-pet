/*
 * servo_move.ino — AIBO motion core + bench test
 *
 * Board: ESP32-D0WD-V3 (WROOM-32 DevKit), esp32 core 3.x, ESP32Servo 3.2.1
 * Port:  /dev/cu.usbserial-0001   FQBN: esp32:esp32:esp32
 *
 * WHY THIS REPLACES THE STEP-AND-DELAY VERSION
 * Measured on the actual board: a 140 deg move took 5.57 s whether the step
 * delay was 2 ms or 27 ms. servo.write() blocks on the 50 Hz PWM frame, so
 * the "one degree per delay()" loop is pinned near 25 steps/s and its speed
 * knob does nothing. It also blocks, which is fatal once four servos, I2S
 * audio and a LED ring have to share the loop.
 *
 * Instead: hold a float position per joint and advance it toward its target
 * by (speed * dt) once per 20 ms frame -- one write per servo per frame, the
 * exact rate the servo consumes. Motion is smooth, speed is real deg/s, and
 * loop() never blocks.
 *
 * WIRING (per servo)
 *   brown  GND ---- ESP32 GND        <-- common ground, always
 *   red    V+  ---- 5V  (VIN/VBUS -- bypasses the 3V3 regulator)
 *   orange SIG ---- GPIO (below)
 * 1000-2200uF across 5V/GND at the board. One unloaded MG996R off a 3 A
 * charger is fine; four under load needs the split rail.
 *
 * Serial @ 115200:
 *   0..180    move active joint      j0..j3  select joint
 *   v<n>      speed, deg/s (default 120)
 *   s         sweep 30<->150         m  endpoints    c  centre
 *   a / d     attach / detach        t  timing self-test
 *   ?         help
 */
#include <ESP32Servo.h>
#include <esp_system.h>

// Classic ESP32: 6-11 are SPI flash, 34-39 are input-only, 0/2/12/15 are
// strapping pins (GPIO12 high at boot = 1.8 V flash = dead board).
// 13/14/27/26 avoid all of it.
static const int PINS[4] = {13, 14, 27, 26};
static const char* NAMES[4] = {"base", "shoulder", "elbow", "head"};
static const int N_ACTIVE = 1;          // bench: just GPIO13 for now

static const int  US_MIN = 500, US_MAX = 2500, SERVO_HZ = 50;
static const uint32_t FRAME_US = 1000000UL / SERVO_HZ;   // 20 000

struct Joint {
  Servo   servo;
  float   pos = 90, target = 90;
  float   speed = 120.0f;               // deg/s
  bool    attached = false;
} J[4];

int active = 0;
uint32_t lastFrame = 0;

void attachJoint(int i) {
  J[i].servo.setPeriodHertz(SERVO_HZ);
  J[i].servo.attach(PINS[i], US_MIN, US_MAX);
  J[i].servo.write((int)J[i].pos);
  J[i].attached = true;
}

// one 20 ms frame: advance every joint toward its target, write once
bool tick(float dt) {
  bool moving = false;
  for (int i = 0; i < N_ACTIVE; i++) {
    Joint &j = J[i];
    if (!j.attached) continue;
    float d = j.target - j.pos;
    float step = j.speed * dt;
    if (fabsf(d) <= step) j.pos = j.target;
    else                  j.pos += (d > 0 ? step : -step);
    j.servo.write((int)lroundf(j.pos));
    if (j.pos != j.target) moving = true;
  }
  return moving;
}

// block until the current move finishes (bench use only; the real firmware
// just lets tick() run inside the main loop)
uint32_t settle(uint32_t timeout_ms = 8000) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    uint32_t now = micros();
    if (now - lastFrame >= FRAME_US) {
      float dt = (now - lastFrame) / 1e6f;
      lastFrame = now;
      if (!tick(dt)) return millis() - t0;
    }
  }
  return millis() - t0;
}

void moveTo(float a) {
  J[active].target = constrain(a, 0.0f, 180.0f);
  uint32_t ms = settle();
  Serial.printf("%s -> %.0f deg in %lu ms (%.1f deg/s cmd)\n",
                NAMES[active], J[active].target, (unsigned long)ms, J[active].speed);
}

void timingTest() {
  Serial.println(F("timing self-test (140 deg each way):"));
  for (float v : {60.0f, 120.0f, 240.0f, 400.0f}) {
    J[active].speed = v;
    J[active].target = 20; settle();
    uint32_t t0 = millis();
    J[active].target = 160; settle();
    uint32_t ms = millis() - t0;
    Serial.printf("   %6.0f deg/s -> %5lu ms  (%.1f ms/deg, actual %.0f deg/s)\n",
                  v, (unsigned long)ms, ms / 140.0f, 140000.0f / ms);
  }
  J[active].speed = 120;
  Serial.println(F("done"));
}

void help() {
  Serial.println(F("\n--- AIBO motion core ---"));
  Serial.printf("  active joint: %s (GPIO%d)  speed %.0f deg/s\n",
                NAMES[active], PINS[active], J[active].speed);
  Serial.println(F("  0..180 move | v<n> speed | j<0-3> joint"));
  Serial.println(F("  s sweep | m endpoints | c centre | t timing | a/d attach\n"));
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\nAIBO motion core"));
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("reset reason: %s\n",
                r == ESP_RST_BROWNOUT ? "*** BROWNOUT -- supply sagged ***"
                : r == ESP_RST_POWERON ? "power-on (normal)" : "other/normal");
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < N_ACTIVE; i++) attachJoint(i);
  lastFrame = micros();
  help();
}

void loop() {
  uint32_t now = micros();
  if (now - lastFrame >= FRAME_US) {          // non-blocking: this is the
    float dt = (now - lastFrame) / 1e6f;      // only place servos are written
    lastFrame = now;
    tick(dt);
  }
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;
  char c = cmd[0];

  if (isDigit(c))      moveTo(cmd.toFloat());
  else if (c == 'v') { J[active].speed = constrain(cmd.substring(1).toFloat(), 5, 600);
                       Serial.printf("speed = %.0f deg/s\n", J[active].speed); }
  else if (c == 'j') { active = constrain(cmd.substring(1).toInt(), 0, N_ACTIVE - 1);
                       Serial.printf("active = %s\n", NAMES[active]); }
  else if (c == 'c')   moveTo(90);
  else if (c == 'm') { moveTo(10); delay(300); moveTo(170); delay(300); moveTo(90); }
  else if (c == 't')   timingTest();
  else if (c == 's') { Serial.println(F("sweeping -- send anything to stop"));
                       while (!Serial.available()) { moveTo(30); moveTo(150); }
                       Serial.read(); Serial.println(F("stopped")); }
  else if (c == 'd') { for (int i=0;i<N_ACTIVE;i++){ J[i].servo.detach(); J[i].attached=false; }
                       Serial.println(F("detached")); }
  else if (c == 'a') { for (int i=0;i<N_ACTIVE;i++) attachJoint(i);
                       Serial.println(F("attached")); }
  else help();
}
