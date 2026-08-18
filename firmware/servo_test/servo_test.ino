/*
 * servo_test.ino — AIBO MG996R bench test
 *
 * Board: ESP32-D0WD-V3 (ESP32-WROOM-32 DevKit), esp32 core 3.x
 * Port:  /dev/cu.usbserial-0001   FQBN: esp32:esp32:esp32
 *
 *  ##  POWERING FROM THE BOARD'S 5V PIN  ##
 *  Fine for ONE unloaded servo off a real 5V/3A charger. The 5V pin is
 *  wired to USB VBUS, so this does NOT load the 3.3V regulator. What
 *  limits you instead: the charger, the USB cable's resistance, and the
 *  VBUS protection diode some DevKits fit (often 1 A).
 *  NOT fine off a laptop USB port (500 mA), and not fine for the finished
 *  lamp -- 4 servos under load needs the split rail.
 *
 *  MG996R (brown/red/orange)      ESP32
 *    brown   GND  ------------------------ GND
 *    red     V+   ------------------------ 5V   (a.k.a. VIN)
 *    orange  SIG  ------------------------ GPIO 13
 *
 *  Put the 1000-2200uF cap directly across the board's 5V and GND pins.
 *  That cap is what actually supplies the inrush spike -- without it this
 *  setup browns out on the first direction change.
 *
 *  This sketch reports the reset reason on every boot, so if the supply
 *  is not coping it will say BROWNOUT instead of leaving you guessing.
 *
 * Serial @ 115200. Commands:
 *    0..180   go to that angle
 *    s        sweep 30 <-> 150 until a key is pressed
 *    c        centre (90)
 *    m        min/max endpoint check (10 / 170)
 *    d        detach (servo goes limp)
 *    a        re-attach
 *    ?        help
 */
#include <ESP32Servo.h>
#include <esp_system.h>

// ---- pins -----------------------------------------------------------------
// Classic ESP32 rules: 6-11 are the SPI flash (never touch), 34-39 are
// input-only (cannot drive a servo), and 0/2/12/15 are strapping pins --
// GPIO12 especially, a pull-up there at boot sets the flash to 1.8 V and the
// chip will not start. 13/14/27/26 dodge all of it.
static const int PIN_SERVO   = 13;   // this test
static const int PIN_BASE    = 13;   // final AIBO map, for later
static const int PIN_SHOULDER = 14;
static const int PIN_ELBOW   = 27;
static const int PIN_HEAD    = 26;

// ---- MG996R timing --------------------------------------------------------
// 500-2500 us is the full 180 deg travel. The library's 544-2400 default
// leaves a few degrees on the table at each end.
static const int US_MIN = 500;
static const int US_MAX = 2500;
static const int SERVO_HZ = 50;

Servo servo;
int angle = 90;
bool attached = false;

void attachServo() {
  servo.setPeriodHertz(SERVO_HZ);
  servo.attach(PIN_SERVO, US_MIN, US_MAX);
  attached = true;
  Serial.printf("attached on GPIO%d  (%d-%d us @ %d Hz)\n",
                PIN_SERVO, US_MIN, US_MAX, SERVO_HZ);
}

static int STEP_MS = 15;   // raise this if you see brownouts

void goTo(int a, int step_ms = STEP_MS) {
  a = constrain(a, 0, 180);
  if (!attached) attachServo();
  int dir = (a > angle) ? 1 : -1;
  while (angle != a) {                 // ease instead of slamming: a full-speed
    angle += dir;                      // step is a current spike, and a spike
    servo.write(angle);                // on a weak supply is a brownout
    delay(step_ms);
  }
  Serial.printf("angle = %d\n", angle);
}

void help() {
  Serial.println(F("\n--- AIBO MG996R test ---"));
  Serial.println(F("  0..180  go to angle      s  sweep"));
  Serial.println(F("  c       centre (90)      m  endpoints (10/170)"));
  Serial.println(F("  d       detach           a  attach"));
  Serial.println(F("  +/-     slower/faster step (raise if it browns out)"));
  Serial.printf("  signal GPIO%d | servo on its OWN 5V | grounds tied\n\n",
                PIN_SERVO);
}

void reportReset() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print(F("reset reason: "));
  switch (r) {
    case ESP_RST_POWERON:  Serial.println(F("power-on (normal)")); break;
    case ESP_RST_SW:       Serial.println(F("software (normal)")); break;
    case ESP_RST_BROWNOUT:
      Serial.println(F("*** BROWNOUT ***"));
      Serial.println(F("  The 5V rail collapsed -- the servo pulled more than"));
      Serial.println(F("  your supply could give. In order of what fixes it:"));
      Serial.println(F("   1. add/enlarge the cap across 5V-GND on the board"));
      Serial.println(F("   2. charger, not a laptop port; short thick cable"));
      Serial.println(F("   3. raise STEP_MS below (slower = smaller spike)"));
      Serial.println(F("   4. give the servo its own 5V, grounds tied"));
      break;
    case ESP_RST_PANIC:    Serial.println(F("panic/exception")); break;
    case ESP_RST_WDT:
    case ESP_RST_TASK_WDT: Serial.println(F("watchdog")); break;
    default:               Serial.printf("other (%d)\n", (int)r); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\nAIBO servo test booting"));
  reportReset();
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  attachServo();
  servo.write(angle);
  help();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;

  char c = cmd[0];
  if (isDigit(c)) {
    goTo(cmd.toInt());
  } else if (c == 's') {
    Serial.println(F("sweeping -- send anything to stop"));
    while (!Serial.available()) { goTo(30); goTo(150); }
    Serial.read();
    Serial.println(F("stopped"));
  } else if (c == 'c') {
    goTo(90);
  } else if (c == 'm') {
    Serial.println(F("endpoint check"));
    goTo(10); delay(400); goTo(170); delay(400); goTo(90);
  } else if (c == 'd') {
    servo.detach(); attached = false;
    Serial.println(F("detached -- servo is limp, no holding torque"));
  } else if (c == 'a') {
    attachServo();
  } else if (c == '+') {
    STEP_MS += 5; Serial.printf("step = %d ms (gentler)\n", STEP_MS);
  } else if (c == '-') {
    STEP_MS = max(2, STEP_MS - 5); Serial.printf("step = %d ms (snappier)\n", STEP_MS);
  } else {
    help();
  }
}
