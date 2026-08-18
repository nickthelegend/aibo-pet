/*
 * servo_diag.ino — find out WHERE the chain is broken.
 *
 * The servo is not moving. Three things must ALL be true; test them one at
 * a time instead of guessing:
 *   1. the GPIO actually drives        -> 'h' / 'l' / 'b'   (multimeter/LED)
 *   2. real servo pulses reach it      -> '1' '2' '3'       (horn must jump)
 *   3. the servo has 5V and shared gnd -> measure its own red/brown
 *
 * Pulses come straight from the LEDC hardware peripheral -- no servo
 * library, no timer ISR. 50 Hz, 16-bit: duty = us/20000 * 65535.
 *
 * Board: ESP32-D0WD-V3, GPIO13 (silkscreen "D13" on a DOIT DevKit V1).
 */
static const int PIN = 13;
static const int HZ  = 50;
static const int RES = 16;

bool pwm_on = false;

void setPulse(int us) {
  if (!pwm_on) { ledcAttach(PIN, HZ, RES); pwm_on = true; }
  uint32_t duty = (uint32_t)((us / 20000.0f) * ((1UL << RES) - 1));
  ledcWrite(PIN, duty);
  Serial.printf("  %d us @ %d Hz  (duty %lu / %lu)\n",
                us, HZ, (unsigned long)duty, (unsigned long)((1UL << RES) - 1));
}

void toDigital() {
  if (pwm_on) { ledcDetach(PIN); pwm_on = false; }
  pinMode(PIN, OUTPUT);
}

void help() {
  Serial.println(F("\n=== servo diagnostic — GPIO13 ==="));
  Serial.println(F("STEP 1  is the pin driving?   black probe on GND"));
  Serial.println(F("  h  hold HIGH  -> ~3.3 V     l  hold LOW -> ~0 V"));
  Serial.println(F("  b  blink 1 Hz x10"));
  Serial.println(F("STEP 2  do pulses reach it?   horn should JUMP each time"));
  Serial.println(F("  1  1000us (~0deg)   2  1500us (~90deg)   3  2000us (~180deg)"));
  Serial.println(F("  w  wiggle 1000<->2000 five times   0  pulses off"));
  Serial.println(F("STEP 3  power — probe the SERVO's own red/brown wires:"));
  Serial.println(F("  must read 4.8-5.2 V. If 0 V, red is not on 5V — that's it."));
  Serial.println(F("  3.3 V there = you're on the wrong rail; MG996R won't turn.\n"));
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\nservo diagnostic ready (LEDC hardware PWM)"));
  toDigital();
  digitalWrite(PIN, LOW);
  help();
}

void loop() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n'); c.trim();
  if (!c.length()) return;
  switch (c[0]) {
    case 'h': toDigital(); digitalWrite(PIN, HIGH);
              Serial.println(F("HIGH — expect ~3.3 V on GPIO13")); break;
    case 'l': toDigital(); digitalWrite(PIN, LOW);
              Serial.println(F("LOW — expect ~0 V")); break;
    case 'b': toDigital(); Serial.println(F("blinking..."));
              for (int i=0;i<10;i++){digitalWrite(PIN,HIGH);delay(500);
                                     digitalWrite(PIN,LOW);delay(500);}
              Serial.println(F("blink done")); break;
    case '1': setPulse(1000); break;
    case '2': setPulse(1500); break;
    case '3': setPulse(2000); break;
    case 'w': Serial.println(F("wiggling 5x — WATCH THE HORN"));
              for (int i=0;i<5;i++){ setPulse(1000); delay(700);
                                     setPulse(2000); delay(700); }
              setPulse(1500); Serial.println(F("wiggle done")); break;
    case '0': if (pwm_on) { ledcDetach(PIN); pwm_on = false; }
              toDigital(); digitalWrite(PIN, LOW);
              Serial.println(F("pulses off — servo limp")); break;
    default:  help();
  }
}
