#include <Servo.h>

const uint8_t PAN_PIN  = 9;   // SG-5010
const uint8_t TILT_PIN = 10;  // SG-90
const int PAN_MIN  = 10;      // adjust for mechanical end-stops
const int PAN_MAX  = 170;
const int TILT_MIN = 20;
const int TILT_MAX = 160;

Servo panServo;
Servo tiltServo;

void setup() {
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write((PAN_MIN + PAN_MAX) / 2);   // start centred
  tiltServo.write((TILT_MIN + TILT_MAX) / 2);
  Serial.begin(115200);
}

void loop() {
  static String cmd = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(cmd);
      cmd = "";
    } else {
      cmd += c;
    }
  }
}

void processCommand(const String& s) {
  // Expected format: "P### T###\n"  e.g. "P090 T045"
  int pIndex = s.indexOf('P');
  int tIndex = s.indexOf('T');
  if (pIndex == -1 || tIndex == -1) return;

  int panVal  = s.substring(pIndex + 1, pIndex + 4).toInt();
  int tiltVal = s.substring(tIndex + 1, tIndex + 4).toInt();

  panVal  = constrain(panVal,  PAN_MIN,  PAN_MAX);
  tiltVal = constrain(tiltVal, TILT_MIN, TILT_MAX);

  panServo.write(panVal);
  tiltServo.write(tiltVal);
}
