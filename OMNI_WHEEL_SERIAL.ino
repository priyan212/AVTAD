// Omni-Wheel Movement with 4 Cytron Drivers

const int dirPins[4] = {22, 23, 24, 25}; // M1 to M4 DIR
const int pwmPins[4] = {2, 3, 4, 5};     // M1 to M4 PWM
int motorSpeed = 200;  // Speed (0–255)

String input = "";

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(pwmPins[i], OUTPUT);
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void setMotor(int m, bool forward) {
  digitalWrite(dirPins[m], forward ? HIGH : LOW);
  analogWrite(pwmPins[m], motorSpeed);
}

void stopAll() {
  for (int i = 0; i < 4; i++) {
    analogWrite(pwmPins[i], 0);
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd == "F") {
    for (int i = 0; i < 4; i++) setMotor(i, true);
  }
  else if (cmd == "B") {
    for (int i = 0; i < 4; i++) setMotor(i, false);
  }
  else if (cmd == "L") {
    setMotor(0, false); setMotor(1, true); setMotor(2, true); setMotor(3, false);
  }
  else if (cmd == "R") {
    setMotor(0, true); setMotor(1, false); setMotor(2, false); setMotor(3, true);
  }
  else if (cmd == "S") {
    stopAll();
  }
}