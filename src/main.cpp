#include <Arduino.h>
#include <PID_v1.h>

#define SSR_PIN 2
#define PRT_PIN A6

#define PRT_RESISTOR 1100

#define TIME_WINDOW 100
#define PID_WINDOW TIME_WINDOW * 2
#define SAMPLE_TIME 250

double currentTemp;
double targetTemp = 0;
double onTimeMs;
double rawOnTimeMs;
PID pid(&currentTemp, &onTimeMs, &targetTemp, 0.58, 0.004, 0, P_ON_M, DIRECT);

void setup() {
  analogReference(EXTERNAL);

  Serial.begin(115200);

  pid.SetOutputLimits(0, TIME_WINDOW);
  pid.SetSampleTime(PID_WINDOW);

  pinMode(SSR_PIN, OUTPUT);
}

void printStatus() {
  Serial.println();
  Serial.print("Temp: ");
  Serial.println(currentTemp);
  Serial.print("Target: ");
  Serial.println(targetTemp);
  Serial.print("onTime: ");
  Serial.println(onTimeMs);
  Serial.print("Kp: ");
  Serial.println(pid.GetKp());
  Serial.print("Ki: ");
  Serial.println(pid.GetKi(), 3);
  Serial.print("Kd: ");
  Serial.println(pid.GetKd());
}

void handleCommand() {
  char cmd = (char)Serial.read();
  if (cmd == 'p') {
    Serial.write("Kp: ");
    double p = Serial.parseFloat();
    pid.SetTunings(p, pid.GetKi(), pid.GetKd(), P_ON_M);
  } else if (cmd == 'i') {
    Serial.write("Ki: ");
    double i = Serial.parseFloat();
    pid.SetTunings(pid.GetKp(), i, pid.GetKd(), P_ON_M);
  } else if (cmd == 'd') {
    Serial.write("Kd: ");
    double d = Serial.parseFloat();
    pid.SetTunings(pid.GetKp(), pid.GetKi(), d, P_ON_M);
  } else if (cmd == 's') {
    Serial.write("Set: ");
    targetTemp = Serial.parseFloat();
    if (targetTemp == 0) {
      pid.SetMode(MANUAL);
      onTimeMs = 0;
    } else {
      pid.SetMode(AUTOMATIC);
    }
  }
}

double prtCMASum;
int prtCMACount = 0;

double getTemp() {
  prtCMACount = 0;

  double rPRT = PRT_RESISTOR * ((2 / (prtCMASum / 1024)) - 1);
  double temp = (rPRT - 1000) / 3.851;

  return temp;
}

unsigned long lastTempReading = 0;
unsigned long lastWrite = 0;

void loop() {
  unsigned long now = millis();

  prtCMASum =
      (analogRead(PRT_PIN) + (prtCMACount * prtCMASum)) / (prtCMACount + 1);
  prtCMACount += 1;

  if (now - lastTempReading > SAMPLE_TIME) {
    currentTemp = getTemp();
    lastTempReading = now;
  }

  pid.Compute();

  unsigned long windowTime = now % TIME_WINDOW;
  if (windowTime < onTimeMs) {
    digitalWrite(SSR_PIN, HIGH);
  } else {
    digitalWrite(SSR_PIN, LOW);
  }

  if (now - lastWrite > 500) {
    lastWrite = now;
    printStatus();
  }

  if (Serial.available() > 0) {
    handleCommand();
  }
}
