#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <SPI.h>

#define TIME_WINDOW 512
#define PID_WINDOW TIME_WINDOW * 2
#define SAMPLE_TIME 250

#define SSR_PIN 2
#define PRT_PIN A6

#define PRT_RESISTOR 1100

double prtCMASum;
int prtCMACount = 0;

double currentTemp;
double targetTemp = 0;
double onTimeMs;
double rawOnTimeMs;
PID pid(&currentTemp, &onTimeMs, &targetTemp, 13.74, 0.05, 0, P_ON_M, DIRECT);

bool tune = false;
PID_ATune pid_autotune(&currentTemp, &onTimeMs);

void setup() {
  analogReference(EXTERNAL);

  Serial.begin(115200);

  pid.SetOutputLimits(0, TIME_WINDOW);
  pid.SetSampleTime(PID_WINDOW);

  pid_autotune.SetNoiseBand(2);
  pid_autotune.SetOutputStep(TIME_WINDOW / 2);
  pid_autotune.SetControlType(0);
  pid_autotune.SetLookbackSec(30);

  pinMode(SSR_PIN, OUTPUT);
}

unsigned long lastTempReading = 0;
unsigned long lastWrite = 0;

double getTemp() {
  prtCMACount = 0;

  double rPRT = PRT_RESISTOR * ((2 / (prtCMASum / 1024)) - 1);
  // if (abs(rawTemp - targetTemp) < 3) {
  //   rawTemp = targetTemp;
  // }

  return (rPRT - 1000) / 3.851;
}

void loop() {
  unsigned long now = millis();

  prtCMASum =
      (analogRead(PRT_PIN) + (prtCMACount * prtCMASum)) / (prtCMACount + 1);
  prtCMACount += 1;

  if (now - lastTempReading > SAMPLE_TIME) {
    currentTemp = getTemp();
    lastTempReading = now;
  }

  if (tune) {
    if (pid_autotune.Runtime()) {
      Serial.print("Kp: ");
      Serial.println(pid_autotune.GetKp());
      Serial.print("Ki: ");
      Serial.println(pid_autotune.GetKi());
      Serial.print("Kd: ");
      Serial.println(pid_autotune.GetKd());

      digitalWrite(SSR_PIN, LOW);
      while (1)
        ;
    }
  } else {
    // if (currentTemp - targetTemp < -50) {
    //   pid.SetMode(MANUAL);
    //   onTimeMs = TIME_WINDOW;
    // } else if (currentTemp - targetTemp > 20) {
    //   pid.SetMode(MANUAL);
    //   onTimeMs = 0;
    // } else {
    pid.SetMode(AUTOMATIC);
    // }

    pid.Compute();
  }

  unsigned long windowTime = now % TIME_WINDOW;
  if (windowTime < onTimeMs) {
    digitalWrite(SSR_PIN, HIGH);
  } else {
    digitalWrite(SSR_PIN, LOW);
  }

  if (now - lastWrite > 500) {
    Serial.println();
    Serial.print("R: ");
    Serial.println(prtCMASum);
    Serial.print("Temp: ");
    Serial.println(currentTemp);
    Serial.print("Target: ");
    Serial.println(targetTemp);
    Serial.print("onTime: ");
    Serial.println(onTimeMs);
    Serial.print("Kp: ");
    Serial.println(pid.GetKp());
    Serial.print("Ki: ");
    Serial.println(pid.GetKi());
    Serial.print("Kd: ");
    Serial.println(pid.GetKd());
    if (pid.GetMode() == MANUAL)
      Serial.println("MANUAL");
    lastWrite = now;
  }

  if (Serial.available() > 0) {
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
    } else if (cmd == ' ') {
      onTimeMs = TIME_WINDOW / 2;
      tune = !tune;
    }

    Serial.print("Tune: ");
    Serial.println(tune);
  }
}
