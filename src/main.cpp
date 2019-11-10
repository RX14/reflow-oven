#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <SPI.h>
#include <max6675.h>

#define TIME_WINDOW 512
// Double Exponential Smoothing factors
#define DATA_SMOOTHING 0.33d
#define TREND_SMOOTHING 0.2d

MAX6675 thermocouple;

double currentTemp;
double tempDeriv = 0;
double targetTemp = 200;
double onTimeMs;
PID pid(&currentTemp, &onTimeMs, &targetTemp, 23.71, 1.02, 0, P_ON_M, DIRECT);

bool tune = false;
PID_ATune pid_autotune(&currentTemp, &onTimeMs);

void setup() {
  Serial.begin(9600);

  SPI.begin();
  thermocouple.begin(A0);
  delay(300);

  currentTemp = thermocouple.readCelsius();
  pid.SetOutputLimits(0, TIME_WINDOW);
  pid.SetMode(AUTOMATIC);

  pid_autotune.SetNoiseBand(1);
  pid_autotune.SetOutputStep(TIME_WINDOW / 4);
  pid_autotune.SetControlType(0);
  pid_autotune.SetLookbackSec(5);

  pinMode(9, OUTPUT);
}

unsigned long lastTempReading = 0;
unsigned long lastWrite = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastTempReading > 250) {
    double measuredTemp = thermocouple.readCelsius();
    lastTempReading = now;

    double prevTemp = currentTemp;
    currentTemp = DATA_SMOOTHING * measuredTemp +
                  (1 - DATA_SMOOTHING) * (prevTemp + tempDeriv);
    tempDeriv = TREND_SMOOTHING * (currentTemp - prevTemp) +
                (1 - TREND_SMOOTHING) * tempDeriv;
  }

  if (tune) {
    if (pid_autotune.Runtime()) {
      Serial.print("Kp: ");
      Serial.println(pid_autotune.GetKp());
      Serial.print("Ki: ");
      Serial.println(pid_autotune.GetKi());
      Serial.print("Kd: ");
      Serial.println(pid_autotune.GetKd());

      digitalWrite(9, LOW);
      while (1)
        ;
    }
  } else {
    pid.Compute();
  }

  unsigned long windowTime = now % TIME_WINDOW;
  if (windowTime < onTimeMs) {
    digitalWrite(9, HIGH);
  } else {
    digitalWrite(9, LOW);
  }

  if (now - lastWrite > 500) {
    Serial.println();
    Serial.print("Temp: ");
    Serial.println(currentTemp);
    Serial.print("Deriv: ");
    Serial.println(tempDeriv);
    Serial.print("Target: ");
    Serial.println(targetTemp);
    Serial.print("onTime: ");
    Serial.println(onTimeMs);
    Serial.flush();
    lastWrite = now;
  }

  if (Serial.available() > 0) {
    Serial.read();

    onTimeMs = TIME_WINDOW / 2;
    tune = !tune;

    Serial.print("Tune: ");
    Serial.println(tune);
  }
}
