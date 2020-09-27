#include <Arduino.h>
#include <PID_v1.h>

#define SSR_PIN 2
#define PRT_PIN A6

#define PRT_RESISTOR 1100

#define TIME_WINDOW 100
#define PID_WINDOW TIME_WINDOW * 2
#define SAMPLE_TIME 250

double currentTemp;
double setpoint = 0;
double onTimeMs;
PID pid(&currentTemp, &onTimeMs, &setpoint, 0.25, 0.0012, 0, P_ON_E, DIRECT);

void setup() {
  analogReference(EXTERNAL);

  Serial.begin(115200);

  pid.SetOutputLimits(0, TIME_WINDOW);
  pid.SetSampleTime(PID_WINDOW);

  pinMode(SSR_PIN, OUTPUT);
}

typedef enum { IDLE, PREHEAT, SOAK, REFLOW, COOLDOWN } State;

State state = IDLE;
unsigned long stateTimer = 0;

char const *describeState(State state) {
  switch (state) {
  case IDLE:
    return "IDLE";
  case PREHEAT:
    return "PREHEAT";
  case SOAK:
    return "SOAK";
  case REFLOW:
    return "REFLOW";
  case COOLDOWN:
    return "COOLDOWN";
  }
}

void startReflow() {
  pid.SetMode(AUTOMATIC);
  stateTimer = 0;
  state = PREHEAT;
}

State nextState() {
  switch (state) {
  case IDLE:
    return IDLE;
  case PREHEAT:
    if (currentTemp >= 100) {
      pid.SetMode(AUTOMATIC);
      stateTimer = 0;
      return SOAK;
    } else {
      return PREHEAT;
    }
  case SOAK:
    if (currentTemp >= 165) {
      pid.SetMode(MANUAL);
      stateTimer = 0;
      return REFLOW;
    } else {
      return SOAK;
    }
  case REFLOW:
    if (currentTemp >= 215) {
      pid.SetMode(MANUAL);
      stateTimer = 0;
      return COOLDOWN;
    } else {
      return REFLOW;
    }
  case COOLDOWN:
    if (currentTemp <= 50) {
      pid.SetMode(MANUAL);
      stateTimer = 0;
      return IDLE;
    } else {
      return COOLDOWN;
    }
  }
}

void handleState() {
  switch (state) {
  case IDLE:
    break;
  case PREHEAT:
    setpoint = 110;
    break;
  case SOAK:
    //   const double slope = (165.0 - 100.0) / (120000.0);
    //   setpoint = 110 + (slope * stateTimer);
    // } break;
    setpoint = 180;
    break;
  case REFLOW:
    setpoint = 0;
    onTimeMs = TIME_WINDOW;
    break;
  case COOLDOWN:
    setpoint = 0;
    onTimeMs = 0;
    break;
  }
}

void printStatus() {
  Serial.println();
  Serial.print("Temp: ");
  Serial.println(currentTemp);
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
  Serial.print("onTime: ");
  Serial.println(onTimeMs);
  Serial.print("Kp: ");
  Serial.println(pid.GetKp());
  Serial.print("Ki: ");
  Serial.println(pid.GetKi(), 4);
  Serial.print("Kd: ");
  Serial.println(pid.GetKd());
  Serial.print("PID State: ");
  Serial.println(pid.GetMode() == MANUAL ? "MANUAL" : "AUTOMATIC");
  Serial.print("Reflow State: ");
  Serial.println(describeState(state));
}

void handleCommand() {
  char cmd = (char)Serial.read();
  if (cmd == 'p') {
    Serial.write("Kp: ");
    double p = Serial.parseFloat();
    pid.SetTunings(p, pid.GetKi(), pid.GetKd());
  } else if (cmd == 'i') {
    Serial.write("Ki: ");
    double i = Serial.parseFloat();
    pid.SetTunings(pid.GetKp(), i, pid.GetKd());
  } else if (cmd == 'd') {
    Serial.write("Kd: ");
    double d = Serial.parseFloat();
    pid.SetTunings(pid.GetKp(), pid.GetKi(), d);
  } else if (cmd == 's') {
    Serial.write("Setpoint: ");
    setpoint = Serial.parseFloat();
    if (setpoint == 0) {
      pid.SetMode(MANUAL);
      onTimeMs = 0;
    } else {
      pid.SetMode(AUTOMATIC);
    }
  } else if (cmd == 'g') {
    startReflow();
  } else if (cmd == 'q') {
    pid.SetMode(MANUAL);
    state = IDLE;
    setpoint = 0;
    onTimeMs = 0;
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
unsigned long lastNow = 0;

void loop() {
  unsigned long now = millis();

  prtCMASum =
      (analogRead(PRT_PIN) + (prtCMACount * prtCMASum)) / (prtCMACount + 1);
  prtCMACount += 1;

  if (now - lastTempReading > SAMPLE_TIME) {
    currentTemp = getTemp();
    lastTempReading = now;
  }

  stateTimer += now - lastNow;
  state = nextState();
  handleState();

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

  lastNow = now;
}
