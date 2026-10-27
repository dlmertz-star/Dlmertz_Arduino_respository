const int  PWM_PIN         = 18;
const int  TACH_PIN        = 19;
const int  POT_PIN         = 34;
const int  PWM_FREQ        = 25000;
const int  PWM_BITS        = 8;
const bool INVERT_PWM      = true;   // true if GPIO18 drives an NPN base
const int  PULSES_PER_REV  = 2;
const uint32_t MIN_EDGE_US = 5000;

float potFiltered = 0;
int   lastPercent = -1;

const uint32_t MIN_LOW_US    = 1000;    // real low phase is many ms; glitches are µs
const uint32_t MIN_PERIOD_US = 10000;   // max accepted ~3000 RPM at 2 pulses/rev

volatile uint32_t fallUs = 0, lastGoodUs = 0, pulseCount = 0;
volatile uint32_t lastLowWidth = 0, lastPeriod = 0;

void IRAM_ATTR tachISR() {
  uint32_t now = micros();
  if (digitalRead(TACH_PIN) == LOW) {
    fallUs = now;                        // a low phase starts
  } else if (fallUs) {
    uint32_t low = now - fallUs;
    fallUs = 0;
    if (low >= MIN_LOW_US && (now - lastGoodUs) >= MIN_PERIOD_US) {
      lastLowWidth = low;
      lastPeriod   = now - lastGoodUs;
      lastGoodUs   = now;
      pulseCount++;                      // count on the rising edge
    }
  }
}

void setFanPercent(int percent) {
  percent = constrain(percent, 0, 100);
  int duty = percent * 255 / 100;
  if (INVERT_PWM) duty = 255 - duty;
  ledcWrite(PWM_PIN, duty);
}

int readPotPercent() {
  // average 16 samples to tame ADC noise, then smooth over time
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(POT_PIN);
  float raw = sum / 16.0f;
  potFiltered += 0.2f * (raw - potFiltered);

  // the ESP32 ADC is unreliable near its ends, so add a small dead zone
  const float LOW_END = 100, HIGH_END = 3995;
  float pct = (potFiltered - LOW_END) * 100.0f / (HIGH_END - LOW_END);
  return constrain((int)(pct + 0.5f), 0, 100);
}

void setup() {
  Serial.begin(115200);
  pinMode(TACH_PIN, INPUT);                   // external 10k pull-up to 3V3
  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db); // full 0-3.3 V range
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_BITS);
  potFiltered = analogRead(POT_PIN);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, CHANGE);
}

void loop() {
  static uint32_t lastPot = 0, lastReport = 0;

  if (millis() - lastPot >= 20) {             // update speed 50 times a second
    lastPot = millis();
    int percent = readPotPercent();
    if (abs(percent - lastPercent) >= 1) {    // only write on a real change
      lastPercent = percent;
      setFanPercent(percent);
    }
  }

  if (millis() - lastReport >= 1000) {
    lastReport += 1000;
    noInterrupts();
    uint32_t count = pulseCount; pulseCount = 0;
    uint32_t lw = lastLowWidth, per = lastPeriod;
    interrupts();
    uint32_t rpm = count * 60 / PULSES_PER_REV;
    Serial.printf("set: %d%%  pulses/s: %lu  RPM: %lu  low: %lu us  period: %lu us\n",
              lastPercent, count, rpm, lw, per);
  }
}