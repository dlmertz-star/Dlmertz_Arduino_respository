/*
 * ESP32 temperature-controlled 4-pin PWM fan
 *  - 0 RPM below T0 (fan supply is switched off by a P-MOSFET)
 *  - RPM target rises linearly from a minimum running speed at T0 to max at T1
 *  - inner loop: feed-forward + PI on measured RPM (tach feedback)
 *  - fail-safe: sensor lost -> fan at 100 %
 *
 * Arduino-ESP32 core 2.x or 3.x.
 * Libraries: OneWire + DallasTemperature (DS18B20)
 *            or "DHT sensor library" + "Adafruit Unified Sensor" (DHT11/DHT22)
 */
#include <Arduino.h>

// ======================= USER CONFIGURATION =======================
#define SENSOR_DS18B20 1
#define SENSOR_DHT     2
#define SENSOR_TYPE    SENSOR_DS18B20      // <- pick your sensor
#define DHT_TYPE       DHT22               // DHT11 or DHT22 (only if SENSOR_DHT)

// Pins (none of these are strapping / flash / input-only pins)
#define PIN_PWM     25   // -> Q1 base (through 1k): 25 kHz PWM, inverted by the NPN
#define PIN_ENABLE  27   // -> Q3 base (through 1k): switches the fan's 12 V
#define PIN_TACH    26   // <- fan tach (open collector, 10k pull-up to 3.3 V)
#define PIN_TEMP     4   // <-> temperature sensor data

// Temperature law
const float T0_C     = 30.0f;   // below this (minus hysteresis): fan stopped
const float T1_C     = 50.0f;   // at/above this: full speed
const float T_HYST_C = 1.0f;    // fan stops at T0 - T_HYST_C

// Fan characteristics
const float   RPM_MIN_FRAC     = 0.25f;    // lowest reliable running speed / max speed
const float   RPM_MAX_DEFAULT  = 1800.0f;  // used if calibration is off or fails
const bool    CALIBRATE_RPM_MAX = true;    // spin fan at 100 % for ~6 s at boot to measure max RPM
const uint8_t TACH_PPR         = 2;        // tach pulses per revolution (standard: 2)

// PI loop (error normalised to max RPM, output = duty 0..1)
const float    KP          = 0.30f;
const float    KI          = 0.40f;    // per second
const float    TRIM_LIMIT  = 0.40f;    // integrator clamp (anti-windup)
const uint32_t CTL_PERIOD_MS = 500;
const float    KICK_DUTY   = 0.60f;    // start-up kick when leaving the stopped state
const uint32_t KICK_MS     = 1000;

// Temperature acquisition
const float   TEMP_FILTER_ALPHA = 0.3f;    // EMA filter
const uint8_t SENSOR_FAIL_LIMIT = 5;       // consecutive failed reads -> fail-safe
#if SENSOR_TYPE == SENSOR_DHT
const uint32_t TEMP_PERIOD_MS = 2000;
#else
const uint32_t TEMP_PERIOD_MS = 1000;
#endif
// ==================================================================

// ---------------- Temperature sensor abstraction ----------------
#if SENSOR_TYPE == SENSOR_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
  OneWire oneWire(PIN_TEMP);
  DallasTemperature ds(&oneWire);
  void sensorBegin() {
    ds.begin();
    ds.setWaitForConversion(false);   // non-blocking
    ds.requestTemperatures();
  }
  bool sensorRead(float &t) {
    float v = ds.getTempCByIndex(0);
    ds.requestTemperatures();          // start next conversion
    if (v == DEVICE_DISCONNECTED_C || v == 85.0f) return false;  // 85 = power-on default
    t = v;
    return true;
  }
#elif SENSOR_TYPE == SENSOR_DHT
  #include <DHT.h>
  DHT dht(PIN_TEMP, DHT_TYPE);
  void sensorBegin() { dht.begin(); }
  bool sensorRead(float &t) {
    float v = dht.readTemperature();
    if (isnan(v)) return false;
    t = v;
    return true;
  }
#else
  #error "Unknown SENSOR_TYPE"
#endif

// ---------------- PWM output (LEDC, 25 kHz) ----------------
const uint32_t PWM_FREQ_HZ = 25000;
const uint8_t  PWM_BITS    = 8;
const uint32_t PWM_MAX     = (1u << PWM_BITS) - 1;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  void pwmBegin()            { ledcAttach(PIN_PWM, PWM_FREQ_HZ, PWM_BITS); }
  void pwmWrite(uint32_t v)  { ledcWrite(PIN_PWM, v); }
#else
  const uint8_t PWM_CH = 0;
  void pwmBegin()            { ledcSetup(PWM_CH, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(PIN_PWM, PWM_CH); }
  void pwmWrite(uint32_t v)  { ledcWrite(PWM_CH, v); }
#endif

// duty = fan duty 0..1. The NPN inverts the signal: GPIO high => fan PWM line low.
void fanOutput(bool on, float duty) {
  duty = constrain(duty, 0.0f, 1.0f);
  pwmWrite((uint32_t)lroundf((1.0f - duty) * PWM_MAX));
  digitalWrite(PIN_ENABLE, on ? HIGH : LOW);
}

// ---------------- Tachometer ----------------
volatile uint32_t tachCount  = 0;
volatile uint32_t tachLastUs = 0;

void IRAM_ATTR onTach() {
  uint32_t now = micros();
  if ((uint32_t)(now - tachLastUs) > 300) {   // ignore glitches < 300 us
    tachCount++;
    tachLastUs = now;
  }
}

float readRpm(uint32_t dtMs) {
  static uint32_t last = 0;
  uint32_t c = tachCount;
  uint32_t d = c - last;
  last = c;
  return d * 60000.0f / ((float)TACH_PPR * (float)dtMs);
}

float calibrateRpmMax() {
  fanOutput(true, 1.0f);
  delay(4000);              // spin-up
  readRpm(1000);            // reset the pulse baseline
  delay(2000);
  float rpm = readRpm(2000);
  Serial.printf("Calibration: max RPM = %.0f\n", rpm);
  if (rpm < 200.0f) {
    Serial.println("WARN: no usable tach signal, using RPM_MAX_DEFAULT");
    return RPM_MAX_DEFAULT;
  }
  return rpm;
}

// ---------------- Control state ----------------
float    rpmMax       = RPM_MAX_DEFAULT;
float    tempFiltered = NAN;
bool     tempValid    = false;
uint8_t  failCount    = 0;
bool     fanOn        = false;
float    integ        = 0.0f;
uint32_t kickUntil    = 0;
uint32_t lastTempMs   = 0;
uint32_t lastCtlMs    = 0;

void updateTemperature() {
  float t;
  if (sensorRead(t)) {
    failCount = 0;
    tempFiltered = isnan(tempFiltered) ? t : tempFiltered + TEMP_FILTER_ALPHA * (t - tempFiltered);
    tempValid = true;
  } else {
    if (failCount < 255) failCount++;
    if (failCount >= SENSOR_FAIL_LIMIT) tempValid = false;
  }
}

void controlStep(uint32_t now, uint32_t dtMs) {
  float rpm = readRpm(dtMs);
  bool  wantRun;
  float target = 0.0f;

  if (!tempValid) {                       // fail-safe: sensor lost
    wantRun = true;
    target  = rpmMax;
  } else {
    if (!fanOn && tempFiltered >= T0_C)               wantRun = true;
    else if (fanOn && tempFiltered < T0_C - T_HYST_C) wantRun = false;
    else                                              wantRun = fanOn;

    if (wantRun) {
      float x = constrain((tempFiltered - T0_C) / (T1_C - T0_C), 0.0f, 1.0f);
      target = rpmMax * (RPM_MIN_FRAC + (1.0f - RPM_MIN_FRAC) * x);
    }
  }

  float duty = 0.0f;

  if (!wantRun) {                         // stopped
    if (fanOn) { fanOutput(false, 0.0f); fanOn = false; }
    integ = 0.0f;
  } else if (!fanOn) {                    // start-up kick
    fanOn = true;
    integ = 0.0f;
    kickUntil = now + KICK_MS;
    duty = KICK_DUTY;
    fanOutput(true, duty);
  } else if ((int32_t)(kickUntil - now) > 0) {
    duty = KICK_DUTY;                     // still kicking, PI frozen
  } else {                                // feed-forward + PI on RPM
    float ff = target / rpmMax;
    float e  = (target - rpm) / rpmMax;
    integ += KI * e * (dtMs / 1000.0f);
    integ  = constrain(integ, -TRIM_LIMIT, TRIM_LIMIT);
    duty   = constrain(ff + KP * e + integ, 0.0f, 1.0f);
    fanOutput(true, duty);

    static uint32_t lowSince = 0;         // stall warning
    if (rpm < 0.3f * target) {
      if (lowSince == 0) lowSince = now;
      else if (now - lowSince > 5000) Serial.println("WARN: fan stalled or tach lost?");
    } else {
      lowSince = 0;
    }
  }

  Serial.printf("T=%.1f valid=%d target=%.0f rpm=%.0f duty=%.2f\n",
                tempFiltered, tempValid, target, rpm, duty);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);
  pwmBegin();
  pinMode(PIN_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), onTach, FALLING);
  sensorBegin();

  rpmMax = CALIBRATE_RPM_MAX ? calibrateRpmMax() : RPM_MAX_DEFAULT;
  fanOutput(false, 0.0f);

  lastTempMs = lastCtlMs = millis();
  enableLoopWDT();                        // reboot if loop() ever hangs
}

void loop() {
  uint32_t now = millis();
  if (now - lastTempMs >= TEMP_PERIOD_MS) {
    lastTempMs = now;
    updateTemperature();
  }
  if (now - lastCtlMs >= CTL_PERIOD_MS) {
    uint32_t dt = now - lastCtlMs;
    lastCtlMs = now;
    controlStep(now, dt);
  }
  delay(5);
}
