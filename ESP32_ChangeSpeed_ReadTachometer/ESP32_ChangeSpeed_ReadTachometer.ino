#include <Arduino.h>

// Pin Definitions
const int PWM_PIN = 18;
const int TACH_PIN = 19;

// Fan Specifications
const int MAX_FAN_RPM = 5000; 
const int MIN_FAN_RPM = 1500; 

// PWM Configuration (ESP32 Arduino Core 3.0+ Compatible)
const int PWM_FREQ = 25000;    
const int PWM_RES = 8;         

// Global Variables for Tracking Pulse Counter
volatile unsigned long pulseCount = 0;

volatile unsigned long lastPulseTime = 0;
const unsigned long DEBOUNCE_TIME_US = 4000; // 4ms lockout (~5000 RPM max limit safety)

// Replace your old tachISR() function with this one:
void IRAM_ATTR tachISR() {
  unsigned long currentTime = esp_timer_get_time(); // Get precise time in microseconds
  if (currentTime - lastPulseTime >= DEBOUNCE_TIME_US) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
}

void loop() {
  // Target duty cycles we WANT the fan to see (25%, 50%, 75%, 100%)
  int targetDutyCycles[] = {64, 128, 192, 255}; 

  for (int i = 0; i < 4; i++) {
    int targetDuty = targetDutyCycles[i];
    float targetPercent = (targetDuty / 255.0) * 100.0;
    
    // INVERSION: Flip the duty cycle to compensate for the transistor inverter circuit
    int invertedDuty = 255 - targetDuty;
    
    // 1. Command the inverted PWM change to the ESP32 pin
    ledcWrite(PWM_PIN, invertedDuty);
    
    // 2. Wait 5 seconds for the physical fan blades to stabilize
    Serial.print("\nSetting PWM Target to "); Serial.print(targetPercent, 1); Serial.println("%... Waiting for stabilization.");
    delay(5000); 

    // 3. Measure RPM over a 1-second window
    pulseCount = 0;
    delay(1000); 
    
    // Calculate actual RPM based on 2 pulses per revolution
    unsigned long readings = pulseCount; 
    float actualRPM = (readings * 60.0) / 2.0;
    
    // Calculate expected RPM across the 1500 to 5000 RPM range
    float expectedRPM = MIN_FAN_RPM + ((targetPercent / 100.0) * (MAX_FAN_RPM - MIN_FAN_RPM));

    // 4. Print results
    Serial.println("====== AUDIT RESULTS ======");
    Serial.print("Target PWM    : "); Serial.print(targetPercent, 1); Serial.println("%");
    Serial.print("ESP32 Out PWM : "); Serial.print((invertedDuty / 255.0) * 100.0, 1); Serial.println("% (Inverted)");
    Serial.print("Expected RPM  : ~"); Serial.println(expectedRPM, 0);
    Serial.print("Actual RPM    : "); Serial.println(actualRPM, 0);
    
    // Calculate Deviation
    float variance = ((actualRPM - expectedRPM) / expectedRPM) * 100.0;
    Serial.print("Discrepancy   : "); Serial.print(variance, 1); Serial.println("%");
  }
  
  delay(10000);
}