// Pin definitions
const int PWM_PIN = 18;  
const int TACH_PIN = 27;  
const int POT_PIN = 34;  
 
// PWM settings
const int PWM_FREQ = 25000;   
const int PWM_RESOLUTION = 8;  
 
// Tachometer variables
volatile unsigned long tachPulseCount = 0;
unsigned long lastTachTime = 0;
const unsigned long TACH_SAMPLE_TIME = 1000;  // Sample period in milliseconds
 
// Interrupt service routine for tachometer
void IRAM_ATTR tachISR() {
  tachPulseCount = tachPulseCount + 1;
}
 
void setup() {
 
  // Start Serial Monitor
  Serial.begin(115200);
  Serial.println("Fan Control with Tachometer Starting...");
 
  // Configure PWM
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
 
  // Configure tachometer pin with interrupt
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);
 
  // Set initial fan speed to zero
  ledcWrite(PWM_PIN, 0);
 
  // Initialize timing
  lastTachTime = millis();
 
  Serial.println("Fan controller ready!");
  Serial.println("Turn potentiometer to adjust fan speed");
}
 
void loop() {
 
  // Read potentiometer value (0-4095 for 12-bit ADC)
  int potValue = analogRead(POT_PIN);
 
  // Convert to PWM duty cycle (0-255 for 8-bit resolution)
  int fanSpeed = map(potValue, 0, 4095, 0, 255);
 
  // Apply PWM to fan
  ledcWrite(PWM_PIN, 255-fanSpeed);
 
  // Calculate speed percentage
  int speedPercent = map(fanSpeed, 0, 255, 0, 100);
 
  // Calculate RPM every second
  unsigned long currentTime = millis();
  if (currentTime - lastTachTime >= TACH_SAMPLE_TIME) {
    // Calculate RPM (2 pulses per revolution for most fans)
    unsigned long rpm = (tachPulseCount * 60000) / (TACH_SAMPLE_TIME * 2);
 
    // Estimate target RPM based on fan speed (values for Noctua NF-A4x20)
    // This fan has max RPM of 5000, adjust for your specific fan
    unsigned long targetRPM = map(speedPercent, 0, 100, 0, 5000);
 
    // Display results
    Serial.print("Pot: ");
    Serial.print(potValue);
    Serial.print(" | PWM: ");
    Serial.print(fanSpeed);
    Serial.print(" | Speed: ");
    Serial.print(speedPercent);
    Serial.print("% | Target: ");
    Serial.print(targetRPM);
    Serial.print(" RPM | Actual: ");
    Serial.print(rpm);
    Serial.println(" RPM");
 
    // Reset counters
    tachPulseCount = 0;
    lastTachTime = currentTime;
  }
 
  delay(50);  // Small delay for stable readings
}
