#include <esp32-hal-ledc.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2 
//Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.
#define DHTTYPE DHT11  

const int fanPwmPin =27;   // Yellow wire
const int fanTachPin = 25;  // Blue wire

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Volatile variables used in the Interrupt Service Routine (ISR)
volatile unsigned long pulseCount = 0;
unsigned long lastMillis = 0;
unsigned long speedChangeMillis = 0;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int currentSpeedStep = 0;
const int speeds[] = {32,64,128,255}; // ~25%, ~50%, 100% duty cycle

// Replace the ISR-based approach with this polling method, called every loop() iteration
int lastStableState = HIGH;
unsigned long lastChangeTime = 0;
const unsigned long STABLE_TIME_US = 500; // pin must hold new state this long to count

void pollTach() {
  int reading = digitalRead(fanTachPin);
  unsigned long now = micros();

  if (reading != lastStableState) {
    if (now - lastChangeTime > STABLE_TIME_US) {
      if (reading == LOW) {  // falling edge confirmed stable
        pulseCount++;
      }
      lastStableState = reading;
      lastChangeTime = now;
    }
  } else {
    lastChangeTime = now; // reset timer while state is unchanged
  }
}
// Interrupt Service Routine - runs every time the pulse transitions from HIGH to LO
void displayRPM(unsigned long num) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("RPM: "); 
  display.println(num);
  display.display();
}
void setup() {
  Serial.begin(115200);
  Wire.begin(32, 33);

  Serial.print("Tach pin in use: ");
  Serial.println(fanTachPin);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (true); // halt here rather than crashing later
  }
  display.clearDisplay();
  display.display();

  // Set up PWM (25 kHz, 8-bit resolution)
  pinMode(fanPwmPin, OUTPUT);
  ledcAttach(fanPwmPin, 25000, 8); 
  
  // Set up Tachometer pin with internal pull-up and attach interrupt
  pinMode(fanTachPin, INPUT_PULLUP);
  
  // Set initial fan speed
  ledcWrite(fanPwmPin, speeds[currentSpeedStep]);
  Serial.println("Fan Control Initialized.");
}

void loop() {
  pollTach();
  unsigned long currentMillis = millis();
  
  // Calculate and print RPM once every second
  if (currentMillis - lastMillis >= 1000) {
    // Temporarily detach interrupt or read volatile safely
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0; // Reset count for the next second
    interrupts();
    
    // Formula: (Pulses per second * 60 seconds) / 2 pulses per revolution
    unsigned long rpm = (pulses * 60) / 2;
    Serial.print("Raw pulses: ");
    Serial.print(pulses);
    Serial.print(" | RPM: ");
    Serial.println(rpm);
    displayRPM(rpm);
    
    lastMillis = currentMillis;
  }
  
  // Change the speed step every 5 seconds
  if (currentMillis - speedChangeMillis >= 10000) {
    currentSpeedStep = (currentSpeedStep + 1) % 4;
    ledcWrite(fanPwmPin, speeds[currentSpeedStep]);
    speedChangeMillis = currentMillis;
    Serial.print("--> Changing speed to PWM duty: ");
    Serial.println(speeds[currentSpeedStep]);
  }
}
