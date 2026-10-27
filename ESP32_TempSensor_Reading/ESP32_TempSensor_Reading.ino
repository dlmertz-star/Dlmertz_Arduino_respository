#include <DHT.h>

#define DHTPIN 14
#define DHTTYPE DHT11

const int FAN_PWM_PIN = 25:      // pick your PWM-capable GPIO
const int FAN_TACH_PIN = 26;
const int pwmFreq = 25000;  // 25 kHz is standard for 4-pin PC fans
const int pwmRes = 8;       // 8-bit resolution: 0-255
const int FAN_MAX_RMP=4000;
const int FAN_MIN_RMP=1500;
const int FAN_PULSES_PER_REV

float Kp = 0.08;
float Ki = 0.02;
// MIN_TEMP=
// MAX_TEMP=

DHT dht(DHTPIN,DHTTYPE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  ledcAttach(fanPin, pwmFreq, pwmRes);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);

  float temp=dht.readTemperature();
  float hum=dht.readHumidity();

  if(isnan(temp) || isnan(hum)) {
    Serial.println("Failed to read from DHT11!");
    return; 
  }
  ledcWrite(fanPin, dutyCycle);  // dutyCycle: 0-255
  // if (temp>MAX_TEMP) then fan speed= 100%
  // if (temp< MIN_TEMP) then fan speed= 0%
  // else then fan speed 

  Serial.printf("Temp: %.1f C   Humidity: %.1f %%/n", temp, hum);
}
