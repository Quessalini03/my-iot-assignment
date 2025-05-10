#include <Arduino.h>
#include <DHT20.h>
#include <Wire.h>
#include <pins_arduino.h>


const int fanPin = GPIO_NUM_6; // Fan control pin
const int lightPin = GPIO_NUM_2; // Light control pin
const int ledPin = GPIO_NUM_48; // LED control pin

// Map 70% (0–100) to 0–1023 for PWM output
int translate(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
  return map(value, fromLow, fromHigh, toLow, toHigh);
}

void TaskLEDControl(void *pvParameters) {
  pinMode(ledPin, OUTPUT); // Initialize LED pin
  int ledState = 0;
  while(1) {
    if (ledState == 0) {
      digitalWrite(ledPin, HIGH); // Turn ON LED
    } else {
      digitalWrite(ledPin, LOW); // Turn OFF LED
    }
    ledState = 1 - ledState;
    vTaskDelay(6000);
  }
}

void TaskFanControl(void *pvParameters) {
  pinMode(fanPin, OUTPUT);
  int fanState = 0;
  int pwmValue = 0;
  while(1) {
    if (fanState == 0) {
      pwmValue = translate(50, 0, 100, 0, 1023);
    } else {
      pwmValue = translate(0, 0, 100, 0, 1023); 
    }
    analogWrite(fanPin, pwmValue * 255 / 1023);  // Adjust to 8-bit if needed
    fanState = 1 - fanState;
    vTaskDelay(2000);
  }
}

void TaskTemperature_Humidity(void *pvParameters){
  DHT20 dht20;
  Wire.begin(GPIO_NUM_11, GPIO_NUM_12);
  dht20.begin();
  while(1){
    dht20.read();
    double temperature = dht20.getTemperature();
    double humidity = dht20.getHumidity();

    Serial.print("Temp: "); Serial.print(temperature); Serial.print(" *C ");
    Serial.print(" Humidity: "); Serial.print(humidity); Serial.print(" %");
    Serial.println();
    
    vTaskDelay(5000);
  }

}

void TaskLight(void *pvParameters) {
  pinMode(lightPin, INPUT);
  while(1) {
    uint16_t lightRaw = analogRead(lightPin);
    long light = map(lightRaw, 0, 4095, 0, 100);
    Serial.print("Light: "); Serial.print(light); Serial.println(" %");
    Serial.println();
    vTaskDelay(2000);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200U);
  xTaskCreate(TaskLEDControl, "LED Control", 2048, NULL, 2, NULL);
  xTaskCreate(TaskTemperature_Humidity, "Temperature & Humidity", 2048, NULL, 2, NULL);
  xTaskCreate(TaskFanControl, "Fan Control", 2048, NULL, 2, NULL);
  xTaskCreate(TaskLight, "Light Control", 2048, NULL, 2, NULL);
  
}

void loop() {
  
}