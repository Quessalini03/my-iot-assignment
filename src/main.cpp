#define CONFIG_THINGSBOARD_ENABLE_DEBUG false
#include <Arduino.h>
#include <DHT20.h>
#include <Wire.h>
#include <pins_arduino.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
// #include <OTA_Firmware_Update.h>
#include <ThingsBoard.h>
// #include <Shared_Attribute_Update.h>
// #include <Attribute_Request.h>
#include <Espressif_Updater.h>
// #include <Server_Side_RPC.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//Shared Attributes Configuration
constexpr uint8_t MAX_ATTRIBUTES = 2U; //
constexpr std::array<const char*, MAX_ATTRIBUTES> 
SHARED_ATTRIBUTES = 
{
  "POWER",
  "ledState"
};

int translate(int value, int fromLow, int fromHigh, int toLow, int toHigh);

// ==============  Connections  ==============
constexpr int16_t TELEMETRY_SEND_INTERVAL = 5000U;
uint32_t previousTelemetrySend; 

constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char TOKEN_MCU[] = "drk0og684x3iblpov2pp";
constexpr char TOKEN_FAN[] = "q4aq498zp7smm08j62jj";
constexpr char TOKEN_TEMP_HUMID[] = "q4aq498zp7smm08j62jj";
constexpr char TOKEN_LIGHT[] = "9pmlxcmj7jaclqnji59n";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr char TEMPERATURE_KEY[] = "temperature";
constexpr char HUMIDITY_KEY[] = "humidity";
constexpr char LIGHT_KEY[] = "light";
constexpr char FAN_KEY[] = "speed";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 1024;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 1024;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 10000U * 1000U;
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
// constexpr char TELEMETRY_TOPIC[] = "v1/devices/me/telemetry";
// constexpr char RPC_REQUEST_TOPIC[] = "v1/devices/me/rpc/request/+";
// constexpr char RPC_RESPONSE_TOPIC[] = "v1/devices/me/rpc/response/";

// ===============  Constants  ==============
constexpr int16_t telemetrySendInterval = 10000U;

// ==============  Firmware Related  ==============

// Firmware title and version used to compare with remote version, to check if an update is needed.
// Title needs to be the same and version needs to be different --> downgrading is possible
constexpr char CURRENT_FIRMWARE_TITLE[] = "BLINKY";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.1";
// Maximum amount of retries we attempt to download each firmware chunck over MQTT
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
// Size of each firmware chunck downloaded over MQTT,
// increased packet size, might increase download speed
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;

// ==============  Connecting  ==============
void requestTimedOut() {
  Serial.printf("Attribute request timed out did not receive a response in (%llu) microseconds. Ensure client is connected to the MQTT broker and that the keys actually exist on the target device\n", REQUEST_TIMEOUT_MICROSECONDS);
}
// Initialize underlying client, used to establish a connection
WiFiClient espClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
PubSubClient rpcClient(espClient);
// // Initialize used apis
// OTA_Firmware_Update<> ota;
// Shared_Attribute_Update<1U, MAX_ATTRIBUTES> shared_update;
// Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
// Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
// const std::array<IAPI_Implementation*, 4U> apis = {
//     &rpc,
//     &shared_update,
//     &attr_request,
//     &ota
// };

// Initialize ThingsBoard instance with the maximum needed buffer size
// ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);
// ThingsBoard tb2(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE);
// Initalize the Updater client instance used to flash binary to flash memory
// Espressif_Updater<> updater;
// Statuses for updating
// bool shared_update_subscribed = false;
// bool currentFWSent = false;
// bool updateRequestSent = false;
// bool requestedShared = false;

ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void update_starting_callback() {
}
void finished_callback(const bool & success) {
  if (success) {
    Serial.println("Done, Reboot now");
    esp_restart();
    return;
  }
  Serial.println("Downloading firmware failed");
}
void progress_callback(const size_t & current, const size_t & total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}
void processSharedAttributeUpdate(const JsonObjectConst &data) {
  //Info
  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
}

void processSharedAttributeRequest(const JsonObjectConst &data) {
  //Info
  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
}

// ==============  Pin Definitions  ==============
const int fanPin = GPIO_NUM_6; // Fan control pin
const int lightSensorPin = GPIO_NUM_2; // Light control pin
const int ledPin = 48; // LED control pin
const int tempHumidityPin[] = {GPIO_NUM_11, GPIO_NUM_12}; // DHT20 SDA and SCL pins

// ==============  Devices Attributes  ==============
bool attributesChanged = false; // Flag to indicate if attributes have changed
float fanSpeed = 0.0; // Fan speed percentage

// ==============  RPCs  ==============
RPC_Response setFanSpeed(const RPC_Data &data){
    Serial.println("Received Fan speed");
    float newSpeed = data;
    fanSpeed = newSpeed; // Update the global fan speed variable
    Serial.print("Fan speed changed: ");
    Serial.println(newSpeed);
    int pwmValue = translate((int) newSpeed, 0, 100, 0, 1023); 
    
    pinMode(fanPin, OUTPUT);
    analogWrite(fanPin, pwmValue * 255 / 1023);  // Adjust to 8-bit if needed
    attributesChanged = true;
    return RPC_Response("setFanSpeed", newSpeed);
}

// int ledMode = 0; // 0 for OFF, 1 for ON

RPC_Response setLEDState(const RPC_Data &data) {
    Serial.println("Received LED state");
    bool newState = data;
    Serial.print("LED state changed: ");
    Serial.println(newState);
    
    digitalWrite(ledPin, newState);
    
    attributesChanged = true;
    return RPC_Response("setLEDState", newState);
}

// bool setLEDState(bool newState) {
//     Serial.print("Setting LED state to: ");
//     Serial.println(newState ? "ON" : "OFF");
//     digitalWrite(ledPin, newState ? HIGH : LOW);
//     return true;
// } 

// float setFanSpeed(float newSpeed) {
//     Serial.print("Setting Fan speed to: ");
//     Serial.println(newSpeed);
//     int pwmValue = translate((int)newSpeed, 0, 100, 0, 1023); 
//     analogWrite(fanPin, pwmValue * 255 / 1023);  // Adjust to 8-bit if needed
//     return newSpeed;
// }

const std::array<RPC_Callback, 2U> callbacks = {
  RPC_Callback{ "setFanSpeed", setFanSpeed },
  RPC_Callback{ "setLEDState", setLEDState }
};

void taskCoreIoTConnect(void *pvParameters) {
  bool rpcSubscribed = false;
  while(1) {
    if (WiFi.status() != WL_CONNECTED) {
    }  
    else if (!tb.connected()) {
      // Reconnect to the ThingsBoard server,
      // if a connection was disrupted or has not yet been established
      Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN_MCU);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN_MCU, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        return;
      }

      if (!rpcSubscribed) {
        Serial.println("Subscribing for RPC...");
        if (!tb.RPC_Subscribe(callbacks.begin(), callbacks.end())) {
          Serial.println("Failed to subscribe for RPC");
          return;
        }
        Serial.println("RPC Subscription done");
        rpcSubscribed = true;
      }
    //   if (!requestedShared) {
    //     Serial.println("Requesting shared attributes...");
    //     const Attribute_Request_Callback<MAX_ATTRIBUTES> sharedCallback(&processSharedAttributeRequest, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, SHARED_ATTRIBUTES);
    //     requestedShared = attr_request.Shared_Attributes_Request(sharedCallback);
    //     if (!requestedShared) {
    //       Serial.println("Failed to request shared attributes");
    //     }
    //   }

    // if (!shared_update_subscribed){
    //     Serial.println("Subscribing for shared attribute updates...");
    //     const Shared_Attribute_Callback<MAX_ATTRIBUTES> callback(&processSharedAttributeUpdate, SHARED_ATTRIBUTES);
    //     if (!shared_update.Shared_Attributes_Subscribe(callback)) {
    //     Serial.println("Failed to subscribe for shared attribute updates");
    //     // continue;
    //     }
    //     Serial.println("Subscribe done");
    //     shared_update_subscribed = true;
    //   }
    // }  
    // if (!currentFWSent) {
    //   currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
    // }
    // if (!updateRequestSent) {
    //   Serial.print(CURRENT_FIRMWARE_TITLE);
    //   Serial.println(CURRENT_FIRMWARE_VERSION);
    //   Serial.println("Firwmare Update ...");
    //   const OTA_Update_Callback callback(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION, &updater, &finished_callback, &progress_callback, &update_starting_callback, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
    //   updateRequestSent = ota.Start_Firmware_Update(callback);
    //   if(updateRequestSent) {
    //     delay(500);
    //     Serial.println("Firwmare Update Subscription...");
    //     updateRequestSent = ota.Subscribe_Firmware_Update(callback);
    //   }
    }

    // if (WiFi.status() != WL_CONNECTED) {
    // }
    // else if (!rpcClient.connected()) {
    //   reconnect();
    // }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Map (0–100) to 0–1023 for PWM output
int translate(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
  return map(value, fromLow, fromHigh, toLow, toHigh);
}

void TaskLEDControl(void *pvParameters) {
  int ledState = 0;
  while(1) {
    if (ledState == 0) {
      digitalWrite(ledPin, HIGH); // Turn ON LED
    } else {
      digitalWrite(ledPin, LOW); // Turn OFF LED
    }
    ledState = 1 - ledState;
    vTaskDelay(6000 / portTICK_PERIOD_MS); // Delay for 6 seconds
  }
}

void TaskFanControl(void *pvParameters) {
  int pwmValue = 0;
  while(1) {

    Serial.print("Fan Speed: ");
    Serial.print(fanSpeed);
    Serial.println(" %");

    if (tb.connected()) {
      // Send telemetry data to ThingsBoard
      const Telemetry fanTelemetry(FAN_KEY, fanSpeed);
      const Telemetry deviceTelemetry("device", "Fan");
      const Telemetry telemetryData[] = {fanTelemetry, deviceTelemetry};
      
      tb.sendTelemetry(telemetryData, 2);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskTemperature_Humidity(void *pvParameters){
  DHT20 dht20;
  // Wire.begin(GPIO_NUM_11, GPIO_NUM_12);
  Wire.begin(tempHumidityPin[0], tempHumidityPin[1]);
  dht20.begin();
  uint32_t previousTemperatureHumiditySend = millis();
  while(1){
    if (millis() - previousTemperatureHumiditySend > telemetrySendInterval) {
      previousTemperatureHumiditySend = millis();

      dht20.read();
      
      float temperature = dht20.getTemperature();
      float humidity = dht20.getHumidity();

      if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT20 sensor!");
      } else {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");

        if (tb.connected()) {
          const Telemetry tempTelemetry(TEMPERATURE_KEY, temperature);
          const Telemetry humidityTelemetry(HUMIDITY_KEY, humidity);
          const Telemetry deviceTelemetry("device", "HuTem");
          const Telemetry telemetryData[] = {tempTelemetry, humidityTelemetry, deviceTelemetry};

          tb.sendTelemetry(telemetryData, 3);
        }
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
  }

}

void TaskLight(void *pvParameters) {
  uint32_t previousLightSend = millis();
  while(1) {
    if (millis() - previousLightSend > telemetrySendInterval) {
      previousLightSend = millis();

      uint16_t lightRaw = analogRead(lightSensorPin);
      long light = map(lightRaw, 0, 4095, 0, 100);
      Serial.print("Light: "); Serial.print(light); Serial.println(" %");
      Serial.println();

      if (tb.connected()) {
        // Send telemetry data to ThingsBoard
        const Telemetry lightTelemetry(LIGHT_KEY, light);
        const Telemetry deviceTelemetry("device", "Light");
        const Telemetry telemetryData[] = {lightTelemetry, deviceTelemetry};
        
        tb.sendTelemetry(telemetryData, 2);
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
  }
}

void taskWifiControl(void *pvParameters) {
  delay(10);
  Serial.println("taskWifiControl started");
  wl_status_t status = WiFi.status();
  while(1) {
    status = WiFi.status();
    if (status != WL_CONNECTED) {
        InitWiFi();
        Serial.print("Reconnecting...");
    }
    // Serial.print("Checking wifi...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void taskThingsBoard(void *pvParameters) {
  while(1) {
    if (WiFi.status() == WL_CONNECTED) {
      tb.loop();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(ledPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(lightSensorPin, INPUT);
  
  delay(1000);
  xTaskCreate(taskWifiControl, "Wifi Control", 4096, NULL, 2, NULL);
  xTaskCreate(taskCoreIoTConnect, "Core IoT Connect", 20480, NULL, 2, NULL);
  xTaskCreate(taskThingsBoard, "ThingsBoard", 2048, NULL, 2, NULL);
  xTaskCreate(TaskTemperature_Humidity, "Temperature & Humidity", 2049, NULL, 2, NULL);
  xTaskCreate(TaskLight, "Light Control", 2049, NULL, 2, NULL);
  xTaskCreate(TaskFanControl, "Fan Control", 2048, NULL, 2, NULL);
  // xTaskCreate(TaskLEDControl, "LED Control", 2049, NULL, 2, NULL);
  
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to prevent busy loop
}