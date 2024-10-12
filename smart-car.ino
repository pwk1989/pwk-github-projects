#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "esp_system.h"    // Include ESP system library
#include <driver/ledc.h>   // Include the LEDC driver

const char* ssid = "";
const char* password = "";

WebSocketsClient webSocket;

// Motor control pins
const int enA = 32; // PWM pin for speed control (Motor A)
const int in1 = 25;
const int in2 = 26;

const int enB = 33; // PWM pin for speed control (Motor B)
const int in3 = 27;
const int in4 = 14;

// Current speed level (0-255)
int speedLevel = 0; // Default to 0 (Neutral)

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins (direction pins)
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Prepare and configure the LEDC timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_8_BIT, // 8-bit resolution
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = 1000,             // 1 kHz frequency
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Configure LEDC channel for enA (Motor A)
  ledc_channel_config_t ledc_channel0 = {
    .gpio_num       = enA,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0, // Initial duty cycle
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel0);

  // Configure LEDC channel for enB (Motor B)
  ledc_channel_config_t ledc_channel1 = {
    .gpio_num       = enB,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_1,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0, // Initial duty cycle
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel1);

  // Stop motors initially
  stopMoving();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize WebSocket
  webSocket.begin("192.168.1.11", 3000, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  Serial.println("Connecting to WebSocket server...");
}

void loop() {
  webSocket.loop();
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      stopMoving();
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      break;
    case WStype_TEXT:
      {
        String message = String((char *)payload);
        Serial.println("Received message: " + message);
        handleCommand(message);
      }
      break;
    default:
      break;
  }
}

void handleCommand(String message) {
  // Parse JSON message
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  String cmd = doc["command"];
  int gear = doc["gear"];

  // Map gear level to speed (0-255)
  if (gear == 0) {
    speedLevel = 0;
  } else {
    speedLevel = map(gear, 1, 5, 51, 255); // Gear 1=51, Gear 5=255
  }

  if (speedLevel == 0 || cmd == "stop") {
    stopMoving();
    return;
  }

  if (cmd == "forward") {
    moveForward();
  } else if (cmd == "backward") {
    moveBackward();
  } else if (cmd == "left") {
    turnLeft();
  } else if (cmd == "right") {
    turnRight();
  } else if (cmd == "stop") {
    stopMoving();
  }
}

void setMotorSpeed(ledc_channel_t channel, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void moveForward() {
  setMotorSpeed(LEDC_CHANNEL_0, speedLevel);
  setMotorSpeed(LEDC_CHANNEL_1, speedLevel);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveBackward() {
  setMotorSpeed(LEDC_CHANNEL_0, speedLevel);
  setMotorSpeed(LEDC_CHANNEL_1, speedLevel);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft() {
  setMotorSpeed(LEDC_CHANNEL_0, speedLevel / 2);
  setMotorSpeed(LEDC_CHANNEL_1, speedLevel);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight() {
  setMotorSpeed(LEDC_CHANNEL_0, speedLevel);
  setMotorSpeed(LEDC_CHANNEL_1, speedLevel / 2);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopMoving() {
  setMotorSpeed(LEDC_CHANNEL_0, 0);
  setMotorSpeed(LEDC_CHANNEL_1, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
