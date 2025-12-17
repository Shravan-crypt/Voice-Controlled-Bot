#include <WiFi.h>
#include <esp_now.h>

#define AIN1 25
#define AIN2 33
#define PWMA 32
#define BIN1 27
#define BIN2 14
#define PWMB 12
#define STBY 26

typedef struct struct_message {
  char command[10];
} struct_message;

struct_message incomingMessage;

void forward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
}

void backward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
}

void left() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
}

void right() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
}

void stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingMessage, data, sizeof(incomingMessage));
  Serial.print("Received: ");
  Serial.println(incomingMessage.command);

  if (strcmp(incomingMessage.command, "Forward") == 0) forward();
  else if (strcmp(incomingMessage.command, "Backward") == 0) backward();
  else if (strcmp(incomingMessage.command, "Left") == 0) left();
  else if (strcmp(incomingMessage.command, "Right") == 0) right();
  else stop();
}

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stop();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(onReceive);
}

void loop() {}
