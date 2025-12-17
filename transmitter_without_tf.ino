#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

#define I2S_WS  15
#define I2S_SD  32
#define I2S_SCK 14

uint8_t receiverMac[] = {0x14, 0x33, 0x5C, 0x0B, 0x66, 0x64};

typedef struct struct_message {
  char command[10];
} struct_message;

struct_message message;

//
const int SAMPLES = 512;
const double SAMPLE_RATE = 16000;

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT FFT = ArduinoFFT(vReal,vImag, SAMPLES, SAMPLE_RATE);
//

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  setupMic();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  //
  Serial.println("FFT voice Sender ready");

}

void setupMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_start(I2S_NUM_0);
}

//
String classifyCommand() {
  double forwardEnergy = 0;
  double backwardEnergy = 0;
  double leftEnergy = 0;
  double rightEnergy = 0;

  for (int i = 1; i < SAMPLES / 2; i++) {
    double freq = (i * SAMPLE_RATE) / SAMPLES;
    double mag = vReal[i];

    if (freq >= 300 && freq <= 800) forwardEnergy += mag;
    if (freq >= 500 && freq <= 1200) backwardEnergy += mag;
    if (freq >= 200 && freq <= 600) leftEnergy += mag;
    if (freq >= 700 && freq <= 1500) rightEnergy += mag;
  }
  Serial.printf("F: %.0f | B: %.0f | L: %.0f | R: %.0f\n", forwardEnergy, backwardEnergy, leftEnergy, rightEnergy);

  double maxVal = max(max(forwardEnergy, backwardEnergy), max(leftEnergy, rightEnergy));
  if (maxVal == forwardEnergy) return "Forward";
  if (maxVal == backwardEnergy) return "Backward";
  if (maxVal == leftEnergy) return "Left";
  if (maxVal == rightEnergy) return "Right";
}
//

void loop() {
  /*int32_t samples[512];
  size_t bytes_read;
  i2s_read(I2S_NUM_0, (char*)samples, sizeof(samples), &bytes_read, portMAX_DELAY);

  long sum = 0;
  for (int i = 0; i < bytes_read / 4; i++) sum += abs(samples[i]);
  float avg = sum / (bytes_read / 4);

  if (avg > 5000000) {
    strcpy(message.command, "Backward"); 
  } else {
    strcpy(message.command, "Stop"); 
  }*/

  //
  int32_t sample32;
  size_t readBytes;

  for (int i=0; i < SAMPLES; i++) {
    i2s_read(I2S_NUM_0, &sample32, sizeof(sample32), &readBytes, portMAX_DELAY);
    vReal[i] = sample32 / 50000.0;
    vImag[i] = 0;
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  String result = classifyCommand();
  strcpy(message.command, result.c_str());
  //

  esp_now_send(receiverMac, (uint8_t *)&message, sizeof(message));
  //Serial.printf("Sent: %s | avg: %.0f\n", message.command, avg);
  Serial.println("Sent Command: " + result);

  delay(5000);
}
