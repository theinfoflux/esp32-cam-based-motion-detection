#include "esp_camera.h"
#include <WiFi.h>

// Replace with your WiFi credentials
const char* ssid = "AndroidAP3DEC";
const char* password = "12345678";

// CAMERA_MODEL_AI_THINKER pinout
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Flash LED pin
#define FLASH_LED_PIN 4  

// Motion detection settings
const float pixelThreshold = 20.0;        // avg pixel difference threshold
const int minConsecutiveFrames = 3;       // must detect motion for 3 frames
const unsigned long cooldownTime = 5000;  // 5 sec cooldown

camera_fb_t *prev_frame = NULL;
int motionFrameCount = 0;
unsigned long lastMotionTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;   // grayscale for comparison

  if(psramFound()){
    config.frame_size = FRAMESIZE_QQVGA;  // 160x120 fast + enough
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.fb_count = 1;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  camera_fb_t *current_frame = esp_camera_fb_get();
  if (!current_frame) {
    Serial.println("Camera capture failed");
    return;
  }

  if (prev_frame != NULL) {
    long totalDiff = 0;
    int count = 0;

    // Compare fewer pixels (every 20th pixel → faster, less noise)
    for (size_t i = 0; i < current_frame->len; i += 20) {
      totalDiff += abs(current_frame->buf[i] - prev_frame->buf[i]);
      count++;
    }

    float avgDiff = (float)totalDiff / count;
    Serial.printf("Avg pixel diff: %.2f\n", avgDiff);

    // Motion detection logic
    if (avgDiff > pixelThreshold) {
      motionFrameCount++;
    } else {
      motionFrameCount = 0;
    }

    if (motionFrameCount >= minConsecutiveFrames && millis() - lastMotionTime > cooldownTime) {
      Serial.println("🚨 Motion Detected!");
      digitalWrite(FLASH_LED_PIN, HIGH);
      delay(1000);
      digitalWrite(FLASH_LED_PIN, LOW);

      lastMotionTime = millis();  // reset cooldown
      motionFrameCount = 0;       // reset
    }

    esp_camera_fb_return(prev_frame);
  }

  prev_frame = current_frame;
}
