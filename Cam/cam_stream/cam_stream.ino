#include "esp_camera.h"
#include "fb_gfx.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_timer.h>
#include <time.h>

// ==== CAMERA PINOUT (ESP32-S3-CAM AI-Thinker style) ====
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      10
#define SIOD_GPIO_NUM      40
#define SIOC_GPIO_NUM      39

#define Y9_GPIO_NUM        39
#define Y8_GPIO_NUM        41
#define Y7_GPIO_NUM        42
#define Y6_GPIO_NUM        21
#define Y5_GPIO_NUM        38
#define Y4_GPIO_NUM        47
#define Y3_GPIO_NUM        48
#define Y2_GPIO_NUM        45
#define VSYNC_GPIO_NUM     2
#define HREF_GPIO_NUM      46
#define PCLK_GPIO_NUM      1

// ==== WIFI AP CONFIG ====
const char* ssid     = "ESP32CAM";
const char* password = "12345678";

WiFiServer server(80);

// ========== Camera setup ==========
void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;   // direct draw support

  config.frame_size = FRAMESIZE_QVGA; // 320x240 for speed
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    ESP.restart();
  }
}

// ========== Draw time ==========
void drawTime(camera_fb_t* fb) {
  // Wrap camera buffer as fb_data_t
  fb_data_t fb_data;
  fb_data.width = fb->width;
  fb_data.height = fb->height;
  fb_data.data = fb->buf;
  fb_data.bytes_per_pixel = 2;      // RGB565 = 2 bytes
  fb_data.format = FB_RGB565;       // important

  // Get uptime as hh:mm:ss
  unsigned long ms = millis();
  int sec = ms / 1000;
  int h = (sec / 3600) % 24;
  int m = (sec / 60) % 60;
  int s = sec % 60;

  char text[16];
  sprintf(text, "%02d:%02d:%02d", h, m, s);

  // Draw in bottom-left
  fb_gfx_print(&fb_data, 5, fb->height - 12, 0xFFFF, text);
}

// ========== HTTP Stream ==========
void streamTask(WiFiClient client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;

    // Draw overlay
    drawTime(fb);

    // Encode to JPEG
    uint8_t* jpg_buf = NULL;
    size_t jpg_len = 0;
    bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
    esp_camera_fb_return(fb);

    if (!ok) continue;

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", jpg_len);
    client.write(jpg_buf, jpg_len);
    client.print("\r\n");
    free(jpg_buf);

    delay(100); // ~10 fps
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  startCamera();

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

// ========== Main loop ==========
void loop() {
  WiFiClient client = server.available();
  if (client) {
    // Wait for HTTP request
    while (client.connected() && !client.available()) delay(1);
    String req = client.readStringUntil('\r');
    client.readStringUntil('\n'); // discard \n

    if (req.indexOf("GET /stream") >= 0) {
      streamTask(client);
    } else {
      // simple page with link
      client.println("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                     "<html><body><h1>ESP32-CAM</h1>"
                     "<img src='/stream'>"
                     "</body></html>");
    }
    client.stop();
  }
}
