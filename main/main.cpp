#include <stdio.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_psram.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dl_image_jpeg.hpp"
#include "pedestrian_detect.hpp"
#include <stdlib.h>
//marker line to figure out if user entered or exited
#define LineY 80

//Range in which centroid assumes its the same pedestrian
#define samePedestrianX 10
#define samePedestrianY 15

#define wifiSSID "YOUR_WIFI_SSID"
#define wifiPASSWORD "YOUR_WIFI_PASSWORD"

static bool g_has_psram = false;
static const char* TAG = "stream";


typedef struct {
    int x1;
    int x2;
    int y1;
    int y2;

    int centroidX;
    int centroidY;
} Pedestrian;


//setup for OV3660 camera
static void camera_init_or_abort() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  //parallel data bus (Y2..Y9)
  config.pin_d0 = 11;  // Y2
  config.pin_d1 = 9;   // Y3
  config.pin_d2 = 8;   // Y4
  config.pin_d3 = 10;  // Y5
  config.pin_d4 = 12;  // Y6
  config.pin_d5 = 18;  // Y7
  config.pin_d6 = 17;  // Y8
  config.pin_d7 = 16;  // Y9

  //clocks and sync
  config.pin_xclk = 15;
  config.pin_pclk = 13;
  config.pin_vsync = 6;
  config.pin_href = 7;

  //SCCB (I2C)
  config.pin_sccb_sda = 4;
  config.pin_sccb_scl = 5;

  //power/reset not wired
  config.pin_pwdn = -1;
  config.pin_reset = -1;

  //stable for OV3660; you can try 20000000 if your board is stable
  config.xclk_freq_hz = 10000000;

  //low-res raw frames for fast ML and overlay
  config.pixel_format = PIXFORMAT_GRAYSCALE; //fastest
  config.frame_size = FRAMESIZE_QQVGA;  //160x120
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  //not used for RGB565 capture, relevant only for PIXFORMAT_JPEG
  config.jpeg_quality = 40;


  //check if psram available to use for buffering
  size_t ps_bytes = esp_psram_get_size();
  g_has_psram = (ps_bytes > 0);
  if (g_has_psram) 
  {
    ESP_LOGI(TAG, "PSRAM detected: %u bytes", (unsigned)ps_bytes);
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
  } 
  else 
  {
    ESP_LOGW(TAG, "No PSRAM detected, using DRAM FB");
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
  }

  //initialize cam
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
    abort();
  }

  //tuning
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    ESP_LOGI(TAG, "Sensor PID: 0x%04x", s->id.PID);  // expect 0x3660
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
    // Ensure framesize is applied (usually not needed if set in config)
    s->set_framesize(s, FRAMESIZE_QQVGA);
  }
}


static void wifi_init()
{
  //initialize NVS
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  //intialize wifi with default config
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  wifi_config_t  wconfig = {
    .sta = {.ssid = wifiSSID, .password = wifiPASSWORD},  
  };

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wconfig);
  esp_wifi_start();

  ESP_LOGI(TAG, "wifi_init_sta finished.");
  ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", wifiSSID, wifiPASSWORD);

}


//calculate centroid for x or y depending on input
void calculateCentroid(Pedestrian* p1)
{
    int cx = (p1->x1 + p1->x2)/2;
    int cy = (p1->y1 + p1->y2)/2;
    p1->centroidX = cx;
    p1->centroidY = cy;
}


//check if pedestrian is same as last frame
//This is achieved by seeing if the centroids are closer enough especially on the X axis
bool samePedestrian(Pedestrian p1, Pedestrian p2)
{  
    int diffX = p1.centroidX - p2.centroidX;
    int diffY = p1.centroidY - p2.centroidY;


    //check if between ranges
    return (abs(diffX) <= samePedestrianX) && (abs(diffY) <= samePedestrianY);
}

 



extern "C" void app_main(void)
{
    
}