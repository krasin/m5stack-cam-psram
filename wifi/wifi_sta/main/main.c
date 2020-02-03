#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_event_loop.h"
#include "esp_http_server.h"
#include "config.h"

static const char* TAG = "camera";
#define CAM_USE_WIFI

// TODO: define WiFi SSID and password
//#define ESP_WIFI_SSID ""
//#define ESP_WIFI_PASS ""

#define MAX_STA_CONN  1

static EventGroupHandle_t s_wifi_event_group;
static esp_ip4_addr_t s_ip_addr;
const int CONNECTED_BIT = BIT0;
extern void led_brightness(int duty);

// Forward declare cam clock stop/start. Note (new?)  signature.
esp_err_t camera_enable_out_clock(camera_config_t *config);
void camera_disable_out_clock();

static bool is_camera_initialized = false;

static camera_config_t camera_config = {
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz
    .xclk_freq_hz = CAM_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_SVGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 15, //0-63 lower number means higher quality
    .fb_count = 2 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

static void wifi_init_softap();
static esp_err_t http_server_init();

bool ensure_camera_init() {
  if (is_camera_initialized) {
    ESP_LOGI(TAG, "ensure_camera_init: already initialized\n");
    ESP_LOGI(TAG, "enabling camera out clock...\n");
    camera_enable_out_clock(&camera_config);
    ESP_LOGI(TAG, "camera out clock enabled!\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    return true;
  }

  esp_err_t err = esp_camera_init(&camera_config);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ensure_camera_init: camera Init Failed");
    return false;
  }

  is_camera_initialized = true;

#ifdef FISH_EYE_CAM
  // flip img, other cam setting view sensor.h
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
  ESP_LOGI(TAG, "ensure_camera_init: successfully initialized\n");
  return true;
}

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_INFO);
    
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

#ifdef CAM_USE_WIFI
    wifi_init_softap();
    led_brightness(20);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    http_server_init();
#endif
}

#ifdef CAM_USE_WIFI

static char kErrCameraInitFailed[] = "camera failed to initialize!";

esp_err_t jpg_httpd_handler(httpd_req_t *req){
    esp_err_t res = ESP_OK;
    bool ok = ensure_camera_init();
    if (!ok) {
      ESP_LOGE(TAG, "Camera initialization failed");
      res = httpd_resp_set_type(req, "text/html");
      if(res == ESP_OK) {
	res = httpd_resp_send(req, kErrCameraInitFailed, strlen(kErrCameraInitFailed));
      }
      return;
    }
    camera_fb_t * fb = NULL;
    size_t fb_len = 0;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    res = httpd_resp_set_type(req, "image/jpeg");
    if(res == ESP_OK){
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }

    if(res == ESP_OK){
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPG: %uKB %ums", (uint32_t)(fb_len/1024), (uint32_t)((fr_end - fr_start)/1000));
    camera_disable_out_clock();
    ESP_LOGI(TAG, "JPG: camera clock disabled");
    return res;
}

static esp_err_t http_server_init(){
    httpd_handle_t server;
    httpd_uri_t jpeg_uri = {
        .uri = "/jpg",
        .method = HTTP_GET,
        .handler = jpg_httpd_handler,
        .user_ctx = NULL
    };

    httpd_config_t http_options = HTTPD_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(httpd_start(&server, &http_options));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &jpeg_uri));

    return ESP_OK;
}

static esp_err_t event_handler(void* ctx, system_event_t* event) 
{
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
      s_ip_addr = event->event_info.got_ip.ip_info.ip;
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d", MAC2STR(event->event_info.sta_connected.mac),
               event->event_info.sta_connected.aid);
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d", MAC2STR(event->event_info.sta_disconnected.mac),
               event->event_info.sta_disconnected.aid);
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    default:
      break;
  }
  return ESP_OK;
}

static void wifi_init_softap() 
{
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
      .sta = {
           .ssid = ESP_WIFI_SSID,
           .password = ESP_WIFI_PASS
      },
  };
  ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

#endif
