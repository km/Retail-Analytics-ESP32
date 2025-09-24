    #include <stdio.h>
    #include <string.h>
    #include <vector>
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
    #include "esp_task_wdt.h"
    #include "dl_image_jpeg.hpp"
    #include "pedestrian_detect.hpp"
    #include <stdlib.h>
    #include <queue>
    
    //marker line to figure out if user entered or exited
    #define LineY 60

    //Range in which centroid assumes its the same pedestrian
    #define samePedestrianX 10
    #define samePedestrianY 15
    #define pedestrianThreshold 0.75

    #define wifiSSID ""
    #define wifiPASSWORD ""
    #define wifiCONNECTEDBIT BIT0
    #define wifiFAILBIT      BIT1

    //too track condition of wifi
    static EventGroupHandle_t s_wifi_event_group;

    static int s_retry_num = 0;
    static bool g_has_psram = false;
    static const char* TAG = "stream";

    //queues
    QueueHandle_t camera_queue;  //Camera to ML
    QueueHandle_t stream_queue;  //ML to Stream


    //pedestrian detector
    static PedestrianDetect* pmodel = nullptr;
    struct Pedestrian {
        int x1;
        int x2;
        int y1;
        int y2;
        int centroidX;
        int centroidY;

        Pedestrian() = default;

        Pedestrian& operator=(const Pedestrian& other) = default;


    };

    typedef struct {
        uint8_t *buf;
        size_t len;
    } jpeg_frame;


    std::vector<Pedestrian> currentPedestrians;

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
    bool samePedestrian(const Pedestrian& p1, const Pedestrian& p2)
    {  
        int diffX = p1.centroidX - p2.centroidX;
        int diffY = p1.centroidY - p2.centroidY;


        //check if between ranges
        return (abs(diffX) <= samePedestrianX) && (abs(diffY) <= samePedestrianY);
    }




    //convert RGB888 to RGB565 format
    inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }

    //precomputed circle mask to do less calcuations and avoid nested loops
    static const int8_t circle_offsets_r3[][2] = {
    { 0, 0},
    {-1, 0}, { 1, 0}, { 0,-1}, { 0, 1},
    {-1,-1}, {-1, 1}, { 1,-1}, { 1, 1},
    {-2, 0}, { 2, 0}, { 0,-2}, { 0, 2},
    {-2,-1}, { 2,-1}, {-2, 1}, { 2, 1},
    {-1,-2}, { 1,-2}, {-1, 2}, { 1, 2},
    {-3, 0}, { 3, 0}, { 0,-3}, { 0, 3}
    };
    //draw centroid radius 3 according to the mask
    void draw_point_rgb565(uint8_t *buf, int width, int height, int x, int y, uint8_t r, uint8_t g, uint8_t b) 
    {
    //map 2d into 1d
        uint16_t *pix = (uint16_t*)buf;
        uint16_t color = rgb565(r, g, b);

        for (size_t i = 0; i < sizeof(circle_offsets_r3)/sizeof(circle_offsets_r3[0]); i++) {
            int nx = x + circle_offsets_r3[i][0];
            int ny = y + circle_offsets_r3[i][1];
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            //formula to access 2d array in 1d y*width + x
                pix[ny * width + nx] = color;
            }
        }
    }

    //draw entrance line
    void draw_line_rgb565(uint8_t *buf, int width, uint8_t r, uint8_t g, uint8_t b) 
    {
        //map 2d into 1d
        uint16_t *pix = (uint16_t*)buf;
        uint16_t color = rgb565(r, g, b);

        
        for (int x = 0; x < width; x++) 
        {
        pix[LineY* width + x] = color;
        }
        
    }

    //add new pedestrians into current list (only apply this with a list that has been filtered for same pedestrians)
    void updatePedestrians(const std::vector<Pedestrian>& newPedestrians)
    {
        currentPedestrians.insert(currentPedestrians.end(), newPedestrians.begin(), newPedestrians.end());
    }

    //remove non existent pedestrians & check for crossing the line
    void prunePedestrians(const std::vector<Pedestrian>& newPedestrians)
    {
        std::vector<Pedestrian> updatedPedestrians;

        for (auto &oldPed : currentPedestrians) 
        {
            bool found = false;
            for (auto &newPed : newPedestrians) 
            {
                if (samePedestrian(oldPed, newPed)) 
                {
                    found = true;
                    updatedPedestrians.push_back(newPed);
                    //print found new ped
                    ESP_LOGI(TAG, "Pedestrian still in frame at (%d, %d)", newPed.centroidX, newPed.centroidY);
                    //check if crossed the line
                    if ((oldPed.centroidY > LineY) && (newPed.centroidY <= LineY)) 
                    {
                        ESP_LOGI(TAG, "Pedestrian exited at (%d, %d)", newPed.centroidX, newPed.centroidY);
                    } 
                    else if ((oldPed.centroidY < LineY) && (newPed.centroidY >= LineY)) 
                    {
                        ESP_LOGI(TAG, "Pedestrian entered at (%d, %d)", newPed.centroidX, newPed.centroidY);
                    }
                    break;
                }
            }
            //if not found it means pedestrian has left the frame
            if (!found) 
            {
                ESP_LOGI(TAG, "Pedestrian left the frame at (%d, %d)", oldPed.centroidX, oldPed.centroidY);
            }
        }

        currentPedestrians = updatedPedestrians;
    }
    //get new filtered pedestrians in the new list that arent already in the current list
    std::vector<Pedestrian> filterNewPedestrians(const std::vector<Pedestrian>& newPedestrians)
    {
        std::vector<Pedestrian> filteredPedestrians;

        for (const auto& newPed : newPedestrians) 
        {
            bool isNew = true;
            for (const auto& currPed : currentPedestrians) 
            {
                if (samePedestrian(newPed, currPed)) 
                {
                    isNew = false;
                    break;
                }
            }
            if (isNew) 
            {
                filteredPedestrians.push_back(newPed);
            }
        }

        return filteredPedestrians;
    }


    //run model
    auto run_pedestrian_detect(uint8_t* image_data, int image_width, int image_height) -> std::vector<Pedestrian>
    {
        std::vector<Pedestrian> pedestrians;
        //prep image
        dl::image::img_t img{image_data, (uint16_t)image_width, (uint16_t)image_height, dl::image::DL_IMAGE_PIX_TYPE_RGB565};

        //run model
        auto& results = pmodel->run(img);
        //parse results
        for (const auto& r : results) 
        {
            if (r.score < pedestrianThreshold)
            {
            continue; //skip if not confident enough
            } 
            else
            {
                //ESP_LOGI(TAG, "Pedestrian detected with confidence: %.2f, box coords: x1:%d, y1:%d, x2:%d, y2:%d", 
                //   r.score, r.box[0], r.box[1], r.box[2], r.box[3]);

                //add to pedestrians list
                Pedestrian p;
                p.x1 = r.box[0];
                p.y1 = r.box[1];
                p.x2 = r.box[2];
                p.y2 = r.box[3];
                calculateCentroid(&p);
                pedestrians.push_back(p);
                

                
            }
            
        }

        return pedestrians;
    }



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

    //stable for OV3660 you can try 20000000 if your board is stable
    config.xclk_freq_hz = 20000000;

    //low-res raw frames for fast ML and overlay
    config.pixel_format = PIXFORMAT_RGB565; //fastest
    config.frame_size = FRAMESIZE_QQVGA;  //160x120
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    //not used for RGB565 or Grayscale capture, relevant only for PIXFORMAT_JPEG
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


    //wifi event handler
    static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
    {
        //check if start is called
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            //check if failed and retry up to 5 times
        } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (s_retry_num < 5) {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "retry to connect to the AP %d/5", s_retry_num);
            } 
            else
            {
                //set FreeRTOS event group bit to notify connection failed
                xEventGroupSetBits(s_wifi_event_group, wifiFAILBIT);
            }
        
            
        } 
        //successfully connected and got IP
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, wifiCONNECTEDBIT);
        }
    }
    static void wifi_init(void)
    {
        //create event group to track wifi connection
        s_wifi_event_group = xEventGroupCreate();

        //initialize net 
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        //register the events
        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler,NULL,&instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&wifi_event_handler,NULL,&instance_got_ip));

        wifi_config_t wifi_config = {
            .sta = {
                .ssid = wifiSSID,
                .password = wifiPASSWORD,
            },
        };

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "wifi_init_sta finished.");

        //check if connected or failed to retry
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, wifiCONNECTEDBIT| wifiFAILBIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if (bits & wifiCONNECTEDBIT) 
        {
            ESP_LOGI(TAG, "Connected to SSID:%s", wifiSSID);
        } 
        else if (bits & wifiFAILBIT) 
        {
            ESP_LOGE(TAG, "Failed to connect to SSID:%s", wifiSSID);
        } 
        else 
        {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
        }
    }

    //setup stream handler
    esp_err_t stream_handler(httpd_req_t *req)
    {
        
        camera_fb_t *fb = NULL;
        char part_buf[64];
        //set response headers and boundary for stream
        static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
        static const char* _STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
        static const char* _STREAM_BOUNDARY_INITIAL = "--123456789000000000000987654321\r\n";
        static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

        httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
        esp_err_t res = NULL;
        esp_wifi_set_ps(WIFI_PS_NONE);
        //send chunks of jpeg frames
        while (httpd_req_to_sockfd(req) >= 0) 
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            fb = esp_camera_fb_get();

            if (!fb) {
                ESP_LOGE(TAG, "Camera capture failed");
                return ESP_FAIL;
            }
            
            int width = fb->width;
            int height = fb->height;

            //make a copy of the raw frame buffer 
            std::vector<uint8_t> rgb_copy(fb->len);
            memcpy(rgb_copy.data(), fb->buf, fb->len);
            draw_line_rgb565(rgb_copy.data(), width, 0,255, 0);

            //free frame buffer to be reused
            esp_camera_fb_return(fb);
            fb = NULL;

            auto results = run_pedestrian_detect(rgb_copy.data(), width, height);
            
            //draw centroids across each pedestrian detected
            for (auto &det : results) {
                    draw_point_rgb565(rgb_copy.data(), width, height, det.centroidX, det.centroidY, 255, 0, 0);
                
            }

            //get filtered new pedestrians
            auto newPedestrians = filterNewPedestrians(results);
            //clean up current pedestrians list and check for crossing the line
            prunePedestrians(results);
            //add new pedestrians to current list
            updatePedestrians(newPedestrians);

            //convert rgb565 to jpeg for streaming
            size_t jpg_buf_len = 0;
            uint8_t *jpg_buf = NULL;
            bool jpeg_converted = fmt2jpg(
                rgb_copy.data(),      // raw RGB565 data
                rgb_copy.size(),      // input buffer length
                width,            // image width
                height,           // image height
                PIXFORMAT_RGB565,     // pixel format of input
                40,                   // JPEG quality (0-63, lower = faster/smaller)
                &jpg_buf,             // output buffer pointer
                &jpg_buf_len          // output buffer length
            );

            //esp_camera_fb_return(fb);
            fb = NULL;

            if (!jpeg_converted) {
                ESP_LOGE(TAG, "JPEG compression failed");
                continue;
            }

            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY_INITIAL, strlen(_STREAM_BOUNDARY));
            if (res != ESP_OK) { free(jpg_buf); break; }
            snprintf((char *)part_buf, 64, _STREAM_PART, jpg_buf_len);
            
            res = httpd_resp_send_chunk(req, (const char *)part_buf, strlen((const char *)part_buf));
            if (res != ESP_OK) break;
            if (res != ESP_OK) { free(jpg_buf); break; }

            res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
            free(jpg_buf);
            if (res != ESP_OK) { break; }
        }

        return res;
    }
    static const char kIndexHtml[] =
        "<!doctype html><html><head><meta name='viewport' "
        "content='width=device-width,initial-scale=1'/>"
        "<title>ESP32S3 OV3660 Stream</title></head><body style='margin:0;"
        "background:#000;display:flex;justify-content:center;align-items:center;"
        "height:100vh'>"
        "<img src='/stream' style='max-width:100%;height:auto;object-fit:contain'/>"
        "</body></html>";

    static esp_err_t index_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
    }

    //start http server
    httpd_handle_t start_webserver(void)
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();

        config.lru_purge_enable = true;
        config.recv_wait_timeout = 30; 
        config.send_wait_timeout = 30; 
        config.stack_size = 8192;
        httpd_handle_t server = NULL;
        if (httpd_start(&server, &config) == ESP_OK)
        {
            httpd_uri_t stream_uri = {
                .uri       = "/stream",
                .method    = HTTP_GET,
                .handler   = stream_handler,
                .user_ctx  = NULL
            };

            httpd_uri_t root = {.uri = "/",
                        .method = HTTP_GET,
                        .handler = index_handler,
                        .user_ctx = nullptr};


            httpd_register_uri_handler(server, &root);
            httpd_register_uri_handler(server, &stream_uri);
        }
        return server;
    }


    //task report to api
    void report_task(void* pvParameters)
    {
        while (1) 
        {
            //report every 10 seconds
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
    //task refactor

    //task to capture images from camera and queue
    void camera_task(void* pvParameters)
    {
        while (1) 
        {
            camera_fb_t *fb = NULL;
            fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "Camera capture failed");
                continue;
            }
            //send to queue
            xQueueSend(camera_queue, &fb, portMAX_DELAY);  
            vTaskDelay(pdMS_TO_TICKS(1));

        }
    }


    //ml task to run model and queue results
    void ml_task(void* pvParameters)
    {
        static bool s_wdt_registered = false;
        if (!s_wdt_registered) {
                esp_err_t e = esp_task_wdt_add(NULL);
                if (e == ESP_OK) {
                    s_wdt_registered = true;
                    ESP_LOGI(TAG, "ml_task registered with Task WDT");
                } else {
                    // If TWDT wasn't initialized for tasks, this will fail.
                    // We just won't call esp_task_wdt_reset() in that case.
                    ESP_LOGW(TAG, "ml_task WDT add failed: %s", esp_err_to_name(e));
                }
        }
        while (1) 
        {
    
            camera_fb_t *fb = NULL;
            //wait for image from camera task
            if (xQueueReceive(camera_queue, &fb, portMAX_DELAY) == pdTRUE) 
            {
                //received frame
                if (fb) 
                {
                    int width = fb->width;
                    int height = fb->height;
        
                    auto results = run_pedestrian_detect(fb->buf, width, height);
                    //draw line
                    draw_line_rgb565(fb->buf, width, 0,255, 0);
                    vTaskDelay(pdMS_TO_TICKS(1));
                    //draw centroids across each pedestrian detected
                    for (auto &det : results) {
                            draw_point_rgb565(fb->buf, width, height, det.centroidX, det.centroidY, 255, 0, 0);
       

                    }

                    //get filtered new pedestrians
                    auto newPedestrians = filterNewPedestrians(results);
                    vTaskDelay(pdMS_TO_TICKS(1));
                    //clean up current pedestrians list and check for crossing the line
                    prunePedestrians(results);
                                     
                    vTaskDelay(pdMS_TO_TICKS(1));

                    //add new pedestrians to current list
                    updatePedestrians(newPedestrians);
                       

                    //convert rgb565 to jpeg for streaming
                    size_t jpg_buf_len = 0;
                    uint8_t *jpg_buf = NULL;
                    vTaskDelay(pdMS_TO_TICKS(1));

                    (void)esp_task_wdt_reset();
                    bool jpeg_converted = fmt2jpg(
                        fb->buf,
                        fb->len, 
                        fb->width, 
                        fb->height,
                        PIXFORMAT_RGB565,
                        20, 
                        &jpg_buf, 
                        &jpg_buf_len    
                    );

                    esp_camera_fb_return(fb);
                    fb = NULL;
                  
                    if (!jpeg_converted) {
                        ESP_LOGE(TAG, "JPEG compression failed");
                        vTaskDelay(pdMS_TO_TICKS(1));
                        continue;
                    }

                  
                    
                    jpeg_frame frame = { .buf = jpg_buf, .len = jpg_buf_len };

                    //send to stream queue
                   //xQueueSend(stream_queue, &frame, portMAX_DELAY);   
                     if (xQueueSend(stream_queue, &frame, 0) != pdPASS)
                        {
                            jpeg_frame stale;
                            if (xQueueReceive(stream_queue, &stale, 0) == pdTRUE) {
                                free(stale.buf);
                            }
                            (void)xQueueSend(stream_queue, &frame, 0);
                      }

                        (void)esp_task_wdt_reset();
                        vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
        }
    }


    esp_err_t stream_handler_pipeline(httpd_req_t *req)
    {
        
        jpeg_frame fb = {0};
        char part_buf[64];
        //set response headers and boundary for stream
        static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
        static const char* _STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
        static const char* _STREAM_BOUNDARY_INITIAL = "--123456789000000000000987654321\r\n";
        static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

        httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
        esp_err_t res = ESP_OK;
        esp_wifi_set_ps(WIFI_PS_NONE);

        //send chunks of jpeg frames
        while (httpd_req_to_sockfd(req) >= 0) 
        {
            //get frame
            if(xQueueReceive(stream_queue, &fb, portMAX_DELAY) == pdTRUE)
            {
        

                res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
                snprintf((char *)part_buf, 64, _STREAM_PART, fb.len);
                
                res = httpd_resp_send_chunk(req, (const char *)part_buf, strlen((const char *)part_buf));
                if (res != ESP_OK) break;

                res = httpd_resp_send_chunk(req, (const char *)fb.buf, fb.len);
                if (res != ESP_OK) { break; }

                //free jpeg buffer so no memory leak
                free(fb.buf); 
            }

        }

        return res;
    }


    httpd_handle_t start_webserver_pipeline(void)
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();

        config.lru_purge_enable = true;
        config.recv_wait_timeout = 30; 
        config.send_wait_timeout = 30; 
        config.stack_size = 8192;
        httpd_handle_t server = NULL;
        if (httpd_start(&server, &config) == ESP_OK)
        {
            httpd_uri_t stream_uri = {
                .uri       = "/stream",
                .method    = HTTP_GET,
                .handler   = stream_handler_pipeline,
                .user_ctx  = NULL
            };

            httpd_uri_t root = {.uri = "/",
                        .method = HTTP_GET,
                        .handler = index_handler,
                        .user_ctx = nullptr};


            httpd_register_uri_handler(server, &root);
            httpd_register_uri_handler(server, &stream_uri);
        }
        return server;
    }



    extern "C" void app_main(void)
    {
      
        ESP_ERROR_CHECK(esp_task_wdt_deinit());
        esp_task_wdt_config_t twdt_cfg = {
            .timeout_ms = 10000,
            .idle_core_mask = 0, //0 = do not monitor Idle0/Idle1
            .trigger_panic = true,
        };
        ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_cfg));
        
        ESP_ERROR_CHECK(nvs_flash_init());

        //connect to wifi
        wifi_init();  
        vTaskDelay(pdMS_TO_TICKS(2000));  
        //start cam
        camera_init_or_abort();    
        pmodel = new PedestrianDetect();
        //create tasks
        camera_queue = xQueueCreate(5, sizeof(camera_fb_t*));
        stream_queue = xQueueCreate(1, sizeof(jpeg_frame));

        xTaskCreatePinnedToCore(&camera_task, "camera_task", 4096, NULL, 7, NULL, 0);

	    xTaskCreatePinnedToCore(&ml_task, "ml_task", 16384, NULL, 6, NULL, 1);

        start_webserver_pipeline();  

        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);
        
        ESP_LOGI(TAG, "ESP32 IP: " IPSTR, IP2STR(&ip_info.ip));
    }