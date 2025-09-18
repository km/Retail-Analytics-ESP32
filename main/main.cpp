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

typedef struct {
    int x1;
    int x2;
    int y1;
    int y2;

    int centroidX;
    int centroidY;
} Pedestrian;


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