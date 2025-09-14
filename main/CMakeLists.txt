idf_component_register(
  SRCS "main.cpp"
  INCLUDE_DIRS "."
  REQUIRES
    esp32-camera
    esp_http_server
    esp_wifi
    nvs_flash
    esp_event
    esp_netif
    esp-dl
    pedestrian_detect
    esp_psram
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)