#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "cJSON.h"
#include <string.h>
#include "driver/adc.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

static const char *TAG = "Controller";

cJSON *state = NULL;
char state_str[2000];
bool connected_to_server = false;

long int current_time = 0;
long int start_time = 0;
long int cycletime = 0;

void send_state(void);

#include "services/store.c"
#include "services/websocket.c"
#include "services/station.c"
#include "services/scale.c"

void app_main(void)
{
  printf("Scale Controller - Version 0.1\n");

  ESP_ERROR_CHECK(nvs_flash_init());

  restore_state();

  if (cJSON_GetObjectItem(state,"start_time")) {
    start_time = cJSON_GetObjectItem(state,"start_time")->valueint;
    printf("loaded start_time: %lu\n", start_time);
  }

  if (cJSON_GetObjectItem(state,"time")) {
    current_time = cJSON_GetObjectItem(state,"time")->valueint;
    printf("loaded current time: %lu\n", current_time);
  }

  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  strcpy(device_id,get_char("device_id"));
  if (strcmp(device_id,"")==0) {
    // xTaskCreate(&websocket_utilities_task, "websocket_utilities_task", 10000, NULL, 5, NULL);
  } else {
    printf("pulled device_id from storage: %s\n", device_id);
  }

  strcpy(token,get_char("token"));
  if (strcmp(token,"")==0) {
    strcpy(token,device_id);
    printf("no token found, setting token as device id: %s\n", token);
  } else {
    printf("pulled token from storage: %s\n", token);
  }

  station_main();

  xTaskCreate(&websocket_utilities_task, "websocket_utilities_task", 10000, NULL, 5, NULL);
  xTaskCreate(&websocket_relay_task, "websocket_relay_task", 20000, NULL, 5, NULL);
  xTaskCreate(&scale_task, "scale_task", 20000, NULL, 5, NULL);

  while (1) {
    current_time++;
    start_time = cJSON_GetObjectItem(state,"start_time")->valueint;
    cycletime = current_time - start_time;
    cJSON *time_json = cJSON_CreateNumber(current_time);
    cJSON_ReplaceItemInObjectCaseSensitive(state,"time",time_json);

    cJSON *cycletime_json = cJSON_CreateNumber(cycletime);
    cJSON_ReplaceItemInObjectCaseSensitive(state,"cycletime",cycletime_json);

    send_state();
    store_state(state);
    // ESP_LOGI(TAG, "Free memory: %d bytes", esp_get_free_heap_size());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
