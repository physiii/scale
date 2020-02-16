char temperature_service_message[2000];
char temperature_service_message_in[2000];
bool temperature_service_message_ready = false;
bool raising_temp = false;
bool lowering_temp = false;
bool tared = false;
int TARE_COUNT = 6;
int REPEAT_CNT = 12;
float SCALE_THRESH = 1;


float scale_bias = 36000;
float SCALE_SLOPE = 4825;

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hx711.h>
#include <math.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define PD_SCK_GPIO 4
#define DOUT_GPIO   5
#else
#define PD_SCK_GPIO 23
#define DOUT_GPIO   22
#endif

bool THERM_INVERT_IO = CONFIG_THERM_INVERT_IO;

float TEMP_SET_POINT = CONFIG_TEMP_SET_POINT;
float TEMP_WINDOW = 1.0;

struct scale
{
  int control_pos_io;
  int control_neg_io;
  bool invert_io;
  float value;
  float prev_value;
  int32_t reading;
  float accumulator;
  float alpha;
  float variance;
};

void scale_task(void *pvParameters)
{
    hx711_t dev = {
        .dout = DOUT_GPIO,
        .pd_sck = PD_SCK_GPIO,
        .gain = HX711_GAIN_A_64
    };

    // initialize device
    while (1)
    {
        esp_err_t r = hx711_init(&dev);
        if (r == ESP_OK)
            break;
        printf("Could not initialize HX711: %d (%s)\n", r, esp_err_to_name(r));
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    struct scale sc;
    sc.alpha = 0.4;
    sc.variance = 0.3;
    sc.accumulator = 0;
    int tare_count = 0;
    int repeat_cnt = 0;
    float sum = 0;
    // read from device
    while (1)
    {
        esp_err_t r = hx711_wait(&dev, 200);
        if (r != ESP_OK)
        {
            printf("Device not found: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        r = hx711_read_data(&dev, &sc.reading);
        sc.reading=fabs(sc.reading);
        if (r != ESP_OK)
        {
            printf("Could not read data: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }


        // exponential moving average
        // accumulator = (alpha * new_value) + (1.0 - alpha) * accumulator
        // The closer alpha is to one the more older values will contribute.
        sc.accumulator = (sc.alpha * sc.reading) + (1 - sc.alpha) * sc.accumulator;
        sc.value = (sc.accumulator - scale_bias)/SCALE_SLOPE;

        if (sc.value > SCALE_THRESH) {
          cJSON *number = cJSON_CreateNumber(sc.value);
          cJSON_ReplaceItemInObjectCaseSensitive(state,"weight",number);
          if (fabs(sc.value - sc.prev_value) < sc.variance) {
            repeat_cnt++;
            if (repeat_cnt > REPEAT_CNT) {
              printf("Log weight on server: %f\n", sc.value);
              cJSON * log = cJSON_CreateObject();
              cJSON_ReplaceItemInObjectCaseSensitive(log,"weight",number);
              send_log(log);
              repeat_cnt = 0;
            }
          } else {
            repeat_cnt = 0;
          }
        }

        if (!tared && tare_count >= TARE_COUNT) {
          scale_bias = sum / TARE_COUNT;
          printf("Tare complete: count %d\tsum :%f\tbias: %f\n", TARE_COUNT, sum, scale_bias);
          tared = true;
        } else {
          sum+=sc.reading;
          tare_count++;
        }

        sc.prev_value = sc.value;
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
