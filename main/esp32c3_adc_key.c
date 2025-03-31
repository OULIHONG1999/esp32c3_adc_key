#include <string.h>
#include <key/adc_key_driver.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


const static char *TAG = "EXAMPLE";


void app_main(void) {
    init_adc_key();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


