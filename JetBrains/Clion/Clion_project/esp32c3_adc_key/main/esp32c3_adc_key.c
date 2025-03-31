#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ADC校准结构体
static esp_adc_cal_characteristics_t *adc_chars;

// ADC通道配置
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_db_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

void app_main(void)
{
    // 配置ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);

    // 校准ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    // 打印ADC校准值类型
    printf("eFuse Two Point: Supported\n");

    // 无限循环读取ADC值
    while (1) {
        uint32_t adc_reading = 0;
        // 多次采样取平均值
        for (int i = 0; i < 64; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= 64;

        // 打印原始ADC值
        printf("Raw: %d\n", adc_reading);

        // 打印校准后的电压值
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Voltage: %dmV\n", voltage);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}