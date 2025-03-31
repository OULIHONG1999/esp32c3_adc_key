//
// Created by Administrator on 2025/3/29.
//

#include "adc_key_driver.h"

#include <esp_bit_defs.h>
#include <esp_efuse_table.h>
#include <limits.h>
#include <driver/gpio.h>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

#include <hal/gpio_types.h>
#define KEY_BOOT GPIO_NUM_9


#define LOG_TAG_CONST       KEY
#define LOG_TAG             "[KEY]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
const static char *TAG = "ADC_INIT";

#define RIGHT_ADC1_CHAN0          ADC_CHANNEL_0
#define LEFT_ADC1_CHAN1          ADC_CHANNEL_1
#define ADC_ATTEN                 ADC_ATTEN_DB_12


static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                 adc_cali_handle_t *out_handle);

static adc_oneshot_unit_handle_t adc_key_handle;
static adc_oneshot_unit_init_cfg_t init_adc_key_config = {
    .unit_id = ADC_UNIT_1,
};

static adc_oneshot_chan_cfg_t adc_key_config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN,
};


static adc_key_t adc_key = {0};
//                                  松开        上       左      下         右       左上      左下     右下      右上      中
// unsigned short right_key_val[10] = {4095, 1582, 2123, 3257, 2232, 1132, 1800, 1880, 1163, 0};
// unsigned short left_key_val[10] =  {4095, 3274, 2680, 2120, 1581, 2158, 1594, 1130, 1390, 8};

//                                  松开        下        右       左       右下      左下     上       右上      左上     中
unsigned short left_key_val[10] = {4095, 3257, 2232, 2123, 1880, 1800, 1582, 1163, 1132, 0};
//                                  松开        上       左        左上      下       左下      右       右上     右下      中
unsigned short right_key_val[10] = {4095, 3274, 2680, 2158, 2120, 1594, 1581, 1390, 1130, 0};


unsigned short find_closest_index(unsigned short arr[], int size, unsigned short value) {
    int closest_index = -1;
    int min_diff = INT_MAX; // 初始化为最大整数值

    for (int i = 0; i < size; i++) {
        int diff = abs(arr[i] - value); // 计算当前元素与目标值的差值
        if (diff < min_diff) {
            min_diff = diff; // 更新最小差值
            closest_index = i; // 更新最接近元素的下标
        }
    }

    return closest_index; // 返回最接近元素的下标
}

unsigned int io_get_key_value(void) {
    adc_cali_handle_t adc1_cali_chan0_handle = adc_key.adc1_cali_chan0_handle;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_key_handle , RIGHT_ADC1_CHAN0, &adc_key.adc_raw[0][0]));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_key.adc_raw[0][0], &adc_key.voltage[0][0]));

    ESP_ERROR_CHECK(adc_oneshot_read(adc_key_handle , LEFT_ADC1_CHAN1, &adc_key.adc_raw[1][0]));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_key.adc_raw[1][0], &adc_key.voltage[1][0]));

    // ESP_LOGI(TAG, "adc_raw[0][0]:%d, voltage[0][0]:%d, adc_raw[1][0]:%d, voltage[1][0]:%d",
    //          adc_key.adc_raw[0][0], adc_key.voltage[0][0], adc_key.adc_raw[1][0], adc_key.voltage[1][0]);

    // 左按键
    unsigned short left_key_index = find_closest_index(left_key_val, 10, adc_key.adc_raw[1][0]);
    // 右按键
    unsigned short right_key_index = find_closest_index(right_key_val, 10, adc_key.adc_raw[0][0]);

    // 读BOOT IO的值
    const u8 boot_val = gpio_get_level(KEY_BOOT);
    if (boot_val == 0) {
        left_key_index = 9;
    }

    if (left_key_index == 0 && right_key_index == 0) {
        return NO_KEY;
    }

    const unsigned int key_val = (left_key_index << 16) | right_key_index;
    // printf("left_key_index:%d, right_key_index:%d, key_val:%x\n", left_key_index, right_key_index, key_val);
    return key_val;
}


//按键驱动扫描参数列表
struct key_driver_para iokey_scan_para = {
    .scan_time = 10, //按键扫描频率, 单位: ms
    .last_key = NO_KEY, //上一次get_value按键值, 初始化为NO_KEY;
    .filter_time = 4, //按键消抖延时;
    .long_time = 30, //按键判定长按数量
    .hold_time = (30 + 15), //按键判定HOLD数量
    .click_delay_time = 20, //按键被抬起后等待连击延时数量
    .key_type = KEY_DRIVER_TYPE_IO,
    .get_value = io_get_key_value,
};

//=======================================================//
// 按键扫描函数: 扫描所有注册的按键驱动
//=======================================================//
static void key_driver_scan(void *_scan_para) {
    struct key_driver_para *scan_para = (struct key_driver_para *) _scan_para;
    int key_event = 0;
    u32 cur_key_value = NO_KEY;
    u32 key_value = 0;
    static volatile u8 is_key_active = 0;
    cur_key_value = scan_para->get_value();

    if (cur_key_value != NO_KEY) {
        is_key_active = 35; //35*10Ms
    } else if (is_key_active) {
        is_key_active--;
    }

    //===== 按键消抖处理
    if (cur_key_value != scan_para->filter_value && // 检测到不同按键
        scan_para->filter_time // 尝试过消抖计数
    ) {
        //当前按键值与上一次按键值如果不相等, 重新消抖处理, 注意filter_time != 0;
        scan_para->filter_cnt = 0; //消抖次数清0, 重新开始消抖
        scan_para->filter_value = cur_key_value; //记录上一次的按键值
        return; //第一次检测, 返回不做处理
    }
    //当前按键值与上一次按键值相等, filter_cnt开始累加;
    if (scan_para->filter_cnt < scan_para->filter_time) {
        scan_para->filter_cnt++;
        return;
    }

    //===== 按键消抖结束, 开始判断按键类型(单击, 双击, 长按, 多击, HOLD, (长按/HOLD)抬起)
    if (cur_key_value != scan_para->last_key) {
        // 检测到按键状态变化
        if (cur_key_value == NO_KEY) {
            // 按键松开
            //cur_key = NO_KEY; last_key = valid_key -> 按键被抬起
            if (scan_para->press_cnt >= scan_para->long_time) //长按/HOLD状态之后被按键抬起;  按键松开超过预设值，则发送抬起消息
            {
                key_event = KEY_EVENT_UP;
                key_value = scan_para->last_key;
                goto _notify; //发送抬起消息
            }

            scan_para->click_delay_cnt = 1; //按键等待下次连击延时开始
        } else {
            //cur_key = valid_key, last_key = NO_KEY -> 按键被按下
            scan_para->press_cnt = 1; //用于判断long和hold事件的计数器重新开始计时;
            if (cur_key_value != scan_para->notify_value) {
                //第一次单击/连击时按下的是不同按键, 单击次数重新开始计数
                scan_para->click_cnt = 1;
                scan_para->notify_value = cur_key_value;
            } else {
                scan_para->click_cnt++; //单击次数累加
            }
        }
        goto _scan_end; //返回, 等待延时时间到
    } else {
        // 依然检测到是上次的按键
        //cur_key = last_key -> 没有按键按下/按键长按(HOLD)
        if (cur_key_value == NO_KEY) {
            //last_key = NO_KEY; cur_key = NO_KEY -> 没有按键按下
            if (scan_para->click_cnt > 0) {
                //有按键需要消息需要处理
                if (scan_para->click_delay_cnt > scan_para->click_delay_time) {
                    // 表示当前识别到连击后，按键抬起的时间超过设定值，开始对连击的时间进行统计和处理
                    //按键被抬起后延时到
                    //TODO: 在此可以添加任意多击事件
                    if (scan_para->click_cnt >= 5) {
                        key_event = KEY_EVENT_FIRTH_CLICK; //五击
                    } else if (scan_para->click_cnt >= 4) {
                        key_event = KEY_EVENT_FOURTH_CLICK; //4击
                    } else if (scan_para->click_cnt >= 3) {
                        key_event = KEY_EVENT_TRIPLE_CLICK; //三击
                    } else if (scan_para->click_cnt >= 2) {
                        key_event = KEY_EVENT_DOUBLE_CLICK; //双击
                    } else {
                        key_event = KEY_EVENT_CLICK; //单击
                    }
                    key_value = scan_para->notify_value;
                    goto _notify;
                } else {
                    //按键抬起后等待下次延时时间未到
                    scan_para->click_delay_cnt++;
                    goto _scan_end; //按键抬起后延时时间未到, 返回
                }
            } else {
                goto _scan_end; //没有按键需要处理
            }
        } else {
            //last_key = valid_key; cur_key = valid_key, press_cnt累加用于判断long和hold
            scan_para->press_cnt++;
            if (scan_para->press_cnt == scan_para->long_time) {
                key_event = KEY_EVENT_LONG;
            } else if (scan_para->press_cnt == scan_para->hold_time) {
                key_event = KEY_EVENT_HOLD;
                scan_para->press_cnt = scan_para->long_time;
            } else {
                goto _scan_end; //press_cnt没到长按和HOLD次数, 返回
            }
            //press_cnt没到长按和HOLD次数, 发消息
            key_value = cur_key_value;
            goto _notify;
        }
    }

_notify:
    scan_para->click_cnt = 0; //单击次数清0
    scan_para->notify_value = NO_KEY;

    printf("key_value: 0x%08lx, event: %d\n", key_value, key_event);

_scan_end:
    scan_para->last_key = cur_key_value;
}


void init_adc_key(void) {
    // 初始化io9
    esp_efuse_write_field_bit(ESP_EFUSE_VDD_SPI_AS_GPIO);
    // 配置 GPIO 引脚
    gpio_reset_pin(KEY_BOOT);
    gpio_set_direction(KEY_BOOT, GPIO_MODE_INPUT);
    // 设置下拉
    gpio_set_pull_mode(KEY_BOOT, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_adc_key_config , &adc_key_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_key_handle, RIGHT_ADC1_CHAN0, &adc_key_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_key_handle, LEFT_ADC1_CHAN1, &adc_key_config));


    // 曲线拟合
    bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, RIGHT_ADC1_CHAN0, ADC_ATTEN,
                                                      &adc_key.adc1_cali_chan0_handle);
    bool do_calibration1_chan1 = adc_calibration_init(ADC_UNIT_1, LEFT_ADC1_CHAN1, ADC_ATTEN,
                                                      &adc_key.adc1_cali_chan1_handle);

    if (do_calibration1_chan0 || do_calibration1_chan1) {
        ESP_LOGI(TAG, "Calibration Success");
    } else {
        ESP_LOGW(TAG, "Calibration Failed");
    }

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &key_driver_scan,
        .arg = &iokey_scan_para,
        .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 5000));
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                 adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
