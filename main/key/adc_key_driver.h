//
// Created by Administrator on 2025/3/29.
//

#ifndef ADC_KEY_DRIVER_H
#define ADC_KEY_DRIVER_H

#ifndef SYS_KEY_DRIVER_H
#define SYS_KEY_DRIVER_H
#include <stdint.h>
#include <esp_adc/adc_cali.h>


typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

#define ONE_PORT_TO_LOW 		0 		//按键一个端口接低电平, 另一个端口接IO
#define ONE_PORT_TO_HIGH		1 		//按键一个端口接高电平, 另一个端口接IO
#define DOUBLE_PORT_TO_IO		2		//按键两个端口接IO
#define CUST_DOUBLE_PORT_TO_IO	3

enum {
    KEY_EVENT_CLICK,
    KEY_EVENT_LONG,
    KEY_EVENT_HOLD,
    KEY_EVENT_UP,
    KEY_EVENT_DOUBLE_CLICK,
    KEY_EVENT_TRIPLE_CLICK,
    KEY_EVENT_FOURTH_CLICK,
    KEY_EVENT_FIRTH_CLICK,
    KEY_EVENT_USER,
    KEY_EVENT_MAX,
};


typedef enum __KEY_DRIVER_TYPE {
    KEY_DRIVER_TYPE_IO = 0x0,
    KEY_DRIVER_TYPE_AD,
    KEY_DRIVER_TYPE_RTCVDD_AD,
    KEY_DRIVER_TYPE_IR,
    KEY_DRIVER_TYPE_TOUCH,
    KEY_DRIVER_TYPE_CTMU_TOUCH,
    KEY_DRIVER_TYPE_RDEC,
    KEY_DRIVER_TYPE_SLIDEKEY,
    KEY_DRIVER_TYPE_SOFTKEY,
    KEY_DRIVER_TYPE_BRIGHTNESS,
    KEY_DRIVER_TYPE_VOICE,

    KEY_DRIVER_TYPE_MAX,
} KEY_DRIVER_TYPE;



#define NO_KEY 		0xff


#define KEY_NOT_SUPPORT  0x01


struct key_driver_para {
    const u32 scan_time;	//按键扫描频率, 单位ms
    u32 last_key;  			//上一次get_value按键值
    //== 用于消抖类参数
    u32 filter_value; 		//用于按键消抖
    u32 filter_cnt;  		//用于按键消抖时的累加值
    const u32 filter_time;	//当filter_cnt累加到base_cnt值时, 消抖有效
    //== 用于判定长按和HOLD事件参数
    const u32 long_time;  	//按键判定长按数量
    const u32 hold_time;  	//按键判定HOLD数量
    u32 press_cnt;  		 	//与long_time和hold_time对比, 判断long_event和hold_event
    //== 用于判定连击事件参数
    u32 click_cnt;  			//单击次数
    u32 click_delay_cnt;  	//按键被抬起后等待连击事件延时计数
    const u32 click_delay_time;	////按键被抬起后等待连击事件延时数量
    u32 notify_value;  		//在延时的待发送按键值
    u32 key_type;
    u32 (*get_value)(void);
};

//组合按键映射按键值
struct key_remap {
    u8 bit_value;
    u8 remap_value;
};

struct key_remap_data {
    u8 remap_num;
    const struct key_remap *table;
};



struct one_io_key {
    u8 port;
};

struct two_io_key {
    u8 in_port;
    u8 out_port;
};

union key_type {
    struct one_io_key one_io;
    struct two_io_key two_io;
};

struct iokey_port {
    union key_type key_type;
    u8 connect_way;
    u8 key_value;
};

struct iokey_platform_data {
    u8 enable;
    u8 num;
    const struct iokey_port *port;
};

typedef struct {
    adc_cali_handle_t adc1_cali_chan0_handle;
    adc_cali_handle_t adc1_cali_chan1_handle;

    int adc_raw[2][10];
    int voltage[2][10];
} adc_key_t;


// key_driver API:
extern int key_driver_init(void);
void init_adc_key(void);



#endif




#endif //ADC_KEY_DRIVER_H
