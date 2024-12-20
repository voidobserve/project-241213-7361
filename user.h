/******************************************************************************
;  *       @型号                   : MC32F7361
;  *       @创建日期               : 2021.12.21
;  *       @公司/作者              : SINOMCU-FAE
;  *       @晟矽微技术支持         : 2048615934
;  *       @晟矽微官网             : http://www.sinomcu.com/
;  *       @版权                   : 2021 SINOMCU公司版权所有.
;  *---------------------- 建议 ---------------------------------
;  *                   变量定义时使用全局变量
******************************************************************************/
#ifndef USER
#define USER
#include "mc32-common.h"
#include "MC32F7361.h"

/*****************************************************************
;       Function : Define variables
;*****************************************************************/
#if 1

#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long int
#define uint8_t unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long int

#define DEF_SET_BIT0 0x01
#define DEF_SET_BIT1 0x02
#define DEF_SET_BIT2 0x04
#define DEF_SET_BIT3 0x08
#define DEF_SET_BIT4 0x10
#define DEF_SET_BIT5 0x20
#define DEF_SET_BIT6 0x40
#define DEF_SET_BIT7 0x80

#define DEF_CLR_BIT0 0xFE
#define DEF_CLR_BIT1 0xFD
#define DEF_CLR_BIT2 0xFB
#define DEF_CLR_BIT3 0xF7
#define DEF_CLR_BIT4 0xEF
#define DEF_CLR_BIT5 0xDF
#define DEF_CLR_BIT6 0xBF
#define DEF_CLR_BIT7 0x7F

#define FAIL 1
#define PASS 0

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#define USE_MY_DEBUG 1
#define ONE_CYCLE_CNT_MS () // 主循环一次所需的时间，单位：ms

#endif

// ===================================================
// 低功耗相关配置                                   //
// ===================================================
#define ENTER_LOW_POWER_PERIOD_TIME (1000) // 进入低功耗的周期，单位：100us
volatile u16 low_power_time_cnt; // 进入低功耗的时间计数

// ===================================================
// 低电量检测相关配置                                //
// ===================================================
// 使用 VDD/4 通道, 内部2V参考电压
#define POWER_LOW_3_2_V 1638 // 3.2V电压对应的ad值
#define POWER_LOW_3_0_V 1536 // 3.0V电压对应的ad值
enum
{
    BATTERY_NORMAL = 0,          // 电池状态正常
    BATTERY_POWER_LOW_3_2_V, // 电池电压低于3.2V
    BATTERY_POWER_LOW_3_0_V,     // 电池电压低于3.0V
};
volatile u8 battery_status; // 记录电池状态的状态机

// ===================================================
// 32768Hz晶振 计时 相关配置                          //
// ===================================================
volatile u8 keep_time_cnt;

// ===================================================
// 蓝牙相关配置                                      //
// ===================================================
// 蓝牙控制引脚
#if USE_MY_DEBUG
#define BLE_CTL_PIN (P21D)
#define BLE_CTL_PIN_OUT() (P21OE = 1)        // 蓝牙控制引脚切换为输出模式
#define BLE_CTL_PIN_IN() (P21OE = 0)         // 蓝牙控制引脚切换为输入模式
#define BLE_CTL_PIN_HIGH() (BLE_CTL_PIN = 1) // 蓝牙控制引脚输出高电平
#define BLE_CTL_PIN_LOW() (BLE_CTL_PIN = 0)  // 蓝牙控制引脚输出低电平
#else
#define BLE_CTL_PIN (P17D)
#define BLE_CTL_PIN_OUT() (P17OE = 1)        // 蓝牙控制引脚切换为输出模式
#define BLE_CTL_PIN_IN() (P17OE = 0)         // 蓝牙控制引脚切换为输入模式
#define BLE_CTL_PIN_HIGH() (BLE_CTL_PIN = 1) // 蓝牙控制引脚输出高电平
#define BLE_CTL_PIN_LOW() (BLE_CTL_PIN = 0)  // 蓝牙控制引脚输出低电平
#endif

// ======================================================
// 中断服务函数使用到的变量:
u8 abuf;
u8 statusbuf;

//===============Global Variable===============
u8 i; // 循环计数值

// u16 tmp;

// ===================================================
// pwm灯光相关配置                                   //
// ===================================================
#define WHITE_PWM_DUTY_REG (T2DATA)  // 设置白灯pwm占空比的寄存器
#define YELLOW_PWM_DUTY_REG (T0DATA) // 设置黄灯pwm占空比的寄存器
volatile u8 cur_w_pwm_duty;          // 记录白灯对应的pwm占空比
volatile u8 cur_y_pwm_duty;          // 记录黄灯对应的pwm占空比

// 定义当前灯光的状态(用于状态机中)
enum
{
    CUR_LIGHT_STATUS_OFF,
    CUR_LIGHT_STATUS_WHITE,
    CUR_LIGHT_STATUS_YELLOW,
    CUR_LIGHT_STATUS_WHITE_YELLOW,
};
volatile u8 cur_light_status; // 存档当前灯光状态的状态机

// ===================================================
// adc相关配置                                       //
// ===================================================
// 定义adc的通道，用于切换adc通道
enum
{
    ADC_CHANNEL_VDD,
    ADC_CHANNEL_KEY,
};
// #define // 按键对应的    ad值 >> 4
/*
    使用4V参考电压，在4.8V~4.9V供电下，测得
    没有按键按下的时候，对应的电压约为电源VDD的电压
    按键        按下时对应的电压      对应的ad值  对应的ad值>>4
    ad_sel      1.53V               1566        97
    ad_up       0.20V               204         12
    ad_down     2.40V               2457        153
    ad_left     3.63V               3717        232
    ad_right    2.04V               2089        130
    ad_mid      0.94V               962         60

    取(ad值>>4)中间值作为分割线:
    12  60  97  130  153  232
    ad <= 36    up          具有 短按 长按功能
    ad <= 78    mid         具有 短按 长按功能
    ad <= 113   sel         具有 长按 功能
    ad <= 141   right       具有 短按 长按功能
    ad <= 192   down        具有 短按 长按功能
    ad <= 243   left        具有 短按 长按功能

    使用VDD（5V）作为参考电压：
    测量电阻的分压（按键外部都是10K上拉）：
    按键  下拉电阻     按下时对应的电压      对应的ad值  对应的ad值>>4
    up    0R          0V                    0           0    
    mid   2.2K        0.901V                739         46
    sel   4.7K        1.59V                 1309        82
    right 7.5K        2.14V                 1755        110
    // down  10K         2.5V                  2048        128
    down  15K         3V                    2458        154
    left  47K         4.122V                3377        211

    取(ad值>>4)中间值作为分割线:
    0  46  82  110  154  211
    ad <= 23    up          
    ad <= 64    mid         
    ad <= 96    sel         
    ad <= 132    right       
    ad <= 183    down        
    ad <= 233    left        
*/
#define AD_KEY_VAL_NONE 255

#define AD_KEY_VAL_UP 23
#define AD_KEY_VAL_MID 64
#define AD_KEY_VAL_SEL 96
#define AD_KEY_VAL_RIGHT 132
#define AD_KEY_VAL_DOWN 183
#define AD_KEY_VAL_LEFT 233

// 定义按键的id，与 id_table[]中的顺序有关
enum
{
    AD_KEY_ID_UP = 0,
    AD_KEY_ID_MID,
    AD_KEY_ID_SEL,
    AD_KEY_ID_RIGHT,
    AD_KEY_ID_DOWN,
    AD_KEY_ID_LEFT,

    AD_KEY_ID_NONE,
};

// 定义按键的事件
enum
{
    KEY_EVENT_CLICK = 1,
    KEY_EVENT_LONG,
    KEY_EVENT_HOLD,

    KEY_EVENT_NONE,
};

// 存放按键
const u8 id_table[6] = {
    AD_KEY_VAL_UP,
    AD_KEY_VAL_MID,
    AD_KEY_VAL_SEL,
    AD_KEY_VAL_RIGHT,
    AD_KEY_VAL_DOWN,
    AD_KEY_VAL_LEFT,
}; // 中间值，从小到大排列

// 定义按键事件，用于存放到数组中
enum
{
    KYE_UP_CLICK,
    KEY_UP_LONG,
    KEY_UP_HOLD,

    KEY_MID_CLICK,
    KEY_MID_LONG,
    KEY_MID_HOLD,

    KEY_SEL_CLICK,
    KEY_SEL_LONG,
    KEY_SEL_HOLD,

    KEY_RIGHT_CLICK,
    KEY_RIGHT_LONG,
    KEY_RIGHT_HOLD,

    KEY_DOWN_CLICK,
    KEY_DOWN_LONG,
    KEY_DOWN_HOLD,

    KEY_LEFT_CLICK,
    KEY_LEFT_LONG,
    KEY_LEFT_HOLD,
};

// 存放按键对应的按键事件
const u8 key_event_table[6][3] = {
    {KYE_UP_CLICK, KEY_UP_LONG, KEY_UP_HOLD},
    {KEY_MID_CLICK, KEY_MID_LONG, KEY_MID_HOLD},
    {KEY_SEL_CLICK, KEY_SEL_LONG, KEY_SEL_HOLD},
    {KEY_RIGHT_CLICK, KEY_RIGHT_LONG, KEY_RIGHT_HOLD},
    {KEY_DOWN_CLICK, KEY_DOWN_LONG, KEY_DOWN_HOLD},
    {KEY_LEFT_CLICK, KEY_LEFT_LONG, KEY_LEFT_HOLD},
};
// ===================================================
// TM1650相关配置                                    //
// ===================================================
// AIP1650(TM1650) 驱动相关定义：
#define I2C_SCL_PIN (P11D)
#define I2C_SDA_PIN (P04D)

#define I2C_SDA_IN() (P04OE = 0)      // 设置I2C SDA引脚为输入
#define I2C_SDA_OUT() (P04OE = 1)     // 设置I2C SDA引脚为输出
#define I2C_SDA_H() (I2C_SDA_PIN = 1) // 设置I2C SDA引脚为高电平
#define I2C_SDA_L() (I2C_SDA_PIN = 0) // 设置I2C SDA引脚为低电平
#define I2C_SDA_IS_H() (I2C_SDA_PIN)  // 判断I2C SDA引脚是否为高电平

#define I2C_SCL_OUT() (P11OE = 1)     // 设置I2C SCL引脚为输出
#define I2C_SCL_H() (I2C_SCL_PIN = 1) // 设置I2C SCL引脚为高电平
#define I2C_SCL_L() (I2C_SCL_PIN = 0) // 设置I2C SCL引脚为低电平

// TM1650 命令定义
#define TM1650_DISPLAY_ON 0x01  // 显示开
#define TM1650_DISPLAY_OFF 0x00 // 显示关
#define TM1650_BRIGHT_MIN 0x00  // 最小亮度
#define TM1650_BRIGHT_MAX 0x70  // 最大亮度

// 数码管段码表 (0-9)
const uint8_t NUM_TABLE[] = {
    0xEB, // 0
    0x88, // 1
    0xB3, // 2
    0xBA, // 3
    0xD8, // 4
    0x7A, // 5
    0x7B, // 6
    0xA8, // 7
    0xFB, // 8
    0xFA, // 9
};
// ===================================================
// 单线通信相关配置                                   //
// ===================================================
// 单线通信发送/接收状态枚举
typedef enum
{
    IDLE,           // 空闲状态
    TX_START,       // 发送起始信号
    TX_SENDING_BIT, // 发送数据位
    TX_SENDING_LOW, // 发送低电平部分
    RX_START,       // 检测起始信号
    RX_DATA,        // 接收数据位
    RX_END          // 接收完成
} TRxState;

// 单线通信使用的宏
// 时间相关宏定义
#define START_SIGNAL_TIME (100) // 发送起始信号时间 10ms/100us = 100
#define BIT1_HIGH_TIME (10)     // 发送1高电平时间 1ms/100us = 10
#define BIT0_HIGH_TIME (5)      // 发送0高电平时间 0.5ms/100us = 5
#define BIT1_LOW_TIME (5)       // 发送1低电平时间 0.5ms/100us = 5
#define BIT0_LOW_TIME (10)      // 发送0低电平时间 1ms/100us = 10
// 单线通信使用的宏
#define RX_START_MIN_TIME (80)  // 接收起始信号最小时间 8ms/100us = 80
#define RX_START_MAX_TIME (150) // 接收起始信号最大时间 15ms/100us = 150
#define RX_BIT1_MIN_TIME (8)    // 接收1最小时间 0.8ms/100us = 8
#define RX_BIT1_MAX_TIME (12)   // 接收1最大时间 1.2ms/100us = 12
// 单线通信使用的宏
#define SEND_SEND_BIT(x) (1 << (7 - x)) // 发送数据时,使用的位比较
#define RECV_RECV_BIT(x) (1 << (x))     // 接收数据时,使用的位比较

#define SEND_DATA_LEN 3 // 发送数据长度
#define RX_DATA_LEN 6   // 接收数据长度
//
#define set_pin_low() (P13D = 0)  // 设置引脚低电平
#define set_pin_high() (P13D = 1) // 设置引脚高电平

#if USE_MY_DEBUG

#define get_pin_state() (P20D) // 获取引脚状态--测试用

#else

#define get_pin_state() (P16D) // 获取引脚状态

#endif

volatile uint8_t send_buf[SEND_DATA_LEN]; // 单线通信的数据缓冲区
                                          // volatile u8 recv_buf[SEND_DATA_LEN]; // 单线通信的接收缓冲区

// 单线通信使用的全局变量
// 全局变量
volatile uint8_t g_data[RX_DATA_LEN];     // 发送/接收数据缓冲区
static volatile uint8_t g_byte_count;     // 当前字节计数
static volatile uint8_t g_bit_count;      // 当前位计数
static volatile uint8_t g_timer_count;    // 定时器计数
volatile uint8_t g_state = IDLE;   // 当前状态
static volatile uint8_t g_last_pin_state; // 上一次引脚状态
volatile uint8_t pin_state;

// adc采样，滤波函数中使用的变量：
u16 adc_val_tmp; // 临时存放单次转换中采集的ad值
u32 adc_val_sum; // 存放多次采集到的ad值
u16 get_adcmax;
u16 get_adcmin;

volatile u16 adc_val; // 存放采集到的ad值

volatile u8 ad_key_event; // 存放按键检测函数检测到的按键事件
volatile u8 cur_key_id;   // 存放按键检测函数检测到的按键id

volatile u8 set_time_hold_cnt; // 设置时间的长按计数
// volatile u16 cur_time;         // 存放当前时间 0~9999 -> 00:00 ~ 99:99

enum
{
    STATUS_NONE = 0, // 无状态
    STATUS_SET_TIME_HOUR = 1,
    STATUS_SET_TIME_MIN,

};
volatile u8 cur_set_time_status; // 记录当前的状态（状态机）

volatile u16 set_time_delay_cnt;                  // 调节时间时，让数码管闪烁得到时间计时
volatile u16 set_time_done_cnt;                   // 调节时间时，没有操作的持续时间计时
volatile uint8_t thousand, hundred, decade, unit; // 时间 0~9999，每个变量只存一位十进制的数据

//===============Global Function===============
void Sys_Init(void);
void CLR_RAM(void);
void IO_Init(void);
void delay_ms(u16 xms);
void delay_us(u8 xus);
void adc_config(void);
void timer1_config(void); // 定时器配置，使用外部晶振作为时钟源
void timer3_config(void);
void pwm_led_config(void);

// void start_send(uint8_t *data);
void start_send(void);
void timer_100us_isr(void);

void adc_channel_sel(u8 adc_channel);
u16 adc_single_convert(void);
u16 adc_get_val(void);
u8 get_key_id(void);
void ad_key_event_10ms_isr(void);
void key_ad_key_event_deal(void);

// void bsp_i2c_init(void);
// void bsp_i2c_start(void);
void bsp_i2c_tx_byte(uint8_t dat);
u8 bsp_i2c_rx_ack(void);
// void bsp_i2c_stop(void);
void TM1650_WriteByte(uint8_t addr, uint8_t data);
// void TM1650_Clear(void);
void TM1650_Init(void);
// void TM1650_DisplayBit(uint8_t pos, uint8_t num);

void TM1650_DisplayTime(void); // 显示数字(0-9999)

//============Define  Flag=================
typedef union
{
    unsigned char byte;
    struct
    {
        u8 bit0 : 1;
        u8 bit1 : 1;
        u8 bit2 : 1;
        u8 bit3 : 1;
        u8 bit4 : 1;
        u8 bit5 : 1;
        u8 bit6 : 1;
        u8 bit7 : 1;
    } bits;
} bit_flag;
volatile bit_flag flag1;
volatile bit_flag flag2;
// u8 flag_10ms;
#define flag_10ms flag1.bits.bit0
#define flag_is_ble_open flag1.bits.bit1   // 标志位，蓝牙是否开启，0--未开启，1--开启
#define adjust_pwm_dir flag1.bits.bit2     // 调节灯光亮度(pwm占空比)的方向, 0--降低，1--增加
#define flag_recv_time_ack flag1.bits.bit3 // 接收到时间后，返回应答，0--还未接收到时间，1--接收到了时间，返回应答
#define flag_is_send_data flag1.bits.bit4  // 是否要发送数据的标志位，0--不发送数据，1--发送数据

#define flag_is_last_power_low flag1.bits.bit5 // 上次是否检测到低电压
#define flag_is_enter_low_power flag1.bits.bit6 // 是否进入低功耗，0--不进入低功耗，1--进入低功耗

// #define flag_is_ctl_tm1650 flag1.bits.bit7 // 标志位，是否在和屏幕驱动ic通信，0--否，1--正在通信

enum
{
    DEV_USE_NONE,
    DEV_USE_TM1650,
    // DEV_USE_AD,
    // DEV_USE_COMMNUNICATE,
    DEV_USE_ELSE,
};  
volatile u8 cur_dev_use; // 记录当前外设使用情况，如果没有使用外设，立即进入低功耗


// #if USE_MY_DEBUG // 占用约83字节程序空间
#define DEBUG_PIN P22D
#if 0 // 占用约83字节程序空间
// 通过一个引脚输出数据

void send_data_msb(u32 send_data)
{
    // 先发送格式头
    // __set_input_pull_up(); // 高电平
    DEBUG_PIN = 1;
    delay_ms(15);
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(7); //

    for (u8 i = 0; i < 32; i++)
    {
        if ((send_data >> (32 - 1 - i)) & 0x01)
        {
            // 如果要发送逻辑1
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(10); //
        }
        else
        {
            // 如果要发送逻辑0
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(5); //
        }
    }

    // 最后，设置为低电平
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(1);
    DEBUG_PIN = 1;
    delay_ms(1);
    DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

#endif

/**************************** end of file *********************************************/
