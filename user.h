/******************************************************************************
;  *       @�ͺ�                   : MC32F7361
;  *       @��������               : 2021.12.21
;  *       @��˾/����              : SINOMCU-FAE
;  *       @����΢����֧��         : 2048615934
;  *       @����΢����             : http://www.sinomcu.com/
;  *       @��Ȩ                   : 2021 SINOMCU��˾��Ȩ����.
;  *---------------------- ���� ---------------------------------
;  *                   ��������ʱʹ��ȫ�ֱ���
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
#define ONE_CYCLE_CNT_MS () // ��ѭ��һ�������ʱ�䣬��λ��ms

#endif

// ������������
#if USE_MY_DEBUG
#define BLE_CTL_PIN (P17D)
#define BLE_CTL_PIN_OUT() (P17OE = 1)        // �������������л�Ϊ���ģʽ
#define BLE_CTL_PIN_IN() (P17OE = 0)         // �������������л�Ϊ����ģʽ
#define BLE_CTL_PIN_HIGH() (BLE_CTL_PIN = 1) // ����������������ߵ�ƽ
#define BLE_CTL_PIN_LOW() (BLE_CTL_PIN = 0)  // ����������������͵�ƽ
#else
#define BLE_CTL_PIN (P17D)
#define BLE_CTL_PIN_OUT() (P17OE = 1)        // �������������л�Ϊ���ģʽ
#define BLE_CTL_PIN_IN() (P17OE = 0)         // �������������л�Ϊ����ģʽ
#define BLE_CTL_PIN_HIGH() (BLE_CTL_PIN = 1) // ����������������ߵ�ƽ
#define BLE_CTL_PIN_LOW() (BLE_CTL_PIN = 0)  // ����������������͵�ƽ
#endif

// ======================================================
// �жϷ�����ʹ�õ��ı���:
u8 abuf;
u8 statusbuf;

//===============Global Variable===============
u16 i; // ѭ������ֵ

u16 tmp;

// ===================================================
// pwm�ƹ��������                                   //
// ===================================================
#define WHITE_PWM_DUTY_REG (T2DATA)  // ���ð׵�pwmռ�ձȵļĴ���
#define YELLOW_PWM_DUTY_REG (T0DATA) // ���ûƵ�pwmռ�ձȵļĴ���
volatile u8 cur_w_pwm_duty;          // ��¼�׵ƶ�Ӧ��pwmռ�ձ�
volatile u8 cur_y_pwm_duty;          // ��¼�Ƶƶ�Ӧ��pwmռ�ձ�

// ���嵱ǰ�ƹ��״̬(����״̬����)
enum
{
    CUR_LIGHT_STATUS_OFF,
    CUR_LIGHT_STATUS_WHITE,
    CUR_LIGHT_STATUS_YELLOW,
    CUR_LIGHT_STATUS_WHITE_YELLOW,
};
volatile u8 cur_light_status; // �浵��ǰ�ƹ�״̬��״̬��

// ===================================================
// adc�������                                       //
// ===================================================
// ����adc��ͨ���������л�adcͨ��
enum
{
    ADC_CHANNEL_VDD,
    ADC_CHANNEL_KEY,
};
// #define // ������Ӧ��    adֵ >> 4
/*
    ʹ��4V�ο���ѹ����4.8V~4.9V�����£����
    û�а������µ�ʱ�򣬶�Ӧ�ĵ�ѹԼΪ��ԴVDD�ĵ�ѹ
    ����        ����ʱ��Ӧ�ĵ�ѹ      ��Ӧ��adֵ  ��Ӧ��adֵ>>4
    ad_sel      1.53V               1566        97
    ad_up       0.20V               204         12
    ad_down     2.40V               2457        153
    ad_left     3.63V               3717        232
    ad_right    2.04V               2089        130
    ad_mid      0.94V               962         60

    ȡ(adֵ>>4)�м�ֵ��Ϊ�ָ���:
    12  60  97  130  153  232
    ad <= 36    up          ���� �̰� ��������
    ad <= 78    mid         ���� �̰� ��������
    ad <= 113   sel         ���� ���� ����
    ad <= 141   right       ���� �̰� ��������
    ad <= 192   down        ���� �̰� ��������
    ad <= 243   left        ���� �̰� ��������
*/
#define AD_KEY_VAL_NONE 255
// #define _AD_KEY_VAL_SEL 97
// #define _AD_KEY_VAL_UP 12
// #define _AD_KEY_VAL_DOWN 153

#define AD_KEY_VAL_SEL 113
#define AD_KEY_VAL_UP 36
#define AD_KEY_VAL_DOWN 192
#define AD_KEY_VAL_LEFT 243
#define AD_KEY_VAL_RIGHT 141
#define AD_KEY_VAL_MID 78

// ���尴����id���� id_table[]�е�˳���й�
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

// ���尴�����¼�
enum
{
    KEY_EVENT_CLICK = 0,
    KEY_EVENT_LONG,
    KEY_EVENT_HOLD,

    KEY_EVENT_NONE,
};

// ��Ű���
const u8 id_table[6] = {
    AD_KEY_VAL_UP,
    AD_KEY_VAL_MID,
    AD_KEY_VAL_SEL,
    AD_KEY_VAL_RIGHT,
    AD_KEY_VAL_DOWN,
    AD_KEY_VAL_LEFT,
}; // �м�ֵ����С��������

// ���尴���¼������ڴ�ŵ�������
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

// ��Ű�����Ӧ�İ����¼�
const u8 key_event_table[6][3] = {
    {KYE_UP_CLICK, KEY_UP_LONG, KEY_UP_HOLD},
    {KEY_MID_CLICK, KEY_MID_LONG, KEY_MID_HOLD},
    {KEY_SEL_CLICK, KEY_SEL_LONG, KEY_SEL_HOLD},
    {KEY_RIGHT_CLICK, KEY_RIGHT_LONG, KEY_RIGHT_HOLD},
    {KEY_DOWN_CLICK, KEY_DOWN_LONG, KEY_DOWN_HOLD},
    {KEY_LEFT_CLICK, KEY_LEFT_LONG, KEY_LEFT_HOLD},
};

// AIP1650(TM1650) ������ض��壺
#define I2C_SCL_PIN (P11D)
#define I2C_SDA_PIN (P04D)

#define I2C_SDA_IN() (P04OE = 0)      // ����I2C SDA����Ϊ����
#define I2C_SDA_OUT() (P04OE = 1)     // ����I2C SDA����Ϊ���
#define I2C_SDA_H() (I2C_SDA_PIN = 1) // ����I2C SDA����Ϊ�ߵ�ƽ
#define I2C_SDA_L() (I2C_SDA_PIN = 0) // ����I2C SDA����Ϊ�͵�ƽ
#define I2C_SDA_IS_H() (I2C_SDA_PIN)  // �ж�I2C SDA�����Ƿ�Ϊ�ߵ�ƽ

#define I2C_SCL_OUT() (P11OE = 1)     // ����I2C SCL����Ϊ���
#define I2C_SCL_H() (I2C_SCL_PIN = 1) // ����I2C SCL����Ϊ�ߵ�ƽ
#define I2C_SCL_L() (I2C_SCL_PIN = 0) // ����I2C SCL����Ϊ�͵�ƽ

// TM1650 �����
#define TM1650_DISPLAY_ON 0x01  // ��ʾ��
#define TM1650_DISPLAY_OFF 0x00 // ��ʾ��
#define TM1650_BRIGHT_MIN 0x00  // ��С����
#define TM1650_BRIGHT_MAX 0x70  // �������

// ����ܶ���� (0-9)
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

// ����ͨ�ŷ���/����״̬ö��
typedef enum
{
    IDLE,           // ����״̬
    TX_START,       // ������ʼ�ź�
    TX_SENDING_BIT, // ��������λ
    TX_SENDING_LOW, // ���͵͵�ƽ����
    RX_START,       // �����ʼ�ź�
    RX_DATA,        // ��������λ
    RX_END          // �������
} TRxState;

// ����ͨ��ʹ�õĺ�
// ʱ����غ궨��
#define START_SIGNAL_TIME (100) // ������ʼ�ź�ʱ�� 10ms/100us = 100
#define BIT1_HIGH_TIME (10)     // ����1�ߵ�ƽʱ�� 1ms/100us = 10
#define BIT0_HIGH_TIME (5)      // ����0�ߵ�ƽʱ�� 0.5ms/100us = 5
#define BIT1_LOW_TIME (5)       // ����1�͵�ƽʱ�� 0.5ms/100us = 5
#define BIT0_LOW_TIME (10)      // ����0�͵�ƽʱ�� 1ms/100us = 10
// ����ͨ��ʹ�õĺ�
#define RX_START_MIN_TIME (80)  // ������ʼ�ź���Сʱ�� 8ms/100us = 80
#define RX_START_MAX_TIME (150) // ������ʼ�ź����ʱ�� 15ms/100us = 150
#define RX_BIT1_MIN_TIME (8)    // ����1��Сʱ�� 0.8ms/100us = 8
#define RX_BIT1_MAX_TIME (12)   // ����1���ʱ�� 1.2ms/100us = 12

// ����ͨ��ʹ�õĺ�
#define BIT(x) (1 << x)
#define SEND_DATA_LEN 3                   // �������ݳ���
#define RX_DATA_LEN 3                     // �������ݳ���
volatile uint8_t send_buf[SEND_DATA_LEN]; // ����ͨ�ŵ����ݻ�����
// volatile u8 recv_buf[SEND_DATA_LEN]; // ����ͨ�ŵĽ��ջ�����

// ����ͨ��ʹ�õ�ȫ�ֱ���
// ȫ�ֱ���
static uint8_t g_data[RX_DATA_LEN]; // ����/�������ݻ�����
static uint8_t g_byte_count;        // ��ǰ�ֽڼ���
static uint8_t g_bit_count;         // ��ǰλ����
static uint8_t g_timer_count;       // ��ʱ������
static uint8_t g_state = IDLE;      // ��ǰ״̬
static uint8_t g_last_pin_state;    // ��һ������״̬
uint8_t pin_state;

#define set_pin_low() (P13D = 0)  // �������ŵ͵�ƽ
#define set_pin_high() (P13D = 1) // �������Ÿߵ�ƽ
#define get_pin_state() (P16D)    // ��ȡ����״̬

// adc�������˲�������ʹ�õı�����
u16 adc_val_tmp; // ��ʱ��ŵ���ת���вɼ���adֵ
u32 adc_val_sum; // ��Ŷ�βɼ�����adֵ
u16 get_adcmax;
u16 get_adcmin;

volatile u16 adc_val; // ��Ųɼ�����adֵ

volatile u8 ad_key_event; // ��Ű�����⺯����⵽�İ����¼�
volatile u8 cur_key_id;   // ��Ű�����⺯����⵽�İ���id

volatile u8 set_time_hold_cnt; // ����ʱ��ĳ�������
// volatile u16 cur_time;         // ��ŵ�ǰʱ�� 0~9999 -> 00:00 ~ 99:99

enum
{
    STATUS_NONE = 0, // ��״̬
    STATUS_SET_TIME_HOUR = 1,
    STATUS_SET_TIME_MIN,

};
volatile u8 cur_status; // ��¼��ǰ��״̬��״̬����

volatile u16 set_time_delay_cnt;         // ����ʱ��ʱ�����������˸�õ�ʱ���ʱ
volatile u16 set_time_done_cnt;          // ����ʱ��ʱ��û�в����ĳ���ʱ���ʱ
uint8_t thousand, hundred, decade, unit; // ʱ�� 0~9999
// u8 hour, min;

//===============Global Function===============
void Sys_Init(void);
void CLR_RAM(void);
void IO_Init(void);
void delay_ms(u16 xms);
void delay_us(u8 xus);
void adc_config(void);
void timer1_config(void); // ��ʱ�����ã�ʹ���ⲿ������Ϊʱ��Դ
void timer3_config(void);
void pwm_led_config(void);

void start_send(uint8_t *data);
void timer_100us_isr(void);

void adc_channel_sel(u8 adc_channel);
u16 adc_single_convert(void);
u16 adc_get_val(void);
u8 get_key_id(void);
void ad_key_event_10ms_isr(void);
void key_ad_key_event_deal(void);

// void bsp_i2c_init(void);
void bsp_i2c_start(void);
void bsp_i2c_tx_byte(uint8_t dat);
u8 bsp_i2c_rx_ack(void);
void bsp_i2c_stop(void);
void TM1650_WriteByte(uint8_t addr, uint8_t data);
void TM1650_Clear(void);
void TM1650_Init(void);
void TM1650_DisplayBit(uint8_t pos, uint8_t num);

void TM1650_DisplayNum(); // ��ʾ����(0-9999)

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
// u8 flag_10ms;
#define flag_10ms flag1.bits.bit0
#define flag_is_ble_open flag1.bits.bit1 // ��־λ�������Ƿ�����0--δ������1--����
#define adjust_pwm_dir flag1.bits.bit2   // ���ڵƹ�����(pwmռ�ձ�)�ķ���, 0--���ͣ�1--����
// #define _adjust_pwm_dit flag1.bits.bit3 //

// #define flag_set_time flag1.bits.bit1
// #define g_last_pin_state flag1.bits.bit1 // ����ͨ��--��һ������״̬
// #define pin_state flag1.bits.bit2 // ����ͨ��--��һ������״̬

#if USE_MY_DEBUG
// ͨ��һ�������������
#define DEBUG_PIN P22D
void send_data_msb(u32 send_data)
{
    // �ȷ��͸�ʽͷ
    // __set_input_pull_up(); // �ߵ�ƽ
    DEBUG_PIN = 1;
    delay_ms(15);
    // __set_output_open_drain(); // �͵�ƽ
    DEBUG_PIN = 0;
    delay_ms(7); //

    for (u8 i = 0; i < 32; i++)
    {
        if ((send_data >> (32 - 1 - i)) & 0x01)
        {
            // ���Ҫ�����߼�1
            // __set_input_pull_up();  	   // �ߵ�ƽ
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // �͵�ƽ
            DEBUG_PIN = 0;
            delay_ms(10); //
        }
        else
        {
            // ���Ҫ�����߼�0
            // __set_input_pull_up();  	   // �ߵ�ƽ
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // �͵�ƽ
            DEBUG_PIN = 0;
            delay_ms(5); //
        }
    }

    // �������Ϊ�͵�ƽ
    // __set_output_open_drain(); // �͵�ƽ
    DEBUG_PIN = 0;
    delay_ms(1);
    DEBUG_PIN = 1;
    delay_ms(1);
    DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

#endif

/**************************** end of file *********************************************/
