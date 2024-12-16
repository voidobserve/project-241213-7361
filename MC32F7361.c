/******************************************************************************
;  *       @型号                 : MC32F7361
;  *       @创建日期             : 2021.12.21
;  *       @公司/作者            : SINOMCU-FAE
;  *       @晟矽微技术支持       : 2048615934
;  *       @晟矽微官网           : http://www.sinomcu.com/
;  *       @版权                 : 2021 SINOMCU公司版权所有.
;  *----------------------摘要描述---------------------------------
;  *                  ADC
;  *                  上电等待AD稳定、零点校准
;  *                  ADC时钟为HIRC32分频1M，12位数据H8L4、内部2V、15ADCLK
;  *                  定时器2MS，P10 采集电压值
;  *                  采集电压值<1.5V:P11为低电平;>1.5V:P11为高电平
******************************************************************************/

#include "user.h"

// 毫秒级延时 (误差：在1%以内，1ms、10ms、100ms延时的误差均小于1%)
// 前提条件：FCPU = FHOSC / 4
void delay_ms(u16 xms)
{
    while (xms)
    {
        u16 i = 572;
        while (i--)
        {
            Nop();
        }
        xms--; // 把 --操作放在while()判断条件外面，更节省空间
    }
}

// 不准确的微秒级延时，只是为了给IIC驱动提供us延时
void delay_us(u8 xus)
{
    while (xus)
    {
        Nop();
        xus--; // 把 --操作放在while()判断条件外面，更节省空间
    }
}

/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            : 清RAM
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR0 = 0; FSR0 < 0xff; FSR0++)
    {
        INDF0 = 0x00;
    }
    FSR0 = 0xFF;
    INDF0 = 0x00;
}
/************************************************
;  *    @函数名            : IO_Init
;  *    @说明              : IO初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void IO_Init(void)
{
    IOP0 = 0x00;   // io口数据位
    OEP0 = 0x3F;   // io口方向 1:out  0:in
    PUP0 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP0 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P0ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP1 = 0x00;   // io口数据位
    OEP1 = 0xFF;   // io口方向 1:out  0:in
    PUP1 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP1 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P1ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP2 = 0x00; // io口数据位
    OEP2 = 0x0F; // io口方向 1:out  0:in
    PUP2 = 0x00; // io口上拉电阻   1:enable  0:disable
    PDP2 = 0x00; // io口下拉电阻   1:enable  0:disablea

    PMOD = 0x00;  // P00、P01、P13 io端口值从寄存器读，推挽输出
    DRVCR = 0x80; // 普通驱动
}

void adc_config(void)
{
    // P12,AD按键检测引脚
    P12OE = 0;    // 输入模式
    P12DC = 1;    // 模拟功能
    ADCR0 = 0xFB; // 使能ADC、12位数据H8L4
    ADCR1 = 0x80; // adc时钟32分频、内部2V参考电压
    ADCR2 = 0xFF; // 固定为15ADCLK

    ADEN = 1;
}

void timer0_config(void)
{
    // T0CR = DEF_SET_BIT0 | DEF_SET_BIT2; // 定时模式,CPU,32分频
    // T0CNT = 250 - 1;                    // 1ms
    // T0LOAD = 250 - 1;
    // T0EN = 1;
    // T0IE = 1;
}

void timer1_config(void)
{
    // T1CR = DEF_SET_BIT0 | DEF_SET_BIT2 | DEF_SET_BIT3 | DEF_SET_BIT4; // 定时模式,FLOSC,32分频
    T1CR = 0x15; // 时钟源选择FLOSC，32分频，32Khz(32768) / 32 约为 1Khz，实际上每隔0.9765625ms计数一次
    // T1CNT = 2 - 1; //
    T1LOAD = 10 - 1; //
    T1EN = 1;
    T1IE = 1;
}

void timer3_config(void)
{
    T3CR = DEF_SET_BIT0 | DEF_SET_BIT2; // 定时模式,CPU,32分频
    T3CNT = 250 - 1;                    // 1ms
    T3LOAD = 250 - 1;
    T3EN = 1;
    T3IE = 1;
}

// 配置PWM，默认占空比为 0%
void pwm_led_config(void)
{
    // PWM0O1 PWM2
    T0CR = DEF_SET_BIT6 | DEF_SET_BIT0 | DEF_SET_BIT1; // 使能PWM,CPU,8分频
    T0CNT = 255 - 1;
    T0LOAD = 255 - 1; //
    T0DATA = 0;

    T2CR = DEF_SET_BIT6 | DEF_SET_BIT0 | DEF_SET_BIT1; // 使能PWM,CPU,8分频
    T2CNT = 255 - 1;
    T2LOAD = 255 - 1; //
    T2DATA = 0;

    PWMCR1 = 0x00; // pwm工作于普通模式
    PWMCR3 = 0x04; // PWM0选择P02端口输出
    T0EN = 1;
    T2EN = 1;
}

// 启动发送
void start_send(uint8_t *data)
{
    if (g_state != IDLE)
        return; // 如果正在收发则返回

    // 复制要发送的数据
    for (i = 0; i < SEND_DATA_LEN; i++)
    {
        g_data[i] = data[i];
    }
    g_byte_count = 0;
    g_bit_count = 0;
    g_timer_count = 0;
    g_state = TX_START;

    set_pin_low(); // 开始发送起始信号
}

// 100us定时器中断服务函数
void timer_100us_isr(void)
{
    // 发送处理
    if (g_state == TX_START || g_state == TX_SENDING_BIT || g_state == TX_SENDING_LOW)
    {
        switch (g_state)
        {
        case TX_START:
            if (++g_timer_count >= START_SIGNAL_TIME)
            {
                g_timer_count = 0;
                g_state = TX_SENDING_BIT;
                set_pin_high();
            }
            break;

        case TX_SENDING_BIT:
            if (g_data[g_byte_count] & BIT(g_bit_count))
            {
                // 发送1: 高电平1ms
                if (++g_timer_count >= BIT1_HIGH_TIME)
                {
                    g_timer_count = 0;
                    g_state = TX_SENDING_LOW;
                    set_pin_low();
                }
            }
            else
            {
                // 发送0: 高电平0.5ms
                if (++g_timer_count >= BIT0_HIGH_TIME)
                {
                    g_timer_count = 0;
                    g_state = TX_SENDING_LOW;
                    set_pin_low();
                }
            }
            break;

        case TX_SENDING_LOW:
            if (g_data[g_byte_count] & BIT(g_bit_count))
            {
                // 发送1: 低电平0.5ms
                if (++g_timer_count >= BIT1_LOW_TIME)
                {
                    g_timer_count = 0;
                    if (++g_bit_count >= 8)
                    {
                        g_bit_count = 0;
                        if (++g_byte_count >= SEND_DATA_LEN)
                        {
                            g_state = IDLE;
                        }
                    }
                    if (g_state != IDLE)
                    {
                        g_state = TX_SENDING_BIT;
                        set_pin_high();
                    }
                }
            }
            else
            {
                // 发送0: 低电平1ms
                if (++g_timer_count >= BIT0_LOW_TIME)
                {
                    g_timer_count = 0;
                    if (++g_bit_count >= 8)
                    {
                        g_bit_count = 0;
                        if (++g_byte_count >= SEND_DATA_LEN)
                        {
                            g_state = IDLE;
                        }
                    }
                    if (g_state != IDLE)
                    {
                        g_state = TX_SENDING_BIT;
                        set_pin_high();
                    }
                }
            }
            break;
        }
        return;
    }

    // 接收处理
    pin_state = get_pin_state();

    switch (g_state)
    {
    case IDLE:
        if (pin_state == 0)
        { // 检测到起始信号
            g_timer_count = 0;
            g_state = RX_START;
            g_byte_count = 0;
            g_bit_count = 0;
            // memset(g_data, 0, sizeof(g_data));
            for (i = 0; i < 9; i++)
            {
                g_data[i] = 0;
            }
        }
        break;

    case RX_START:
        g_timer_count++;
        if (pin_state == 1)
        { // 起始信号结束
            // 8ms~15ms的起始信号
            if (g_timer_count >= RX_START_MIN_TIME && g_timer_count <= RX_START_MAX_TIME)
            {
                g_timer_count = 0;
                g_state = RX_DATA;
            }
            else
            { // 无效信号
                g_state = IDLE;
            }
        }
        break;

    case RX_DATA:
        g_timer_count++;
        if (pin_state != g_last_pin_state)
        { // 检测到电平变化
            if (g_last_pin_state == 1)
            { // 高电平结束
                // 判断数据1: 高电平0.8ms~1.2ms (8~12个计数)
                // 判断数据0: 高电平0.3ms~0.7ms (3~7个计数)
                if (g_timer_count >= RX_BIT1_MIN_TIME && g_timer_count <= RX_BIT1_MAX_TIME)
                { // 接收到1
                    g_data[g_byte_count] |= BIT(g_bit_count);
                }
                // 如果是3~7个计数则为0，不需要设置数据位
                g_bit_count++;
                if (g_bit_count >= 8)
                {
                    g_bit_count = 0;
                    g_byte_count++;
                    if (g_byte_count >= RX_DATA_LEN)
                    {
                        g_state = RX_END; // 接收完成
                    }
                }
            }
            g_timer_count = 0;
        }
        break;

    case RX_END:
        // 在这里可以处理接收到的数据g_data
        g_state = IDLE;
        break;
    }
    g_last_pin_state = pin_state;
}

// 使用示例
void example(void)
{
    uint8_t send_buf[SEND_DATA_LEN] = {0x01, 0x02, 0x03};
    // 发送新数据
    start_send(send_buf);

    // 接收到的数据可以在RX_END状态下处理
}

void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();

    adc_config();
    // timer1_config();
    TM1650_Init();
    pwm_led_config();
    timer3_config();

    // 充电检测引脚，只检测电平
    P03PD = 1; // 下拉
    P03OE = 1; // 充电检测脚，输入模式

    // 蓝牙控制引脚，配置为输入模式
    BLE_CTL_PIN_IN();

    ad_key_event = KEY_EVENT_NONE;
    cur_key_id = AD_KEY_ID_NONE;

    GIE = 1;
}

#if 1

void adc_channel_sel(u8 adc_channel)
{
    switch (adc_channel)
    {
    case ADC_CHANNEL_KEY:
        ADCR0 = 0x7F; // AN7通道，使能ADC
        ADCR1 = 0x82; // adc时钟 == FHIRC/32，使用内部4V参考电压
        break;

    case ADC_CHANNEL_VDD:
        ADCR0 = 0xAF; // 1/VDD通道，使能ADC
        ADCR1 = 0x80; // adc时钟 == FHIRC/32，使用内部2V参考电压

    default:
        break;
    }

    delay_ms(1);
}

u16 adc_single_convert(void)
{
    ADEOC = 0;
    while (!ADEOC)
        ; // 等待转换完成
    adc_val_tmp = ADRH;
    adc_val_tmp = adc_val_tmp << 4 | (ADRL & 0x0F);
    return adc_val_tmp;
}

u16 adc_get_val(void)
{
    adc_val_sum = 0;
    get_adcmax = 0;
    get_adcmin = 0xFFFF;

    for (i = 0; i < 20; i++)
    {
        adc_val_tmp = adc_single_convert(); // 这里补充上adc单次转换获得的数据
        if (i < 2)
            continue; // 丢弃前两次采样的
        if (adc_val_tmp > get_adcmax)
            get_adcmax = adc_val_tmp; // 更新当前采集到的最大值
        if (adc_val_tmp < get_adcmin)
            get_adcmin = adc_val_tmp; // 更新当前采集到的最小值
        adc_val_sum += adc_val_tmp;
    }

    adc_val_sum -= get_adcmax;        // 去掉一个最大
    adc_val_sum -= get_adcmin;        // 去掉一个最小
    adc_val_tmp = (adc_val_sum >> 4); // 除以16，取平均值

    return adc_val_tmp;
}

u8 get_key_id(void)
{
    adc_val = adc_single_convert() >> 4;
    for (i = 0; i < AD_KEY_ID_NONE; i++)
    {
        if (adc_val <= id_table[i])
            return i;
    }
    return AD_KEY_ID_NONE;
}

void ad_key_event_10ms_isr(void)
{
    static u8 last = AD_KEY_ID_NONE;
    static u8 count = 0;
    // 有按键按下时返回 AD_KEY_ID ，无按键返回 KEY_ID_NONE
    // 找到 id_table[] 中对应的按键:
    cur_key_id = get_key_id();

    // if (cur_key_id < AD_KEY_ID_NONE)
    //     send_data_msb(cur_key_id); // 测试发现这里可以得到对应的按键id

    if (last != cur_key_id) // 如果有按下 / 抬起动作
    {
        if (last == AD_KEY_ID_NONE)
        {
            // 按下
            count = 0;
        }
        else if (cur_key_id == AD_KEY_ID_NONE)
        {
            // 抬起
            if (count < 75)
            {
                // 短按  -->  ad_key_event
                ad_key_event = KEY_EVENT_CLICK;
            }
        }
        else
        {
            // 按下按键时中途又按下其他按键，这种情况不考虑
        }
    }
    else if (cur_key_id != AD_KEY_ID_NONE)
    {
        // 持续按下
        count++;

        if (count == 70)
        {
            // 长按  -->  ad_key_event
            ad_key_event = KEY_EVENT_LONG;
        }
        else if (count >= 70 + 10)
        {
            // 持续长按  -->  ad_key_event
            ad_key_event = KEY_EVENT_HOLD;
            count = 70;
        }
    }

    last = cur_key_id;
}

void key_ad_key_event_deal(void)
{
    if (!flag_10ms)
        return;

    flag_10ms = 0;
    // 10ms调用一次，得到 cur_key_id 和 ad_key_event ，然后对照查表
    ad_key_event_10ms_isr();

    // 如果检测到按键，这里发送给蓝牙IC
    switch (cur_key_id)
    {
    case AD_KEY_ID_UP:
        /* code */
        break;
    case AD_KEY_ID_MID:
        /* code */
        break;
    case AD_KEY_ID_SEL:
        /* code */
        break;
    case AD_KEY_ID_RIGHT:
        /* code */
        break;
    case AD_KEY_ID_DOWN:
        /* code */
        break;
    case AD_KEY_ID_LEFT:
        /* code */
        break;

    default:
        break;
    }

    // if (cur_key_id < AD_KEY_ID_NONE)
    if (cur_key_id < AD_KEY_ID_NONE && ad_key_event != KEY_EVENT_NONE)
    {
        // send_data_msb(cur_key_id); // 能检测到按键id,但是event==3
        // send_data_msb(ad_key_event);

        switch (key_event_table[cur_key_id][ad_key_event])
        {
            // case KYE_UP_CLICK: //
            //     break;
        case KEY_UP_LONG: // 长按时，开/关蓝牙

            if (flag_is_ble_open)
            { // 关闭蓝牙
                BLE_CTL_PIN_IN();
                flag_is_ble_open = 0;
            }
            else
            { // 打开蓝牙
                BLE_CTL_PIN_OUT();
                BLE_CTL_PIN_LOW(); // 输出低电平
                flag_is_ble_open = 1;
            }

            break;

        case KEY_UP_HOLD:
            break;

        case KEY_MID_CLICK:

            break;

        case KEY_MID_HOLD:

            break;

        case KEY_SEL_HOLD:
            // P22D = ~P22D;
            // 刚进入这里，已经过了800ms
            set_time_hold_cnt++;
            if (set_time_hold_cnt >= 220)
            {
                set_time_hold_cnt = 0;
                cur_status = STATUS_SET_TIME_HOUR;
            }

            break;

        case KEY_RIGHT_CLICK:

            set_time_done_cnt = 0;
            if (STATUS_SET_TIME_HOUR == cur_status)
            {
                if (cur_time < 2300)
                    cur_time += 100;
            }
            else if (STATUS_SET_TIME_MIN == cur_status)
            {
                if ((cur_time % 100) < 59)
                {
                    cur_time++;
                }
            }

            break;

        case KEY_RIGHT_HOLD:

            break;

        case KEY_DOWN_CLICK:

            break;

        case KEY_DOWN_HOLD:

            break;

        case KEY_LEFT_CLICK:

            set_time_done_cnt = 0;
            if (STATUS_SET_TIME_HOUR == cur_status)
            {
                if (cur_time > 0)
                    cur_time -= 100;
            }
            else if (STATUS_SET_TIME_MIN == cur_status)
            {
                if ((cur_time % 100) > 0)
                {
                    cur_time--;
                }
            }

            break;

        case KEY_LEFT_HOLD:

            break;

        default:
            break;
        }

        ad_key_event = KEY_EVENT_NONE; // 处理完事件后，清除
    }
}

#endif

void bsp_i2c_init(void)
{
    I2C_SDA_OUT();
    I2C_SCL_OUT();
    I2C_SDA_H();
    delay_ms(5);
}
void bsp_i2c_start(void)
{
    I2C_SDA_OUT();
    I2C_SCL_OUT();
    I2C_SDA_H();
    I2C_SCL_H();
    delay_us(5);
    I2C_SDA_L();
    delay_us(5);
    I2C_SCL_L();
}
void bsp_i2c_tx_byte(uint8_t dat)
{
    u8 i;
    I2C_SDA_OUT();
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
        {
            I2C_SDA_H();
        }
        else
        {
            I2C_SDA_L();
        }
        delay_us(5);
        I2C_SCL_H();
        delay_us(5);
        I2C_SCL_L();
        dat <<= 1;
    }
}
u8 bsp_i2c_rx_ack(void)
{
    // u8 ret = false;
    u8 ret = 0;
    I2C_SDA_IN();
    delay_us(5);
    I2C_SCL_H();
    delay_us(5);
    if (!I2C_SDA_IS_H())
    {
        // ret = true;
        ret = 1;
    }
    I2C_SCL_L();
    return ret;
}
void bsp_i2c_stop(void)
{
    I2C_SDA_OUT();
    I2C_SDA_L();
    delay_us(5);
    I2C_SCL_H();
    delay_us(5);
    I2C_SDA_H();
}

// IIC写一个字节
void TM1650_WriteByte(uint8_t addr, uint8_t data)
{
    bsp_i2c_start();
    bsp_i2c_tx_byte(addr);
    bsp_i2c_rx_ack();
    bsp_i2c_tx_byte(data);
    bsp_i2c_rx_ack();
    bsp_i2c_stop();
}

// 清空显示
void TM1650_Clear(void)
{
    // 清空4个数码管
    TM1650_WriteByte(0x68, 0x00); // 第1位
    TM1650_WriteByte(0x6A, 0x00); // 第2位
    TM1650_WriteByte(0x6C, 0x00); // 第3位
    TM1650_WriteByte(0x6E, 0x00); // 第4位
}

// 初始化TM1650
void TM1650_Init(void)
{
    bsp_i2c_init();

    // 设置数据命令
    TM1650_WriteByte(0x48, 0x01); // 开启显示

    // 设置亮度(最大亮度)
    TM1650_WriteByte(0x48, TM1650_DISPLAY_ON | TM1650_BRIGHT_MAX);

    // 清空显示
    TM1650_Clear();
}

// 显示单个数字
void TM1650_DisplayBit(uint8_t pos, uint8_t num)
{
    uint8_t addr;

    // if (pos > 4 || num > 9)
    //     return; // 参数检查

    // 计算位置对应的地址
    addr = 0x68 + (pos - 1) * 2;

    // 写入段码
    TM1650_WriteByte(addr, NUM_TABLE[num]);
}

// 显示数字(0-9999)
void TM1650_DisplayNum(uint16_t num)
{
    uint8_t thousand, hundred, decade, unit;

    if (num > 9999)
        return; // 参数检查

    // 分离各个位
    thousand = num / 1000;
    hundred = (num % 1000) / 100;
    decade = (num % 100) / 10;
    unit = num % 10;

    // 显示各个位
    TM1650_DisplayBit(4, thousand);
    TM1650_DisplayBit(3, hundred);
    TM1650_DisplayBit(2, decade);
    TM1650_DisplayBit(1, unit);
}

/************************************************
;  *    @函数名            : main
;  *    @说明              : 主程序
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void main(void)
{
    Sys_Init();

    while (1)
    {
        adc_channel_sel(ADC_CHANNEL_KEY);
        key_ad_key_event_deal();

        if (STATUS_SET_TIME_HOUR == cur_status)
        { // 设置小时
            // TM1650_DisplayNum(cur_time);
            // delay_ms(500);
            // TM1650_WriteByte(0x6E, 0x00); // 第4位
            // TM1650_WriteByte(0x6C, 0x00); // 第3位
            // delay_ms(500);

            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayNum(cur_time);
            }
            else
            {
                TM1650_WriteByte(0x6E, 0x00); // 第4位
                TM1650_WriteByte(0x6C, 0x00); // 第3位

                if (set_time_delay_cnt >= 1000 - 1)
                {
                    set_time_delay_cnt = 0;
                }
            }

            if (set_time_done_cnt >= 5000)
            {
                cur_status = STATUS_SET_TIME_MIN;
                set_time_done_cnt = 0;
            }
        }
        else if (STATUS_SET_TIME_MIN == cur_status)
        { // 设置分钟
            // TM1650_DisplayNum(cur_time);
            // delay_ms(500);
            // TM1650_WriteByte(0x6A, 0x00); // 第2位
            // TM1650_WriteByte(0x68, 0x00); // 第1位
            // delay_ms(500);

            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayNum(cur_time);
            }
            else
            {
                TM1650_WriteByte(0x6A, 0x00); // 第2位
                TM1650_WriteByte(0x68, 0x00); // 第1位

                if (set_time_delay_cnt >= 1000 - 1)
                {
                    set_time_delay_cnt = 0;
                }
            }

            if (set_time_done_cnt >= 5000)
            {
                cur_status = STATUS_NONE;
                set_time_done_cnt = 0;
            }
        }
    }
}
/************************************************
;  *    @函数名            : interrupt
;  *    @说明              : 中断函数
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;
    swapar _PFLAG;
    movra _statusbuf;
    __endasm;

    // if (T0IF & T0IE)
    if (T1IF & T1IE)
    {
        static u16 timer1_cnt = 0;
        timer1_cnt++;
        if (timer1_cnt >= 1024)
        {
            P12D = ~P12D;
            timer1_cnt = 0;
        }

        // if (timer0_cnt < 10)
        // {
        //     timer0_cnt++;
        // }

        // if (timer0_cnt >= 10)
        // {
        //     timer0_cnt = 0;
        //     flag_10ms = 1;
        // }

        T1IF = 0;
    }

    if (T3IF & T3IE)
    {
        static u8 timer3_cnt = 0;
        timer3_cnt++;
        if (timer3_cnt >= 10)
        {
            flag_10ms = 1;
            timer3_cnt = 0;
        }

        if (STATUS_SET_TIME_HOUR == cur_status ||
            STATUS_SET_TIME_MIN == cur_status)
        {
            set_time_delay_cnt++;
            set_time_done_cnt++;
        }

        T3IF = 0;
    }

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
