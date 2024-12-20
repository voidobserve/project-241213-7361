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

        __asm;
        clrwdt; // 喂狗
        __endasm;
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
    // 切换通道时才配置ADCR1
    // ADCR1 = 0x80; // adc时钟32分频、内部2V参考电压
    ADCR2 = 0xFF; // 固定为15ADCLK

    ADEN = 1;
}

// void timer0_config(void)
// {
// T0CR = DEF_SET_BIT0 | DEF_SET_BIT2; // 定时模式,CPU,32分频
// T0CNT = 250 - 1;                    // 1ms
// T0LOAD = 250 - 1;
// T0EN = 1;
// T0IE = 1;
// }

// 定时器配置，使用外部晶振作为时钟源
void timer1_config(void)
{
    // T1CR = DEF_SET_BIT0 | DEF_SET_BIT2 | DEF_SET_BIT3 | DEF_SET_BIT4; // 定时模式,FLOSC,32分频
    T1CR = 0x15; // 时钟源选择FLOSC，32分频，32Khz(32768) / 32 约为 1Khz，实际上每隔0.9765625ms计数一次
    // T1CR = 0x00;     // 测试用的频率
    T1LOAD = 10 - 1; //
    // T1LOAD = 50 - 1; // 测试用的重装载值
    T1EN = 1;
    T1IE = 1;
}

//
void timer3_config(void)
{
    T3CR = 0x03; // 定时模式,CPU,8分频
    // T3CNT = 100 - 1;
    T3LOAD = 100 - 1; // 8分频后，这里是100us触发一次中断
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
    // T0DATA = 0; // 初始值就是0

    T2CR = DEF_SET_BIT6 | DEF_SET_BIT0 | DEF_SET_BIT1; // 使能PWM,CPU,8分频
    T2CNT = 255 - 1;
    T2LOAD = 255 - 1; //
    // T2DATA = 0;  // 初始值就是0

    PWMCR1 = 0x00; // pwm工作于普通模式
    PWMCR3 = 0x04; // PWM0选择P02端口输出
    T0EN = 1;
    T2EN = 1;
}

// 启动发送
// void start_send(uint8_t *data)
void start_send(void)
{
    if (g_state != IDLE)
        return; // 如果正在收发则返回

    // 复制要发送的数据
    for (i = 0; i < SEND_DATA_LEN; i++)
    {
        // g_data[i] = data[i];
        g_data[i] = send_buf[i];
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

        case TX_SENDING_BIT: // 发送高电平部分
            if (g_data[g_byte_count] & SEND_SEND_BIT(g_bit_count))
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

        case TX_SENDING_LOW: // 发送低电平部分
            if (g_data[g_byte_count] & SEND_SEND_BIT(g_bit_count))
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

        default:
            break;
        }

        return;
    }
    else
    {
        set_pin_high();
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
                    g_data[g_byte_count] |= RECV_RECV_BIT(g_bit_count);
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
        // 在这里可以处理接收到的数据 g_data

        if (g_data[0] == 0xA6)
        {
            thousand = g_data[1] >> 4 & 0x0F;
            hundred = g_data[1] & 0x0F;
            decade = g_data[2] >> 4 & 0x0F;
            unit = g_data[2] & 0x0F;
            flag_recv_time_ack = 1;
        }

        g_state = IDLE;
        break;
    }
    g_last_pin_state = pin_state;
}

// // 使用示例
// void example(void)
// {
//     uint8_t send_buf[SEND_DATA_LEN] = {0x01, 0x02, 0x03};
//     // 发送新数据
//     start_send(send_buf);

//     // 接收到的数据可以在 RX_END 状态下处理
// }

void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();

    adc_config();
    timer1_config(); // 使用外部晶振作为时钟源的定时器
    TM1650_Init();   // LED驱动IC配置
    pwm_led_config();
    timer3_config();

    // 充电检测引脚，只检测电平
    P03PD = 1; // 下拉
    P03OE = 1; // 充电检测脚，输入模式

    // 蓝牙控制引脚，配置为输入模式
    BLE_CTL_PIN_IN();

    // 单线通信的数据接收引脚：
#if USE_MY_DEBUG
    P20PU = 1; // 测试用的数据接收脚
    P20OE = 0;
#else
    P16PU = 1; // 上拉
    P16OE = 0; // 输入模式
#endif

    delay_ms(1); // 等待系统稳定，特别是定时器

    GIE = 1;
}

#if 1

void adc_channel_sel(u8 adc_channel)
{
    switch (adc_channel)
    {
    case ADC_CHANNEL_KEY:
        ADCR0 = 0x7F; // AN7通道，使能ADC
        ADCR1 = 0x83; // adc时钟 == FHIRC/32，使用VDD（5V）参考电压
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
    // adc_val = adc_get_val() >> 4;
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
    static volatile u8 last_key_id = AD_KEY_ID_NONE;
    static volatile u8 count = 0;                      // 长按计数
    static volatile u8 filter_cnt = 0;                 // 按键消抖，使用的变量
    static volatile u8 filter_key_id = AD_KEY_ID_NONE; // 消抖时记录上一次采集到的按键id
    // 有按键按下时返回 AD_KEY_ID ，无按键返回 KEY_ID_NONE
    // 找到 id_table[] 中对应的按键:
    cur_key_id = get_key_id();

    if (cur_key_id != filter_key_id && filter_cnt)
    { // 如果有按键按下
        filter_cnt = 0;
        filter_key_id = cur_key_id;
        return;
    }

    if (filter_cnt < 2)
    { // 如果检测到相同的按键按下/松开
        // 防止计数溢出
        filter_cnt++;
        return;
    }

    // if (cur_key_id < AD_KEY_ID_NONE)
    //     send_data_msb(cur_key_id); // 测试发现这里可以得到对应的按键id

    if (last_key_id != cur_key_id) // 如果有按下 / 抬起动作
    {
        if (last_key_id == AD_KEY_ID_NONE)
        { // 按下
            count = 0;
        }
        else if (cur_key_id == AD_KEY_ID_NONE)
        { // 抬起
            if (count < 75)
            {
                // 短按  -->  ad_key_event
                ad_key_event = KEY_EVENT_CLICK;
            }
            else
            { // 长按 / 长按持续后松手

                // if (AD_KEY_ID_MID == last_key_id && KEY_EVENT_HOLD == ad_key_event) // 不能使用这个判断条件
                if (AD_KEY_ID_MID == last_key_id)
                {
                    // 如果是调节灯光亮度的按键长按后又松手
                    // adjust_pwm_dir = !adjust_pwm_dir; // 改变灯光调节的方向 （该语句比下面到的语句更占用程序空间）
                    if (adjust_pwm_dir)
                    {
                        adjust_pwm_dir = 0;
                    }
                    else
                    {
                        adjust_pwm_dir = 1;
                    }
                } // if (AD_KEY_ID_MID == last_key_id)
                else if (AD_KEY_ID_RIGHT == last_key_id)
                {
                    if (STATUS_NONE == cur_set_time_status)
                    {
                        send_buf[1] = 0x64;
                        flag_is_send_data = 1;
                    }
                }
                else if (AD_KEY_ID_LEFT == last_key_id)
                {
                    if (STATUS_NONE == cur_set_time_status)
                    {
                        send_buf[1] = 0x54;
                        flag_is_send_data = 1;
                    }
                }
                else if (AD_KEY_ID_DOWN == last_key_id)
                {
                    send_buf[1] = 0x14;
                    flag_is_send_data = 1;
                }
            }
        }
        else
        {
            // 按下按键时中途又按下其他按键，这种情况不考虑
        }
    }
    else if (cur_key_id != AD_KEY_ID_NONE)
    { // 持续按下
        count++;
        if (count == 75)
        { // 长按  -->  ad_key_event
            ad_key_event = KEY_EVENT_LONG;
        }
        else if (count >= 75 + 15)
        { // 持续长按  -->  ad_key_event
            ad_key_event = KEY_EVENT_HOLD;
            count = 75;
        }
    }

    last_key_id = cur_key_id;
}

void key_ad_key_event_deal(void)
{
    if (!flag_10ms)
        return;

    flag_10ms = 0;
    // 10ms调用一次，得到 cur_key_id 和 ad_key_event ，然后对照查表
    ad_key_event_10ms_isr();

    if (cur_key_id < AD_KEY_ID_NONE &&
        ad_key_event != KEY_EVENT_NONE &&
        ad_key_event != 0)
    {
        // send_data_msb(cur_key_id); // 能检测到按键id
        // send_data_msb(ad_key_event);

        switch (key_event_table[cur_key_id][ad_key_event])
        {
        case KYE_UP_CLICK: //
            send_buf[1] = 0x41;
            flag_is_send_data = 1;
            break;
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
            // 灯的模式切换按键，短按

            switch (cur_light_status)
            {
            case CUR_LIGHT_STATUS_OFF:
                // 开灯
                WHITE_PWM_DUTY_REG = cur_w_pwm_duty;
                cur_light_status = CUR_LIGHT_STATUS_WHITE;
                break;
            case CUR_LIGHT_STATUS_WHITE:
                WHITE_PWM_DUTY_REG = 0;
                YELLOW_PWM_DUTY_REG = cur_y_pwm_duty;
                cur_light_status = CUR_LIGHT_STATUS_YELLOW;
                break;
            case CUR_LIGHT_STATUS_YELLOW:
                WHITE_PWM_DUTY_REG = cur_w_pwm_duty;
                YELLOW_PWM_DUTY_REG = cur_y_pwm_duty;
                cur_light_status = CUR_LIGHT_STATUS_WHITE_YELLOW;
                break;
            default: // CUR_LIGHT_STATUS_WHITE_YELLOW == cur_light_status
                WHITE_PWM_DUTY_REG = 0;
                YELLOW_PWM_DUTY_REG = 0;
                cur_light_status = CUR_LIGHT_STATUS_OFF;
                break;
            }

            break;

        case KEY_MID_HOLD:

            if (adjust_pwm_dir)
            { // 增大亮度
                if (CUR_LIGHT_STATUS_WHITE == cur_light_status ||
                    CUR_LIGHT_STATUS_WHITE_YELLOW == cur_light_status)
                {

                    if (cur_w_pwm_duty < 255 - 19)
                    {
                        cur_w_pwm_duty += 18;
                    }
                    else
                    {
                        cur_w_pwm_duty = 255;
                    }

                    WHITE_PWM_DUTY_REG = cur_w_pwm_duty;
                }

                if (CUR_LIGHT_STATUS_YELLOW == cur_light_status ||
                    CUR_LIGHT_STATUS_WHITE_YELLOW == cur_light_status)
                {
                    if (cur_y_pwm_duty < 255 - 19)
                    {
                        cur_y_pwm_duty += 18;
                    }
                    else
                    {
                        cur_y_pwm_duty = 255;
                    }

                    YELLOW_PWM_DUTY_REG = cur_y_pwm_duty;
                }
            } // if (adjust_pwm_dir)
            else
            { // 减小亮度
                if (CUR_LIGHT_STATUS_WHITE == cur_light_status ||
                    CUR_LIGHT_STATUS_WHITE_YELLOW == cur_light_status)
                {

                    if (cur_w_pwm_duty > 25 + 19)
                    {
                        cur_w_pwm_duty -= 18;
                    }
                    else
                    {
                        cur_w_pwm_duty = 25;
                    }

                    WHITE_PWM_DUTY_REG = cur_w_pwm_duty;
                }

                if (CUR_LIGHT_STATUS_YELLOW == cur_light_status ||
                    CUR_LIGHT_STATUS_WHITE_YELLOW == cur_light_status)
                {
                    if (cur_y_pwm_duty > 25 + 19)
                    {
                        cur_y_pwm_duty -= 18;
                    }
                    else
                    {
                        cur_y_pwm_duty = 25;
                    }

                    YELLOW_PWM_DUTY_REG = cur_y_pwm_duty;
                }
            } // if (adjust_pwm_dir) else

            break;

        case KEY_SEL_HOLD:
            // DEBUG_PIN = ~DEBUG_PIN;
            // 刚进入这里，已经过了750 + 150ms
            set_time_hold_cnt++;
            if (set_time_hold_cnt >= 14) // 14 * 150ms == 2100ms
            {
                set_time_hold_cnt = 0;
                cur_set_time_status = STATUS_SET_TIME_HOUR;
            }

            break;

        case KEY_RIGHT_CLICK:

            if (STATUS_NONE != cur_set_time_status)
            { // 如果在调节时间
                set_time_done_cnt = 0;
                set_time_delay_cnt = 0;
                if (STATUS_SET_TIME_HOUR == cur_set_time_status)
                {
                    hundred++;
                    if (thousand < 2) // 0~9H,10~19H
                    {
                        if (hundred >= 10)
                        {
                            hundred = 0;
                            thousand++;
                        }
                    }
                    else // 20~23H
                    {
                        if (hundred >= 4)
                        {
                            hundred--;
                        }
                    }

                    TM1650_DisplayTime();
                }
                else if (STATUS_SET_TIME_MIN == cur_set_time_status)
                {
                    unit++;
                    if (unit >= 10)
                    {
                        if (decade < 5)
                        { // 如果未满50min （59 min）
                            unit = 0;
                            decade++;
                        }
                        else
                        { // 如果已经超过59min，不进行调节
                            unit--;
                        }
                    }

                    TM1650_DisplayTime();
                }
            }
            else
            { // 如果没有在调节时间
                send_buf[1] = 0x61;
                flag_is_send_data = 1;
            }

            break;

        case KEY_RIGHT_LONG:
            if (STATUS_NONE == cur_set_time_status)
            {
                send_buf[1] = 0x62;
                flag_is_send_data = 1;
            }
            break;

        case KEY_RIGHT_HOLD:
            if (STATUS_NONE == cur_set_time_status)
            { // 如果没有在调节时间
                send_buf[1] = 0x63;
                flag_is_send_data = 1;
            }
            break;

        case KEY_DOWN_CLICK:
            send_buf[1] = 0x11;
            flag_is_send_data = 1;
            break;

        case KEY_DOWN_LONG:
            send_buf[1] = 0x12;
            flag_is_send_data = 1;
            break;

        case KEY_DOWN_HOLD:
            send_buf[1] = 0x13;
            flag_is_send_data = 1;
            break;

        case KEY_LEFT_CLICK:

            if (STATUS_NONE != cur_set_time_status)
            { // 如果在调节时间
                set_time_done_cnt = 0;
                set_time_delay_cnt = 0;
                if (STATUS_SET_TIME_HOUR == cur_set_time_status)
                {
                    if (thousand > 0) // 10H及以上
                    {
                        if (hundred > 0)
                        {
                            hundred--;
                        }
                        else // hundred == 0
                        {
                            thousand--;
                            hundred = 9;
                        }
                    }
                    else // 0~9H
                    {
                        if (hundred > 0)
                        {
                            hundred--;
                        }
                    }

                    TM1650_DisplayTime();
                }
                else if (STATUS_SET_TIME_MIN == cur_set_time_status)
                {
                    if (decade > 0) // 10min及以上
                    {
                        if (unit > 0)
                        {
                            unit--;
                        }
                        else
                        {
                            decade--;
                            unit = 9;
                        }
                    }
                    else
                    {
                        if (unit > 0)
                        {
                            unit--;
                        }
                    }

                    TM1650_DisplayTime();
                }
            }
            else
            { // 如果没有在调节时间
                send_buf[1] = 0x51;
                flag_is_send_data = 1;
            }

            break;

        case KEY_LEFT_LONG:
            if (STATUS_NONE == cur_set_time_status)
            { // 如果没有在调节时间
                send_buf[1] = 0x52;
                flag_is_send_data = 1;
            }
            break;

        case KEY_LEFT_HOLD:
            if (STATUS_NONE == cur_set_time_status)
            { // 如果没有在调节时间
                send_buf[1] = 0x53;
                flag_is_send_data = 1;
            }
            break;

        default:
            break;
        }

        ad_key_event = KEY_EVENT_NONE; // 处理完事件后，清除
    }
}

#endif

// void bsp_i2c_start(void)
// {

// }
void bsp_i2c_tx_byte(uint8_t dat)
{
    // u8 i;
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
// void bsp_i2c_stop(void)
// {
//     I2C_SDA_OUT();
//     I2C_SDA_L();
//     delay_us(5);
//     I2C_SCL_H();
//     delay_us(5);
//     I2C_SDA_H();
// }

// IIC写一个字节
void TM1650_WriteByte(uint8_t addr, uint8_t data)
{
    cur_dev_use = DEV_USE_TM1650;
    // bsp_i2c_start();
    I2C_SDA_OUT();
    I2C_SCL_OUT();
    I2C_SDA_H();
    I2C_SCL_H();
    delay_us(5);
    I2C_SDA_L();
    delay_us(5);
    I2C_SCL_L();

    bsp_i2c_tx_byte(addr);
    bsp_i2c_rx_ack();
    bsp_i2c_tx_byte(data);
    bsp_i2c_rx_ack();
    // bsp_i2c_stop();
    I2C_SDA_OUT();
    I2C_SDA_L();
    delay_us(5);
    I2C_SCL_H();
    delay_us(5);
    I2C_SDA_H();
    cur_dev_use = DEV_USE_NONE;
}

// 初始化TM1650
void TM1650_Init(void)
{
    // bsp_i2c_init();
    I2C_SDA_OUT();
    I2C_SCL_OUT();
    I2C_SDA_H();
    delay_ms(5);

    // 设置数据命令
    // TM1650_WriteByte(0x48, 0x01); // 开启显示

    // 开启显示、设置亮度(最大亮度)
    TM1650_WriteByte(0x48, TM1650_DISPLAY_ON | TM1650_BRIGHT_MAX);

    // 清空显示
    // TM1650_Clear();
}

// 显示数字(0-9999)
void TM1650_DisplayTime(void)
{
    // 从右往左数，第1位
    TM1650_WriteByte(0x68, NUM_TABLE[unit]);           // 第1位
    TM1650_WriteByte(0x6A, NUM_TABLE[decade]);         // 第2位
    TM1650_WriteByte(0x6C, NUM_TABLE[hundred] | 0x04); // 第3位
    TM1650_WriteByte(0x6E, NUM_TABLE[thousand]);       // 第4位
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

    ad_key_event = KEY_EVENT_NONE;
    cur_key_id = AD_KEY_ID_NONE;
    cur_w_pwm_duty = 255;
    cur_y_pwm_duty = 255;

    send_buf[0] = 0xA5;

    cur_set_time_status = STATUS_SET_TIME_HOUR; // 调试用

    while (1)
    {
        adc_channel_sel(ADC_CHANNEL_KEY);
        key_ad_key_event_deal();

        // { // 低电量检测：
        //     static u8 power_low_3_2_v_cnt = 0;
        //     static u8 power_low_3_0_v_cnt = 0;
        //     static u8 normal_power_cnt = 0; // 电压正常的计数
        //     adc_channel_sel(ADC_CHANNEL_VDD);
        //     adc_val = adc_get_val();
        //     if (adc_val <= POWER_LOW_3_0_V)
        //     { // 如果检测到电量低于3.0V
        //         power_low_3_0_v_cnt++;
        //         power_low_3_2_v_cnt = 0;
        //         normal_power_cnt = 0;
        //     }
        //     else if (adc_val <= POWER_LOW_3_2_V)
        //     { // 如果检测到低电量
        //         power_low_3_2_v_cnt++;
        //         power_low_3_0_v_cnt = 0;
        //         normal_power_cnt = 0;
        //     }
        //     else if (adc_val >= POWER_LOW_3_2_V + 50)
        //     { // 如果检测到电压恢复
        //         normal_power_cnt++;
        //         power_low_3_0_v_cnt = 0;
        //         power_low_3_2_v_cnt = 0;
        //     }

        //     if (power_low_3_0_v_cnt >= 100)
        //     {
        //         power_low_3_0_v_cnt = 0;
        //         // 如果连续多次检测到低电量
        //         // 发送低电量信息
        //         battery_status = BATTERY_POWER_LOW_3_0_V;
        //     }
        //     else if (power_low_3_2_v_cnt >= 100)
        //     {
        //         power_low_3_2_v_cnt = 0;
        //         battery_status = BATTERY_POWER_LOW_3_2_V;
        //     }
        //     else if (normal_power_cnt >= 100)
        //     {
        //         normal_power_cnt = 0;
        //         battery_status = BATTERY_NORMAL;
        //     }

        //     if (BATTERY_POWER_LOW_3_0_V == battery_status)
        //     {                     // 电池电量低于3.0V  关闭蓝牙
        //         BLE_CTL_PIN_IN(); // 输入模式
        //         flag_is_ble_open = 0;
        //     }
        //     else if (BATTERY_POWER_LOW_3_2_V == battery_status)
        //     {
        //     }
        //     else
        //     {
        //     }

        // } // 低电量检测

        if (STATUS_SET_TIME_HOUR == cur_set_time_status)
        { // 设置小时
            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayTime();
            }
            else
            {
                TM1650_WriteByte(0x6E, 0x00);        // 第4位
                TM1650_WriteByte(0x6C, 0x00 | 0x04); // 第3位

                if (set_time_delay_cnt >= 1000 - 1)
                {
                    set_time_delay_cnt = 0;
                }
            }

            if (set_time_done_cnt >= 5000)
            {
                cur_set_time_status = STATUS_SET_TIME_MIN;
                set_time_done_cnt = 0;
            }
        }
        else if (STATUS_SET_TIME_MIN == cur_set_time_status)
        { // 设置分钟
            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayTime();
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
                cur_set_time_status = STATUS_NONE;
                set_time_done_cnt = 0;
            }
        }
        else // STATUS_NONE == cur_set_time_status
        {
            TM1650_DisplayTime(); // 显示当前时间
        }

        if (flag_is_send_data)
        {
            send_buf[2] = ~send_buf[1];
            start_send();
            flag_is_send_data = 0;
        }
        else if (flag_recv_time_ack)
        { // 如果收到了时间，进行应答
            send_buf[1] = 0x83;
            send_buf[2] = 0x7C;
            start_send();
            flag_recv_time_ack = 0;
        }

        // 低功耗：
        // 灯光关闭 且 每隔一段时间 就进入低功耗

//         if (flag_is_enter_low_power &&
//             CUR_LIGHT_STATUS_OFF == cur_light_status &&
//             IDLE == g_state &&
//             DEV_USE_NONE == cur_dev_use)
//         // if (flag_is_enter_low_power)
//         // if (CUR_LIGHT_STATUS_OFF == cur_set_time_status &&
//         //     IDLE == g_state &&
//         //     DEV_USE_NONE == cur_dev_use)
//         {
//             DEBUG_PIN = 1;
//             flag_is_enter_low_power = 0;
//             GIE = 0;  // 屏蔽总中断
//             ADEN = 0; // 关闭ad
//             T3EN = 0; // 关闭定时器

// #if USE_MY_DEBUG
//             P20KE = 1; // 使能IO的键盘中断
// #else
//             P16KE = 1; // 使能IO的键盘中断
// #endif

//             // 休眠前关闭外设 AD等 使能唤醒条件中断
//             KBIE = 1; // 使能键盘中断
//             HFEN = 0; // 关闭高速时钟
//             LFEN = 1;
//             Nop();
//             Nop();
//             Stop();
//             Nop();
//             Nop();
//             HFEN = 1; // 使能高速时钟
//             // LFEN = 1;
//             KBIE = 0; // 关闭键盘中断
//             ADEN = 1; // 使能adc
//             T3EN = 1; // 使能定时器
//             GIE = 1;  // 使能总中断

//             // Sys_Init();
//             DEBUG_PIN = 0;
//         }

        __asm;
        clrwdt; // 喂狗
        __endasm;

    } // while(1)
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

    // 使用32768Hz晶振作为时钟源的定时器:(目前测试统计时间没有问题)
    if (T1IF & T1IE)
    {
        static volatile u16 timer1_cnt = 0;
        timer1_cnt++;
        if (timer1_cnt >= 1024)
        {
            /*
                使用外部32768Hz晶振时,32分频后, 每 0.9765625 ms 计数一次,
                T1LOAD = 10 - 1时,每 9.765625 ms 触发一次中断
                理论上触发1024次中断时,刚好计满 10 000 ms, 10s
            */
            timer1_cnt = 0;

            keep_time_cnt++;
            if (keep_time_cnt >= 6)
            {                      // 如果计满60s
                keep_time_cnt = 0; // 清除计时值

                unit++;
                if (unit >= 10)
                {
                    unit = 0;
                    decade++;
                    if (decade >= 6)
                    {
                        decade = 0;
                        hundred++;
                        if (thousand < 2)
                        {
                            if (hundred >= 10)
                            {
                                hundred = 0;
                                thousand++;
                            }
                        }
                        else
                        {
                            if (hundred >= 4)
                            {
                                thousand = 0;
                                hundred = 0;
                                decade = 0;
                                unit = 0;
                            }
                        }
                    }
                }
            }
        }
        T1IF = 0;
    }

    if (T3IF & T3IE)
    {
        // 目前是100us触发一次

        { // ad按键扫描时间计数
            static volatile u8 ad_key_scan_time_cnt = 0;
            ad_key_scan_time_cnt++;
            if (ad_key_scan_time_cnt >= 100) // 100us * 100 == 10ms
            {
                flag_10ms = 1; // 用于ad按键扫描
                ad_key_scan_time_cnt = 0;
            }
        }

        { // 调节时间时，实现LED闪烁效果的计数
            static volatile u8 __set_time_cnt = 0;
            __set_time_cnt++;

            if (__set_time_cnt >= 10) // 1ms进入一次
            {
                __set_time_cnt = 0;
                if (STATUS_SET_TIME_HOUR == cur_set_time_status ||
                    STATUS_SET_TIME_MIN == cur_set_time_status)
                {
                    set_time_delay_cnt++;
                    set_time_done_cnt++;
                }
                else
                {
                    set_time_delay_cnt = 0;
                    set_time_done_cnt = 0;
                }
            }
        }

        // { // 低电量处理：
        //     static volatile u8 power_low_cnt = 0;

        //     if (BATTERY_POWER_LOW_3_2_V == battery_status)
        //     {
        //         power_low_cnt++;
        //         if (power_low_cnt >= 150) // 低电量15s后，发送一次低电压报警
        //         {
        //             power_low_cnt = 0;
        //             // send_buf[1] = 0x81;
        //             // flag_is_send_data = 1;
        //         }
        //     }
        //     else
        //     { // 不是低电量报警的状态(3.0V~3.2V)，清空计数值
        //         power_low_cnt = 0;
        //     }
        // }

        { // 低功耗：
            // if (CUR_LIGHT_STATUS_OFF == cur_set_time_status &&
            //     IDLE == g_state &&
            //     DEV_USE_NONE == cur_dev_use)
            {
                low_power_time_cnt++;
                if (low_power_time_cnt >= ENTER_LOW_POWER_PERIOD_TIME)
                {
                    flag_is_enter_low_power = 1;
                    low_power_time_cnt = 0;
                }
            }
            // else
            {
                // low_power_time_cnt = 0;
            }
        }

        // 单线数据发送&接收处理：
        timer_100us_isr();

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
