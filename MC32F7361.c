/******************************************************************************
;  *       @�ͺ�                 : MC32F7361
;  *       @��������             : 2021.12.21
;  *       @��˾/����            : SINOMCU-FAE
;  *       @����΢����֧��       : 2048615934
;  *       @����΢����           : http://www.sinomcu.com/
;  *       @��Ȩ                 : 2021 SINOMCU��˾��Ȩ����.
;  *----------------------ժҪ����---------------------------------
;  *                  ADC
;  *                  �ϵ�ȴ�AD�ȶ������У׼
;  *                  ADCʱ��ΪHIRC32��Ƶ1M��12λ����H8L4���ڲ�2V��15ADCLK
;  *                  ��ʱ��2MS��P10 �ɼ���ѹֵ
;  *                  �ɼ���ѹֵ<1.5V:P11Ϊ�͵�ƽ;>1.5V:P11Ϊ�ߵ�ƽ
******************************************************************************/

#include "user.h"

// ���뼶��ʱ (����1%���ڣ�1ms��10ms��100ms��ʱ������С��1%)
// ǰ��������FCPU = FHOSC / 4
void delay_ms(u16 xms)
{
    while (xms)
    {
        u16 i = 572;
        while (i--)
        {
            Nop();
        }
        xms--; // �� --��������while()�ж��������棬����ʡ�ռ�
    }
}

// ��׼ȷ��΢�뼶��ʱ��ֻ��Ϊ�˸�IIC�����ṩus��ʱ
void delay_us(u8 xus)
{
    while (xus)
    {
        Nop();
        xus--; // �� --��������while()�ж��������棬����ʡ�ռ�
    }
}

/************************************************
;  *    @������          : CLR_RAM
;  *    @˵��            : ��RAM
;  *    @�������        :
;  *    @���ز���        :
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
;  *    @������            : IO_Init
;  *    @˵��              : IO��ʼ��
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
void IO_Init(void)
{
    IOP0 = 0x00;   // io������λ
    OEP0 = 0x3F;   // io�ڷ��� 1:out  0:in
    PUP0 = 0x00;   // io����������   1:enable  0:disable
    PDP0 = 0x00;   // io����������   1:enable  0:disable
    P0ADCR = 0x00; // io����ѡ��  1:ģ������  0:ͨ��io

    IOP1 = 0x00;   // io������λ
    OEP1 = 0xFF;   // io�ڷ��� 1:out  0:in
    PUP1 = 0x00;   // io����������   1:enable  0:disable
    PDP1 = 0x00;   // io����������   1:enable  0:disable
    P1ADCR = 0x00; // io����ѡ��  1:ģ������  0:ͨ��io

    IOP2 = 0x00; // io������λ
    OEP2 = 0x0F; // io�ڷ��� 1:out  0:in
    PUP2 = 0x00; // io����������   1:enable  0:disable
    PDP2 = 0x00; // io����������   1:enable  0:disablea

    PMOD = 0x00;  // P00��P01��P13 io�˿�ֵ�ӼĴ��������������
    DRVCR = 0x80; // ��ͨ����
}

void adc_config(void)
{
    // P12,AD�����������
    P12OE = 0;    // ����ģʽ
    P12DC = 1;    // ģ�⹦��
    ADCR0 = 0xFB; // ʹ��ADC��12λ����H8L4
    ADCR1 = 0x80; // adcʱ��32��Ƶ���ڲ�2V�ο���ѹ
    ADCR2 = 0xFF; // �̶�Ϊ15ADCLK

    ADEN = 1;
}

// void timer0_config(void)
// {
// T0CR = DEF_SET_BIT0 | DEF_SET_BIT2; // ��ʱģʽ,CPU,32��Ƶ
// T0CNT = 250 - 1;                    // 1ms
// T0LOAD = 250 - 1;
// T0EN = 1;
// T0IE = 1;
// }

// ��ʱ�����ã�ʹ���ⲿ������Ϊʱ��Դ
void timer1_config(void)
{
    // T1CR = DEF_SET_BIT0 | DEF_SET_BIT2 | DEF_SET_BIT3 | DEF_SET_BIT4; // ��ʱģʽ,FLOSC,32��Ƶ
    T1CR = 0x15; // ʱ��Դѡ��FLOSC��32��Ƶ��32Khz(32768) / 32 ԼΪ 1Khz��ʵ����ÿ��0.9765625ms����һ��
    // T1CNT = 2 - 1; //
    T1LOAD = 10 - 1; //
    T1EN = 1;
    T1IE = 1;
}

//
void timer3_config(void)
{
    T3CR = 0x03; // ��ʱģʽ,CPU,8��Ƶ
    // T3CNT = 100 - 1;
    T3LOAD = 100 - 1; // 8��Ƶ��������100us����һ���ж�
    T3EN = 1;
    T3IE = 1;
}

// ����PWM��Ĭ��ռ�ձ�Ϊ 0%
void pwm_led_config(void)
{
    // PWM0O1 PWM2
    T0CR = DEF_SET_BIT6 | DEF_SET_BIT0 | DEF_SET_BIT1; // ʹ��PWM,CPU,8��Ƶ
    T0CNT = 255 - 1;
    T0LOAD = 255 - 1; //
    // T0DATA = 0; // ��ʼֵ����0

    T2CR = DEF_SET_BIT6 | DEF_SET_BIT0 | DEF_SET_BIT1; // ʹ��PWM,CPU,8��Ƶ
    T2CNT = 255 - 1;
    T2LOAD = 255 - 1; //
    // T2DATA = 0;  // ��ʼֵ����0

    PWMCR1 = 0x00; // pwm��������ͨģʽ
    PWMCR3 = 0x04; // PWM0ѡ��P02�˿����
    T0EN = 1;
    T2EN = 1;
}

// ��������
void start_send(uint8_t *data)
{
    if (g_state != IDLE)
        return; // ��������շ��򷵻�

    // ����Ҫ���͵�����
    for (i = 0; i < SEND_DATA_LEN; i++)
    {
        g_data[i] = data[i];
    }
    g_byte_count = 0;
    g_bit_count = 0;
    g_timer_count = 0;
    g_state = TX_START;

    set_pin_low(); // ��ʼ������ʼ�ź�
}

// 100us��ʱ���жϷ�����
void timer_100us_isr(void)
{
    // ���ʹ���
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
                // ����1: �ߵ�ƽ1ms
                if (++g_timer_count >= BIT1_HIGH_TIME)
                {
                    g_timer_count = 0;
                    g_state = TX_SENDING_LOW;
                    set_pin_low();
                }
            }
            else
            {
                // ����0: �ߵ�ƽ0.5ms
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
                // ����1: �͵�ƽ0.5ms
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
                // ����0: �͵�ƽ1ms
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
        // P13PU = 1; // ����
        // P13OE = 1; // ����ģʽ
        set_pin_high();
    }

    // ���մ���
    pin_state = get_pin_state();

    switch (g_state)
    {
    case IDLE:
        if (pin_state == 0)
        { // ��⵽��ʼ�ź�
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
        { // ��ʼ�źŽ���
            // 8ms~15ms����ʼ�ź�
            if (g_timer_count >= RX_START_MIN_TIME && g_timer_count <= RX_START_MAX_TIME)
            {
                g_timer_count = 0;
                g_state = RX_DATA;
            }
            else
            { // ��Ч�ź�
                g_state = IDLE;
            }
        }
        break;

    case RX_DATA:
        g_timer_count++;
        if (pin_state != g_last_pin_state)
        { // ��⵽��ƽ�仯
            if (g_last_pin_state == 1)
            { // �ߵ�ƽ����
                // �ж�����1: �ߵ�ƽ0.8ms~1.2ms (8~12������)
                // �ж�����0: �ߵ�ƽ0.3ms~0.7ms (3~7������)
                if (g_timer_count >= RX_BIT1_MIN_TIME && g_timer_count <= RX_BIT1_MAX_TIME)
                { // ���յ�1
                    g_data[g_byte_count] |= BIT(g_bit_count);
                }
                // �����3~7��������Ϊ0������Ҫ��������λ
                g_bit_count++;
                if (g_bit_count >= 8)
                {
                    g_bit_count = 0;
                    g_byte_count++;
                    if (g_byte_count >= RX_DATA_LEN)
                    {
                        g_state = RX_END; // �������
                    }
                }
            }
            g_timer_count = 0;
        }
        break;

    case RX_END:
        // ��������Դ�����յ�������g_data

        // send_data_msb(send_buf[0] << 8 | send_buf[]);

        g_state = IDLE;
        break;
    }
    g_last_pin_state = pin_state;
}

// // ʹ��ʾ��
// void example(void)
// {
//     uint8_t send_buf[SEND_DATA_LEN] = {0x01, 0x02, 0x03};
//     // ����������
//     start_send(send_buf);

//     // ���յ������ݿ�����RX_END״̬�´���
// }

void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();

    adc_config();
    timer1_config();
    TM1650_Init();
    pwm_led_config();
    timer3_config();

    // ��������ţ�ֻ����ƽ
    P03PD = 1; // ����
    P03OE = 1; // �����ţ�����ģʽ

    // �����������ţ�����Ϊ����ģʽ
    BLE_CTL_PIN_IN();

    // ����ͨ�ŵ����ݽ������ţ�
    P16PU = 1; // ����
    P16OE = 0; // ����ģʽ

    ad_key_event = KEY_EVENT_NONE;
    cur_key_id = AD_KEY_ID_NONE;
    cur_w_pwm_duty = 255;
    cur_y_pwm_duty = 255;

    GIE = 1;
}

#if 1

void adc_channel_sel(u8 adc_channel)
{
    switch (adc_channel)
    {
    case ADC_CHANNEL_KEY:
        ADCR0 = 0x7F; // AN7ͨ����ʹ��ADC
        ADCR1 = 0x82; // adcʱ�� == FHIRC/32��ʹ���ڲ�4V�ο���ѹ
        break;

    case ADC_CHANNEL_VDD:
        ADCR0 = 0xAF; // 1/VDDͨ����ʹ��ADC
        ADCR1 = 0x80; // adcʱ�� == FHIRC/32��ʹ���ڲ�2V�ο���ѹ

    default:
        break;
    }

    delay_ms(1);
}

u16 adc_single_convert(void)
{
    ADEOC = 0;
    while (!ADEOC)
        ; // �ȴ�ת�����
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
        adc_val_tmp = adc_single_convert(); // ���ﲹ����adc����ת����õ�����
        if (i < 2)
            continue; // ����ǰ���β�����
        if (adc_val_tmp > get_adcmax)
            get_adcmax = adc_val_tmp; // ���µ�ǰ�ɼ��������ֵ
        if (adc_val_tmp < get_adcmin)
            get_adcmin = adc_val_tmp; // ���µ�ǰ�ɼ�������Сֵ
        adc_val_sum += adc_val_tmp;
    }

    adc_val_sum -= get_adcmax;        // ȥ��һ�����
    adc_val_sum -= get_adcmin;        // ȥ��һ����С
    adc_val_tmp = (adc_val_sum >> 4); // ����16��ȡƽ��ֵ

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
    static volatile u8 last_key_id = AD_KEY_ID_NONE;
    static volatile u8 count = 0; // ��������
    // static volatile u8 filter_cnt = 0;                 // ����������ʹ�õı���
    // static volatile u8 filter_key_id = AD_KEY_ID_NONE; // ����ʱ��¼��һ�βɼ����İ���id
    // �а�������ʱ���� AD_KEY_ID ���ް������� KEY_ID_NONE
    // �ҵ� id_table[] �ж�Ӧ�İ���:
    cur_key_id = get_key_id();

    // if (cur_key_id != filter_key_id && cur_key_id != AD_KEY_ID_NONE)
    // if (cur_key_id != filter_key_id && filter_cnt)
    // if (cur_key_id != filter_key_id)
    // { // ����а�������
    //     filter_cnt = 0;
    //     filter_key_id = cur_key_id;
    //     return;
    // }
    // else
    // {
    //     if (filter_cnt < 2)
    //     { // �����⵽��ͬ�İ�������/�ɿ�
    //         // ��ֹ�������
    //         filter_cnt++;
    //         return;
    //     }
    // }

    // if (filter_cnt < 2)
    // { // �����⵽��ͬ�İ�������/�ɿ�
    //     // ��ֹ�������
    //     filter_cnt++;
    //     // ad_key_event = KEY_EVENT_NONE; // ������䣬���޷�ʶ��̰�
    //     return;
    // }

    // if (cur_key_id < AD_KEY_ID_NONE)
    //     send_data_msb(cur_key_id); // ���Է���������Եõ���Ӧ�İ���id

    if (last_key_id != cur_key_id) // ����а��� / ̧����
    {
        if (last_key_id == AD_KEY_ID_NONE)
        { // ����
            count = 0;
        }
        else if (cur_key_id == AD_KEY_ID_NONE)
        { // ̧��
            if (count < 75)
            {
                // �̰�  -->  ad_key_event
                ad_key_event = KEY_EVENT_CLICK;
            }
            else
            {
                // ���� / ��������������
                // if (AD_KEY_ID_MID == last_key_id && KEY_EVENT_HOLD == ad_key_event) // ����ʹ������ж�����
                if (AD_KEY_ID_MID == last_key_id)
                {
                    // ����ǵ��ڵƹ����ȵİ���������������
                    // DEBUG_PIN = ~DEBUG_PIN;
                    // adjust_pwm_dir = !adjust_pwm_dir; // �ı�ƹ���ڵķ��� �����������浽������ռ�ó���ռ䣩
                    if (adjust_pwm_dir)
                    {
                        adjust_pwm_dir = 0;
                    }
                    else
                    {
                        adjust_pwm_dir = 1;
                    }
                } // if (AD_KEY_ID_MID == last_key_id)
            }
        }
        else
        {
            // ���°���ʱ��;�ְ��������������������������
        }
    }
    else if (cur_key_id != AD_KEY_ID_NONE)
    { // ��������
        count++;
        if (count == 75)
        { // ����  -->  ad_key_event
            ad_key_event = KEY_EVENT_LONG;
        }
        else if (count >= 75 + 15)
        { // ��������  -->  ad_key_event
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
    // 10ms����һ�Σ��õ� cur_key_id �� ad_key_event ��Ȼ����ղ��
    ad_key_event_10ms_isr();

    // �����⵽���������﷢�͸�����IC
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

    if (cur_key_id < AD_KEY_ID_NONE && ad_key_event != KEY_EVENT_NONE)
    {
        // send_data_msb(cur_key_id); // �ܼ�⵽����id
        // send_data_msb(ad_key_event);

        switch (key_event_table[cur_key_id][ad_key_event])
        {
            // case KYE_UP_CLICK: //
            //     break;
        case KEY_UP_LONG: // ����ʱ����/������

            if (flag_is_ble_open)
            { // �ر�����
                BLE_CTL_PIN_IN();
                flag_is_ble_open = 0;
            }
            else
            { // ������
                BLE_CTL_PIN_OUT();
                BLE_CTL_PIN_LOW(); // ����͵�ƽ
                flag_is_ble_open = 1;
            }

            break;

        case KEY_UP_HOLD:
            break;

        case KEY_MID_CLICK:
            // �Ƶ�ģʽ�л��������̰�
            // DEBUG_PIN = ~DEBUG_PIN;

            switch (cur_light_status)
            {
            case CUR_LIGHT_STATUS_OFF:
                // ����
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
            // switch (cur_light_status)
            // {
            // case CUR_LIGHT_STATUS_OFF:
            //     // �޲���
            //     break;
            // case CUR_LIGHT_STATUS_WHITE:

            // DEBUG_PIN = ~DEBUG_PIN;

            if (adjust_pwm_dir)
            { // ��������
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
            { // ��С����
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
            // }
            break;

        case KEY_SEL_HOLD:
            // P22D = ~P22D;
            // �ս�������Ѿ�����750 + 150ms
            set_time_hold_cnt++;
            if (set_time_hold_cnt >= 14) // 14 * 150ms == 2100ms
            {
                set_time_hold_cnt = 0;
                cur_status = STATUS_SET_TIME_HOUR;
            }

            break;

        case KEY_RIGHT_CLICK:

            set_time_done_cnt = 0;
            set_time_delay_cnt = 0;
            if (STATUS_SET_TIME_HOUR == cur_status)
            {
                // if (cur_time < 2300)
                // {
                //     cur_time += 100;
                // }

                // if (hour < 23)
                // {
                //     hour++;
                // }

                // TM1650_DisplayNum(cur_time);
                TM1650_DisplayNum();
            }
            else if (STATUS_SET_TIME_MIN == cur_status)
            {
                // if ((cur_time % 100) < 59)
                // {
                //     cur_time++;
                // }

                // if (min < 59)
                // {
                //     min++;
                // }

                TM1650_DisplayNum();
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
            set_time_delay_cnt = 0;
            if (STATUS_SET_TIME_HOUR == cur_status)
            {
                // if (cur_time > 0)
                // {
                //     cur_time -= 100;
                // }

                // if (hour > 0)
                // {
                //     hour--;
                // }

                if (thousand < 2 && hundred < 3)
                {
                    hundred++;
                    if (hundred > 2)
                    {
                        hundred = 0;
                        thousand++;
                    }
                }

                TM1650_DisplayNum();
            }
            else if (STATUS_SET_TIME_MIN == cur_status)
            {
                // if ((cur_time % 100) > 0)
                // {
                //     cur_time--;
                // }

                // if (min > 0)
                // {
                //     min--;
                // }

                TM1650_DisplayNum();
            }

            break;

        case KEY_LEFT_HOLD:

            break;

        default:
            break;
        }

        ad_key_event = KEY_EVENT_NONE; // �������¼������
    }
}

#endif

// void bsp_i2c_init(void)
// {
//     I2C_SDA_OUT();
//     I2C_SCL_OUT();
//     I2C_SDA_H();
//     delay_ms(5);
// }
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

// IICдһ���ֽ�
void TM1650_WriteByte(uint8_t addr, uint8_t data)
{
    bsp_i2c_start();
    bsp_i2c_tx_byte(addr);
    bsp_i2c_rx_ack();
    bsp_i2c_tx_byte(data);
    bsp_i2c_rx_ack();
    bsp_i2c_stop();
}

// �����ʾ
// void TM1650_Clear(void)
// {
//     // ���4�������
//     TM1650_WriteByte(0x68, 0x00); // ��1λ
//     TM1650_WriteByte(0x6A, 0x00); // ��2λ
//     TM1650_WriteByte(0x6C, 0x00); // ��3λ
//     TM1650_WriteByte(0x6E, 0x00); // ��4λ
// }

// ��ʾð��
// void TM1650_show_colon(void)
// {
//     TM1650_WriteByte(0x6E, )
// }

// ��ʼ��TM1650
void TM1650_Init(void)
{
    // bsp_i2c_init();
    I2C_SDA_OUT();
    I2C_SCL_OUT();
    I2C_SDA_H();
    delay_ms(5);

    // ������������
    // TM1650_WriteByte(0x48, 0x01); // ������ʾ

    // ��������(�������)
    TM1650_WriteByte(0x48, TM1650_DISPLAY_ON | TM1650_BRIGHT_MAX);

    // �����ʾ
    // TM1650_Clear();
}

// ��ʾʱ���еĵ������� 0~9��
// void TM1650_DisplayBit(uint8_t pos, uint8_t num)
// {
//     uint8_t addr;

//     // if (pos > 4 || num > 9)
//     //     return; // �������

//     // ����λ�ö�Ӧ�ĵ�ַ
//     addr = 0x68 + (pos - 1) * 2;

//     if (addr == 0x6C)
//     {
//         TM1650_WriteByte(addr, NUM_TABLE[num] | 0x04);
//     }
//     else
//     {
//         // д�����
//         TM1650_WriteByte(addr, NUM_TABLE[num]);
//     }
// }

// ��ʾ����(0-9999)
void TM1650_DisplayNum(void)
{

    // if (num > 9999)
    //     return; // �������

    // �������λ
    // thousand = num / 1000;
    // hundred = (num % 1000) / 100;
    // decade = (num % 100) / 10;
    // unit = num % 10;

    // ��ʾ����λ
    // TM1650_DisplayBit(4, thousand);
    // TM1650_DisplayBit(3, hundred);
    // TM1650_DisplayBit(2, decade);
    // TM1650_DisplayBit(1, unit);
    // TM1650_WriteByte(0x68, NUM_TABLE[hour / 10]);        // ��1λ
    // TM1650_WriteByte(0x6A, NUM_TABLE[hour - hour / 10]); // ��2λ
    // TM1650_WriteByte(0x6C, NUM_TABLE[min / 10] | 0x04);  // ��3λ
    // TM1650_WriteByte(0x6E, NUM_TABLE[min - min / 10]);   // ��4λ
    TM1650_WriteByte(0x68, NUM_TABLE[unit]);           // ��1λ
    TM1650_WriteByte(0x6A, NUM_TABLE[decade]);         // ��2λ
    TM1650_WriteByte(0x6C, NUM_TABLE[hundred] | 0x04); // ��3λ
    TM1650_WriteByte(0x6E, NUM_TABLE[thousand]);       // ��4λ
}

/************************************************
;  *    @������            : main
;  *    @˵��              : ������
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
void main(void)
{
    Sys_Init();

    while (1)
    {
        adc_channel_sel(ADC_CHANNEL_KEY);
        key_ad_key_event_deal();

        { // �͵�����⣺
            static u8 low_power_cnt = 0;
            adc_channel_sel(ADC_CHANNEL_VDD);
            adc_val = adc_get_val();
            if (adc_val <= 2048)
            { // �����⵽�͵���
                low_power_cnt++;
            }
            else
            { // �����һ�μ�ⲻ���͵���
                low_power_cnt = 0;
            }

            if (low_power_cnt >= 10)
            {
                low_power_cnt = 0;

                // ���������μ�⵽�͵���
                // ���͵͵�����Ϣ
            }
        } // �͵������

        if (STATUS_SET_TIME_HOUR == cur_status)
        { // ����Сʱ
            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayNum();
            }
            else
            {
                TM1650_WriteByte(0x6E, 0x00);        // ��4λ
                TM1650_WriteByte(0x6C, 0x00 | 0x04); // ��3λ

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
        { // ���÷���
            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayNum();
            }
            else
            {
                TM1650_WriteByte(0x6A, 0x00); // ��2λ
                TM1650_WriteByte(0x68, 0x00); // ��1λ

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
        else // STATUS_NONE == cur_status
        {
            TM1650_DisplayNum(); // ��ʾ��ǰʱ��
            // TM1650_WriteByte(0x6C, 0x04);
        }

        send_buf[0] = 0xA5;
        send_buf[1] = 0x81;
        send_buf[2] = 0x7E;
        start_send(send_buf);
        delay_ms(100);

    } // while(1)
}
/************************************************
;  *    @������            : interrupt
;  *    @˵��              : �жϺ���
;  *    @�������          :
;  *    @���ز���          :
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
        static volatile u16 timer1_cnt = 0;
        timer1_cnt++;
        if (timer1_cnt >= 1024)
        {
            // P12D = ~P12D;
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
        // Ŀǰ��100us����һ��

        static volatile u8 timer3_cnt = 0;
        timer3_cnt++;

        if (timer3_cnt >= 100) // 100us * 100 == 10ms
        {
            flag_10ms = 1; // ����ad����ɨ��
            timer3_cnt = 0;
        }

        { //
            static u8 __set_time_cnt = 0;
            __set_time_cnt++;

            if (__set_time_cnt >= 10) // 1ms����һ��
            {
                __set_time_cnt = 0;
                if (STATUS_SET_TIME_HOUR == cur_status ||
                    STATUS_SET_TIME_MIN == cur_status)
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
