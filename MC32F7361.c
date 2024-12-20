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

        __asm;
        clrwdt; // ι��
        __endasm;
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
    // �л�ͨ��ʱ������ADCR1
    // ADCR1 = 0x80; // adcʱ��32��Ƶ���ڲ�2V�ο���ѹ
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
    // T1CR = 0x00;     // �����õ�Ƶ��
    T1LOAD = 10 - 1; //
    // T1LOAD = 50 - 1; // �����õ���װ��ֵ
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
// void start_send(uint8_t *data)
void start_send(void)
{
    if (g_state != IDLE)
        return; // ��������շ��򷵻�

    // ����Ҫ���͵�����
    for (i = 0; i < SEND_DATA_LEN; i++)
    {
        // g_data[i] = data[i];
        g_data[i] = send_buf[i];
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

        case TX_SENDING_BIT: // ���͸ߵ�ƽ����
            if (g_data[g_byte_count] & SEND_SEND_BIT(g_bit_count))
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

        case TX_SENDING_LOW: // ���͵͵�ƽ����
            if (g_data[g_byte_count] & SEND_SEND_BIT(g_bit_count))
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
                    g_data[g_byte_count] |= RECV_RECV_BIT(g_bit_count);
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
        // ��������Դ�����յ������� g_data

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

// // ʹ��ʾ��
// void example(void)
// {
//     uint8_t send_buf[SEND_DATA_LEN] = {0x01, 0x02, 0x03};
//     // ����������
//     start_send(send_buf);

//     // ���յ������ݿ����� RX_END ״̬�´���
// }

void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();

    adc_config();
    timer1_config(); // ʹ���ⲿ������Ϊʱ��Դ�Ķ�ʱ��
    TM1650_Init();   // LED����IC����
    pwm_led_config();
    timer3_config();

    // ��������ţ�ֻ����ƽ
    P03PD = 1; // ����
    P03OE = 1; // �����ţ�����ģʽ

    // �����������ţ�����Ϊ����ģʽ
    BLE_CTL_PIN_IN();

    // ����ͨ�ŵ����ݽ������ţ�
#if USE_MY_DEBUG
    P20PU = 1; // �����õ����ݽ��ս�
    P20OE = 0;
#else
    P16PU = 1; // ����
    P16OE = 0; // ����ģʽ
#endif

    delay_ms(1); // �ȴ�ϵͳ�ȶ����ر��Ƕ�ʱ��

    GIE = 1;
}

#if 1

void adc_channel_sel(u8 adc_channel)
{
    switch (adc_channel)
    {
    case ADC_CHANNEL_KEY:
        ADCR0 = 0x7F; // AN7ͨ����ʹ��ADC
        ADCR1 = 0x83; // adcʱ�� == FHIRC/32��ʹ��VDD��5V���ο���ѹ
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
    static volatile u8 count = 0;                      // ��������
    static volatile u8 filter_cnt = 0;                 // ����������ʹ�õı���
    static volatile u8 filter_key_id = AD_KEY_ID_NONE; // ����ʱ��¼��һ�βɼ����İ���id
    // �а�������ʱ���� AD_KEY_ID ���ް������� KEY_ID_NONE
    // �ҵ� id_table[] �ж�Ӧ�İ���:
    cur_key_id = get_key_id();

    if (cur_key_id != filter_key_id && filter_cnt)
    { // ����а�������
        filter_cnt = 0;
        filter_key_id = cur_key_id;
        return;
    }

    if (filter_cnt < 2)
    { // �����⵽��ͬ�İ�������/�ɿ�
        // ��ֹ�������
        filter_cnt++;
        return;
    }

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
            { // ���� / ��������������

                // if (AD_KEY_ID_MID == last_key_id && KEY_EVENT_HOLD == ad_key_event) // ����ʹ������ж�����
                if (AD_KEY_ID_MID == last_key_id)
                {
                    // ����ǵ��ڵƹ����ȵİ���������������
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

    if (cur_key_id < AD_KEY_ID_NONE &&
        ad_key_event != KEY_EVENT_NONE &&
        ad_key_event != 0)
    {
        // send_data_msb(cur_key_id); // �ܼ�⵽����id
        // send_data_msb(ad_key_event);

        switch (key_event_table[cur_key_id][ad_key_event])
        {
        case KYE_UP_CLICK: //
            send_buf[1] = 0x41;
            flag_is_send_data = 1;
            break;
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

            break;

        case KEY_SEL_HOLD:
            // DEBUG_PIN = ~DEBUG_PIN;
            // �ս�������Ѿ�����750 + 150ms
            set_time_hold_cnt++;
            if (set_time_hold_cnt >= 14) // 14 * 150ms == 2100ms
            {
                set_time_hold_cnt = 0;
                cur_set_time_status = STATUS_SET_TIME_HOUR;
            }

            break;

        case KEY_RIGHT_CLICK:

            if (STATUS_NONE != cur_set_time_status)
            { // ����ڵ���ʱ��
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
                        { // ���δ��50min ��59 min��
                            unit = 0;
                            decade++;
                        }
                        else
                        { // ����Ѿ�����59min�������е���
                            unit--;
                        }
                    }

                    TM1650_DisplayTime();
                }
            }
            else
            { // ���û���ڵ���ʱ��
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
            { // ���û���ڵ���ʱ��
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
            { // ����ڵ���ʱ��
                set_time_done_cnt = 0;
                set_time_delay_cnt = 0;
                if (STATUS_SET_TIME_HOUR == cur_set_time_status)
                {
                    if (thousand > 0) // 10H������
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
                    if (decade > 0) // 10min������
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
            { // ���û���ڵ���ʱ��
                send_buf[1] = 0x51;
                flag_is_send_data = 1;
            }

            break;

        case KEY_LEFT_LONG:
            if (STATUS_NONE == cur_set_time_status)
            { // ���û���ڵ���ʱ��
                send_buf[1] = 0x52;
                flag_is_send_data = 1;
            }
            break;

        case KEY_LEFT_HOLD:
            if (STATUS_NONE == cur_set_time_status)
            { // ���û���ڵ���ʱ��
                send_buf[1] = 0x53;
                flag_is_send_data = 1;
            }
            break;

        default:
            break;
        }

        ad_key_event = KEY_EVENT_NONE; // �������¼������
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

// IICдһ���ֽ�
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

    // ������ʾ����������(�������)
    TM1650_WriteByte(0x48, TM1650_DISPLAY_ON | TM1650_BRIGHT_MAX);

    // �����ʾ
    // TM1650_Clear();
}

// ��ʾ����(0-9999)
void TM1650_DisplayTime(void)
{
    // ��������������1λ
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

    ad_key_event = KEY_EVENT_NONE;
    cur_key_id = AD_KEY_ID_NONE;
    cur_w_pwm_duty = 255;
    cur_y_pwm_duty = 255;

    send_buf[0] = 0xA5;

    cur_set_time_status = STATUS_SET_TIME_HOUR; // ������

    while (1)
    {
        adc_channel_sel(ADC_CHANNEL_KEY);
        key_ad_key_event_deal();

        // { // �͵�����⣺
        //     static u8 power_low_3_2_v_cnt = 0;
        //     static u8 power_low_3_0_v_cnt = 0;
        //     static u8 normal_power_cnt = 0; // ��ѹ�����ļ���
        //     adc_channel_sel(ADC_CHANNEL_VDD);
        //     adc_val = adc_get_val();
        //     if (adc_val <= POWER_LOW_3_0_V)
        //     { // �����⵽��������3.0V
        //         power_low_3_0_v_cnt++;
        //         power_low_3_2_v_cnt = 0;
        //         normal_power_cnt = 0;
        //     }
        //     else if (adc_val <= POWER_LOW_3_2_V)
        //     { // �����⵽�͵���
        //         power_low_3_2_v_cnt++;
        //         power_low_3_0_v_cnt = 0;
        //         normal_power_cnt = 0;
        //     }
        //     else if (adc_val >= POWER_LOW_3_2_V + 50)
        //     { // �����⵽��ѹ�ָ�
        //         normal_power_cnt++;
        //         power_low_3_0_v_cnt = 0;
        //         power_low_3_2_v_cnt = 0;
        //     }

        //     if (power_low_3_0_v_cnt >= 100)
        //     {
        //         power_low_3_0_v_cnt = 0;
        //         // ���������μ�⵽�͵���
        //         // ���͵͵�����Ϣ
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
        //     {                     // ��ص�������3.0V  �ر�����
        //         BLE_CTL_PIN_IN(); // ����ģʽ
        //         flag_is_ble_open = 0;
        //     }
        //     else if (BATTERY_POWER_LOW_3_2_V == battery_status)
        //     {
        //     }
        //     else
        //     {
        //     }

        // } // �͵������

        if (STATUS_SET_TIME_HOUR == cur_set_time_status)
        { // ����Сʱ
            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayTime();
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
                cur_set_time_status = STATUS_SET_TIME_MIN;
                set_time_done_cnt = 0;
            }
        }
        else if (STATUS_SET_TIME_MIN == cur_set_time_status)
        { // ���÷���
            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayTime();
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
                cur_set_time_status = STATUS_NONE;
                set_time_done_cnt = 0;
            }
        }
        else // STATUS_NONE == cur_set_time_status
        {
            TM1650_DisplayTime(); // ��ʾ��ǰʱ��
        }

        if (flag_is_send_data)
        {
            send_buf[2] = ~send_buf[1];
            start_send();
            flag_is_send_data = 0;
        }
        else if (flag_recv_time_ack)
        { // ����յ���ʱ�䣬����Ӧ��
            send_buf[1] = 0x83;
            send_buf[2] = 0x7C;
            start_send();
            flag_recv_time_ack = 0;
        }

        // �͹��ģ�
        // �ƹ�ر� �� ÿ��һ��ʱ�� �ͽ���͹���

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
//             GIE = 0;  // �������ж�
//             ADEN = 0; // �ر�ad
//             T3EN = 0; // �رն�ʱ��

// #if USE_MY_DEBUG
//             P20KE = 1; // ʹ��IO�ļ����ж�
// #else
//             P16KE = 1; // ʹ��IO�ļ����ж�
// #endif

//             // ����ǰ�ر����� AD�� ʹ�ܻ��������ж�
//             KBIE = 1; // ʹ�ܼ����ж�
//             HFEN = 0; // �رո���ʱ��
//             LFEN = 1;
//             Nop();
//             Nop();
//             Stop();
//             Nop();
//             Nop();
//             HFEN = 1; // ʹ�ܸ���ʱ��
//             // LFEN = 1;
//             KBIE = 0; // �رռ����ж�
//             ADEN = 1; // ʹ��adc
//             T3EN = 1; // ʹ�ܶ�ʱ��
//             GIE = 1;  // ʹ�����ж�

//             // Sys_Init();
//             DEBUG_PIN = 0;
//         }

        __asm;
        clrwdt; // ι��
        __endasm;

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

    // ʹ��32768Hz������Ϊʱ��Դ�Ķ�ʱ��:(Ŀǰ����ͳ��ʱ��û������)
    if (T1IF & T1IE)
    {
        static volatile u16 timer1_cnt = 0;
        timer1_cnt++;
        if (timer1_cnt >= 1024)
        {
            /*
                ʹ���ⲿ32768Hz����ʱ,32��Ƶ��, ÿ 0.9765625 ms ����һ��,
                T1LOAD = 10 - 1ʱ,ÿ 9.765625 ms ����һ���ж�
                �����ϴ���1024���ж�ʱ,�պü��� 10 000 ms, 10s
            */
            timer1_cnt = 0;

            keep_time_cnt++;
            if (keep_time_cnt >= 6)
            {                      // �������60s
                keep_time_cnt = 0; // �����ʱֵ

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
        // Ŀǰ��100us����һ��

        { // ad����ɨ��ʱ�����
            static volatile u8 ad_key_scan_time_cnt = 0;
            ad_key_scan_time_cnt++;
            if (ad_key_scan_time_cnt >= 100) // 100us * 100 == 10ms
            {
                flag_10ms = 1; // ����ad����ɨ��
                ad_key_scan_time_cnt = 0;
            }
        }

        { // ����ʱ��ʱ��ʵ��LED��˸Ч���ļ���
            static volatile u8 __set_time_cnt = 0;
            __set_time_cnt++;

            if (__set_time_cnt >= 10) // 1ms����һ��
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

        // { // �͵�������
        //     static volatile u8 power_low_cnt = 0;

        //     if (BATTERY_POWER_LOW_3_2_V == battery_status)
        //     {
        //         power_low_cnt++;
        //         if (power_low_cnt >= 150) // �͵���15s�󣬷���һ�ε͵�ѹ����
        //         {
        //             power_low_cnt = 0;
        //             // send_buf[1] = 0x81;
        //             // flag_is_send_data = 1;
        //         }
        //     }
        //     else
        //     { // ���ǵ͵���������״̬(3.0V~3.2V)����ռ���ֵ
        //         power_low_cnt = 0;
        //     }
        // }

        { // �͹��ģ�
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

        // �������ݷ���&���մ���
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
