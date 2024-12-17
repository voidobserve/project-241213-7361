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

void timer0_config(void)
{
    // T0CR = DEF_SET_BIT0 | DEF_SET_BIT2; // ��ʱģʽ,CPU,32��Ƶ
    // T0CNT = 250 - 1;                    // 1ms
    // T0LOAD = 250 - 1;
    // T0EN = 1;
    // T0IE = 1;
}

void timer1_config(void)
{
    // T1CR = DEF_SET_BIT0 | DEF_SET_BIT2 | DEF_SET_BIT3 | DEF_SET_BIT4; // ��ʱģʽ,FLOSC,32��Ƶ
    T1CR = 0x15; // ʱ��Դѡ��FLOSC��32��Ƶ��32Khz(32768) / 32 ԼΪ 1Khz��ʵ����ÿ��0.9765625ms����һ��
    // T1CNT = 2 - 1; //
    T1LOAD = 10 - 1; //
    T1EN = 1;
    T1IE = 1;
}

void timer3_config(void)
{
    T3CR = DEF_SET_BIT0 | DEF_SET_BIT2; // ��ʱģʽ,CPU,32��Ƶ
    T3CNT = 250 - 1;                    // 1ms
    T3LOAD = 250 - 1;
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
    T0DATA = 0;

    T2CR = DEF_SET_BIT6 | DEF_SET_BIT0 | DEF_SET_BIT1; // ʹ��PWM,CPU,8��Ƶ
    T2CNT = 255 - 1;
    T2LOAD = 255 - 1; //
    T2DATA = 0;

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
        }
        return;
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
        g_state = IDLE;
        break;
    }
    g_last_pin_state = pin_state;
}

// ʹ��ʾ��
void example(void)
{
    uint8_t send_buf[SEND_DATA_LEN] = {0x01, 0x02, 0x03};
    // ����������
    start_send(send_buf);

    // ���յ������ݿ�����RX_END״̬�´���
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

    // ��������ţ�ֻ����ƽ
    P03PD = 1; // ����
    P03OE = 1; // �����ţ�����ģʽ

    // �����������ţ�����Ϊ����ģʽ
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
    static u8 last = AD_KEY_ID_NONE;
    static u8 count = 0;
    // �а�������ʱ���� AD_KEY_ID ���ް������� KEY_ID_NONE
    // �ҵ� id_table[] �ж�Ӧ�İ���:
    cur_key_id = get_key_id();

    // if (cur_key_id < AD_KEY_ID_NONE)
    //     send_data_msb(cur_key_id); // ���Է���������Եõ���Ӧ�İ���id

    if (last != cur_key_id) // ����а��� / ̧����
    {
        if (last == AD_KEY_ID_NONE)
        {
            // ����
            count = 0;
        }
        else if (cur_key_id == AD_KEY_ID_NONE)
        {
            // ̧��
            if (count < 75)
            {
                // �̰�  -->  ad_key_event
                ad_key_event = KEY_EVENT_CLICK;
            }
        }
        else
        {
            // ���°���ʱ��;�ְ��������������������������
        }
    }
    else if (cur_key_id != AD_KEY_ID_NONE)
    {
        // ��������
        count++;

        if (count == 70)
        {
            // ����  -->  ad_key_event
            ad_key_event = KEY_EVENT_LONG;
        }
        else if (count >= 70 + 10)
        {
            // ��������  -->  ad_key_event
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

    // if (cur_key_id < AD_KEY_ID_NONE)
    if (cur_key_id < AD_KEY_ID_NONE && ad_key_event != KEY_EVENT_NONE)
    {
        // send_data_msb(cur_key_id); // �ܼ�⵽����id,����event==3
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

            break;

        case KEY_MID_HOLD:

            break;

        case KEY_SEL_HOLD:
            // P22D = ~P22D;
            // �ս�������Ѿ�����800ms
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

        ad_key_event = KEY_EVENT_NONE; // �������¼������
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
void TM1650_Clear(void)
{
    // ���4�������
    TM1650_WriteByte(0x68, 0x00); // ��1λ
    TM1650_WriteByte(0x6A, 0x00); // ��2λ
    TM1650_WriteByte(0x6C, 0x00); // ��3λ
    TM1650_WriteByte(0x6E, 0x00); // ��4λ
}

// ��ʼ��TM1650
void TM1650_Init(void)
{
    bsp_i2c_init();

    // ������������
    TM1650_WriteByte(0x48, 0x01); // ������ʾ

    // ��������(�������)
    TM1650_WriteByte(0x48, TM1650_DISPLAY_ON | TM1650_BRIGHT_MAX);

    // �����ʾ
    TM1650_Clear();
}

// ��ʾ��������
void TM1650_DisplayBit(uint8_t pos, uint8_t num)
{
    uint8_t addr;

    // if (pos > 4 || num > 9)
    //     return; // �������

    // ����λ�ö�Ӧ�ĵ�ַ
    addr = 0x68 + (pos - 1) * 2;

    // д�����
    TM1650_WriteByte(addr, NUM_TABLE[num]);
}

// ��ʾ����(0-9999)
void TM1650_DisplayNum(uint16_t num)
{
    uint8_t thousand, hundred, decade, unit;

    if (num > 9999)
        return; // �������

    // �������λ
    thousand = num / 1000;
    hundred = (num % 1000) / 100;
    decade = (num % 100) / 10;
    unit = num % 10;

    // ��ʾ����λ
    TM1650_DisplayBit(4, thousand);
    TM1650_DisplayBit(3, hundred);
    TM1650_DisplayBit(2, decade);
    TM1650_DisplayBit(1, unit);
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

        if (STATUS_SET_TIME_HOUR == cur_status)
        { // ����Сʱ
            // TM1650_DisplayNum(cur_time);
            // delay_ms(500);
            // TM1650_WriteByte(0x6E, 0x00); // ��4λ
            // TM1650_WriteByte(0x6C, 0x00); // ��3λ
            // delay_ms(500);

            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayNum(cur_time);
            }
            else
            {
                TM1650_WriteByte(0x6E, 0x00); // ��4λ
                TM1650_WriteByte(0x6C, 0x00); // ��3λ

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
            // TM1650_DisplayNum(cur_time);
            // delay_ms(500);
            // TM1650_WriteByte(0x6A, 0x00); // ��2λ
            // TM1650_WriteByte(0x68, 0x00); // ��1λ
            // delay_ms(500);

            if (set_time_delay_cnt < 500)
            {
                TM1650_DisplayNum(cur_time);
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
    }
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
