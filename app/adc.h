#define ADC_CHANNEL_BATTERY 0
#define ADC_CHANNEL_TEMP    1

#define ADC_CHANNEL_NUM 2

static uint8_t bat_level_to_st = 0xff;
static uint8_t backup_bat_level = 0xff;

static nrf_saadc_value_t adc_buf[2][ADC_CHANNEL_NUM];
static uint16_t m_batt_lvl_in_milli_volts = 0; //!< Current battery level.
static uint16_t m_last_volts = 0;
static uint16_t count_usb_ins = 0;
static HW_VER_t hw_ver = HW_VER_INVALID;

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 90   // 270=0.3v  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

#include <math.h>
#define R_NTC   10000
#define R_FIXED 10000
#define B_VALUE 3380
#define ADC_MAX 1024

static int voltage_to_resistance(int voltage_mv)
{
    return R_FIXED * voltage_mv / (3300 - voltage_mv);
}

static int resistance_to_temperature(int resistance)
{
    float temperature = (B_VALUE / (log((float)resistance / R_NTC) + (B_VALUE / 298.15))) - 273.15;
    return temperature;
}

static bool check_mv_in_range(uint16_t value, uint16_t target, uint16_t percentage)
{
    uint16_t range = target * percentage / 100;
    return (value >= (target - range)) && (value <= (target + range));
}

HW_VER_t get_hw_ver(void)
{
    return hw_ver;
}

static HW_VER_t get_hw_ver_from_adc(uint16_t adc_voltage)
{
    if(check_mv_in_range(adc_voltage, HW_VER_V_PURE, HW_VER_ADC_VOLTAGE_TOLERANCE))
    {
        return HW_VER_V_PURE;
    }
    else if(check_mv_in_range(adc_voltage, HW_VER_V_2_0, HW_VER_ADC_VOLTAGE_TOLERANCE))
    {
        return HW_VER_V_2_0;
    }
    else if(check_mv_in_range(adc_voltage, HW_VER_V_1_X, HW_VER_ADC_VOLTAGE_TOLERANCE))
    {
        return HW_VER_V_1_X;
    }
    return HW_VER_INVALID;
}

uint8_t get_battery_level(void)
{
    return bat_level_to_st;
}

void set_battery_level(uint8_t level)
{
    bat_level_to_st = level;
}

uint8_t get_backup_bat_level(void)
{
    return backup_bat_level;
}

void set_backup_bat_level(uint8_t level)
{
    backup_bat_level = level;
}

static uint8_t calc_bat_level(uint16_t mv_value)
{
    uint8_t percentage_batt_level;
    uint8_t bat_level;

    percentage_batt_level = battery_level_in_percent(mv_value);

    switch(percentage_batt_level)
    {
        case 100:
            bat_level = 4;
            break;
        case 75:
            bat_level = 3;
            break;
        case 50:
            bat_level = 2;
            break;
        case 25:
            bat_level = 1;
            break;
        case 0:
            bat_level = 0;
            break;
        default:
            break;
    }
    return bat_level;
}

static void saadc_event_handler(nrf_drv_saadc_evt_t const* p_evt)
{
    static uint16_t charge_time = 0;
    static uint8_t power_change_flag = 0;
    static uint8_t bk_level = 0;
    static uint8_t charge_flag = 0;

    if(p_evt->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;

        uint32_t err_code;
        err_code = nrf_drv_saadc_buffer_convert(p_evt->data.done.p_buffer, ADC_CHANNEL_NUM);
        APP_ERROR_CHECK(err_code);

        for(int i = 0; i < p_evt->data.done.size; i++)
        {
            NRF_LOG_INFO("adc_result is %d , %d mv", p_evt->data.done.p_buffer[i], ADC_RESULT_IN_MILLI_VOLTS(p_evt->data.done.p_buffer[i]));
        }

        // Get the ADC result for the battery channel
        adc_result = p_evt->data.done.p_buffer[ADC_CHANNEL_BATTERY];
        if(adc_result > 1024)
        {
            adc_result = 1024;
        }
        if(get_usb_ins_flag())
        {
            NRF_LOG_INFO("Usb charge mode.");
        }

        m_batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        NRF_LOG_INFO("m_batt_lvl_in_milli_volts is %u mV", m_batt_lvl_in_milli_volts);

        if(m_last_volts == 0)
        {
            NRF_LOG_INFO("m_last_volts----0");
            if(m_batt_lvl_in_milli_volts > 2000 && m_batt_lvl_in_milli_volts <= 3600)
            {
                m_last_volts = m_batt_lvl_in_milli_volts;
                NRF_LOG_INFO("-2 voltage is   %u mV", m_batt_lvl_in_milli_volts);
            }
            else
            {
                NRF_LOG_INFO("-3 voltage is 0 mV");
                m_last_volts = 0;
            }
            if(get_usb_ins_flag())
            {
                m_batt_lvl_in_milli_volts -= 100;
                NRF_LOG_INFO("-1 voltage is  %u mV", m_batt_lvl_in_milli_volts);
            }
        }
        if(get_usb_ins_flag())
        {
            if(m_batt_lvl_in_milli_volts > 2000 && m_batt_lvl_in_milli_volts <= 3600)
            {
                m_batt_lvl_in_milli_volts -= 100;
            }
            else
            {
                m_batt_lvl_in_milli_volts = m_last_volts;
            }

            if(m_batt_lvl_in_milli_volts > m_last_volts)
            {
                m_last_volts = m_batt_lvl_in_milli_volts;
            }
            if(bat_level_to_st == 0xFF)
            {
                m_last_volts -= 168;
                bat_level_to_st = calc_bat_level(m_last_volts);
            }

            if(power_change_flag == 0)
            {
                power_change_flag = 1;
            }
            NRF_LOG_INFO("charge adc is %u mV", m_last_volts);
            NRF_LOG_INFO("bat_level_to_st is %d", bat_level_to_st);
            count_usb_ins++;
            if(0 == bat_level_to_st)
            {
                charge_time = 60; // 5 minuts
            }
            else if(1 == bat_level_to_st)
            {
                charge_time = 240; // 20 minuts
            }
            else if((2 == bat_level_to_st) || (3 == bat_level_to_st))
            {
                charge_time = 300; // 25 minutes
            }

            if(count_usb_ins > charge_time)
            {
                count_usb_ins = 0;
                if(bat_level_to_st < 4)
                {
                    NRF_LOG_INFO("increase level");
                    bat_level_to_st += 1;
                }
            }
        }
        else
        {
            count_usb_ins = 0;

            if(power_change_flag == 1)
            {
                m_last_volts = m_batt_lvl_in_milli_volts;
            }
            else
            {
                if(m_batt_lvl_in_milli_volts < 3000)
                {
                    m_last_volts = m_batt_lvl_in_milli_volts;
                }
                else if(m_batt_lvl_in_milli_volts < m_last_volts)
                {
                    if(m_last_volts - m_batt_lvl_in_milli_volts < 40)
                    {
                        m_last_volts = m_batt_lvl_in_milli_volts;
                    }
                }
            }
            NRF_LOG_INFO("no charge adc is %u mV", m_last_volts);
            NRF_LOG_INFO("m_batt_lvl_in_milli_volts %u mV", m_batt_lvl_in_milli_volts);

            if(power_change_flag == 1)
            {
                bk_level = calc_bat_level(m_last_volts);
                NRF_LOG_INFO("bk_level is %u\n", bk_level);
                if(bk_level < bat_level_to_st)
                {
                    bat_level_to_st = bk_level;
                    NRF_LOG_INFO("power change bat_level_to_st is %u", bat_level_to_st);
                    power_change_flag = 0;
                }
            }
            else
            {
                bat_level_to_st = calc_bat_level(m_last_volts);
                NRF_LOG_INFO("bat_level_to_st is %u", bat_level_to_st);
            }
        }

        err_code = ble_bas_battery_level_update(&m_bas, battery_level_in_percent(m_last_volts), BLE_CONN_HANDLE_ALL);
        if((err_code != NRF_SUCCESS) &&
           (err_code != NRF_ERROR_INVALID_STATE) &&
           (err_code != NRF_ERROR_RESOURCES) &&
           (err_code != NRF_ERROR_BUSY) &&
           (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
        {
            APP_ERROR_HANDLER(err_code);
        }

        adc_result = p_evt->data.done.p_buffer[ADC_CHANNEL_TEMP];
        int temperature = resistance_to_temperature(voltage_to_resistance(ADC_RESULT_IN_MILLI_VOLTS(adc_result)));
        NRF_LOG_INFO("temperature adc_result is %d mv,%d C", ADC_RESULT_IN_MILLI_VOLTS(adc_result), temperature);

        if(get_hw_ver() == HW_VER_V_2_0)
        {
            NRF_LOG_INFO("hardware version is 2.0, charge flag is %d", charge_flag);
            if((temperature > 45 || temperature < 0) && charge_flag)
            {
                NRF_LOG_INFO("temperature is out of range, charge off");
                charge_flag = 0;
                charge_off();
            }
            else if((temperature > 3 && temperature < 42) && !charge_flag)
            {
                NRF_LOG_INFO("temperature is in range, charge on");
                charge_flag = 1;
                charge_on();
            }
        }

        nrf_drv_saadc_uninit();
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
        NVIC_ClearPendingIRQ(SAADC_IRQn);
    }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config0 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    err_code = nrf_drv_saadc_channel_init(0, &config0);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config2 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    err_code = nrf_drv_saadc_channel_init(2, &config2);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(adc_buf[0], ADC_CHANNEL_NUM);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(adc_buf[1], ADC_CHANNEL_NUM);
    APP_ERROR_CHECK(err_code);
}

void adc_get_hw_ver(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    config.resistor_p = NRF_SAADC_RESISTOR_PULLUP;
    err_code = nrf_drv_saadc_channel_init(1, &config);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_value_t adc_result;

    err_code = nrfx_saadc_sample_convert(1, &adc_result);
    APP_ERROR_CHECK(err_code);

    uint16_t hw_ver_adc_voltage = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
    hw_ver = get_hw_ver_from_adc(hw_ver_adc_voltage);
    nrf_drv_saadc_uninit();
}
