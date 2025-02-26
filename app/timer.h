APP_TIMER_DEF(m_battery_timer_id);  /**< Battery timer. */
APP_TIMER_DEF(m_data_out_timer_id); /**< 100ms timer. */
APP_TIMER_DEF(m_1s_timer_id);

static volatile uint8_t one_second_counter = 0;

void battery_level_meas_timeout_handler(void* p_context)
{
    UNUSED_PARAMETER(p_context);
    adc_configure();
    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

void data_timeout_handler(void* p_context)
{
    UNUSED_PARAMETER(p_context);
    ble_nus_state_reset();
}

void m_1s_timeout_hander(void* p_context)
{
    static uint8_t long_termflag = 0;
    static uint8_t flag_ble = 0;
    static uint8_t back_storage_value = 0;
    static bool first_read = false;

    UNUSED_PARAMETER(p_context);

    one_second_counter++;

    // feed wdt
    nrf_drv_wdt_channel_feed(m_channel_id);

#ifdef UART_TRANS
    if(first_read == false)
    {
        send_ble_data_to_st(UART_CMD_ADV_NAME, ble_adv_name, 0x12);
        send_ble_data_to_st(UART_CMD_BLE_VERSION, FW_REVISION, strlen(FW_REVISION));
        send_ble_data_to_st_byte(UART_CMD_BLE_CON_STA, VALUE_DISCONNECT);
        first_read = true;
    }
    if(bat_level_to_st != 0xff)
    {
        if(long_termflag == 0)
        {
            long_termflag = 1;
            app_timer_stop(m_battery_timer_id);
            app_timer_start(m_battery_timer_id, BATTERY_MEAS_LONG_INTERVAL, NULL);
            NRF_LOG_INFO("Start long term time");
        }
    }
    if((backup_bat_level != bat_level_to_st) || (long_termflag == 1))
    {
        NRF_LOG_INFO("backup_bat_level %d ", backup_bat_level);
        NRF_LOG_INFO("bat_level_to_st %d ", bat_level_to_st);
        if(long_termflag == 1)
        {
            long_termflag = 2;
        }
        if(get_usb_ins_flag())
        {
            if(backup_bat_level != 0xFF && bat_level_to_st != 0xFF)
            {
                backup_bat_level = (bat_level_to_st - backup_bat_level > 1) ? (++backup_bat_level) : bat_level_to_st;
            }
            else
            {
                if(bat_level_to_st == 0xff)
                {
                    set_battery_level(backup_bat_level);
                }
                else
                {
                    backup_bat_level = bat_level_to_st;
                }
            }
            NRF_LOG_INFO("usb insert %d \n", backup_bat_level);
        }
        else
        {
            if(backup_bat_level != 0xFF)
            {
                if(backup_bat_level > bat_level_to_st)
                {
                    backup_bat_level = (backup_bat_level - bat_level_to_st > 1) ? (--backup_bat_level) : bat_level_to_st;
                }
                else if(backup_bat_level < bat_level_to_st)
                {
                    if((bat_level_to_st - backup_bat_level) >= 2)
                    {
                        backup_bat_level = bat_level_to_st;
                    }
                    else
                    {
                        set_battery_level(backup_bat_level);
                    }
                }
            }
            else
            {
                backup_bat_level = bat_level_to_st;
                NRF_LOG_INFO("first init", bat_level_to_st);
            }
            NRF_LOG_INFO("no charge %d \n", backup_bat_level);
        }
        send_ble_data_to_st_byte(UART_CMD_BAT_PERCENT, backup_bat_level);
        if(bat_level_flag == 1)
        {
            if(back_storage_value != backup_bat_level)
            {
                back_storage_value = backup_bat_level;
                bat_level_flag = 2;
            }
        }
        else if(bat_level_flag == 0)
        {
            bat_level_flag = 1;
        }
        NRF_LOG_INFO("bat_level_flag is %d\n", bat_level_flag);
    }

    if(one_second_counter == 1 && flag_ble == 0)
    {
        flag_ble = 1;
        send_ble_data_to_st_byte(UART_CMD_CTL_BLE, ((ble_status_flag - 1) ^ 1));
    }
#endif

    if(one_second_counter >= 20)
    {
        one_second_counter = 0;
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_1s_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                m_1s_timeout_hander);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_data_out_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                data_timeout_handler);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_1s_timer_id, ONE_SECOND_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    // Start battery timer
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    // Start data out timer
    // err_code = app_timer_start(m_data_out_timer_id, RCV_DATA_TIMEOUT_INTERVAL, NULL);
    // APP_ERROR_CHECK(err_code);
}

void start_data_out_timer(void)
{
    ret_code_t err_code = app_timer_start(m_data_out_timer_id, RCV_DATA_TIMEOUT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

void stop_data_out_timer(void)
{
    ret_code_t err_code = app_timer_stop(m_data_out_timer_id);
    APP_ERROR_CHECK(err_code);
}
