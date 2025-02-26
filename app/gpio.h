void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    twi_read_data();
}

static void gpiote_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(TWI_STATUS_GPIO, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(TWI_STATUS_GPIO, true);
}

static void gpio_init(void)
{
    // Detect USB insert status.
    nrf_gpio_cfg_input(USB_INS_PIN, NRF_GPIO_PIN_NOPULL);
    // nrf_gpio_cfg(CHARGE_CTL_PIN,NRF_GPIO_PIN_DIR_OUTPUT ,NRF_GPIO_PIN_INPUT_DISCONNECT,NRF_GPIO_PIN_PULLUP ,NRF_GPIO_PIN_S0S1,NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg_output(CHARGE_CTL_PIN);
    charge_off();
    // nrf_gpio_cfg_input(TWI_STATUS_GPIO,NRF_GPIO_PIN_PULLUP);
    gpiote_init();
}

bool get_usb_ins_flag(void)
{
    return nrf_gpio_pin_read(USB_INS_PIN) ? true : false;
}
