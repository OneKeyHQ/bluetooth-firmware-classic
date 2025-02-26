
static uint8_t nus_recv_data_buff[NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH];
static uint16_t nus_recv_data_len = 0;
static uint8_t rcv_head_flag = 0;

static uint8_t* ble_nus_send_buf;
static uint16_t ble_nus_send_len, ble_nus_send_offset;

static void ble_nus_send_packet(uint8_t* data, uint16_t data_len)
{
    ret_code_t err_code;
    uint16_t length = 0;

    do
    {
        length = data_len > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : data_len;
        err_code = ble_nus_data_send(&m_nus, data, &length, m_conn_handle);
        if((err_code != NRF_ERROR_INVALID_STATE) &&
           (err_code != NRF_ERROR_RESOURCES) &&
           (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while(err_code == NRF_ERROR_RESOURCES);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t* p_evt)
{
    static uint32_t msg_len;
    uint32_t pad;
    // uint8_t *rcv_data=(uint8_t *)p_evt->params.rx_data.p_data;
    // uint32_t rcv_len=p_evt->params.rx_data.length;

    if(p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        // NRF_LOG_INFO("Received data from BLE NUS.");
        // NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        nus_recv_data_len = p_evt->params.rx_data.length;
        memcpy(nus_recv_data_buff, (uint8_t*)p_evt->params.rx_data.p_data, nus_recv_data_len);

        if(rcv_head_flag == DATA_INIT)
        {
            if(nus_recv_data_buff[0] == '?' && nus_recv_data_buff[1] == '#' && nus_recv_data_buff[2] == '#')
            {
                set_i2c_data_flag(false);
                if(nus_recv_data_len < 9)
                {
                    return;
                }
                else
                {
                    msg_len = (uint32_t)((nus_recv_data_buff[5] << 24) +
                                         (nus_recv_data_buff[6] << 16) +
                                         (nus_recv_data_buff[7] << 8) +
                                         (nus_recv_data_buff[8]));
                    pad = ((nus_recv_data_len + 63) / 64) + 8;
                    if(msg_len > nus_recv_data_len - pad)
                    {
                        msg_len -= nus_recv_data_len - pad;
                        rcv_head_flag = DATA_DATA;
                        start_data_out_timer();
                    }
                }
            }
            else if(nus_recv_data_buff[0] == 0x5A && nus_recv_data_buff[1] == 0xA5 && nus_recv_data_buff[2] == 0x07 && nus_recv_data_buff[3] == 0x1 && nus_recv_data_buff[4] == 0x03)
            {
                ble_adv_switch_flag = BLE_OFF_ALWAYS;
                return;
            }
        }
        else
        {
            if(nus_recv_data_buff[0] == '?')
            {
                pad = (nus_recv_data_len + 63) / 64;
                if(nus_recv_data_len - pad > msg_len)
                {
                    rcv_head_flag = DATA_INIT;
                    nus_recv_data_len = msg_len + (msg_len + 62) / 63;
                    msg_len = 0;
                    stop_data_out_timer();
                }
                else
                {
                    msg_len -= nus_recv_data_len - pad;
                }
            }
            else
            {
                rcv_head_flag = DATA_INIT;
            }
        }
        i2c_master_write(nus_recv_data_buff, nus_recv_data_len);
    }
    else if(p_evt->type == BLE_NUS_EVT_TX_RDY)
    {
        uint32_t length = 0;

        length = ble_nus_send_len - ble_nus_send_offset;
        if(length > 0)
        {
            length = length > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : length;
            ble_nus_send_packet(ble_nus_send_buf + ble_nus_send_offset, length);
            ble_nus_send_offset += length;
        }
        else
        {
            ble_nus_send_len = 0;
            ble_nus_send_offset = 0;
        }
    }
}

void ble_nus_send(uint8_t* data, uint16_t data_len)
{
    ret_code_t err_code;
    uint16_t length = 0;
    uint16_t offset = 0;

    if(data_len == 0)
    {
        return;
    }

    ble_nus_send_buf = data;
    ble_nus_send_len = data_len;

    length = ble_nus_send_len > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : ble_nus_send_len;
    ble_nus_send_offset = length;
    ble_nus_send_packet(ble_nus_send_buf, length);
}

void ble_nus_state_reset(void)
{
    rcv_head_flag = DATA_INIT;
}
