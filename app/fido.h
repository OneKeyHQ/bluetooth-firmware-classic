#define FIDO_DATA_STATE_IDLE 0
#define FIDO_DATA_STATE_RECV 1

static uint8_t* ble_fido_send_buf;
static uint16_t ble_fido_send_len, ble_fido_send_offset;
static uint8_t fido_sequence_number;

static uint8_t fido_recv_buf[1024];
static uint8_t fido_data_state = FIDO_DATA_STATE_IDLE;
static uint16_t fido_recv_len, fido_recv_offset;

static void ble_fido_send_packet(uint8_t* data, uint16_t data_len)
{
    ret_code_t err_code;
    uint16_t length = 0;

    do
    {
        length = data_len > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : data_len;
        err_code = ble_fido_data_send(&m_fido, data, &length, m_conn_handle);
        if((err_code != NRF_ERROR_INVALID_STATE) &&
           (err_code != NRF_ERROR_RESOURCES) &&
           (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while(err_code == NRF_ERROR_RESOURCES);
}

static void fido_data_handler(ble_fido_evt_t* p_evt)
{
    uint8_t* rcv_data = (uint8_t*)p_evt->params.rx_data.p_data;
    uint32_t rcv_len = p_evt->params.rx_data.length;

    if(p_evt->type == BLE_FIDO_EVT_RX_DATA)
    {
        if(fido_data_state == FIDO_DATA_STATE_IDLE)
        {
            fido_sequence_number = 0;
            fido_recv_len = rcv_data[1] << 8 | rcv_data[2];
            if(fido_recv_len > rcv_len - 3)
            {
                if(fido_recv_len > sizeof(fido_recv_buf) - 3)
                {
                    return;
                }
                memcpy(fido_recv_buf, rcv_data, rcv_len);
                fido_recv_offset = rcv_len;
                fido_data_state = FIDO_DATA_STATE_RECV;
            }
            else
            {
                i2c_master_write_fido(rcv_data, rcv_len);
            }
        }
        else if(fido_data_state == FIDO_DATA_STATE_RECV)
        {
            if(rcv_data[0] == fido_sequence_number)
            {
                fido_sequence_number++;
                memcpy(fido_recv_buf + fido_recv_offset, rcv_data + 1, rcv_len - 1);
                fido_recv_offset += rcv_len - 1;
                if(fido_recv_offset >= fido_recv_len - 3)
                {
                    fido_data_state = FIDO_DATA_STATE_IDLE;
                    i2c_master_write_fido(fido_recv_buf, fido_recv_len + 3);
                }
            }
            else
            {
                fido_data_state = FIDO_DATA_STATE_IDLE;
            }
        }
    }
    else if(p_evt->type == BLE_FIDO_EVT_TX_RDY)
    {
        uint8_t fido_packet[BLE_FIDO_MAX_DATA_LEN];
        uint16_t length = ble_fido_send_len - ble_fido_send_offset;
        if(length > 0)
        {
            fido_packet[0] = fido_sequence_number++;
            length = length > BLE_FIDO_MAX_DATA_LEN - 1 ? BLE_FIDO_MAX_DATA_LEN - 1 : length;
            memcpy(fido_packet + 1, ble_fido_send_buf + ble_fido_send_offset, length);
            ble_fido_send_packet(fido_packet, length + 1);
            ble_fido_send_offset += length;
        }
        else
        {
            ble_fido_send_len = 0;
            ble_fido_send_offset = 0;
            fido_sequence_number = 0;
        }
    }
}

void ble_fido_send(uint8_t* data, uint16_t data_len)
{
    ret_code_t err_code;
    uint16_t length = 0;
    uint16_t offset = 0;

    if(data_len == 0)
    {
        return;
    }

    ble_fido_send_buf = data;
    ble_fido_send_len = data_len;
    fido_sequence_number = 0;

    length = ble_fido_send_len > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : ble_fido_send_len;
    ble_fido_send_offset = length;
    ble_fido_send_packet(ble_fido_send_buf, length);
}