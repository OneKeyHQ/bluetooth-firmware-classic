// UART define
#define MAX_TEST_DATA_BYTES (15U) /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE    256   /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE    256   /**< UART RX buffer size. */
// CMD
#define UART_CMD_BLE_CON_STA  0x01
#define UART_CMD_BLE_PAIR_STA 0x02
#define UART_CMD_PAIR_CODE    0x03
#define UART_CMD_ADV_NAME     0x04
#define UART_CMD_BAT_PERCENT  0x05
#define UART_CMD_BLE_VERSION  0x06
#define UART_CMD_CTL_BLE      0x07
#define UART_CMD_RESET_BLE    0x08
#define UART_CMD_DFU_STA      0x0a
#define UART_CMD_BLE_PUBKEY   0x0b
#define UART_CMD_BLE_SIGN     0x0c
#define UART_CMD_BLE_BUILD_ID 0x0d
#define UART_CMD_BLE_HASH     0x0e
#define UART_CMD_BLE_HW_VER   0x0f
// VALUE
#define VALUE_CONNECT    0x01
#define VALUE_DISCONNECT 0x02
#define VALUE_SECCESS    0x01
#define VALUE_FAILED     0x02
// DFU STATUS
#define VALUE_PREPARE_DFU  0x01
#define VALUE_ENTER_DFU    0x02
#define VALUE_ENTER_FAILED 0x03
#define VALUE_RSP_FAILED   0x04
#define VALUE_UNKNOWN_ERR  0x05

// DATA FLAG
#define DATA_INIT 0x00
#define DATA_HEAD 0x01
#define DATA_DATA 0x02

// BLE RSP STATUS
#define CTL_SUCCESSS 0x01
#define CTL_FAILED   0x02

// CHANNEL
#define BLE_CHANNEL  0x01
#define NFC_CHANNEL  0x02
#define UART_CHANNEL 0x03

#define UART_DEF                  0x00
#define ACTIVE_SEND_UART          0x01
#define RESPONESE_NAME_UART       0x02
#define RESPONESE_BAT_UART        0x03
#define RESPONESE_VER_UART        0x04
#define RESPONESE_BLE_PUBKEY      0x05
#define RESPONESE_BLE_PUBKEY_LOCK 0x06
#define RESPONESE_BLE_SIGN        0x07
#define RESPONESE_BLE_STATUS      0x08
#define RESPONESE_BLE_BUILD_ID    0x09
#define RESPONESE_BLE_BLE_HASH    0x0a
#define RESPONESE_BLE_HW_VER      0x0b
#define DEF_RESP                  0xFF

static volatile uint8_t flag_uart_trans = 1;
static uint8_t uart_data_array[64];
static volatile uint8_t trans_info_flag = 0;

static uint8_t calcXor(uint8_t* buf, uint8_t len)
{
    uint8_t tmp = 0;
    uint8_t i;

    for(i = 0; i < len; i++)
    {
        tmp ^= buf[i];
    }
    return tmp;
}
static void uart_put_data(uint8_t* pdata, uint8_t lenth)
{
    uint32_t err_code;

    while(lenth--)
    {
        do
        {
            err_code = app_uart_put(*pdata++);
            if((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                APP_ERROR_CHECK(err_code);
            }
        } while(err_code == NRF_ERROR_BUSY);
    }
}

static void uart_rsp_status(uint8_t rsp)
{
    uint8_t data[3];
    data[0] = 0xA5;
    data[1] = 0x5A;
    data[2] = rsp;
    uart_put_data(data, 3);
}

void send_ble_data_to_st(uint8_t cmd, uint8_t* data, uint8_t len)
{
    uint8_t uart_trans_buff[128];

    uart_trans_buff[0] = UART_TX_TAG;
    uart_trans_buff[1] = UART_TX_TAG2;
    uart_trans_buff[2] = 0x00;
    uart_trans_buff[3] = len + 3;
    uart_trans_buff[4] = cmd;
    uart_trans_buff[5] = len;
    memcpy(&uart_trans_buff[6], data, len);
    uart_trans_buff[uart_trans_buff[3] + 3] = calcXor(uart_trans_buff, (uart_trans_buff[3] + 3));

    uart_put_data(uart_trans_buff, uart_trans_buff[3] + 4);
}

void send_ble_data_to_st_byte(uint8_t cmd, uint8_t data)
{
    uint8_t data_buff[1];
    data_buff[0] = data;
    send_ble_data_to_st(cmd, data_buff, 1);
}

static void rsp_st_uart_cmd(void* p_event_data, uint16_t event_size)
{
    if(trans_info_flag == DEF_RESP)
    {
        return;
    }
    switch(trans_info_flag)
    {
        case RESPONESE_NAME_UART:
            send_ble_data_to_st(UART_CMD_ADV_NAME, (uint8_t*)ble_adv_name, 0x12);
            break;
        case RESPONESE_VER_UART:
            send_ble_data_to_st(UART_CMD_BLE_VERSION, (uint8_t*)FW_REVISION, strlen(FW_REVISION));
            break;
        case RESPONESE_BAT_UART:
            {
                uint8_t bat_level = get_battery_level();
                send_ble_data_to_st(UART_CMD_BAT_PERCENT, &bat_level, 1);
            }
            break;
        case RESPONESE_BLE_PUBKEY:
            {
                ecdsa_key_info_t key_info = {0};
                nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
                if(key_info.key_lock_flag == STORAGE_TRUE_FLAG)
                {
                    send_ble_data_to_st_byte(UART_CMD_BLE_PUBKEY, 0x01);
                }
                else if(key_info.key_flag != STORAGE_TRUE_FLAG)
                {
                    send_ble_data_to_st_byte(UART_CMD_BLE_PUBKEY, 0x02);
                }
                else
                {
                    send_ble_data_to_st(UART_CMD_BLE_PUBKEY, key_info.public_key, sizeof(key_info.public_key));
                }
            }
            break;
        case RESPONESE_BLE_PUBKEY_LOCK:
            {
                ecdsa_key_info_t key_info = {0};
                nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
                if(key_info.key_lock_flag != STORAGE_TRUE_FLAG)
                {
                    ret_code_t rc;
                    key_info.key_lock_flag = STORAGE_TRUE_FLAG;
                    rc = nrf_fstorage_erase(&fstorage, DEVICE_KEY_INFO_ADDR, FDS_PHY_PAGES_IN_VPAGE, NULL);
                    APP_ERROR_CHECK(rc);

                    rc = nrf_fstorage_write(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t), NULL);
                    APP_ERROR_CHECK(rc);

                    wait_for_flash_ready(&fstorage);
                }
                send_ble_data_to_st_byte(UART_CMD_BLE_PUBKEY, 0x00);
            }
            break;
        case RESPONESE_BLE_SIGN:
            {
                uint32_t msg_len = uart_data_array[2] << 8 | uart_data_array[3] - 2;
                ecdsa_key_info_t key_info = {0};
                nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
                if(key_info.key_flag != STORAGE_TRUE_FLAG)
                {
                    send_ble_data_to_st_byte(UART_CMD_BLE_SIGN, 0x01);
                }
                else
                {
                    uint8_t sign_data[64];
                    sign_ecdsa_msg(key_info.private_key, uart_data_array + 5, msg_len, sign_data);
                    send_ble_data_to_st(UART_CMD_BLE_SIGN, sign_data, 64);
                }
            }
            break;
        case RESPONESE_BLE_STATUS:
            send_ble_data_to_st_byte(UART_CMD_CTL_BLE, ((ble_status_flag - 1) ^ 1));
            break;
        case RESPONESE_BLE_BUILD_ID:
            send_ble_data_to_st(UART_CMD_BLE_BUILD_ID, (uint8_t*)BUILD_ID, 7);
            break;
        case RESPONESE_BLE_BLE_HASH:
            {
                ret_code_t err_code = NRF_SUCCESS;
                nrf_crypto_backend_hash_context_t hash_context = {0};
                uint8_t hash[32] = {0};
                size_t hash_len = 32;
                int chunks = 0;
                int app_size = 0;
                uint8_t* code_addr = (uint8_t*)0x26000;
                uint8_t* code_len = (uint8_t*)0x7F018;
                app_size = code_len[0] + code_len[1] * 256 + code_len[2] * 256 * 256;
                chunks = app_size / 512;

                err_code = nrf_crypto_hash_init(&hash_context, &g_nrf_crypto_hash_sha256_info);
                APP_ERROR_CHECK(err_code);
                for(int i = 0; i < chunks; i++)
                {
                    err_code = nrf_crypto_hash_update(&hash_context, code_addr + i * 512, 512);
                    APP_ERROR_CHECK(err_code);
                }
                if(app_size % 512)
                {
                    err_code = nrf_crypto_hash_update(&hash_context, code_addr + chunks * 512, app_size % 512);
                    APP_ERROR_CHECK(err_code);
                }
                err_code = nrf_crypto_hash_finalize(&hash_context, hash, &hash_len);
                APP_ERROR_CHECK(err_code);

                send_ble_data_to_st(UART_CMD_BLE_HASH, hash, 32);
            }
            break;
        case RESPONESE_BLE_HW_VER:
            {
                HW_VER_t hw_ver = get_hw_ver();
                send_ble_data_to_st(UART_CMD_BLE_HW_VER, &hw_ver, 2);
            }
            break;
        default:
            break;
    }
    trans_info_flag = DEF_RESP;
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t* p_event)
{
    static uint8_t index = 0;
    static uint32_t lenth = 0;
    uint8_t xor_byte;

    switch(p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_data_array[index]));
            index++;

            if(1 == index)
            {
                if((UART_TX_TAG != uart_data_array[0]) && (UART_TX_TAG2 != uart_data_array[0]))
                {
                    index = 0;
                    return;
                }
            }
            else if(2 == index)
            {
                if((UART_TX_TAG2 != uart_data_array[1]) && (UART_TX_TAG != uart_data_array[1]))
                {
                    index = 0;
                    return;
                }
            }
            else if(3 == index)
            {
                if((UART_TX_TAG2 == uart_data_array[0]) && (UART_TX_TAG == uart_data_array[1]))
                {
                    index = 0;
                    return;
                }
            }
            else if(4 == index)
            {
                lenth = ((uint32_t)uart_data_array[2] << 8) + uart_data_array[3];
            }
            else if(index >= lenth + 4)
            {
                xor_byte = calcXor(uart_data_array, index - 1);
                if(xor_byte != uart_data_array[index - 1])
                {
                    index = 0;
                    return;
                }
                lenth -= 1;
                switch(uart_data_array[4])
                {
                    case UART_CMD_CTL_BLE:
                        if(BLE_ON_ALWAYS == uart_data_array[6])
                        {
                            ble_adv_switch_flag = BLE_ON_ALWAYS;
                            NRF_LOG_INFO("RCV ble always ON.");
                        }
                        else if((BLE_OFF_ALWAYS == uart_data_array[6]) || (0 == uart_data_array[6]))
                        {
                            ble_adv_switch_flag = BLE_OFF_ALWAYS;
                            NRF_LOG_INFO("RCV ble always OFF.");
                        }
                        else if(BLE_DISCON == uart_data_array[6])
                        {
                            ble_conn_flag = BLE_DISCON;
                            NRF_LOG_INFO("RCV ble flag disconnect.");
                        }
                        else if(BLE_ON_TEMPO == uart_data_array[6])
                        {
                            ble_conn_flag = BLE_ON_TEMPO;
                            NRF_LOG_INFO("RCV ble flag start adv flag.");
                        }
                        else if(BLE_OFF_TEMPO == uart_data_array[6])
                        {
                            ble_conn_flag = BLE_OFF_TEMPO;
                            NRF_LOG_INFO("RCV ble flag stop adv flag.");
                        }
                        else if(BLE_STATUS == uart_data_array[6])
                        {
                            trans_info_flag = RESPONESE_BLE_STATUS;
                            ;
                        }
                        else
                        {

                            NRF_LOG_INFO("Receive flag is %d \n", uart_data_array[6]);
                        }
                        break;
                    case UART_CMD_RESET_BLE:
                        NVIC_SystemReset();
                        break;
                    case UART_CMD_ADV_NAME:
                        trans_info_flag = RESPONESE_NAME_UART;
                        break;
                    case UART_CMD_BAT_PERCENT:
                        trans_info_flag = RESPONESE_BAT_UART;
                        break;
                    case UART_CMD_BLE_VERSION:
                        trans_info_flag = RESPONESE_VER_UART;
                        break;
                    case UART_CMD_BLE_PUBKEY:
                        trans_info_flag = RESPONESE_BLE_PUBKEY;
                        if(lenth == 2)
                        {
                            if(uart_data_array[5] == 0)
                            {
                                trans_info_flag = RESPONESE_BLE_PUBKEY;
                            }
                            else if(uart_data_array[5] == 1)
                            {
                                trans_info_flag = RESPONESE_BLE_PUBKEY_LOCK;
                            }
                        }
                        break;
                    case UART_CMD_BLE_SIGN:
                        trans_info_flag = RESPONESE_BLE_SIGN;
                        break;
                    case UART_CMD_BLE_BUILD_ID:
                        trans_info_flag = RESPONESE_BLE_BUILD_ID;
                        break;
                    case UART_CMD_BLE_HASH:
                        trans_info_flag = RESPONESE_BLE_BLE_HASH;
                        break;
                    case UART_CMD_BLE_HW_VER:
                        trans_info_flag = RESPONESE_BLE_HW_VER;
                        break;
                    default:
                        break;
                }
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void usr_uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params =
        {
            .rx_pin_no = RX_PIN_NUMBER,
            .tx_pin_no = TX_PIN_NUMBER,
            .rts_pin_no = RTS_PIN_NUMBER,
            .cts_pin_no = CTS_PIN_NUMBER,
            .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
            .use_parity = false,
#if defined(UART_PRESENT)
            .baud_rate = NRF_UART_BAUDRATE_115200
#else
            .baud_rate = NRF_UARTE_BAUDRATE_115200
#endif
        };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
