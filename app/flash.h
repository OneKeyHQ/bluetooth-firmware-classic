/* Dummy data to write to flash. */
static uint32_t m_data2 = 0xBADC0FFE;
static uint32_t m_data = 0xABABABAB;
#define STORAGE_TRUE_FLAG 0xa55aa55a

#define BLE_CTL_ADDR         0x6f000
#define BAT_LVL_ADDR         0x70000
#define DEVICE_KEY_INFO_ADDR 0x71000

static void fstorage_evt_handler(nrf_fstorage_evt_t* p_evt)
{
    if(p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch(p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
            {
                NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                             p_evt->len, p_evt->addr);
            }
            break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
            {
                NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                             p_evt->len, p_evt->addr);
            }
            break;

        default:
            break;
    }
}

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x6f000,
    .end_addr   = 0x72000,
};

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ? bootloader_addr : (code_sz * page_sz));
}
void wait_for_flash_ready(nrf_fstorage_t const* p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while(nrf_fstorage_is_busy(p_fstorage))
    {
        idle_state_handle();
    }
}

static void flash_data_write(uint32_t address, uint32_t data)
{
    ret_code_t rc;

    nrf_fstorage_erase(&fstorage, address, FDS_PHY_PAGES_IN_VPAGE, NULL);

    rc = nrf_fstorage_write(&fstorage, address, &data, sizeof(m_data), NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);
}

static void device_key_info_init(void)
{
    ret_code_t rc;
    ecdsa_key_info_t key_info = {0};
    nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
    if(key_info.key_flag != STORAGE_TRUE_FLAG)
    {
        key_info.key_flag = STORAGE_TRUE_FLAG;
        generate_ecdsa_keypair(key_info.private_key, key_info.public_key);

        rc = nrf_fstorage_erase(&fstorage, DEVICE_KEY_INFO_ADDR, FDS_PHY_PAGES_IN_VPAGE, NULL);
        APP_ERROR_CHECK(rc);

        rc = nrf_fstorage_write(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t), NULL);
        APP_ERROR_CHECK(rc);

        wait_for_flash_ready(&fstorage);
    }
}

static void fs_init(void)
{
    ret_code_t rc;

    nrf_fstorage_api_t* p_fs_api;

    p_fs_api = &nrf_fstorage_sd;
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    (void)nrf5_flash_end_addr_get();

    // flash_data_write(BLE_CTL_FLAG,m_data);
}

uint8_t get_ble_ctl_flag(void)
{
    uint32_t len = 4;
    uint8_t data[4];
    nrf_fstorage_read(&fstorage, BLE_CTL_ADDR, data, len);
    if(((0xFF == data[0]) && (0xFF == data[1]) && (0xFF == data[2]) && (0xFF == data[3])) ||
       ((0xFE == data[0]) && (0x0F == data[1]) && (0xDC == data[2]) && (0xBA == data[3])))
    {
        return BLE_ON_ALWAYS;
    }
    else if((0xAB == data[0]) && (0xAB == data[1]) && (0xAB == data[2]) && (0xAB == data[3]))
    {
        return BLE_OFF_ALWAYS;
    }
    else
    {
        return BLE_ON_ALWAYS;
    }
}

void set_ble_ctl_flag(bool flag)
{
    flash_data_write(BLE_CTL_ADDR, flag ? m_data2 : m_data);
}
