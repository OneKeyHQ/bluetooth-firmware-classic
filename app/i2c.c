#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "app_error.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "boards.h"
#include "nrf_drv_twi.h"
#include "sdk_config.h"

#include "nrf_log_default_backends.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"

#include "nrf_delay.h"

#include "i2c.h"

enum
{
    READSTATE_IDLE,
    READSTATE_READ_INFO,
    READSTATE_READ_DATA,
    READSTATE_READ_FIDO_STATUS,
    READSTATE_READ_FIDO_LEN,
    READSTATE_READ_FIDO_DATA,
};

typedef struct
{
    bool flag;
    uint8_t data_type;
    uint8_t data[1024 * 3];
    uint16_t len;
} i2c_data_buffer_t;

static i2c_data_buffer_t i2c_data_buf = {false, {0}, 0, 0};

static uint8_t i2c_buf_dma[255];

// TWI driver
static volatile bool twi_xfer_done = false;
static uint8_t twi_xfer_dir = 0; // 0-write 1-read

void get_i2c_data(uint8_t** data, uint16_t* len)
{
    *data = i2c_data_buf.data;
    *len = i2c_data_buf.len;
}

bool get_i2c_data_flag(void)
{
    return i2c_data_buf.flag;
}

void set_i2c_data_flag(bool flag)
{
    i2c_data_buf.flag = flag;
}

/**
 * @brief TWI master instance.
 *
 * Instance of TWI master driver that will be used for communication with simulated
 * EEPROM memory.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

static void twi_handler(nrf_drv_twi_evt_t const* p_event, void* p_context)
{
    static uint8_t read_state = READSTATE_IDLE;
    static uint32_t data_len = 0;
    uint32_t len;

    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            twi_xfer_done = true;
            if(twi_xfer_dir == 1)
            {
                if(read_state == READSTATE_IDLE)
                {
                    if(i2c_data_buf.data[0] == '?' && i2c_data_buf.data[1] == '#' && i2c_data_buf.data[2] == '#')
                    {
                        read_state = READSTATE_READ_INFO;
                        i2c_data_buf.len = 3;
                        nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data + i2c_data_buf.len, 6); // read id+len bytes len
                    }
                    else if(i2c_data_buf.data[0] == 'f' && i2c_data_buf.data[1] == 'i' && i2c_data_buf.data[2] == 'd')
                    {
                        read_state = READSTATE_READ_FIDO_STATUS;
                        nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data, 1); // read status
                    }
                }
                else if(read_state == READSTATE_READ_INFO)
                {
                    i2c_data_buf.len += 6;
                    data_len = ((uint32_t)i2c_data_buf.data[5] << 24) + (i2c_data_buf.data[6] << 16) + (i2c_data_buf.data[7] << 8) + i2c_data_buf.data[8];
                    if(data_len > sizeof(i2c_data_buf.data))
                    {
                        read_state = READSTATE_IDLE;
                        return;
                    }
                    len = data_len > 255 ? 255 : data_len;
                    if(len > 0)
                    {
                        read_state = READSTATE_READ_DATA;
                        nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data + i2c_data_buf.len, len); // read id+len bytes len
                        data_len -= len;
                        i2c_data_buf.len += len;
                    }
                    else
                    {
                        i2c_data_buf.flag = true;
                        i2c_data_buf.data_type = DATA_TYPE_NUS;
                        read_state = READSTATE_IDLE;
                    }
                }
                else if(read_state == READSTATE_READ_DATA)
                {
                    len = data_len > 255 ? 255 : data_len;
                    if(len > 0)
                    {
                        nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data + i2c_data_buf.len, len); // read id+len bytes len
                        data_len -= len;
                        i2c_data_buf.len += len;
                    }
                    else
                    {
                        i2c_data_buf.flag = true;
                        i2c_data_buf.data_type = DATA_TYPE_NUS;
                        read_state = READSTATE_IDLE;
                    }
                }
                else if(read_state == READSTATE_READ_FIDO_STATUS)
                {
                    i2c_data_buf.len = 1;
                    read_state = READSTATE_READ_FIDO_LEN;
                    nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data + i2c_data_buf.len, 2); // read len
                }
                else if(read_state == READSTATE_READ_FIDO_LEN)
                {
                    i2c_data_buf.len += 2;
                    data_len = ((uint32_t)i2c_data_buf.data[1] << 8) + i2c_data_buf.data[2];
                    if(data_len > sizeof(i2c_data_buf.data))
                    {
                        read_state = READSTATE_IDLE;
                        return;
                    }
                    len = data_len > 255 ? 255 : data_len;
                    if(len > 0)
                    {
                        read_state = READSTATE_READ_FIDO_DATA;
                        nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data + i2c_data_buf.len, len);
                        data_len -= len;
                        i2c_data_buf.len += len;
                    }
                    else
                    {
                        i2c_data_buf.flag = true;
                        i2c_data_buf.data_type = DATA_TYPE_FIDO;
                        read_state = READSTATE_IDLE;
                    }
                }
                else if(read_state == READSTATE_READ_FIDO_DATA)
                {
                    len = data_len > 255 ? 255 : data_len;
                    if(len > 0)
                    {
                        nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data + i2c_data_buf.len, len);
                        data_len -= len;
                        i2c_data_buf.len += len;
                    }
                    else
                    {
                        i2c_data_buf.flag = true;
                        i2c_data_buf.data_type = DATA_TYPE_FIDO;
                        read_state = READSTATE_IDLE;
                    }
                }
            }
            else
            {
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
int twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
        {
            .scl = TWI_SCL_M,
            .sda = TWI_SDA_M,
            .frequency = NRF_DRV_TWI_FREQ_400K,
            .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
            .clear_bus_init = false};

    ret = nrf_drv_twi_init(&m_twi_master, &config, twi_handler, NULL);

    if(NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}

bool i2c_master_write(uint8_t* buf, uint32_t len)
{
    ret_code_t err_code;
    uint32_t offset = 0;

    twi_xfer_dir = 0;
    twi_xfer_done = false;

    NRF_LOG_INFO("twi send data len =%d", len);
    while(len > 255)
    {
        while(nrf_drv_twi_is_busy(&m_twi_master))
            ;
        memcpy(i2c_buf_dma, buf + offset, 255);
        err_code = nrf_drv_twi_tx(&m_twi_master, SLAVE_ADDR, i2c_buf_dma, 255, true);
        offset += 255;
        len -= 255;
    }
    if(len)
    {
        while(nrf_drv_twi_is_busy(&m_twi_master))
            ;
        memcpy(i2c_buf_dma, buf + offset, len);
        err_code = nrf_drv_twi_tx(&m_twi_master, SLAVE_ADDR, i2c_buf_dma, len, false);
    }
    NRF_LOG_INFO("twi send data finish");
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }
    return true;
}

bool i2c_master_write_ex(uint8_t* buf, uint8_t len, bool no_stop)
{
    ret_code_t err_code;

    twi_xfer_dir = 0;
    twi_xfer_done = false;
    NRF_LOG_INFO("twi send data len =%d ,%d", len, no_stop);
    while(nrf_drv_twi_is_busy(&m_twi_master))
        ;
    err_code = nrf_drv_twi_tx(&m_twi_master, SLAVE_ADDR, buf, len, no_stop);
    NRF_LOG_INFO("twi send data finish");
    if(NRF_SUCCESS != err_code)
    {
        NRF_LOG_INFO("twi send error");
        return false;
    }
    return true;
}

bool i2c_master_read(void)
{
    uint32_t offset = 0;
    ret_code_t err_code;

    twi_xfer_dir = 1;
    i2c_data_buf.len = 0;

    err_code = nrf_drv_twi_rx(&m_twi_master, SLAVE_ADDR, i2c_data_buf.data + offset, 3);
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }
    return true;
}

bool i2c_master_write_fido(uint8_t* buf, uint32_t len)
{
    ret_code_t err_code;
    uint32_t offset = 0;

    twi_xfer_dir = 0;
    twi_xfer_done = false;

    NRF_LOG_INFO("twi send fido data len =%d", len);

    while(nrf_drv_twi_is_busy(&m_twi_master))
        ;
    memcpy(i2c_buf_dma, "fid", 3);
    err_code = nrf_drv_twi_tx(&m_twi_master, SLAVE_ADDR, i2c_buf_dma, 3, true);
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }

    while(len > 255)
    {
        while(nrf_drv_twi_is_busy(&m_twi_master))
            ;
        memcpy(i2c_buf_dma, buf + offset, 255);
        err_code = nrf_drv_twi_tx(&m_twi_master, SLAVE_ADDR, i2c_buf_dma, 255, true);
        offset += 255;
        len -= 255;
    }
    if(len)
    {
        while(nrf_drv_twi_is_busy(&m_twi_master))
            ;
        memcpy(i2c_buf_dma, buf + offset, len);
        err_code = nrf_drv_twi_tx(&m_twi_master, SLAVE_ADDR, i2c_buf_dma, len, false);
    }
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }
    NRF_LOG_INFO("twi send fido data finish");
    return true;
}

void twi_read_data(void)
{
    uint32_t counter = 0;
    uint8_t* data;
    uint16_t len = 0;

    if(!i2c_master_read())
    {
        NRF_LOG_INFO("twi read data error");
        return;
    }
    while(false == get_i2c_data_flag())
    {
        counter++;
        nrf_delay_ms(1);
        if(counter > 500)
            return;
    }
    set_i2c_data_flag(false);
    // response data
    get_i2c_data(&data, &len);
    if(i2c_data_buf.data_type == DATA_TYPE_FIDO)
    {
        NRF_LOG_INFO("twi read fido data");
        ble_fido_send(data, len);
    }
    else
    {
        ble_nus_send(data, len);
    }
}
