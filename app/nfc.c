/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup nfc_uart_tag_example_main main.c
 * @{
 * @ingroup nfc_uart_tag_example
 * @brief The application main file of NFC UART Tag example.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "app_error.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "boards.h"
#include "nfc_t4t_lib.h"
#include "sdk_config.h"

#include "nrf_log_default_backends.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"

#include "nrf_delay.h"

#include "nfc.h"

#define MAX_APDU_LEN 1024 /**< Maximal APDU length, Adafruit limitation. */
// #define HEADER_FIELD_SIZE 1      /**< Header field size. */
#define HEADER_FIELD_SIZE 0 /**< no header */

static bool multi_package = false;

// NFC buffer
uint8_t nfc_apdu[253];
uint32_t nfc_apdu_len = 0;
uint8_t nfc_data_out_buf[APDU_BUFF_SIZE];
uint32_t nfc_data_out_len = 0;
bool nfc_multi_packet = false;

extern uint8_t ble_adv_switch_flag;

static void apdu_command(const uint8_t* p_buf, uint32_t data_len);
bool apdu_cmd = false;

/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void* context,
                         nfc_t4t_event_t event,
                         const uint8_t* data,
                         size_t dataLength,
                         uint32_t flags)
{
    ret_code_t err_code;

    (void)context;

    switch(event)
    {
        case NFC_T4T_EVENT_FIELD_ON:
            multi_package = false;
            NRF_LOG_INFO("NFC Tag has been selected. UART transmission can start...");
            break;

        case NFC_T4T_EVENT_FIELD_OFF:
            multi_package = false;
            NRF_LOG_INFO("NFC field lost. Data from UART will be discarded...");
            break;
        case NFC_T4T_EVENT_DATA_IND:
            if(dataLength > APDU_BUFF_SIZE)
            {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
            }

            if(flags != NFC_T4T_DI_FLAG_MORE)
            {
                memcpy(nfc_apdu, data, dataLength);
                nfc_apdu_len = dataLength;
                nfc_multi_packet = false;
                NRF_LOG_INFO("NFC RX data length: %d", dataLength);
                // NRF_LOG_HEXDUMP_INFO(data,dataLength);
                apdu_command(data, dataLength);
                if(nfc_data_out_len > 0)
                {
                    // Send the response PDU over NFC.
                    err_code = nfc_t4t_response_pdu_send(nfc_data_out_buf, nfc_data_out_len + HEADER_FIELD_SIZE);
                    APP_ERROR_CHECK(err_code);
                    nfc_data_out_len = 0;
                }
            }
            else
            {
                multi_package = true;
                memcpy(nfc_apdu, data, dataLength);
                nfc_apdu_len = dataLength;
                apdu_cmd = true;
                nfc_multi_packet = true;
                // i2c_master_write_ex((uint8_t*)data,dataLength,true);
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Function for application main entry.
 */
int nfc_init(void)
{
    ret_code_t err_code;

    /* Set up NFC */
    err_code = nfc_t4t_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);

    /* Start sensing NFC field */
    err_code = nfc_t4t_emulation_start();
    APP_ERROR_CHECK(err_code);
    return 0;
}

static void apdu_command(const uint8_t* p_buf, uint32_t data_len)
{
    static bool reading = false;

    if(multi_package) // multi_package end
    {
        multi_package = false;
        apdu_cmd = true;
        // i2c_master_write_ex((uint8_t*)p_buf,data_len,false);
        nfc_data_out_len = 2;
        memcpy(nfc_data_out_buf, "\x90\x00", nfc_data_out_len);
    }
    else
    {
        if(p_buf[0] == '?')
        {
            set_i2c_data_flag(false);
            apdu_cmd = true;
            reading = false;
            // i2c_master_write_ex((uint8_t*)p_buf,data_len,false);
            nfc_data_out_len = 2;
            memcpy(nfc_data_out_buf, "\x90\x00", nfc_data_out_len);
        }
        else if(p_buf[0] == '#' && p_buf[1] == '*' && p_buf[2] == '*')
        {
            // usart
            if(reading == false)
            {
                if(nrf_gpio_pin_read(TWI_STATUS_GPIO) == 1) // can read
                {
                    i2c_master_read();
                    reading = true;
                }
                nfc_data_out_len = 3;
                memcpy(nfc_data_out_buf, "#**", nfc_data_out_len);
            }
            else
            {
                if(get_i2c_data_flag() == false)
                {
                    nfc_data_out_len = 3;
                    memcpy(nfc_data_out_buf, "#**", nfc_data_out_len);
                }
                else
                {
                    uint8_t* data;
                    set_i2c_data_flag(false);
                    reading = false;
                    get_i2c_data(&data, &nfc_data_out_len);
                    memcpy(nfc_data_out_buf, data, nfc_data_out_len);
                }
            }
        }
        else
        {
            if(p_buf[0] == 0x5A && p_buf[1] == 0xA5 && p_buf[2] == 0x07 && p_buf[3] == 0x1)
            {
                if(p_buf[4] == 0x03)
                {
                    ble_adv_switch_flag = 3;
                }
                else if(p_buf[4] == 0x02)
                {
                    ble_adv_switch_flag = 2;
                }
                nfc_data_out_len = 3;
                memcpy(nfc_data_out_buf, "\xA5\x5\01", nfc_data_out_len);
            }
            else
            {
                nfc_data_out_len = 2;
                memcpy(nfc_data_out_buf, "\x6D\x00", nfc_data_out_len);
            }
        }
    }
}

void nfc_poll(void* p_event_data, uint16_t event_size)
{
    if(apdu_cmd == true)
    {
        apdu_cmd = false;
        i2c_master_write_ex(nfc_apdu, nfc_apdu_len, nfc_multi_packet);
    }
}
/** @} */
