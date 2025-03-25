/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "ble_fido.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_dis.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_lesc.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_bas.h"
#include "app_error.h"
#include "nrf_drv_saadc.h"
#include "sdk_macros.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_scheduler.h"
#include "ble_dfu.h"
#include "nrf_delay.h"
#include "nrf_bootloader_info.h"
#include "nrf_drv_gpiote.h"
#include "nrf_power.h"
#include "nrf_drv_wdt.h"
#include "nrf_fstorage_sd.h"
#include "nrf_fstorage.h"
#include "fds_internal_defs.h"
#include "nrf_crypto_init.h"
#include "ecdsa.h"
#include "nrf_crypto.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "firmware_config.h"

#include "i2c.h"
#include "nfc.h"

#define BLE_DEFAULT      0
#define BLE_CONNECT      1
#define BLE_DISCONNECT   2
#define BLE_DIS_PIN      3
#define BLE_PIN_ERR      4
#define BLE_PIN_TIMEOUT  5
#define BLE_PAIR_SUCCESS 6
#define BLE_PIN_CANCEL   7
#define BLE_RCV_DATA     8
#define BLE_FIDO_DATA    9

#define BLE_DEF        0
#define BLE_ON_ALWAYS  1
#define BLE_OFF_ALWAYS 2
#define BLE_DISCON     3
#define BLE_ON_TEMPO   5
#define BLE_OFF_TEMPO  6
#define BLE_STATUS     7

#define NO_CHARGE  0
#define USB_CHARGE 1
#define ERROR_STA  2

#define INIT_VALUE 0
#define AUTH_VALUE 1

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG  1 /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL 40 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_DURATION 0  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define ADV_ADDL_MANUF_DATA_LEN 6
#define COMPANY_IDENTIFIER      0xFE

// SCHEDULER CONFIGS
#define SCHED_MAX_EVENT_DATA_SIZE 64 //!< Maximum size of the scheduler event data.
#define SCHED_QUEUE_SIZE          5  //!< Size of the scheduler queue.

#define RCV_DATA_TIMEOUT_INTERVAL   APP_TIMER_TICKS(2000)
#define BATTERY_LEVEL_MEAS_INTERVAL APP_TIMER_TICKS(1000) /**< Battery level measurement interval (ticks). */
#define BATTERY_MEAS_LONG_INTERVAL  APP_TIMER_TICKS(5000)
#define MIN_BATTERY_LEVEL           81  /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL           100 /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT     1   /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Minimum acceptable connection interval (10 ms). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Maximum acceptable connection interval (100 ms) */
#define SLAVE_LATENCY                  0                               /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(100)            /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000)          /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define ONE_SECOND_INTERVAL            APP_TIMER_TICKS(1000)

#define TWI_TIMEOUT_COUNTER 10

#define MAX_CONN_PARAM_UPDATE_COUNT 3 /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE 0 /**< Set to 1 to use LESC debug keys, allows you to  use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND            1                            /**< Perform bonding. */
#define SEC_PARAM_MITM            1                            /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_LESC            1                            /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS        0                            /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_DISPLAY_ONLY /**< Display I/O capabilities. */
#define SEC_PARAM_OOB             0                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE    16                           /**< Maximum encryption key size. */

#define PASSKEY_LENGTH     6 /**< Length of pass-key received by the stack for display. */
#define HEAD_NAME_LENGTH   1
#define ADV_NAME_LENGTH    5
#define MAC_ADDRESS_LENGTH 6

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define BLE_GAP_DATA_LENGTH_DEFAULT 27  //!< The stack's default data length.
#define BLE_GAP_DATA_LENGTH_MAX     251 //!< Maximum data length.

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
BLE_BAS_DEF(m_bas);
BLE_FIDO_DEF(m_fido, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
nrf_drv_wdt_channel_id m_channel_id;

static volatile uint8_t ble_evt_flag = 0;
volatile uint8_t ble_adv_switch_flag = 0;
static volatile uint8_t ble_conn_flag = 0;
static uint8_t mac_ascii[24];
static uint8_t mac[6] = {0x42, 0x13, 0xc7, 0x98, 0x95, 0x1a}; // Device MAC address
static char ble_adv_name[20] = {0};

static uint8_t bat_level_flag = 0;

#ifdef BOND_ENABLE
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
#endif
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[] =                        /**< Universally unique service identifiers. */
    {
#if BLE_DIS_ENABLED
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
#endif
        {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_FIDO_SERVICE, BLE_UUID_TYPE_BLE}};

static void idle_state_handle(void);

static uint8_t bond_check_key_flag = INIT_VALUE;
static uint8_t ble_status_flag = 0;

#include "gpio.h"
#include "flash.h"
#include "adc.h"
#include "uart.h"
#include "timer.h"

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch(event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

// lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
} /**< Structure used to identify the battery service. */

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
void on_bas_evt(ble_bas_t* p_bas, ble_bas_evt_t* p_evt)
{
    switch(p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:

            break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:

            break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
            // No implementation needed.
            break;
    }
}

// battery service init
void sys_bas_init(void)
{
    uint32_t err_code;
    ble_bas_init_t bas_init;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler = on_bas_evt;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}
static void advertising_stop(void)
{
    ret_code_t err_code = ble_advertising_stop(&m_advertising);

    APP_ERROR_CHECK(err_code);
}

static void ctl_advertising(void)
{

    ble_status_flag = get_ble_ctl_flag();
    if(ble_status_flag == BLE_ON_ALWAYS)
    {
        advertising_start();
        NRF_LOG_INFO("1-Start adv.\n");
    }
    else if(ble_status_flag == BLE_OFF_ALWAYS)
    {
        NRF_LOG_INFO("2-No Adv.\n");
    }
}

#ifdef BOND_ENABLE
/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const* p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch(p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            {
                pm_conn_sec_status_t conn_sec_status;

                // Check if the link is authenticated (meaning at least MITM).
                err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
                APP_ERROR_CHECK(err_code);

                if(conn_sec_status.mitm_protected)
                {
                    send_ble_data_to_st_byte(UART_CMD_BLE_PAIR_STA, VALUE_SECCESS);
                    nrf_ble_gatt_data_length_set(&m_gatt, m_conn_handle, BLE_GAP_DATA_LENGTH_MAX);
                    NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d",
                                 ble_conn_state_role(p_evt->conn_handle),
                                 p_evt->conn_handle,
                                 p_evt->params.conn_sec_succeeded.procedure);
                }
                else
                {
                    // The peer did not use MITM, disconnect.
                    NRF_LOG_INFO("Collector did not use MITM, disconnecting");
                    err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                    APP_ERROR_CHECK(err_code);
                    err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
            {
                pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
                pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
            }
            break; // PM_EVT_CONN_SEC_CONFIG_REQ

        case PM_EVT_CONN_SEC_FAILED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            send_ble_data_to_st_byte(UART_CMD_BLE_PAIR_STA, VALUE_FAILED);
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();
            break;

        default:
            break;
    }
}
#endif

void mac_address_get(void)
{
    ble_gap_addr_t Mac_address;
    unsigned char i, j = 0;
    //_sd_ble_gap_addr_get(&Mac_address);
    uint32_t err_code = sd_ble_gap_addr_get(&Mac_address);
    APP_ERROR_CHECK(err_code);

    memcpy(mac, Mac_address.addr, 6);
    for(i = 0; i < 6; i++)
    {
        if((mac[i] >> 4) < 0x0a)
        {
            mac_ascii[j] = 0x30 + (mac[i] >> 4);
            j++;
        }
        else
        {
            mac_ascii[j] = 0x31;
            j++;
            mac_ascii[j] = 0x30 + (mac[i] >> 4) % 0x0a;
            j++;
        }

        if((mac[i] & 0x0f) < 0x0a)
        {
            mac_ascii[j] = 0x30 + (mac[i] & 0x0f);
            j++;
        }
        else
        {
            mac_ascii[j] = 0x31;
            j++;
            mac_ascii[j] = 0x30 + (mac[i] & 0x0f) % 0x0a;
            j++;
        }
    }
    memcpy(&ble_adv_name[0], ADV_HEAD_NAME, HEAD_NAME_LENGTH);
    memcpy(&ble_adv_name[HEAD_NAME_LENGTH], mac_ascii, ADV_NAME_LENGTH - HEAD_NAME_LENGTH);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
#ifdef FIXED_PIN
    // set fixed Passkey
    ble_opt_t ble_opt;
    uint8_t g_ucBleTK[6] = "123456";
    ble_opt.gap_opt.passkey.p_passkey = g_ucBleTK;
#endif
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t*)ble_adv_name,
                                          ADV_NAME_LENGTH);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
#ifdef FIXED_PIN
    // set fixed Passkey
    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
    APP_ERROR_CHECK(err_code);
#endif
}

static void send_service_changed(void* p_event_data, uint16_t event_size);

static uint16_t m_ble_gatt_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t* p_gatt, nrf_ble_gatt_evt_t const* p_evt)
{
    if((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_gatt_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_gatt_max_data_len, m_ble_gatt_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void disconnect(uint16_t conn_handle, void* p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if(err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

#include "dfu.h"
#include "nus.h"
#include "fido.h"

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t err_code;
    ble_dis_init_t dis_init;
    ble_nus_init_t nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_fido_init_t fido_init = {0};
#ifdef BUTTONLESS_ENABLED
    ble_dfu_buttonless_init_t dfus_init = {0};
#endif

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
#ifdef BUTTONLESS_ENABLED
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
#endif
    // Initialize Battery Service.
    sys_bas_init();

#if BLE_DIS_ENABLED
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, ble_adv_name);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, HW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, SW_REVISION);

    ble_dis_sys_id_t system_id;
    system_id.manufacturer_id = MANUFACTURER_ID;
    system_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id = &system_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
#endif
    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize FIDO.
    memset(&fido_init, 0, sizeof(fido_init));
    fido_init.data_handler = fido_data_handler;
    err_code = ble_fido_init(&m_fido, &fido_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t* p_evt)
{
    ret_code_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch(ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising");
            break; // BLE_ADV_EVT_FAST

        case BLE_ADV_EVT_IDLE:
            break; // BLE_ADV_EVT_IDLE

        default:
            break;
    }
}

static void send_service_changed(void* p_event_data, uint16_t event_size)
{
    static uint16_t start_handle;
    // const  uint16_t end_handle = 0xFFFF;
    ret_code_t err_code;

    err_code = sd_ble_gatts_initial_user_handle_get(&start_handle);

    if(err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_gatts_initial_user_handle_get() returned %s which should not happen.",
                      nrf_strerror_get(err_code));
        return NRF_ERROR_INTERNAL;
    }

    err_code = sd_ble_gatts_service_changed(m_conn_handle, start_handle, 0xFFFF);
    if((err_code == BLE_ERROR_INVALID_CONN_HANDLE) || (err_code == NRF_ERROR_INVALID_STATE) || (err_code == NRF_ERROR_BUSY))
    {
        /* These errors can be expected when trying to send a Service Changed indication */
        /* if the CCCD is not set to indicate. Thus, set the returning error code to success. */
        NRF_LOG_WARNING("Client did not have the Service Changed indication set to enabled."
                        "Error: 0x%08x",
                        err_code);
        err_code = NRF_SUCCESS;
    }
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const* p_ble_evt, void* p_context)
{
#ifdef BOND_ENABLE
    ret_code_t err_code;

    pm_handler_secure_on_connection(p_ble_evt);

    switch(p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            {
                NRF_LOG_INFO("Disconnected");
                ble_evt_flag = BLE_DISCONNECT;
                bond_check_key_flag = INIT_VALUE;
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                send_ble_data_to_st_byte(UART_CMD_BLE_CON_STA, VALUE_DISCONNECT);
                // Check if the last connected peer had not used MITM, if so, delete its bond information.
                if(m_peer_to_be_deleted != PM_PEER_ID_INVALID)
                {
                    err_code = pm_peer_delete(m_peer_to_be_deleted);
                    APP_ERROR_CHECK(err_code);
                    NRF_LOG_DEBUG("Collector's bond deleted");
                    m_peer_to_be_deleted = PM_PEER_ID_INVALID;
                }
            }
            break;

        case BLE_GAP_EVT_CONNECTED:
            {
                NRF_LOG_INFO("Connected");
                ble_evt_flag = BLE_CONNECT;
                send_ble_data_to_st_byte(UART_CMD_BLE_CON_STA, VALUE_CONNECT);
                m_peer_to_be_deleted = PM_PEER_ID_INVALID;
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
                APP_ERROR_CHECK(err_code);
                nrf_ble_gatt_data_length_set(&m_gatt, m_conn_handle, BLE_GAP_DATA_LENGTH_DEFAULT);
                // Start Security Request timer.
            }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                    {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                    };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
            {
                char passkey[PASSKEY_LENGTH + 1];
                memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
                passkey[PASSKEY_LENGTH] = 0;
                send_ble_data_to_st(UART_CMD_PAIR_CODE, passkey, PASSKEY_LENGTH);
                NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
            }
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                         p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                         p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                         p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                         *((uint8_t*)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                         *((uint8_t*)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            bond_check_key_flag = AUTH_VALUE;
            break;

        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
            NRF_LOG_INFO("BLE_GATTC_EVT_EXCHANGE_MTU_RSP");
            app_sched_event_put(NULL, NULL, send_service_changed);
            break;

        case BLE_GATTS_EVT_HVC:
            NRF_LOG_INFO("BLE_GATTS_EVT_HVC");
            break;

        default:
            // No implementation needed.
            break;
    }
#else
    uint32_t err_code;

    switch(p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                    {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                    };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
#endif
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
#ifdef BOND_ENABLE
/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = 1;
    sec_param.kdist_own.id = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t err_code;
    ble_advertising_init_t init;
    ble_advdata_manuf_data_t manuf_data;
    uint8_t m_addl_adv_manuf_data[MAC_ADDRESS_LENGTH];

    ble_advdata_service_data_t service_data;
    uint8_t service_data_value = 0xC0;

    service_data.service_uuid = BLE_UUID_FIDO_SERVICE;
    service_data.data.size = 1;
    service_data.data.p_data = &service_data_value;

    memset(&init, 0, sizeof(init));

    manuf_data.company_identifier = COMPANY_IDENTIFIER;
    manuf_data.data.size = ADV_ADDL_MANUF_DATA_LEN;
    memcpy(m_addl_adv_manuf_data, mac, MAC_ADDRESS_LENGTH);
    manuf_data.data.p_data = m_addl_adv_manuf_data;

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;
    // init.advdata.p_manuf_specific_data = &manuf_data;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    init.advdata.p_service_data_array = &service_data;
    init.advdata.service_data_count = 1;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

uint32_t get_rtc_counter(void)
{
    return NRF_RTC1->COUNTER;
}
/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    if((BLE_DEF == ble_adv_switch_flag) || (BLE_ON_ALWAYS == ble_adv_switch_flag))
    {
        if(bond_check_key_flag != AUTH_VALUE)
        {
            err_code = nrf_ble_lesc_request_handler();
            APP_ERROR_CHECK(err_code);
        }
    }
    if(NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void system_init()
{
    gpio_init();
#ifdef UART_TRANS
    usr_uart_init();
#endif
}

/**@brief Application main function.
 */
static void wdt_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

static void manage_bat_level(void* p_event_data, uint16_t event_size)
{
    static bool first_read = false;
    uint32_t len = 4;
    uint8_t data[4];

    if(first_read == false)
    {
        first_read = true;
        nrf_fstorage_read(&fstorage, BAT_LVL_ADDR, data, len);
        if(data[0] <= 4)
        {
            set_backup_bat_level(data[0]);
            NRF_LOG_INFO("read storrage level is %d", data[0]);
        }
    }
    if(bat_level_flag == 2)
    {
        bat_level_flag = 1;
        uint32_t m_data3 = (uint32_t)get_backup_bat_level();
        flash_data_write(BAT_LVL_ADDR, m_data3);
        nrf_fstorage_read(&fstorage, BAT_LVL_ADDR, data, len);
        NRF_LOG_INFO("buff0- %d,buff1- %d,buff2- %d,buff3- %d", data[0], data[1], data[2], data[3]);
    }
}
static void check_advertising_stop(void)
{
    if(ble_evt_flag != BLE_DISCONNECT || ble_evt_flag != BLE_DEFAULT)
    {
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        ble_evt_flag = BLE_DISCONNECT;
        NRF_LOG_INFO("Ctl disconnect.");
    }
    nrf_delay_ms(1000);
    advertising_stop();
}
static void ble_ctl_process(void* p_event_data, uint16_t event_size)
{
    if(BLE_OFF_ALWAYS == ble_adv_switch_flag)
    {
        ble_adv_switch_flag = BLE_DEF;
        if(BLE_ON_ALWAYS == ble_status_flag)
        {
            send_ble_data_to_st_byte(UART_CMD_BLE_CON_STA, VALUE_DISCONNECT);
            set_ble_ctl_flag(false);
            ble_status_flag = BLE_OFF_ALWAYS;
            NRF_LOG_INFO("1-Ble disconnect.\n");
            NRF_LOG_INFO("BLE status is %d", ble_evt_flag);

            check_advertising_stop();
        }
        else
        {
#ifdef UART_TRANS
            uart_rsp_status(CTL_FAILED);
#endif
        }
    }
    else if(BLE_ON_ALWAYS == ble_adv_switch_flag)
    {
        ble_adv_switch_flag = BLE_DEF;
        if(BLE_OFF_ALWAYS == ble_status_flag)
        {
            advertising_start();
            set_ble_ctl_flag(true);
            ble_status_flag = BLE_ON_ALWAYS;
            NRF_LOG_INFO("2-Start advertisement.\n");
        }
        else
        {
#ifdef UART_TRANS
            uart_rsp_status(CTL_FAILED);
#endif
        }
    }
    if(BLE_DISCON == ble_conn_flag)
    {
        if(ble_evt_flag != BLE_DISCONNECT || ble_evt_flag != BLE_DEFAULT)
        {
            sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            ble_evt_flag = BLE_DISCONNECT;
            ble_conn_flag = BLE_DEF;
            NRF_LOG_INFO("Ctl disconnect.");
        }
        else
        {
            ble_conn_flag = BLE_DEF;
        }
    }
    else if(BLE_ON_TEMPO == ble_conn_flag)
    {
        if((BLE_OFF_TEMPO == ble_status_flag) || (BLE_OFF_ALWAYS == ble_status_flag))
        {
            advertising_start();
            ble_status_flag = BLE_ON_TEMPO;
        }
        ble_conn_flag = BLE_DEF;
    }
    else if(BLE_OFF_TEMPO == ble_conn_flag)
    {
        if((BLE_ON_TEMPO == ble_status_flag) || (BLE_ON_ALWAYS == ble_status_flag))
        {
            check_advertising_stop();
            ble_status_flag = BLE_OFF_TEMPO;
        }
        ble_conn_flag = BLE_DEF;
    }
}
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
static void main_loop(void)
{
    // app_sched_event_put(NULL,NULL,ble_ctl_process);
    // app_sched_event_put(NULL,NULL,rsp_st_uart_cmd);
    // app_sched_event_put(NULL,NULL,manage_bat_level);
    // app_sched_event_put(NULL,NULL,nfc_poll);

    ble_ctl_process(NULL, 0);
    manage_bat_level(NULL, 0);
    rsp_st_uart_cmd(NULL, 0);

#if NRFX_NFCT_ENABLED
    nfc_poll(NULL, 0);
#endif
}

int main(void)
{
#ifdef BUTTONLESS_ENABLED
    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    ret_code_t err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
#endif
    // Initialize.
    system_init();
    scheduler_init();
    log_init();

    fs_init();
    nrf_crypto_init();
    device_key_info_init();

    adc_get_hw_ver();
    timers_init();
    power_management_init();
    ble_stack_init();
    mac_address_get();
#ifdef BOND_ENABLE
    peer_manager_init();
#endif
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    application_timers_start();
    // Start execution.
    NRF_LOG_INFO("Debug logging for UART over RTT started.");

    ctl_advertising();

    twi_master_init();

#if NRFX_NFCT_ENABLED
    nfc_init();
#endif

    wdt_init();
    // Enter main loop.
    for(;;)
    {
        main_loop();
        app_sched_execute();
        idle_state_handle();
    }
}

/**
 * @}
 */