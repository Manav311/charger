
////v5 PDM mic -> dB level over UART
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrfx_pdm.h"
#include "nrfx_uarte.h"
#include "nrf_delay.h"
#include "app_error.h"


#include "nordic_common.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//// PDM Microphone pins
#define PDM_CLK_PIN  7   // P0.07
#define PDM_DIN_PIN  8   // P0.08

//// UART pins (adjust these to your board)
#define UART_TX_PIN  3   // P0.06
#define UART_RX_PIN  4   // P0.05 (not used but required)

//// PDM buffer configuration
#define PDM_BUFFER_SIZE 512  // 512 samples = 1024 bytes
static int16_t m_pdm_buffer[2][PDM_BUFFER_SIZE];
static uint8_t m_buffer_idx = 0;

//// UART instance
static const nrfx_uarte_t m_uart = NRFX_UARTE_INSTANCE(0);
static volatile bool m_uart_tx_done = true;

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "DUSQ_CHARGER"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define AUTH_TIMEOUT_INTERVAL           APP_TIMER_TICKS(30000)                   /**< Authentication timeout interval (10 seconds). */
#define FIXED_AUTH_ID                   000000                                  /**< Fixed 6-digit authentication ID. */

#define AUTH_SERVICE_UUID_BASE          {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                                         0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define AUTH_SERVICE_UUID               0x1523
#define AUTH_CHAR_UUID                  0x1524

APP_TIMER_DEF(m_auth_timer_id);                                                 /**< Authentication timer instance. */

BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint32_t m_unique_id = 0;                                                /**< 6-digit unique ID for authentication. */
static bool m_authenticated = false;                                            /**< Authentication status flag. */

static uint16_t m_auth_service_handle;                                          /**< Handle of Authentication Service. */
static ble_gatts_char_handles_t m_auth_char_handles;                            /**< Handles of Authentication characteristic. */
static uint8_t m_auth_uuid_type;                                                /**< UUID type for authentication service. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */



/**
 * @brief UART event handler
 */
static void uart_event_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_UARTE_EVT_TX_DONE:
            m_uart_tx_done = true;
            break;
            
        case NRFX_UARTE_EVT_ERROR:
             //Handle error if needed
            m_uart_tx_done = true;
            break;
            
        default:
            break;
    }
}

/**
 * @brief Initialize UART at high baud rate
 */
static void uart_init(void)
{
    nrfx_uarte_config_t uart_config;
    
//     Configure UART manually
    uart_config.pseltxd = UART_TX_PIN;
    uart_config.pselrxd = UART_RX_PIN;
    uart_config.pselcts = NRF_UARTE_PSEL_DISCONNECTED;  // No CTS pin
    uart_config.pselrts = NRF_UARTE_PSEL_DISCONNECTED;  // No RTS pin
    uart_config.p_context = NULL;
    uart_config.hwfc = NRF_UARTE_HWFC_DISABLED;  // No hardware flow control
    uart_config.parity = NRF_UARTE_PARITY_EXCLUDED;
    uart_config.baudrate = NRF_UARTE_BAUDRATE_115200;  // Standard speed for text
    uart_config.interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY;
    
    ret_code_t err_code = nrfx_uarte_init(&m_uart, &uart_config, uart_event_handler);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Send data over UART (non-blocking)
 */
static void uart_send(uint8_t const * p_data, size_t length)
{
     //Wait for previous transmission to complete
    while (!m_uart_tx_done)
    {
        __WFE();
    }
    
    m_uart_tx_done = false;
    ret_code_t err_code = nrfx_uarte_tx(&m_uart, p_data, length);
    APP_ERROR_CHECK(err_code);
}

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function to generate a random 6-digit unique ID.
 *
 * @details Generates a random number between 100000 and 999999.
 */
static void generate_unique_id(void)
{
    uint32_t random_value;

    // Use device ID as seed for random number generation
    NRF_RNG->TASKS_START = 1;

    // Generate random bytes
    while (NRF_RNG->EVENTS_VALRDY == 0);
    random_value = NRF_RNG->VALUE;
    NRF_RNG->EVENTS_VALRDY = 0;

    while (NRF_RNG->EVENTS_VALRDY == 0);
    random_value |= ((uint32_t)NRF_RNG->VALUE) << 8;
    NRF_RNG->EVENTS_VALRDY = 0;

    while (NRF_RNG->EVENTS_VALRDY == 0);
    random_value |= ((uint32_t)NRF_RNG->VALUE) << 16;
    NRF_RNG->EVENTS_VALRDY = 0;

    NRF_RNG->TASKS_STOP = 1;

    // Ensure the ID is within 6-digit range (100000-999999)
    m_unique_id = 100000 + (random_value % 900000);

    NRF_LOG_INFO("Generated Unique ID: %06d", m_unique_id);
}


/**@brief Authentication timeout handler.
 *
 * @details This function will be called when the authentication timer expires.
 *          If the central has not successfully authenticated within 2 seconds,
 *          the connection will be terminated.
 *
 * @param[in] p_context  Unused.
 */
static void auth_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (!m_authenticated && m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("Authentication timeout - disconnecting");
        ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Failed to disconnect: error code 0x%x", err_code);
        }
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module, making it use the scheduler
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create authentication timer
    err_code = app_timer_create(&m_auth_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                auth_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
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


/**@brief Function for handling write events to the authentication characteristic.
 *
 * @param[in] p_data   Pointer to the written data.
 * @param[in] length   Length of the written data.
 */
static void auth_write_handler(uint8_t const * p_data, uint16_t length)
{
    if (length == 6)
    {
        // Convert 6-digit ASCII string to integer
        uint32_t received_id = 0;
        bool valid = true;

        for (int i = 0; i < 6; i++)
        {
            if (p_data[i] >= '0' && p_data[i] <= '9')
            {
                received_id = received_id * 10 + (p_data[i] - '0');
            }
            else
            {
                valid = false;
                break;
            }
        }

        if (valid && received_id == m_unique_id)
        {
            m_authenticated = true;

            // Stop the authentication timer
            ret_code_t err_code = app_timer_stop(m_auth_timer_id);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("Failed to stop auth timer: 0x%x", err_code);
            }

            NRF_LOG_INFO("Authentication successful! ID matched: %06d", received_id);
            bsp_board_led_on(CONNECTED_LED);
        }
        else
        {
            NRF_LOG_WARNING("Authentication failed! Expected: %06d, Received: %06d",
                           m_unique_id, received_id);

            // Disconnect immediately on failed authentication
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("Failed to disconnect: 0x%x", err_code);
                }
            }
        }
    }
    else
    {
        NRF_LOG_WARNING("Invalid authentication data length: %d (expected 6)", length);
    }
}


/**@brief Function for initializing the Authentication Service.
 */
static void auth_service_init(void)
{
    ret_code_t         err_code;
    ble_uuid_t         ble_uuid;
    ble_uuid128_t      base_uuid = {AUTH_SERVICE_UUID_BASE};
    ble_add_char_params_t add_char_params;

    // Add custom base UUID
    err_code = sd_ble_uuid_vs_add(&base_uuid, &m_auth_uuid_type);
    APP_ERROR_CHECK(err_code);

    // Add service
    ble_uuid.type = m_auth_uuid_type;
    ble_uuid.uuid = AUTH_SERVICE_UUID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &m_auth_service_handle);
    APP_ERROR_CHECK(err_code);

    // Add authentication characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = AUTH_CHAR_UUID;
    add_char_params.uuid_type         = m_auth_uuid_type;
    add_char_params.max_len           = 6;
    add_char_params.init_len          = 0;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.write  = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;

    err_code = characteristic_add(m_auth_service_handle,
                                  &add_char_params,
                                  &m_auth_char_handles);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Authentication service initialized");
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF!");
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

    // Initialize Authentication Service
    auth_service_init();
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
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
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            // Reset authentication status and start authentication timer
            m_authenticated = false;
            err_code = app_timer_start(m_auth_timer_id, AUTH_TIMEOUT_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Authentication timer started - 2 seconds to authenticate");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            // Stop authentication timer and reset state
            app_timer_stop(m_auth_timer_id);
            m_authenticated = false;

            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
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
        } break;

        case BLE_GATTS_EVT_WRITE:
        {
            ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

            // Check if write is to authentication characteristic
            if (p_evt_write->handle == m_auth_char_handles.value_handle)
            {
                auth_write_handler(p_evt_write->data, p_evt_write->len);
            }
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
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

        default:
            // No implementation needed.
            break;
    }
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


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            NRF_LOG_INFO("Send button state change.");
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief Integer square root (for RMS calculation without floating point)
 */
static uint32_t isqrt(uint32_t n)
{
    if (n == 0) return 0;
    uint32_t x = n;
    uint32_t y = (x + 1) / 2;
    while (y < x)
    {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

// T3902 Standard Mode: -26 dBFS at 94 dB SPL, 0 dBFS = 120 dB SPL
// PDM gain_l = 0x20 adds ~4 dB attenuation vs default 0x28
// Total offset: 120 + 4 = 124 (in x10 fixed-point: 1240)
#define DBFS_TO_SPL_OFFSET_X10  1240

/**
 * @brief Approximate 20*log10(rms) using integer math, converted to dB SPL.
 *        Uses log2 via leading-zero count, then converts: dB ~ 6.0206 * log2(x)
 *        Returns dB SPL in fixed-point with 1 decimal place (e.g. 723 = 72.3 dB SPL).
 */
static int32_t rms_to_db_spl(uint32_t rms)
{
    if (rms == 0) return 0; // silence = 0 dB SPL (floor)

    // 20*log10(rms/32768) in x10 fixed point
    // 20*log10(32768) * 10 = 903

    // Find integer part of log2
    uint32_t log2_int = 0;
    uint32_t temp = rms;
    while (temp > 1)
    {
        temp >>= 1;
        log2_int++;
    }

    // Fractional part: linear interpolation between powers of 2
    uint32_t pow2 = 1UL << log2_int;
    uint32_t frac_1000 = ((rms - pow2) * 1000UL) / pow2;

    // 20*log10(rms) in x10 fixed point = 60 * log2(rms)
    int32_t dbfs_x10 = (int32_t)(60 * log2_int + (60 * frac_1000) / 1000);
    dbfs_x10 -= 903; // subtract 20*log10(32768)*10

    // Convert dBFS to dB SPL
    int32_t db_spl_x10 = dbfs_x10 + DBFS_TO_SPL_OFFSET_X10;

    if (db_spl_x10 < 0) db_spl_x10 = 0; // floor at 0 dB SPL

    return db_spl_x10;
}

// UART text buffer for dB output
static char m_uart_text[64];

/**
 * @brief PDM event handler - computes dB level and sends over UART
 */
static void pdm_event_handler(nrfx_pdm_evt_t const * p_evt)
{
    if (p_evt->buffer_released != NULL)
    {
        int16_t *p_buf = (int16_t *)p_evt->buffer_released;

        // First pass: compute DC mean to remove T3902's ~3% DC offset
        int32_t sum = 0;
        for (int i = 0; i < PDM_BUFFER_SIZE; i++)
        {
            sum += (int32_t)p_buf[i];
        }
        int32_t dc_mean = sum / PDM_BUFFER_SIZE;

        // Second pass: compute sum of squares with DC removed
        uint64_t sum_sq = 0;
        for (int i = 0; i < PDM_BUFFER_SIZE; i++)
        {
            int32_t sample = (int32_t)p_buf[i] - dc_mean;
            sum_sq += (uint64_t)(sample * sample);
        }

        uint32_t mean_sq = (uint32_t)(sum_sq / PDM_BUFFER_SIZE);
        uint32_t rms = isqrt(mean_sq);

        int32_t db_spl = rms_to_db_spl(rms);

        // db_spl is in x10 fixed point, e.g. 753 = 75.3 dB SPL
        int32_t whole = db_spl / 10;
        int32_t frac = db_spl % 10;
        int len = snprintf(m_uart_text, sizeof(m_uart_text),
                           "dB SPL: %d.%d\r\n", (int)whole, (int)frac);

        uart_send((uint8_t*)m_uart_text, (size_t)len);
    }

    if (p_evt->buffer_requested)
    {
        m_buffer_idx = (m_buffer_idx + 1) % 2;
        nrfx_pdm_buffer_set(m_pdm_buffer[m_buffer_idx], PDM_BUFFER_SIZE);
    }
}

/**
 * @brief Initialize PDM
 */
static void pdm_init(void)
{
    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    
    pdm_config.mode = NRF_PDM_MODE_MONO;
    pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    
     //Use available frequency for nRF52810
    pdm_config.clock_freq = NRF_PDM_FREQ_1032K;
    
  // Reduced gain for high-sensitivity T3902
    pdm_config.gain_l = 0x20;
    pdm_config.gain_r = 0x20;

    ret_code_t err_code = nrfx_pdm_init(&pdm_config, pdm_event_handler);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
     //Initialize UART first
    uart_init();
    
     //Send startup message
    const char startup_msg[] = "PDM Mic dB SPL Monitor\r\n";
    uart_send((uint8_t*)startup_msg, strlen(startup_msg));
    nrf_delay_ms(100);
    
     //Initialize and start PDM
    pdm_init();
      // Initialize.
    log_init();
    leds_init();
    timers_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    nrfx_pdm_start();
        // Set fixed authentication ID
    m_unique_id = FIXED_AUTH_ID;
    NRF_LOG_INFO("Authentication ID set to: %06d", m_unique_id);

    // Start execution.
    NRF_LOG_INFO("Blinky example started.");
    advertising_start();

    
    while (true)
    {
        __WFE();
        __SEV();
        __WFE();
    }
}



