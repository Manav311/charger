
////v4 getting the DATA on uart
#include <stdbool.h>
#include <stdint.h>
#include <string.h>  // Added for strlen()
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrfx_pdm.h"
#include "nrfx_uarte.h"
#include "nrf_delay.h"
#include "app_error.h"

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
    uart_config.baudrate = NRF_UARTE_BAUDRATE_921600;  // High speed for audio
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

/**
 * @brief PDM event handler - sends audio data over UART
 */
static void pdm_event_handler(nrfx_pdm_evt_t const * p_evt)
{
    if (p_evt->buffer_released != NULL)
    {
         //Send the audio buffer over UART
         //Cast int16_t* to uint8_t* for byte transmission
        uart_send((uint8_t*)p_evt->buffer_released, PDM_BUFFER_SIZE * sizeof(int16_t));
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
    const char startup_msg[] = "PDM Audio Streaming via UART\r\n";
    uart_send((uint8_t*)startup_msg, strlen(startup_msg));
    nrf_delay_ms(100);
    
     //Send sync header to help PC detect start of audio data
    const uint8_t sync_header[] = {0xFF, 0xFF, 0xAA, 0xAA, 0x55, 0x55};
    uart_send(sync_header, sizeof(sync_header));
    nrf_delay_ms(100);
    
     //Initialize and start PDM
    pdm_init();
    nrfx_pdm_start();
    
    while (true)
    {
        __WFE();
        __SEV();
        __WFE();
    }
}



////V3 binary data to direct python
//#include <stdbool.h>
//#include <stdint.h>
//#include "nrf.h"
//#include "nrf_gpio.h"
//#include "nrfx_pdm.h"
//#include "nrf_delay.h"
//#include "app_error.h"
//#include "SEGGER_RTT.h"

//// PDM Microphone pins
//#define PDM_CLK_PIN  7   // P0.07
//#define PDM_DIN_PIN  8   // P0.08

//// PDM buffer configuration
//// Increasing buffer size slightly to help with RTT throughput
//#define PDM_BUFFER_SIZE 256

//static int16_t m_pdm_buffer[2][PDM_BUFFER_SIZE];
//static uint8_t m_buffer_idx = 0;

///**
// * @brief PDM event handler
// * Instead of printing ASCII strings, we write raw binary bytes to RTT.
// * This is significantly faster and prevents data loss.
// */
//static void pdm_event_handler(nrfx_pdm_evt_t const * p_evt)
//{
//    if (p_evt->buffer_released != NULL)
//    {
//        // Write the raw bytes of the buffer directly to RTT Buffer 0
//        // Total bytes = PDM_BUFFER_SIZE * 2 (since each sample is 16-bit)
//        SEGGER_RTT_Write(0, p_evt->buffer_released, PDM_BUFFER_SIZE * sizeof(int16_t));
//    }

//    if (p_evt->buffer_requested)
//    {
//        m_buffer_idx = (m_buffer_idx + 1) % 2;
//        nrfx_pdm_buffer_set(m_pdm_buffer[m_buffer_idx], PDM_BUFFER_SIZE);
//    }
//}

///**
// * @brief Initialize PDM
// */
//static void pdm_init(void)
//{
//    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    
//    pdm_config.mode = NRF_PDM_MODE_MONO;
//    pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    
//    // nRF52810 frequency check:
//    // For 16kHz sample rate, we target ~1.024MHz or 1.032MHz clock.
//    // If NRF_PDM_FREQ_1032K is undefined, 1024K is the standard choice for nRF52810.
//    #if defined(NRF_PDM_FREQ_1032K)
//        pdm_config.clock_freq = NRF_PDM_FREQ_1032K;
//    #elif defined(NRF_PDM_FREQ_1024K)
//        pdm_config.clock_freq = NRF_PDM_FREQ_1024K;
//    #else
//        pdm_config.clock_freq = 0x08000000; // Manual register value for ~1MHz
//    #endif
    
//    // Gain setting: 0x50 is approx +20dB. 
//    // If it sounds distorted, reduce this to 0x40 or NRF_PDM_GAIN_DEFAULT (0x28).
//    pdm_config.gain_l = 0x50; 
//    pdm_config.gain_r = 0x50;

//    ret_code_t err_code = nrfx_pdm_init(&pdm_config, pdm_event_handler);
//    APP_ERROR_CHECK(err_code);
//}

//int main(void)
//{
//    // Initialize RTT
//    SEGGER_RTT_Init();
    
//    // Configure RTT Buffer 0 to "Skip" if full to avoid blocking
//    SEGGER_RTT_ConfigUpBuffer(0, "PDM_DATA", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

//    pdm_init();
//    nrfx_pdm_start();
    
//    while (true)
//    {
//        // Enter System ON sleep mode to save power while waiting for interrupts
//        __WFE();
//        __SEV();
//        __WFE();
//    }
//}







////V2 getting raw data and turning the python script to turn this in wav. 
//#include <stdbool.h>
//#include <stdint.h>
//#include "nrf.h"
//#include "nrf_gpio.h"
//#include "nrfx_pdm.h"
//#include "nrf_delay.h"
//#include "app_error.h"
//#include "SEGGER_RTT.h"

//// PDM Microphone pins
//#define PDM_CLK_PIN  7   // P0.07
//#define PDM_DIN_PIN  8   // P0.08

//// PDM buffer configuration
//#define PDM_BUFFER_SIZE 128

//static int16_t m_pdm_buffer[2][PDM_BUFFER_SIZE];
//static uint8_t m_buffer_idx = 0;

///**
//* @brief PDM event handler - ONLY print raw data
//*/
//static void pdm_event_handler(nrfx_pdm_evt_t const * p_evt)
//{
//   if (p_evt->buffer_released != NULL)
//   {
//       int16_t *p_buffer = (int16_t *)p_evt->buffer_released;
        
//       // Print ONLY the raw samples, nothing else
//       for (int i = 0; i < PDM_BUFFER_SIZE; i++)
//       {
//           SEGGER_RTT_printf(0, "%d\n", p_buffer[i]);
//       }
//   }

//   if (p_evt->buffer_requested)
//   {
//       m_buffer_idx = (m_buffer_idx + 1) % 2;
//       nrfx_pdm_buffer_set(m_pdm_buffer[m_buffer_idx], PDM_BUFFER_SIZE);
//   }
//}

///**
//* @brief Initialize PDM
//*/
//static void pdm_init(void)
//{
//   nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    
//   pdm_config.mode = NRF_PDM_MODE_MONO;
//   pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    
//   #ifdef NRF_PDM_FREQ_1280K
//       pdm_config.clock_freq = NRF_PDM_FREQ_1280K;
//   #else
//       pdm_config.clock_freq = NRF_PDM_FREQ_1032K;
//   #endif
    
//   pdm_config.gain_l = NRF_PDM_GAIN_DEFAULT;
//   pdm_config.gain_r = NRF_PDM_GAIN_DEFAULT;

//   nrfx_pdm_init(&pdm_config, pdm_event_handler);
//}

///**
//* @brief Main function
//*/
//int main(void)
//{
//   pdm_init();
//   nrfx_pdm_start();
    
//   // Continuous recording - no prints, just data
//   while (true)
//   {
//       nrf_delay_ms(100);
//   }
//}












////v1 with blink record the data
////#include <stdbool.h>
////#include <stdint.h>
////#include <string.h>
////#include "nrf.h"
////#include "nrf_gpio.h"
////#include "nrfx_pdm.h"
////#include "nrf_log.h"
////#include "nrf_log_ctrl.h"
////#include "nrf_log_default_backends.h"
////#include "nrf_delay.h"
////#include "app_error.h"

////// Custom LED pins
////#define LED_1   23  // P0.23
////#define LED_2   24  // P0.24
////#define LED_3   22  // P0.22

////// PDM Microphone pins
////#define PDM_CLK_PIN  7   // P0.07
////#define PDM_DIN_PIN  8   // P0.08

////// PDM buffer configuration
////#define PDM_BUFFER_SIZE 128

////static int16_t m_pdm_buffer[2][PDM_BUFFER_SIZE];
////static uint8_t m_buffer_idx = 0;

////// Statistics
////static uint32_t m_total_samples = 0;
////static uint32_t m_buffer_count = 0;

////// LED functions
////static void led_init(void)
////{
////    nrf_gpio_cfg_output(LED_1);
////    nrf_gpio_cfg_output(LED_2);
////    nrf_gpio_cfg_output(LED_3);
    
////    nrf_gpio_pin_set(LED_1);
////    nrf_gpio_pin_set(LED_2);
////    nrf_gpio_pin_set(LED_3);
////}

////static void led_indicate_recording(void)
////{
////    nrf_gpio_pin_toggle(LED_1);
////}

////static void led_indicate_data(void)
////{
////    nrf_gpio_pin_toggle(LED_2);
////}

///**
// * @brief PDM event handler - Continuous recording
// */
//static void pdm_event_handler(nrfx_pdm_evt_t const * p_evt)
//{
//    if (p_evt->error)
//    {
//        NRF_LOG_ERROR("PDM Error: 0x%08X", p_evt->error);
//        return;
//    }

//    if (p_evt->buffer_released != NULL)
//    {
//        led_indicate_data();
        
//        int16_t *p_buffer = (int16_t *)p_evt->buffer_released;
//        m_buffer_count++;
        
//        // Print buffer header
//        NRF_LOG_INFO("=== Buffer %d (Samples %d-%d) ===", 
//                     m_buffer_count,
//                     m_total_samples,
//                     m_total_samples + PDM_BUFFER_SIZE - 1);
        
//        // Print all samples in this buffer
//        for (int i = 0; i < PDM_BUFFER_SIZE; i++)
//        {
//            NRF_LOG_INFO("%d", p_buffer[i]);
//            m_total_samples++;
//        }
        
//        NRF_LOG_INFO(""); // Empty line for readability
//        NRF_LOG_FLUSH();
//    }

//    if (p_evt->buffer_requested)
//    {
//        m_buffer_idx = (m_buffer_idx + 1) % 2;
//        nrfx_pdm_buffer_set(m_pdm_buffer[m_buffer_idx], PDM_BUFFER_SIZE);
//    }
//}

///**
// * @brief Initialize PDM
// */
//static void pdm_init(void)
//{
//    nrfx_err_t err_code;

//    // Configure PDM
//    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    
//    pdm_config.mode = NRF_PDM_MODE_MONO;
//    pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    
//    #ifdef NRF_PDM_FREQ_1280K
//        pdm_config.clock_freq = NRF_PDM_FREQ_1280K;
//    #else
//        pdm_config.clock_freq = NRF_PDM_FREQ_1032K;
//    #endif
    
//    pdm_config.gain_l = NRF_PDM_GAIN_DEFAULT;
//    pdm_config.gain_r = NRF_PDM_GAIN_DEFAULT;

//    // Initialize PDM
//    err_code = nrfx_pdm_init(&pdm_config, pdm_event_handler);
//    APP_ERROR_CHECK(err_code);

//    NRF_LOG_INFO("PDM initialized successfully");
//    NRF_LOG_INFO("Sample rate: ~16000 Hz");
//    NRF_LOG_INFO("Buffer size: %d samples", PDM_BUFFER_SIZE);
//    NRF_LOG_INFO("Buffer interval: ~%d ms", (PDM_BUFFER_SIZE * 1000) / 16000);
//}

///**
// * @brief Initialize logging
// */
//static void log_init(void)
//{
//    ret_code_t err_code = NRF_LOG_INIT(NULL);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_DEFAULT_BACKENDS_INIT();
//}

///**
// * @brief Main function
// */
//int main(void)
//{
//    nrfx_err_t err_code;

//    led_init();
//    log_init();
    
//    NRF_LOG_INFO("========================================");
//    NRF_LOG_INFO("PDM Microphone - Continuous Recording");
//    NRF_LOG_INFO("========================================");
//    NRF_LOG_INFO("Recording will continue indefinitely");
//    NRF_LOG_INFO("Each buffer contains %d samples", PDM_BUFFER_SIZE);
//    NRF_LOG_INFO("LED 1: Recording indicator");
//    NRF_LOG_INFO("LED 2: Data received indicator");
//    NRF_LOG_INFO("");
//    NRF_LOG_INFO("Starting in 2 seconds...");
//    NRF_LOG_FLUSH();

//    nrf_delay_ms(2000);

//    pdm_init();

//    // Start PDM
//    err_code = nrfx_pdm_start();
//    APP_ERROR_CHECK(err_code);
    
//    NRF_LOG_INFO("Recording started!");
//    NRF_LOG_INFO("Data format: One sample per line (int16)");
//    NRF_LOG_INFO("========================================");
//    NRF_LOG_INFO("");
//    NRF_LOG_FLUSH();

//    uint32_t led_timer = 0;
//    while (true)
//    {
//        NRF_LOG_FLUSH();
        
//        // Toggle recording LED periodically
//        led_timer++;
//        if (led_timer >= 5)
//        {
//            led_indicate_recording();
//            led_timer = 0;
//        }
        
//        nrf_delay_ms(100);
//    }
//}