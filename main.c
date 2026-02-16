
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

/**
 * @brief Approximate 20*log10(rms) using integer math.
 *        Uses log2 via leading-zero count, then converts: dB ~ 6.0206 * log2(x)
 *        Returns dB in fixed-point with 1 decimal place (e.g. 723 = 72.3 dB).
 *        Result is dBFS (relative to int16 full-scale = 32768).
 */
static int32_t rms_to_dbfs(uint32_t rms)
{
    if (rms == 0) return -9999; // represents -inf

    // 20*log10(rms/32768) = 20*log10(rms) - 20*log10(32768)
    // 20*log10(32768) = 90.31 dB (we use 903 in fixed-point x10)
    // 20*log10(x) = 20 * log2(x) / log2(10) = 20 * log2(x) * 0.30103
    //             = 6.0206 * log2(x)
    // In fixed-point x10: 60.206 * log2(x), approximate as 60 * log2(x)

    // Find integer part of log2 using CLZ
    uint32_t log2_int = 0;
    uint32_t temp = rms;
    while (temp > 1)
    {
        temp >>= 1;
        log2_int++;
    }

    // Fractional part: linear interpolation between powers of 2
    // frac = (rms - 2^log2_int) / 2^log2_int, scaled to 0-1000
    uint32_t pow2 = 1UL << log2_int;
    uint32_t frac_1000 = ((rms - pow2) * 1000UL) / pow2;

    // 20*log10(rms) in x10 fixed point = 60 * (log2_int + frac/1000)
    //                                   = 60*log2_int + 60*frac/1000
    int32_t db_x10 = (int32_t)(60 * log2_int + (60 * frac_1000) / 1000);

    // Subtract 20*log10(32768) * 10 = 903
    db_x10 -= 903;

    return db_x10;
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

        // Compute sum of squares for RMS
        uint64_t sum_sq = 0;
        for (int i = 0; i < PDM_BUFFER_SIZE; i++)
        {
            int32_t sample = (int32_t)p_buf[i];
            sum_sq += (uint64_t)(sample * sample);
        }

        uint32_t mean_sq = (uint32_t)(sum_sq / PDM_BUFFER_SIZE);
        uint32_t rms = isqrt(mean_sq);

        int32_t db = rms_to_dbfs(rms);

        int len;
        if (db <= -9999)
        {
            len = snprintf(m_uart_text, sizeof(m_uart_text), "dBFS: -inf\r\n");
        }
        else
        {
            // db is in x10 fixed point, e.g. -253 means -25.3 dBFS
            int32_t whole = db / 10;
            int32_t frac = db % 10;
            if (db < 0 && frac != 0)
            {
                whole -= 1;
                frac = 10 - (-frac);
            }
            if (frac < 0) frac = -frac;
            len = snprintf(m_uart_text, sizeof(m_uart_text), "dBFS: %d.%d\r\n", (int)whole, (int)frac);
        }

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
    const char startup_msg[] = "PDM Mic dB Level Monitor\r\n";
    uart_send((uint8_t*)startup_msg, strlen(startup_msg));
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



