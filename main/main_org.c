#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

// Your component header file
#include "twai_manager.h"

static const char *TAG = "app_main";

// CAN bus pins (adjust to your hardware)
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

// Global TWAI manager instance
static twai_mgr_inst_t twai_node;

/* -------------------------------------------------------------------------- */
/*                                RX Queue                                    */
/* -------------------------------------------------------------------------- */

// Safe structure for passing data from the ISR to the task
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  len;
    bool     is_fd;
    uint8_t  data[64];
} rx_msg_t;

static QueueHandle_t rx_queue = NULL;

/* -------------------------------------------------------------------------- */
/*                                RX Callback                                 */
/* -------------------------------------------------------------------------- */

/**
 * Called from the hardware interrupt (ISR) context by your twai_manager.
 */
static void app_can_rx_cb(uint8_t node_id, const twai_frame_t *frame, void *user_ctx) {
    if (!rx_queue) return;

    rx_msg_t msg = {0};
    
    // Using the new ESP-IDF v6 API (frame->header...)
    msg.id    = frame->header.id;
    msg.dlc   = frame->header.dlc;
    msg.len   = twai_mgr_dlc_to_len(frame->header.dlc); // Your function!
   

    // Safely copy bytes (source buffer will be destroyed after exiting the ISR)
    if (msg.len > 0 && frame->buffer != NULL) {
        size_t copy_len = (msg.len > 64) ? 64 : msg.len;
        memcpy(msg.data, frame->buffer, copy_len);
    }

    BaseType_t high_task_woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &msg, &high_task_woken);
    portYIELD_FROM_ISR(high_task_woken);
}

/* -------------------------------------------------------------------------- */
/*                              FreeRTOS Tasks                                */
/* -------------------------------------------------------------------------- */

/**
 * Task monitoring errors (e.g., bus shorts) and recovering TWAI from Bus-Off
 */
static void can_health_task(void *pvParameters) {
    twai_mgr_inst_t *inst = (twai_mgr_inst_t *)pvParameters;
    while (1) {
        twai_mgr_health_monitor(inst);
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100 ms
    }
}

/**
 * Task receiving and displaying packets from the queue
 */
static void can_rx_task(void *pvParameters) {
    rx_msg_t msg;
    while (1) {
        // Wait for a packet from the ISR
        if (xQueueReceive(rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "RX -> ID: 0x%03" PRIx32 " | DLC: %u | Bytes: %u | FD: %d", 
                     msg.id, msg.dlc, msg.len, msg.is_fd);
            
            // Print received data in HEX format
            if (msg.len > 0) {
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, msg.data, msg.len, ESP_LOG_INFO);
            }
        }
    }
}

/**
 * Task transmitting your 34-byte frame with automatic DLC calculation
 */
static void can_tx_task(void *pvParameters) {
    twai_mgr_inst_t *inst = (twai_mgr_inst_t *)pvParameters;

    // Your payload (34 bytes)
    uint8_t tx_data[] = {
        0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x1c, 0xfd, 
        0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf5, 
        0x00, 0x00, 0x8a, 0xbb, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 
        0x07, 0x00
    };

    while (1) {
        
        uint8_t computed_dlc = 0;
        while (computed_dlc < 15 && twai_mgr_dlc_to_len(computed_dlc) < sizeof(tx_data)) {
            computed_dlc++;
        }
        computed_dlc = twai_mgr_bound_dlc(computed_dlc); // Safe upper limit

        // 2. Zero out the entire frame structure
        twai_frame_t frame = {0};

        // 3. Configure the header (ESP-IDF v6)
        frame.header.id  = 0x271;               // Target ID (change as needed)
        frame.header.ide = 0;                   // 11-bit standard ID
        frame.header.fdf = 1;                   // CAN-FD frame
        frame.header.brs = 1;                   // Bit Rate Switch (faster data phase)
        frame.header.esi = 0;
        frame.header.dlc = computed_dlc;        // Sets the correct DLC (here: 14)

        // 4. Assign the buffer pointer and the physical length in bytes
        frame.buffer = tx_data;
        frame.buffer_len = twai_mgr_dlc_to_len(computed_dlc); // Will return 48 bytes

        // 5. Transmit asynchronously via manager
        esp_err_t err = twai_mgr_async_transmit(inst, &frame, pdMS_TO_TICKS(50));
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "TX OK -> Frame sent (ID: 0x%03" PRIx32 " DLC: %u)", 
                     frame.header.id, computed_dlc);
        } else {
            ESP_LOGW(TAG, "TX ERROR -> %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Loop every 1 second
    }
}

/* -------------------------------------------------------------------------- */
/*                                Entry Point                                 */
/* -------------------------------------------------------------------------- */

void app_main(void) {
    ESP_LOGI(TAG, "System start. Initializing TWAI for ESP32-C5...");

    // Create a queue that can hold a maximum of 16 received frames
    rx_queue = xQueueCreate(16, sizeof(rx_msg_t));
    if (rx_queue == NULL) {
        ESP_LOGE(TAG, "Error: Out of memory for RX queue!");
        return;
    }

    memset(&twai_node, 0, sizeof(twai_mgr_inst_t));
    
    // Your initialization function! We pass app_can_rx_cb as the RX event handler
    esp_err_t err = twai_mgr_init_custom_node(
        &twai_node,
        CAN_TX_PIN,                     // tx_io
        CAN_RX_PIN,                     // rx_io
        TWAI_BUS_MODE_FD,               // mode = CAN-FD
        500000,                         // arb_bitrate = 500 kbps
        2000000,                        // data_bitrate = 2 Mbps
        16,                             // tx_queue_depth
        false,                          // listen_only = false
        (void *)(uintptr_t)0,           // user_ctx
        app_can_rx_cb                   // Pointer to the RX callback!
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Critical TWAI initialization error: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Driver started. Pinning tasks to core 0...");

    // For ESP32-C5 (single-core RISC-V), we strictly assign tasks to core '0'
    xTaskCreatePinnedToCore(can_health_task, "health", 3072, &twai_node, 5, NULL, 0);
    xTaskCreatePinnedToCore(can_rx_task,     "rx_task",4096, NULL,       6, NULL, 0); // RX has a slightly higher priority (6)
    xTaskCreatePinnedToCore(can_tx_task,     "tx_task",4096, &twai_node, 5, NULL, 0);
}