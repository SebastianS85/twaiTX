#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

// Your custom component header file
#include "twai_manager.h"

static const char *TAG = "MEB_DCDC";

// CAN bus pins (adjust to your hardware)
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

static twai_mgr_inst_t twai_node;

/* -------------------------------------------------------------------------- */
/*                           Constants and Variables                          */
/* -------------------------------------------------------------------------- */

#define TARGET_HV_HIGH 360.0f
#define TARGET_HV_LOW  350.0f
#define TARGET_LV_VOLTAGE 14.4f
#define RAW_LV (uint8_t)(((TARGET_LV_VOLTAGE - 10.6f) / 0.025f) + 0.5f) // ~152 (0x98)

// Arrays for the VW CRC algorithm (PDU constants)
const uint8_t HVK_01_PDU[16]  = {0xED, 0xD6, 0x96, 0x63, 0xA5, 0x12, 0xD5, 0x9A, 0x1E, 0x0D, 0x24, 0xCD, 0x8C, 0xA6, 0x2F, 0x41};
const uint8_t NVEM_10_PDU[16] = {0xD4, 0x22, 0xAD, 0x3F, 0x25, 0xAA, 0x62, 0x5B, 0xDC, 0x73, 0xED, 0xC3, 0x9A, 0x14, 0x2F, 0x3E};
const uint8_t BMS_20_PDU[16]  = {0xEE, 0x80, 0x6E, 0x4E, 0x29, 0xC6, 0x92, 0xC0, 0x65, 0xAA, 0x3A, 0xA1, 0x8F, 0xCD, 0xE6, 0x90};

// State machine phase types
typedef enum {
    PHASE_WAIT_STABLE,
    PHASE_REQUEST,
    PHASE_PRECHARGE,
    PHASE_HOLD
} precharge_phase_t;

// Global state variables (protected by a FreeRTOS Mutex)
static float hv_voltage = 0.0f;
static uint8_t dcdc_mode = 7;
static SemaphoreHandle_t hv_lock;

// RX queue structure for safe ISR-to-Task communication
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  len;
    bool     is_fd;
    uint8_t  data[64];
} rx_msg_t;

static QueueHandle_t rx_queue = NULL;

/* -------------------------------------------------------------------------- */
/*                             VW CRC Algorithm                               */
/* -------------------------------------------------------------------------- */

static uint8_t crc8h2f_continue(const uint8_t *data, size_t len, uint8_t crc, uint8_t final_xor) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x2F;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc ^ final_xor;
}

static uint8_t vw_crc(const uint8_t *data, size_t len, uint8_t pdu_const) {
    // Calculate CRC for the payload (excluding the first byte where the CRC is stored)
    uint8_t crc = crc8h2f_continue(data + 1, len - 1, 0xFF, 0x00);
    // Add the PDU constant to the CRC calculation at the end
    return crc8h2f_continue(&pdu_const, 1, crc, 0xFF);
}

/* -------------------------------------------------------------------------- */
/*                               Frame Builders                               */
/* -------------------------------------------------------------------------- */

static void build_20f(uint8_t counter, uint8_t *out) {
    memset(out, 0, 8);
    out[1] = counter & 0x0F;
    out[4] = 0x40;
    out[6] = 0xFD;
    out[7] = RAW_LV;
    out[0] = vw_crc(out, 8, NVEM_10_PDU[counter & 0x0F]);
}

static void build_503(uint8_t counter, precharge_phase_t phase, uint8_t *out) {
    memset(out, 0, 8);
    if (phase == PHASE_PRECHARGE) {
        out[1] = 0xE0 | (counter & 0x0F);
        out[3] = 0xC8;
    } else if (phase == PHASE_REQUEST) {
        out[1] = 0x60 | (counter & 0x0F);
        out[3] = 0xC8;
    } else { // STANDBY / WAIT_STABLE / HOLD
        out[1] = 0x60 | (counter & 0x0F);
        out[3] = 0xC0;
    }
    out[4] = 0x0F;
    out[6] = 0x60;
    out[0] = vw_crc(out, 8, HVK_01_PDU[counter & 0x0F]);
}

static void build_cf(uint8_t counter, float pack_voltage, float zk_voltage, uint8_t *out) {
    memset(out, 0, 8);
    uint16_t pack_raw = (uint16_t)(pack_voltage / 0.25f);
    uint16_t zk_raw = (uint16_t)((zk_voltage + 1000.0f) / 0.5f);
    
    out[1] = counter & 0x0F;
    out[2] = 0x07;
    out[5] = zk_raw & 0xFF;
    out[6] = ((zk_raw >> 8) & 0x0F) | ((pack_raw & 0x0F) << 4);
    out[7] = (pack_raw >> 4) & 0xFF;
    out[0] = vw_crc(out, 8, BMS_20_PDU[counter & 0x0F]);
}

/**
 * @brief Helper function to dynamically map payload length to CAN-FD DLC
 *        and asynchronously send the frame using the twai_manager component.
 */
static void send_can_frame(twai_mgr_inst_t *inst, uint32_t id, const uint8_t *data, size_t len) {
    uint8_t computed_dlc = 0;
    // Automatically find the correct DLC for the given payload length
    while (computed_dlc < 15 && twai_mgr_dlc_to_len(computed_dlc) < len) computed_dlc++;
    computed_dlc = twai_mgr_bound_dlc(computed_dlc); // Safety cap

    twai_frame_t frame = {0};
    frame.header.id  = id;
    frame.header.ide = 0; // Standard 11-bit ID
    frame.header.fdf = 1; // FD Format
    frame.header.brs = 1; // Bit Rate Switch (Accelerated Data Phase)
    frame.header.esi = 0;
    frame.header.dlc = computed_dlc;
    frame.buffer = (uint8_t *)data; // Cast to non-const for the driver
    frame.buffer_len = twai_mgr_dlc_to_len(computed_dlc); // e.g. maps 34 bytes to 48 wire bytes

    twai_mgr_async_transmit(inst, &frame, pdMS_TO_TICKS(10));
}

/* -------------------------------------------------------------------------- */
/*                              Callback and RX                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Hardware ISR callback triggered by twai_manager when a frame is received.
 */
static void app_can_rx_cb(uint8_t node_id, const twai_frame_t *frame, void *user_ctx) {
    if (!rx_queue) return;
    
    rx_msg_t msg = {0};
    msg.id    = frame->header.id;
    msg.dlc   = frame->header.dlc;
    msg.len   = twai_mgr_dlc_to_len(frame->header.dlc);
   

    // Safely copy payload (the source buffer will be destroyed upon ISR exit)
    if (msg.len > 0 && frame->buffer != NULL) {
        memcpy(msg.data, frame->buffer, (msg.len > 64) ? 64 : msg.len);
    }
    
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &msg, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * @brief Task for decoding specific CAN signals and updating the global state.
 */
static void can_rx_task(void *pvParameters) {
    rx_msg_t msg;
    while (1) {
        if (xQueueReceive(rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            // Decode High Voltage (0x2AE)
            if (msg.id == 0x2AE && msg.len >= 4) {
                uint16_t raw_v = ((msg.data[1] & 0xF0) >> 4) | (msg.data[2] << 4);
                
                xSemaphoreTake(hv_lock, portMAX_DELAY);
                hv_voltage = raw_v * 0.25f;
                xSemaphoreGive(hv_lock);
            }
            // Decode DC-DC Mode (0x0F7)
            else if (msg.id == 0x0F7 && msg.len >= 6) {
                uint8_t mode = (msg.data[5] >> 1) & 0x07;
                
                xSemaphoreTake(hv_lock, portMAX_DELAY);
                dcdc_mode = mode;
                xSemaphoreGive(hv_lock);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                       Main Task (TX & State Machine)                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Monitors bus health and attempts recovery upon a Bus-Off event.
 */
static void can_health_task(void *pvParameters) {
    twai_mgr_inst_t *inst = (twai_mgr_inst_t *)pvParameters;
    while (1) {
        twai_mgr_health_monitor(inst);
        vTaskDelay(pdMS_TO_TICKS(100)); // Run check every 100ms
    }
}

/**
 * @brief Main logic handling the Precharge Phase and CAN transmissions.
 */
static void meb_dcdc_task(void *pvParameters) {
    twai_mgr_inst_t *inst = (twai_mgr_inst_t *)pvParameters;

    ESP_LOGI(TAG, "Precharge band: %.0f-%.0f V", TARGET_HV_LOW, TARGET_HV_HIGH);

    TickType_t now;
    TickType_t next_20f = xTaskGetTickCount();
    TickType_t next_503 = next_20f;
    TickType_t next_271 = next_20f;
    TickType_t next_cf  = next_20f;
    TickType_t next_5bb = next_20f;
    TickType_t last_print = next_20f;

    uint8_t ctr_20f = 0, ctr_503 = 0, ctr_cf = 0;
    precharge_phase_t phase = PHASE_WAIT_STABLE;
    uint32_t stable_count = 0;

    // Decoded data payload for ID 0x271 (34 bytes original)
    // The send_can_frame function will automatically pad this to 48 bytes (DLC 14).
    uint8_t data_271[34] = {
        0x00,0x00,0x00,0x00,0x24,0x00,0x1c,0xfd,0x8f,0x00,0x00,0x00,0x00,0x00,0x00,0xf5,
        0x00,0x00,0x8a,0xbb,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x07,0x00
    };
    uint8_t data_5bb[8] = {0};
    uint8_t buffer[8];

    while (1) {
        now = xTaskGetTickCount();

        // 1. Safely read current values (local copies for this iteration loop)
        xSemaphoreTake(hv_lock, portMAX_DELAY);
        float v = hv_voltage;
        uint8_t mode = dcdc_mode;
        xSemaphoreGive(hv_lock);

        // 2. State Machine Logic
        if (phase == PHASE_WAIT_STABLE && v > 0.5f && v < 10.0f) {
            stable_count++;
            // Assuming this loop runs roughly every 10ms, 5 counts = ~50ms of stability
            if (stable_count >= 5) {
                ESP_LOGW(TAG, "HV stable at %.1f V - requesting precharge", v);
                phase = PHASE_REQUEST;
            }
        } else if (phase == PHASE_WAIT_STABLE) {
            stable_count = 0;
        }
        
        if (phase == PHASE_REQUEST && mode == 1) {
            ESP_LOGW(TAG, "DC-DC confirmed precharge mode - activating");
            phase = PHASE_PRECHARGE;
        }
        
        if (phase == PHASE_PRECHARGE && v >= TARGET_HV_HIGH && v < 1000.0f) {
            ESP_LOGW(TAG, "Target reached: %.1f V - switching to standby", v);
            phase = PHASE_HOLD;
        }
        
        if (phase == PHASE_HOLD && v < TARGET_HV_LOW) {
            ESP_LOGW(TAG, "Voltage dropped to %.1f V - resuming precharge", v);
            phase = PHASE_REQUEST;
        }

        // Print diagnostic logs every 1 second
        if ((now - last_print) >= pdMS_TO_TICKS(1000)) {
            const char* mode_name = (mode == 0) ? "Standby" : (mode == 1) ? "Precharge" : 
                                    (mode == 2) ? "Buck" : (mode == 5) ? "Error" : 
                                    (mode == 7) ? "Init" : "Unknown";
            const char* phase_name = (phase == PHASE_WAIT_STABLE) ? "WAIT_STABLE" : 
                                     (phase == PHASE_REQUEST) ? "REQUEST" : 
                                     (phase == PHASE_PRECHARGE) ? "PRECHARGE" : "HOLD";
                                     
            ESP_LOGI(TAG, "  HV: %.1f V  Mode: %s  Phase: %s", v, mode_name, phase_name);
            last_print = now;
        }

        // 3. Transmit frames based on configured time intervals
        if (now >= next_20f) {
            build_20f(ctr_20f, buffer);
            send_can_frame(inst, 0x20F, buffer, 8);
            ctr_20f = (ctr_20f + 1) & 0x0F;
            next_20f += pdMS_TO_TICKS(50);
        }

        if (now >= next_503) {
            build_503(ctr_503, phase, buffer);
            send_can_frame(inst, 0x503, buffer, 8);
            ctr_503 = (ctr_503 + 1) & 0x0F;
            next_503 += pdMS_TO_TICKS(100);
        }

        if (now >= next_271) {
            send_can_frame(inst, 0x271, data_271, sizeof(data_271));
            next_271 += pdMS_TO_TICKS(100);
        }

        if (now >= next_cf) {
            build_cf(ctr_cf, TARGET_HV_HIGH, v, buffer);
            send_can_frame(inst, 0x0CF, buffer, 8);
            ctr_cf = (ctr_cf + 1) & 0x0F;
            next_cf += pdMS_TO_TICKS(100);
        }

        if (now >= next_5bb) {
            send_can_frame(inst, 0x5BB, data_5bb, 8);
            next_5bb += pdMS_TO_TICKS(500);
        }

        // Delay to yield to FreeRTOS IDLE task, Watchdog, and lower priority tasks
        // Delaying for at least 1 tick (usually 10ms) prevents Task Watchdog Triggers (TWDT)
        vTaskDelay(pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1);
    }
}

/* -------------------------------------------------------------------------- */
/*                                Entry Point                                 */
/* -------------------------------------------------------------------------- */

void app_main(void) {
    ESP_LOGI(TAG, "Starting MEB DC-DC Precharge Controller...");

    hv_lock = xSemaphoreCreateMutex();
    rx_queue = xQueueCreate(32, sizeof(rx_msg_t));

    memset(&twai_node, 0, sizeof(twai_mgr_inst_t));
    
    // Initialize the TWAI node via your custom manager wrapper
    esp_err_t err = twai_mgr_init_custom_node(
        &twai_node, CAN_TX_PIN, CAN_RX_PIN, TWAI_BUS_MODE_FD,
        500000, 2000000, 32, false, NULL, app_can_rx_cb
    );
    ESP_ERROR_CHECK(err);

    // Pinned to core 0 (Required for single-core RISC-V targets like ESP32-C5)
    xTaskCreatePinnedToCore(can_health_task, "health_task", 3072, &twai_node, 5, NULL, 0);
    xTaskCreatePinnedToCore(can_rx_task,     "rx_task",     4096, NULL,       6, NULL, 0);
    xTaskCreatePinnedToCore(meb_dcdc_task,   "dcdc_task",   4096, &twai_node, 5, NULL, 0);
}