/*
 * ESP32 Controller Firmware (ESP-IDF 5.5.2)
 * ==========================================
 * Role: Bridge between PC (USB/UART) and Drone ESP32 (ESP-NOW wireless)
 *
 * Data flow:
 *   PC  ──UART──▶  Controller ESP32  ──ESP-NOW──▶  Drone ESP32
 *   PC  ◀──UART──  Controller ESP32  ◀──ESP-NOW──  Drone ESP32 (telemetry)
 *
 * BEFORE FLASHING:
 *   1. Flash the drone firmware first, note its MAC address from serial monitor
 *   2. Replace DRONE_MAC below with that address
 *   3. Flash this firmware to the controller ESP32
 *
 * HARDWARE:
 *   - Any ESP32 DevKit connected to PC via USB
 *   - UART0 is shared between IDF logging and binary data to PC.
 *     Logging is suppressed after startup so binary protocol is clean.
 */

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "CTRL";

/* =========================================================
 *  CONFIGURATION
 * ========================================================= */

/* Wi-Fi channel — MUST match on both controller and drone */
#define ESPNOW_CHANNEL       1

/* Set to 1 to use Espressif Long-Range mode (~500-800 m).
 * Both devices MUST use the same setting. */
#define USE_LONG_RANGE       1

/* How long (ms) after boot to keep logs visible before
 * suppressing them for clean binary data to the PC script. */
#define LOG_SUPPRESS_DELAY_MS 3000

/* !!!  CHANGE THIS to your drone ESP32's MAC address  !!!
 * To find it: flash drone firmware, run `idf.py monitor`,
 * look for the line "Drone MAC: XX:XX:XX:XX:XX:XX"        */
static uint8_t drone_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* =========================================================
 *  PACKET DEFINITIONS
 *  __attribute__((packed)) ensures no padding between fields.
 *  Both ESP32s MUST use identical struct layouts.
 * ========================================================= */

#define MAGIC_CTRL  0xAB   /* PC -> controller -> drone */
#define MAGIC_TELE  0xCD   /* drone -> controller -> PC */

typedef struct __attribute__((packed)) {
    uint8_t  magic;       /* MAGIC_CTRL = 0xAB                    */
    uint16_t throttle;    /* 1000 (idle) … 2000 (full)            */
    int16_t  roll;        /* -500 (left)  … +500 (right)          */
    int16_t  pitch;       /* -500 (back)  … +500 (forward)        */
    int16_t  yaw;         /* -500 (CCW)   … +500 (CW)             */
    uint8_t  armed;       /* 0 = disarmed, 1 = armed              */
    uint8_t  checksum;    /* XOR of all preceding bytes           */
} control_packet_t;       /* total: 11 bytes                      */

typedef struct __attribute__((packed)) {
    uint8_t  magic;       /* MAGIC_TELE = 0xCD                    */
    uint16_t voltage_mv;  /* battery millivolts                   */
    int16_t  roll_deg;    /* attitude roll  × 10  (123 = 12.3°)   */
    int16_t  pitch_deg;   /* attitude pitch × 10                  */
    int16_t  yaw_deg;     /* attitude yaw   × 10                  */
    uint8_t  flags;       /* bit0=armed, bit1=failsafe            */
    uint8_t  checksum;
} telemetry_packet_t;     /* total: 11 bytes                      */

/* =========================================================
 *  GLOBALS
 * ========================================================= */

/* Queue between UART-RX task and ESP-NOW-TX task */
static QueueHandle_t send_queue;

/* Flag: once true, stop printing logs to UART0 */
static volatile bool logs_suppressed = false;

/* =========================================================
 *  CHECKSUM  —  XOR of all bytes
 * ========================================================= */
static uint8_t compute_checksum(const uint8_t *data, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++)
        cs ^= data[i];
    return cs;
}

/* =========================================================
 *  ESP-NOW CALLBACKS
 *  Called from the Wi-Fi driver task — keep them fast.
 * ========================================================= */

/* Called after we try to send a packet to the drone.
 * IDF 5.5.2 signature: (const esp_now_send_info_t *, esp_now_send_status_t) */
static void on_data_sent(const esp_now_send_info_t *tx_info,
                         esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        if (!logs_suppressed) {
            ESP_LOGW(TAG, "ESP-NOW send failed — drone out of range?");
        }
    }
}

/* Called when the drone sends telemetry back to us.
 * We validate it and forward the raw bytes to the PC over UART0. */
static void on_data_recv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int data_len)
{
    if (data_len != sizeof(telemetry_packet_t))
        return;

    const telemetry_packet_t *tele = (const telemetry_packet_t *)data;

    /* Verify magic byte */
    if (tele->magic != MAGIC_TELE)
        return;

    /* Verify checksum */
    uint8_t expected = compute_checksum(data, data_len - 1);
    if (tele->checksum != expected)
        return;

    /* Forward raw telemetry bytes to PC via UART0 */
    uart_write_bytes(UART_NUM_0, (const char *)data, data_len);
}

/* =========================================================
 *  UART RECEIVE TASK
 *  Reads control_packet_t from PC, validates, queues for TX.
 * ========================================================= */
static void uart_rx_task(void *arg)
{
    const size_t PKT_SIZE = sizeof(control_packet_t);
    uint8_t buf[PKT_SIZE];
    uint8_t byte;

    ESP_LOGI(TAG, "UART RX task started — waiting for packets from PC");

    while (1) {
        /* Strategy: scan for magic byte, then read the rest of the packet.
         * This handles framing recovery if bytes are dropped. */

        /* Step 1: find the magic byte */
        int len = uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(100));
        if (len != 1)
            continue;
        if (byte != MAGIC_CTRL)
            continue;  /* discard non-magic bytes (e.g. stray log text) */

        buf[0] = byte;

        /* Step 2: read remaining bytes of the packet */
        int remaining = PKT_SIZE - 1;
        int got = uart_read_bytes(UART_NUM_0, &buf[1], remaining,
                                  pdMS_TO_TICKS(50));
        if (got != remaining)
            continue;  /* timeout — partial packet, discard */

        control_packet_t *pkt = (control_packet_t *)buf;

        /* Checksum validation */
        uint8_t expected_cs = compute_checksum(buf, PKT_SIZE - 1);
        if (pkt->checksum != expected_cs)
            continue;  /* corrupted */

        /* Sanity-check values to catch garbled data */
        if (pkt->throttle < 900 || pkt->throttle > 2100)
            continue;
        if (pkt->armed > 1)
            continue;

        /* Enqueue for ESP-NOW TX — don't block if full (drop oldest) */
        if (xQueueSend(send_queue, buf, 0) != pdTRUE) {
            /* Queue full — OK, we prefer fresh data anyway */
        }
    }
}

/* =========================================================
 *  ESP-NOW SEND TASK
 *  Pulls packets from queue and sends them to drone.
 * ========================================================= */
static void espnow_tx_task(void *arg)
{
    uint8_t buf[sizeof(control_packet_t)];

    ESP_LOGI(TAG, "ESP-NOW TX task started");

    while (1) {
        if (xQueueReceive(send_queue, buf, portMAX_DELAY) == pdTRUE) {
            esp_err_t ret = esp_now_send(drone_mac, buf,
                                         sizeof(control_packet_t));
            if (ret != ESP_OK && !logs_suppressed) {
                ESP_LOGE(TAG, "esp_now_send error: %s",
                         esp_err_to_name(ret));
            }
        }
    }
}

/* =========================================================
 *  UART INIT
 * ========================================================= */
static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &cfg));
    /* 2 KB RX buffer, 256 B TX buffer */
    ESP_ERROR_CHECK(
        uart_driver_install(UART_NUM_0, 2048, 256, 0, NULL, 0));
}

/* =========================================================
 *  WI-FI + ESP-NOW INIT
 * ========================================================= */
static void wifi_and_espnow_init(void)
{
    /* NVS — Wi-Fi driver stores calibration data here */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

#if USE_LONG_RANGE
    /* Espressif Long Range — ~500-800 m, 512 Kbps.
     * BOTH devices must enable this. */
    ESP_ERROR_CHECK(
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));
    ESP_LOGI(TAG, "Long-Range (LR) mode enabled");
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    /* Print our MAC so the drone firmware can be configured */
    uint8_t my_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X      ║",
             my_mac[0], my_mac[1], my_mac[2],
             my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");

    /* --- ESP-NOW --- */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    /* Register drone as a known peer */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, drone_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW ready on channel %d", ESPNOW_CHANNEL);
    ESP_LOGI(TAG, "Target drone MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             drone_mac[0], drone_mac[1], drone_mac[2],
             drone_mac[3], drone_mac[4], drone_mac[5]);

    /* Warn if drone MAC is still the default broadcast address */
    bool all_ff = true;
    for (int i = 0; i < 6; i++) {
        if (drone_mac[i] != 0xFF) { all_ff = false; break; }
    }
    if (all_ff) {
        ESP_LOGW(TAG, "╔══════════════════════════════════════════════════╗");
        ESP_LOGW(TAG, "║  WARNING: drone_mac is FF:FF:FF:FF:FF:FF         ║");
        ESP_LOGW(TAG, "║  You MUST set it to your drone's actual MAC!     ║");
        ESP_LOGW(TAG, "║  Flash drone first → note its MAC → update here  ║");
        ESP_LOGW(TAG, "╚══════════════════════════════════════════════════╝");
    }
}

/* =========================================================
 *  app_main — ESP-IDF entry point
 * ========================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Controller ESP32 booting (IDF 5.5.2) ===");

    uart_init();
    wifi_and_espnow_init();

    /* Queue: depth 4, each slot = one control_packet_t.
     * Shallow queue ensures we always send the freshest data. */
    send_queue = xQueueCreate(4, sizeof(control_packet_t));
    if (send_queue == NULL) {
        ESP_LOGE(TAG, "FATAL: failed to create queue");
        return;
    }

    xTaskCreate(uart_rx_task,   "uart_rx",   4096, NULL, 5, NULL);
    xTaskCreate(espnow_tx_task, "espnow_tx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Controller ready.");
    ESP_LOGI(TAG, "Run:  python3 pc/controller_pc.py /dev/ttyUSB0");
    ESP_LOGI(TAG, "Logs will be suppressed in %d ms for clean binary data.",
             LOG_SUPPRESS_DELAY_MS);

    /* Give the user time to read the MAC address in the monitor,
     * then suppress all logging so UART0 carries only binary data. */
    vTaskDelay(pdMS_TO_TICKS(LOG_SUPPRESS_DELAY_MS));
    esp_log_level_set("*", ESP_LOG_NONE);
    logs_suppressed = true;

    /* app_main returns — FreeRTOS tasks keep running */
}