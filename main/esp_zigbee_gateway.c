/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Zigbee Gateway Example
 */
#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"     /* For WiFi event group */
#include "driver/usb_serial_jtag.h"
#include "esp_coexist.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_zigbee_gateway.h"
#include "zb_config_platform.h"
#include "esp_phy_init.h"
#include "driver/gpio.h"
#include "esp_http_server.h" /* For Web Server */
#include "secrets.h"         /* WiFi Credentials (excluded from git) */

/* =================================
 *  WIFI CONFIGURATION
 * ================================= */
#define MAX_RETRY          5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

/* Neighbor data storage for web display */
#define MAX_NEIGHBOR_DISPLAY 20
typedef struct {
    uint16_t short_addr;
    uint8_t device_type;
    int8_t rssi;
    uint8_t lqi;
    bool active;
} neighbor_display_t;

static neighbor_display_t s_neighbors[MAX_NEIGHBOR_DISPLAY];
static int s_neighbor_count = 0;
static SemaphoreHandle_t s_neighbor_mutex;
static char s_connection_status[64] = "System Starting... (Waiting for Zigbee)";

static const char *TAG = "ESP_ZB_ROUTER";


/* =================================
 *  WIFI FUNCTIONS
 * ================================= */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* SCAN DIAGNOSTIC */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true
    };
    ESP_LOGW(TAG, "Starting WiFi Scan...");
    esp_wifi_scan_start(&scan_config, true);
    
    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(ap_num * sizeof(wifi_ap_record_t));
    esp_wifi_scan_get_ap_records(&ap_num, ap_list);
    
    ESP_LOGW(TAG, "----------------------------------------------------------------");
    ESP_LOGW(TAG, "Scanned %d Access Points:", ap_num);
    for(int i = 0; i < ap_num; i++) {
        ESP_LOGW(TAG, "SSID: %-32s | RSSI: %d | Auth: %d", ap_list[i].ssid, ap_list[i].rssi, ap_list[i].authmode);
    }
    ESP_LOGW(TAG, "----------------------------------------------------------------");
    free(ap_list);
    /* END SCAN DIAGNOSTIC */

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN, // Accept any security level (Debug)
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    // Start is already called above, but calling it again or connect is fine.
    esp_wifi_connect();

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* =================================
 *  WEB SERVER FUNCTIONS
 * ================================= */
static const char *get_device_type_str(uint8_t type) {
    switch (type) {
    case ESP_ZB_DEVICE_TYPE_COORDINATOR: return "Coord";
    case ESP_ZB_DEVICE_TYPE_ROUTER: return "Router";
    case ESP_ZB_DEVICE_TYPE_ED: return "EndDev";
    default: return "Unkwn";
    }
}

/* Handler for the root URL */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    char resp_buf[2048]; // Buffer large enough for HTML table
    int len = 0;

    len += snprintf(resp_buf + len, sizeof(resp_buf) - len, 
        "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='3'><meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>body{font-family:Arial,sans-serif;text-align:center;}table{width:90%%;margin:20px auto;border-collapse:collapse;}"
        "th,td{border:1px solid #ddd;padding:8px;}th{background-color:#f2f2f2;}tr:nth-child(even){background-color:#f9f9f9;}"
        ".good{color:green;font-weight:bold;}.bad{color:red;}</style>"
        "<title>Zigbee Monitor</title></head><body>"
        "<h1>Zigbee Analyzer</h1>"
        "<p><strong>Status:</strong> %s</p>"
        "<p>Auto-refresh: 3s</p>"
        "<table><tr><th>Addr</th><th>Type</th><th>RSSI (dBm)</th><th>LQI</th></tr>", s_connection_status);

    if (xSemaphoreTake(s_neighbor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_neighbor_count == 0) {
            len += snprintf(resp_buf + len, sizeof(resp_buf) - len, "<tr><td colspan='4'>No neighbors found...</td></tr>");
        } else {
            for (int i = 0; i < s_neighbor_count; i++) {
                // Color code for signal strength
                const char* signal_class = (s_neighbors[i].rssi > -70) ? "good" : (s_neighbors[i].rssi < -85 ? "bad" : "");
                
                len += snprintf(resp_buf + len, sizeof(resp_buf) - len, 
                    "<tr><td>0x%04X</td><td>%s</td><td class='%s'>%d</td><td>%d</td></tr>",
                    s_neighbors[i].short_addr,
                    get_device_type_str(s_neighbors[i].device_type),
                    signal_class,
                    s_neighbors[i].rssi,
                    s_neighbors[i].lqi);
            }
        }
        xSemaphoreGive(s_neighbor_mutex);
    }

    len += snprintf(resp_buf + len, sizeof(resp_buf) - len, "</table></body></html>");
    httpd_resp_send(req, resp_buf, len);
    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Affiche l'URL en gros pour l'utilisateur
    ESP_LOGE(TAG, "==================================================");
    ESP_LOGE(TAG, "WEB INTERFACE READY!");
    ESP_LOGE(TAG, "Open this URL in your browser: http://<YOUR_IP>/");
    ESP_LOGE(TAG, "Wait to see 'got ip: ...' in logs just above");
    ESP_LOGE(TAG, "==================================================");

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

/* Note: Please select the correct console output port based on the development board in menuconfig */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
esp_err_t esp_zb_gateway_console_init(void)
{
    esp_err_t ret = ESP_OK;
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    usb_serial_jtag_vfs_use_driver();
    uart_vfs_dev_register();
    return ret;
}
#endif

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        
        // --- COEXISTENCE INIT SEQUENCE ---
        snprintf(s_connection_status, sizeof(s_connection_status), "Initializing WiFi...");
        
        // 1. Connect/Init WiFi (Includes Scan)
        wifi_init_sta();
        start_webserver();

        // 2. Enable Coexistence Explicitly
        #if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
        ESP_LOGI(TAG, "Enabling Software Coexistence...");
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // Required for Coex
        esp_coex_wifi_i154_enable();
        #endif

        snprintf(s_connection_status, sizeof(s_connection_status), "Zigbee Stack Initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee_address;
            esp_zb_get_long_address(ieee_address);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
                     ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            snprintf(s_connection_status, sizeof(s_connection_status), "Connected! (Chan: %d)", esp_zb_get_current_channel());
        } else {
            ESP_LOGI(TAG, "Network steering failed, restart (status: %s)", esp_err_to_name(err_status));
            snprintf(s_connection_status, sizeof(s_connection_status), "Scanning... (Attempting to Join)");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        ESP_LOGI(TAG, "Production configuration is %s", err_status == ESP_OK ? "ready" : "not present");
        esp_zb_set_node_descriptor_manufacturer_code(ESP_MANUFACTURER_CODE);
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}


static void print_neighbor_signal_rssi(uint8_t param)
{
    static int scan_count = 0;
    esp_zb_nwk_info_iterator_t iterator = ESP_ZB_NWK_INFO_ITERATOR_INIT;
    esp_zb_nwk_neighbor_info_t neighbor;

    if (xSemaphoreTake(s_neighbor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        
        // Reset list before new scan - only reset the count, data will be overwritten
        s_neighbor_count = 0;

        ESP_LOGW(TAG, "========== Scan Voisins #%d (RSSI > -90 dBm) ==========", ++scan_count);
        while (esp_zb_nwk_get_next_neighbor(&iterator, &neighbor) == ESP_OK) {
            // Filter to keep only relevant signals
            if (neighbor.rssi > -90 || neighbor.short_addr == 0x0000) {
                 
                 // Add to Web Display Structure safely
                 if (s_neighbor_count < MAX_NEIGHBOR_DISPLAY) {
                     s_neighbors[s_neighbor_count].short_addr = neighbor.short_addr;
                     s_neighbors[s_neighbor_count].device_type = neighbor.device_type;
                     s_neighbors[s_neighbor_count].rssi = neighbor.rssi;
                     s_neighbors[s_neighbor_count].lqi = neighbor.lqi;
                     
                     s_neighbor_count++;
                 }

                 ESP_LOGI(TAG, "Addr: 0x%04x (%s) | RSSI: %d dBm | LQI: %d", 
                    neighbor.short_addr, 
                    get_device_type_str(neighbor.device_type), 
                    neighbor.rssi, 
                    neighbor.lqi);
            }
        }
        ESP_LOGW(TAG, "=======================================================");
        xSemaphoreGive(s_neighbor_mutex);
    }

    /* Schedule next check in 3 seconds */
    esp_zb_scheduler_alarm((esp_zb_callback_t)print_neighbor_signal_rssi, 0, 3000);
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    // Routers can scan all channels. Set both primary and secondary to all channels to be sure.
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_set_secondary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    
    // Set max transmit power (20dBm) - Useful for external antenna
    esp_zb_set_tx_power(20);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ESP_ZB_GATEWAY_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_attribute_list_t *basic_cluser = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluser, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Start the periodic signal monitoring */
    esp_zb_scheduler_alarm((esp_zb_callback_t)print_neighbor_signal_rssi, 0, 3000);

    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Configure GPIOs for External Antenna on Seeed Studio XIAO ESP32C6
    // Documentation: https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/
    // GPIO 3 (Output, Low)  = Enable RF Switch Control
    // GPIO 14 (Output, High) = Select External U.FL Antenna
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_14);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    gpio_set_level(GPIO_NUM_3, 0);  // Enable RF switch (Active LOW according to Seeed Wiki)
    gpio_set_level(GPIO_NUM_14, 1); // Select External Antenna (HIGH)

    // CHANGE: Move platform config AFTER system init to avoid radio conflicts during WiFi init
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize WiFi & WebServer */
    s_neighbor_mutex = xSemaphoreCreateMutex();
    
    // Wifi Init moved to Zigbee Task (Signal Handler) for correct Coexistence timing
    // wifi_init_sta();
    // start_webserver();

    // Give WiFi a moment to settle before starting Zigbee Radio
    // usage of radio by Zigbee and WiFi is managed by Software Coexistence (enabled in sdkconfig)
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Now configure Zigbee Platform (Radio)
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
