/*
 * CAMMI Senior Design - Wearable BLE Air Mouse Bring-Up Demo Firmware
 * Target: ESP32-S3-N16R8
 * Framework: ESP-IDF (C++ with NimBLE HID Mouse profile)
 *
 * Hardware Peripherals & Pinouts:
 *  1. IMU (MPU-6500) over Espressif SPI Master (SPI2_HOST):
 *     - DRDY INT: GPIO 9
 *     - nCS:      GPIO 10
 *     - MOSI/SDA: GPIO 11
 *     - SCLK/SCL: GPIO 12
 *     - MISO/AD0: GPIO 13
 *  2. Joystick & Mode Switch:
 *     - SW (Mode toggle: Active vs Typing Locked): GPIO 41 (Active-Low)
 *     - Y (Scroll): GPIO 1 (Pin 38 on ESP32-S3 DevKitC-1, ADC1 Channel 0)
 *  3. OLED Display (SSD1306 128x64 I2C):
 *     - SDA: GPIO 4
 *     - SCL: GPIO 5
 *     - Default I2C Address: 0x3C (auto-probes 0x3C / 0x3D)
 *  4. BLE HID Mouse:
 *     - Stack: Apache NimBLE
 *     - Appearance: 0x03C2 (Human Interface Device: Mouse)
 *     - Service: 0x1812 (HID) + 0x180A (DIS) + 0x180F (BAS)
 *     - Report: 4-byte standard mouse report [Buttons, dX, dY, Wheel]
 *
 * FreeRTOS Multi-Core Architecture:
 *  - Core 1:
 *      * imu_sensor_task (Prio 10): 500 Hz DRDY ISR notification, 6-DOF complementary filter, cursor deltas.
 *      * input_poll_task (Prio 6): 50 Hz button debounce & ADC1 joystick polling.
 *      * oled_ui_task    (Prio 2): 10 Hz SSD1306 framebuffer dashboard rendering.
 *  - Core 0:
 *      * ble_hid_tx_task (Prio 5): 100 Hz (10 ms) decoupled delta consumption, BLE HID report transmission, and serial telemetry.
 *      * NimBLE Host Task (Prio 9): NimBLE event loop.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_adc/adc_oneshot.h"

// NimBLE Host & Bluetooth Headers
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "CAMMI_DEMO";

/* =========================================================================
 * 1. PIN DEFINITIONS
 * ========================================================================= */
// MPU-6500 (SPI2_HOST)
#define PIN_IMU_INT         GPIO_NUM_9
#define PIN_IMU_NCS         GPIO_NUM_10
#define PIN_IMU_MOSI        GPIO_NUM_11
#define PIN_IMU_SCLK        GPIO_NUM_12
#define PIN_IMU_MISO        GPIO_NUM_13

// SSD1306 OLED (I2C Master)
#define PIN_OLED_SDA        GPIO_NUM_4
#define PIN_OLED_SCL        GPIO_NUM_5
#define OLED_I2C_ADDR       0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

// Mode switch & Joystick
#define PIN_JOY_SW          GPIO_NUM_41     // Mode toggle button (Active vs Typing Locked)
#define PIN_JOY_VCC_GATE    GPIO_NUM_2      // Joystick VCC switched rail (if power-gated)
#define JOY_ADC_UNIT        ADC_UNIT_1
#define JOY_ADC_CHANNEL     ADC_CHANNEL_0   // GPIO 1 (Pin 38 on DevKitC-1 header)

/* =========================================================================
 * 2. MPU-6500 REGISTER MAP
 * ========================================================================= */
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_INT_PIN_CFG     0x37
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_USER_CTRL       0x6A
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

#define MPU6500_READ_FLAG       0x80
#define MPU6500_EXPECTED_ID     0x70

/* =========================================================================
 * 3. SHARED SYSTEM STATE (Spinlock Decoupled IPC)
 * ========================================================================= */
typedef struct {
    float pitch_deg;
    float roll_deg;
    int32_t acc_dx;
    int32_t acc_dy;
    int8_t scroll_y;
    int raw_y;              // Live raw ADC reading
    bool active_mode;       // true = Air Mouse Active, false = Typing Locked
    uint8_t who_am_i;
    bool imu_online;
    uint32_t drdy_count;
} system_state_t;

static portMUX_TYPE s_state_spinlock = portMUX_INITIALIZER_UNLOCKED;
static system_state_t s_state = {
    .pitch_deg = 0.0f,
    .roll_deg = 0.0f,
    .acc_dx = 0,
    .acc_dy = 0,
    .scroll_y = 0,
    .raw_y = 2048,
    .active_mode = true,
    .who_am_i = 0x00,
    .imu_online = false,
    .drdy_count = 0
};

static spi_device_handle_t s_imu_spi = NULL;
static TaskHandle_t s_imu_task_handle = NULL;
static esp_lcd_panel_handle_t s_oled_panel = NULL;
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/* =========================================================================
 * 4. NIMBLE BLE HID MOUSE PROFILE & DRIVER
 * ========================================================================= */
#define BLE_HID_DEVICE_NAME     "CAMMI Air Mouse"
#define BLE_APPEARANCE_MOUSE    0x03C2

static uint8_t s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_hid_input_handle = 0;
static uint8_t s_protocol_mode = 0x01; // 1 = Report Protocol Mode

// Standard 4-byte USB/BLE HID Mouse Descriptor
static const uint8_t s_mouse_report_map[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xA1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xA1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs) ; padding
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xC0,                          //   END_COLLECTION
    0xC0                           // END_COLLECTION
};

static void ble_hid_advertise(void);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

// Device Information Service (0x180A) Access Callback
static int gatt_svr_chr_access_dis(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == 0x2A29) { // Manufacturer Name
        const char *mfg = "CAMMI Team";
        return os_mbuf_append(ctxt->om, mfg, strlen(mfg)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    } else if (uuid16 == 0x2A24) { // Model Number
        const char *model = "CAMMI-S3";
        return os_mbuf_append(ctxt->om, model, strlen(model)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    } else if (uuid16 == 0x2A50) { // PnP ID
        uint8_t pnp_id[7] = { 0x02, 0x5a, 0x05, 0x00, 0x01, 0x10, 0x01 };
        return os_mbuf_append(ctxt->om, pnp_id, sizeof(pnp_id)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// Battery Service (0x180F) Access Callback
static int gatt_svr_chr_access_bas(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t battery_level = 100; // 100% full for demo
    return os_mbuf_append(ctxt->om, &battery_level, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// HID Service (0x1812) Access Callback
static int gatt_svr_chr_access_hid(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid16 == 0x2A4B) { // Report Map
        return os_mbuf_append(ctxt->om, s_mouse_report_map, sizeof(s_mouse_report_map)) == 0
               ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    } else if (uuid16 == 0x2A4A) { // HID Information
        uint8_t hid_info[4] = { 0x11, 0x01, 0x00, 0x02 }; // bcdHID 1.11, country 0, remote wake
        return os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    } else if (uuid16 == 0x2A4E) { // Protocol Mode
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            return os_mbuf_append(ctxt->om, &s_protocol_mode, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            if (OS_MBUF_PKTLEN(ctxt->om) > 0) {
                os_mbuf_copydata(ctxt->om, 0, 1, &s_protocol_mode);
            }
            return 0;
        }
    } else if (uuid16 == 0x2A4C) { // HID Control Point
        return 0; // Accept writes without error
    } else if (uuid16 == 0x2A4D) { // Input Report
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint8_t dummy_report[4] = { 0, 0, 0, 0 };
            return os_mbuf_append(ctxt->om, dummy_report, sizeof(dummy_report)) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// Report Reference Descriptor (0x2908) Callback
static int gatt_svr_dsc_access_report_ref(uint16_t conn_handle, uint16_t attr_handle,
                                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        uint8_t report_ref[2] = { 0x00, 0x01 }; // Report ID = 0, Report Type = 1 (Input)
        return os_mbuf_append(ctxt->om, report_ref, sizeof(report_ref)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// Static UUID16 definitions (Lvalues compatible with C++ compiler)
static const ble_uuid16_t s_uuid_dis        = BLE_UUID16_INIT(0x180A);
static const ble_uuid16_t s_uuid_dis_mfg    = BLE_UUID16_INIT(0x2A29);
static const ble_uuid16_t s_uuid_dis_model  = BLE_UUID16_INIT(0x2A24);
static const ble_uuid16_t s_uuid_dis_pnp    = BLE_UUID16_INIT(0x2A50);

static const ble_uuid16_t s_uuid_bas        = BLE_UUID16_INIT(0x180F);
static const ble_uuid16_t s_uuid_bas_level  = BLE_UUID16_INIT(0x2A19);

static const ble_uuid16_t s_uuid_hid        = BLE_UUID16_INIT(0x1812);
static const ble_uuid16_t s_uuid_hid_proto  = BLE_UUID16_INIT(0x2A4E);
static const ble_uuid16_t s_uuid_hid_info   = BLE_UUID16_INIT(0x2A4A);
static const ble_uuid16_t s_uuid_hid_ctrl   = BLE_UUID16_INIT(0x2A4C);
static const ble_uuid16_t s_uuid_hid_map    = BLE_UUID16_INIT(0x2A4B);
static const ble_uuid16_t s_uuid_hid_report = BLE_UUID16_INIT(0x2A4D);
static const ble_uuid16_t s_uuid_report_ref = BLE_UUID16_INIT(0x2908);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

static struct ble_gatt_dsc_def s_hid_report_dscs[] = {
    {
        .uuid = &s_uuid_report_ref.u,
        .att_flags = BLE_ATT_F_READ,
        .min_key_size = 0,
        .access_cb = gatt_svr_dsc_access_report_ref,
        .arg = NULL,
    },
    { NULL, 0, 0, NULL, NULL }
};

static const struct ble_gatt_chr_def s_dis_chrs[] = {
    {
        .uuid = &s_uuid_dis_mfg.u,
        .access_cb = gatt_svr_chr_access_dis,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &s_uuid_dis_model.u,
        .access_cb = gatt_svr_chr_access_dis,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &s_uuid_dis_pnp.u,
        .access_cb = gatt_svr_chr_access_dis,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    { NULL, NULL, NULL, NULL, 0, 0, NULL, NULL }
};

static const struct ble_gatt_chr_def s_bas_chrs[] = {
    {
        .uuid = &s_uuid_bas_level.u,
        .access_cb = gatt_svr_chr_access_bas,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    { NULL, NULL, NULL, NULL, 0, 0, NULL, NULL }
};

static const struct ble_gatt_chr_def s_hid_chrs[] = {
    {
        .uuid = &s_uuid_hid_proto.u,
        .access_cb = gatt_svr_chr_access_hid,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &s_uuid_hid_info.u,
        .access_cb = gatt_svr_chr_access_hid,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &s_uuid_hid_ctrl.u,
        .access_cb = gatt_svr_chr_access_hid,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &s_uuid_hid_map.u,
        .access_cb = gatt_svr_chr_access_hid,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &s_uuid_hid_report.u,
        .access_cb = gatt_svr_chr_access_hid,
        .arg = NULL,
        .descriptors = s_hid_report_dscs,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC,
        .min_key_size = 0,
        .val_handle = &s_hid_input_handle,
        .cpfd = NULL,
    },
    { NULL, NULL, NULL, NULL, 0, 0, NULL, NULL }
};

// GATT Services Table
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    // 1. Device Information Service (0x180A)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_uuid_dis.u,
        .includes = NULL,
        .characteristics = s_dis_chrs,
    },
    // 2. Battery Service (0x180F)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_uuid_bas.u,
        .includes = NULL,
        .characteristics = s_bas_chrs,
    },
    // 3. Human Interface Device Service (0x1812)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_uuid_hid.u,
        .includes = NULL,
        .characteristics = s_hid_chrs,
    },
    { 0, NULL, NULL, NULL }
};

#pragma GCC diagnostic pop

// Send standard 4-byte mouse notification to connected BLE host
static void ble_mouse_send_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t scroll)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_hid_input_handle == 0) {
        return;
    }
    uint8_t report[4] = { buttons, (uint8_t)dx, (uint8_t)dy, (uint8_t)scroll };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
    if (om != NULL) {
        ble_gatts_notify_custom(s_conn_handle, s_hid_input_handle, om);
    }
}

static void ble_hid_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    // Advertising flags: General discoverable & BLE only
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Appearance: Mouse (0x03C2) - tells Windows/macOS/Android to display a mouse icon
    fields.appearance = BLE_APPEARANCE_MOUSE;
    fields.appearance_is_present = 1;

    // Device Name
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    // 16-bit Service UUID: HID (0x1812)
    fields.uuids16 = (ble_uuid16_t *)&s_uuid_hid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "BLE advertising started: '%s' (Appearance: Mouse 0x03C2)", BLE_HID_DEVICE_NAME);
    }
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Host connection %s; status=%d",
                 event->connect.status == 0 ? "ESTABLISHED" : "FAILED",
                 event->connect.status);
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            // Initiate security pairing immediately so Windows/macOS/Android complete pairing
            ble_gap_security_initiate(event->connect.conn_handle);
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_hid_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Host disconnected; reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_hid_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_hid_advertise();
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "BLE encryption change; status=%d", event->enc_change.status);
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        {
            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            ble_store_util_delete_peer(&desc.peer_id_addr);
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            struct ble_sm_io pk;
            pk.action = event->passkey.params.action;
            pk.numcmp_accept = 1;
            ble_sm_inject_io(event->passkey.conn_handle, &pk);
        }
        break;

    default:
        break;
    }
    return 0;
}

static void ble_hid_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    ESP_LOGI(TAG, "Bluetooth MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    ble_hid_advertise();
}

static void ble_hid_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE Host Task running on Core 0");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_hid_init(void)
{
    nimble_port_init();

    // Security Manager: Just-Works pairing with encryption & bonding (Required for HID hosts)
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sync_cb = ble_hid_on_sync;

    // Register GATT Services Table
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(s_gatt_svcs);
    ble_gatts_add_svcs(s_gatt_svcs);

    // Set GAP Local Device Name & Mouse Appearance
    ble_svc_gap_device_name_set(BLE_HID_DEVICE_NAME);
    ble_svc_gap_device_appearance_set(BLE_APPEARANCE_MOUSE);

    // Start FreeRTOS Host Task
    nimble_port_freertos_init(ble_hid_host_task);
}

/* =========================================================================
 * 5. ESPRESSIF SPI MASTER DRIVER FOR MPU-6500
 * ========================================================================= */
static esp_err_t mpu6500_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx_data[2] = { (uint8_t)(reg & 0x7F), val };
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx_data;
    return spi_device_polling_transmit(s_imu_spi, &t);
}

static esp_err_t mpu6500_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx_data[2] = { (uint8_t)(reg | MPU6500_READ_FLAG), 0x00 };
    uint8_t rx_data[2] = { 0 };

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    esp_err_t ret = spi_device_polling_transmit(s_imu_spi, &t);
    if (ret == ESP_OK) {
        *val = rx_data[1];
    }
    return ret;
}

static esp_err_t mpu6500_read_burst(uint8_t start_reg, uint8_t *buf, size_t len)
{
    uint8_t tx_data[24] = { 0 };
    uint8_t rx_data[24] = { 0 };
    if (len + 1 > sizeof(tx_data)) {
        return ESP_ERR_INVALID_SIZE;
    }

    tx_data[0] = (uint8_t)(start_reg | MPU6500_READ_FLAG);

    spi_transaction_t t = {};
    t.length = (len + 1) * 8;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    esp_err_t ret = spi_device_polling_transmit(s_imu_spi, &t);
    if (ret == ESP_OK) {
        memcpy(buf, &rx_data[1], len);
    }
    return ret;
}

static void IRAM_ATTR mpu_drdy_isr(void *arg)
{
    BaseType_t hp_task_woken = pdFALSE;
    if (s_imu_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_imu_task_handle, &hp_task_woken);
        portYIELD_FROM_ISR(hp_task_woken);
    }
}

static esp_err_t imu_spi_init(void)
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_IMU_MOSI;
    buscfg.miso_io_num = PIN_IMU_MISO;
    buscfg.sclk_io_num = PIN_IMU_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;                        // SPI Mode 0 (CPOL=0, CPHA=0)
    devcfg.clock_speed_hz = 1 * 1000 * 1000;// 1 MHz safe demo clock
    devcfg.spics_io_num = PIN_IMU_NCS;
    devcfg.queue_size = 1;

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &s_imu_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    // Power-up settling time
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reset device
    mpu6500_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Wake up & select PLL
    mpu6500_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Disable I2C slave interface (force SPI mode)
    mpu6500_write_reg(MPU_REG_USER_CTRL, 0x10);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Query WHO_AM_I
    uint8_t who = 0;
    mpu6500_read_reg(MPU_REG_WHO_AM_I, &who);
    ESP_LOGI(TAG, "MPU-6500 WHO_AM_I = 0x%02X (Expected: 0x%02X)", who, MPU6500_EXPECTED_ID);

    portENTER_CRITICAL(&s_state_spinlock);
    s_state.who_am_i = who;
    s_state.imu_online = (who == MPU6500_EXPECTED_ID);
    portEXIT_CRITICAL(&s_state_spinlock);

    if (who != MPU6500_EXPECTED_ID) {
        ESP_LOGW(TAG, "WHO_AM_I mismatch! Check SPI wiring (MOSI=11, SCLK=12, MISO=13, nCS=10).");
        return ESP_ERR_NOT_FOUND;
    }

    // Configure DLPF (~41 Hz)
    mpu6500_write_reg(MPU_REG_CONFIG, 0x03);

    // Sample Rate = 1000 Hz / (1 + 1) = 500 Hz
    mpu6500_write_reg(MPU_REG_SMPLRT_DIV, 0x01);

    // Full-scale ranges: Gyro +-500 dps (0x08), Accel +-4g (0x08)
    mpu6500_write_reg(MPU_REG_GYRO_CONFIG, 0x08);
    mpu6500_write_reg(MPU_REG_ACCEL_CONFIG, 0x08);

    // Configure INT pin: Active-high, push-pull, 50us pulse, clear on read (0x10)
    mpu6500_write_reg(MPU_REG_INT_PIN_CFG, 0x10);
    // Enable Data Ready Interrupt
    mpu6500_write_reg(MPU_REG_INT_ENABLE, 0x01);

    // Attach GPIO interrupt on PIN_IMU_INT (GPIO 9)
    gpio_config_t int_cfg = {};
    int_cfg.pin_bit_mask = (1ULL << PIN_IMU_INT);
    int_cfg.mode = GPIO_MODE_INPUT;
    int_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    int_cfg.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&int_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_IMU_INT, mpu_drdy_isr, NULL);

    ESP_LOGI(TAG, "MPU-6500 initialized successfully over Espressif SPI master!");
    return ESP_OK;
}

/* =========================================================================
 * 6. CORE 1 REAL-TIME IMU TASK (Priority 10)
 * ========================================================================= */
static void imu_sensor_task(void *arg)
{
    s_imu_task_handle = xTaskGetCurrentTaskHandle();
    uint8_t raw[14];
    float pitch = 0.0f, roll = 0.0f;
    int64_t last_us = esp_timer_get_time();

    while (1) {
        // Fallback timeout prevents deadlock if an interrupt is missed during flash or system events
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.05f) {
            dt = 0.002f; // nominal 500 Hz dt fallback
        }

        if (mpu6500_read_burst(MPU_REG_ACCEL_XOUT_H, raw, 14) != ESP_OK) {
            continue;
        }

        int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
        int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
        int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

        // Scale factors: +-4g -> 8192 LSB/g, +-500 dps -> 65.5 LSB/(deg/s)
        float fax = (float)ax / 8192.0f;
        float fay = (float)ay / 8192.0f;
        float faz = (float)az / 8192.0f;
        float fgx = (float)gx / 65.5f;
        float fgy = (float)gy / 65.5f;
        float fgz = (float)gz / 65.5f;

        // 6-DOF Complementary Filter (98% Gyro integration, 2% Accel gravity reference)
        float acc_pitch = atan2f(fay, sqrtf(fax * fax + faz * faz)) * 57.29578f;
        float acc_roll  = atan2f(-fax, faz) * 57.29578f;
        pitch = 0.98f * (pitch + fgx * dt) + 0.02f * acc_pitch;
        roll  = 0.98f * (roll  + fgy * dt) + 0.02f * acc_roll;

        // Gimbal-guarded cursor delta mapping (attenuate roll compensation near +-90 deg pitch)
        float cos_p = cosf(pitch * 0.0174533f);
        if (cos_p < 0.2f) cos_p = 0.2f;
        int32_t dx = (int32_t)(-fgz * cos_p * 0.35f);
        int32_t dy = (int32_t)(-fgy * 0.35f);

        portENTER_CRITICAL(&s_state_spinlock);
        s_state.pitch_deg = pitch;
        s_state.roll_deg  = roll;
        s_state.drdy_count++;
        if (s_state.active_mode) {
            s_state.acc_dx += dx;
            s_state.acc_dy += dy;
        }
        portEXIT_CRITICAL(&s_state_spinlock);
    }
}

/* =========================================================================
 * 7. CORE 1 INPUT & JOYSTICK POLL TASK (Priority 6, 50 Hz)
 * ========================================================================= */
static int s_joy_center_y = 2048;

static void input_poll_init(void)
{
    // 1. Power-gate / VCC supply on GPIO 2 (Drive HIGH to ensure analog pot has power)
    gpio_config_t gate_cfg = {};
    gate_cfg.pin_bit_mask = (1ULL << PIN_JOY_VCC_GATE);
    gate_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&gate_cfg);
    gpio_set_level(PIN_JOY_VCC_GATE, 1);

    // 2. Mode toggle switch on PIN_JOY_SW (GPIO 41, active-low with pull-up)
    gpio_config_t sw_cfg = {};
    sw_cfg.pin_bit_mask = (1ULL << PIN_JOY_SW);
    sw_cfg.mode = GPIO_MODE_INPUT;
    sw_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    sw_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&sw_cfg);

    // 3. ADC Oneshot setup for Joystick Y on GPIO 1 (Pin 38 on DevKitC-1, ADC1_CH0)
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = JOY_ADC_UNIT;
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err == ESP_OK) {
        adc_oneshot_chan_cfg_t chan_cfg = {};
        chan_cfg.atten = ADC_ATTEN_DB_12;
        chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
        adc_oneshot_config_channel(s_adc_handle, JOY_ADC_CHANNEL, &chan_cfg);
        ESP_LOGI(TAG, "Joystick Y ADC configured on GPIO 1 (ADC1_CH0)");

        // Settle power and calibrate resting center baseline
        vTaskDelay(pdMS_TO_TICKS(50));
        int sum = 0, count = 0;
        for (int i = 0; i < 16; i++) {
            int val = 0;
            if (adc_oneshot_read(s_adc_handle, JOY_ADC_CHANNEL, &val) == ESP_OK) {
                sum += val;
                count++;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (count > 0 && (sum / count) > 200) {
            s_joy_center_y = sum / count;
            ESP_LOGI(TAG, "Calibrated Joystick Y resting center: %d", s_joy_center_y);
        } else {
            ESP_LOGW(TAG, "Joystick resting reading low (%d). Verify VCC/GND or GPIO 2.", count > 0 ? sum / count : 0);
        }
    } else {
        ESP_LOGW(TAG, "ADC init returned %s; scroll will fallback to idle", esp_err_to_name(err));
    }
}

static void input_poll_task(void *arg)
{
    int last_sw = 1;

    while (1) {
        // 1. Poll Mode Switch (SW on GPIO 41)
        int sw = gpio_get_level(PIN_JOY_SW);
        bool toggled = (last_sw == 1 && sw == 0);
        last_sw = sw;

        // 2. Ensure power rail is high and poll Joystick Y on GPIO 1 via ADC1 Channel 0
        gpio_set_level(PIN_JOY_VCC_GATE, 1);
        int raw_y = 2048;
        if (s_adc_handle != NULL) {
            adc_oneshot_read(s_adc_handle, JOY_ADC_CHANNEL, &raw_y);
        }

        // True bidirectional scroll around calibrated center:
        // Pushed UP   (diff > +350) -> Positive scroll (+1 to +3)
        // Pushed DOWN (diff < -350) -> Negative scroll (-1 to -3)
        // Center deadband (|diff| <= 350) -> 0
        int8_t current_scroll = 0;
        if (raw_y < 150) {
            // Safety guard: if unpowered or disconnected, do not spam -3 scroll
            current_scroll = 0;
        } else {
            int diff = raw_y - s_joy_center_y;
            if (diff > 350) {
                current_scroll = (int8_t)((diff - 350) / 300 + 1);
                if (current_scroll > 3) current_scroll = 3;
            } else if (diff < -350) {
                current_scroll = (int8_t)((diff + 350) / 300 - 1);
                if (current_scroll < -3) current_scroll = -3;
            }
        }

        bool log_toggled = false;
        bool active_now = true;

        portENTER_CRITICAL(&s_state_spinlock);
        s_state.scroll_y = current_scroll;
        s_state.raw_y = raw_y;
        if (toggled) {
            s_state.active_mode = !s_state.active_mode;
            s_state.acc_dx = 0;
            s_state.acc_dy = 0;
            log_toggled = true;
            active_now = s_state.active_mode;
        }
        portEXIT_CRITICAL(&s_state_spinlock);

        // Safe console output OUTSIDE of the critical section
        if (log_toggled) {
            ESP_LOGI(TAG, "Mode toggled -> %s", active_now ? "ACTIVE AIR MOUSE" : "TYPING LOCKED");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }
}
//             active_now = s_state.active_mode;
//         }
//         portEXIT_CRITICAL(&s_state_spinlock);

//         // Safe console output OUTSIDE of the critical section (prevents kernel abort/reset!)
//         if (log_toggled) {
//             ESP_LOGI(TAG, "Mode toggled -> %s", active_now ? "ACTIVE AIR MOUSE" : "TYPING LOCKED");
//         }

//         vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
//     }
// }

/* =========================================================================
 * 8. SSD1306 128x64 OLED UI & FONT RENDERER (Core 1, Priority 2)
 * ========================================================================= */
typedef struct {
    char c;
    uint8_t glyph[5];
} font_entry_t;

static const font_entry_t s_font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'+', {0x08, 0x08, 0x3E, 0x08, 0x08}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
};

static void oled_draw_char(uint8_t *fb, int page, int col, char c)
{
    if (page < 0 || page >= 8 || col < 0 || col > 122) return;
    const uint8_t *glyph = NULL;
    for (size_t i = 0; i < sizeof(s_font) / sizeof(s_font[0]); i++) {
        if (s_font[i].c == c) {
            glyph = s_font[i].glyph;
            break;
        }
    }
    for (int i = 0; i < 5; i++) {
        fb[page * OLED_WIDTH + col + i] = glyph ? glyph[i] : 0x00;
    }
}

static void oled_draw_str(uint8_t *fb, int page, int col, const char *str)
{
    while (*str && col <= 122) {
        oled_draw_char(fb, page, col, *str++);
        col += 6;
    }
}

static void oled_ui_init(void)
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = PIN_OLED_SDA;
    bus_cfg.scl_io_num = PIN_OLED_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return;
    }

    // Auto-probe SSD1306 address (0x3C vs 0x3D)
    uint16_t target_addr = OLED_I2C_ADDR;
    if (i2c_master_probe(i2c_bus, 0x3C, 50) == ESP_OK) {
        target_addr = 0x3C;
    } else if (i2c_master_probe(i2c_bus, 0x3D, 50) == ESP_OK) {
        target_addr = 0x3D;
    }

    esp_lcd_panel_io_i2c_config_t io_cfg = {};
    io_cfg.dev_addr = target_addr;
    io_cfg.scl_speed_hz = 100000; // 100 kHz standard mode
    io_cfg.control_phase_bytes = 1;
    io_cfg.dc_bit_offset = 6;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;

    esp_lcd_panel_io_handle_t io_handle = NULL;
    err = esp_lcd_new_panel_io_i2c(i2c_bus, &io_cfg, &io_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_i2c failed: %s", esp_err_to_name(err));
        return;
    }

    esp_lcd_panel_ssd1306_config_t ssd1306_cfg = {};
    ssd1306_cfg.height = OLED_HEIGHT;

    esp_lcd_panel_dev_config_t panel_cfg = {};
    panel_cfg.bits_per_pixel = 1;
    panel_cfg.reset_gpio_num = GPIO_NUM_NC;
    panel_cfg.vendor_config = &ssd1306_cfg;

    err = esp_lcd_new_panel_ssd1306(io_handle, &panel_cfg, &s_oled_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_ssd1306 failed: %s", esp_err_to_name(err));
        return;
    }

    esp_err_t init_err = esp_lcd_panel_init(s_oled_panel);
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 init failed (%s). Continuing demo without OLED display.", esp_err_to_name(init_err));
        s_oled_panel = NULL;
        return;
    }

    esp_lcd_panel_disp_on_off(s_oled_panel, true);
    ESP_LOGI(TAG, "SSD1306 OLED initialized successfully at address 0x%02X!", target_addr);
}

static void oled_ui_task(void *arg)
{
    static uint8_t fb[OLED_WIDTH * OLED_HEIGHT / 8];
    char str_buf[32];

    while (1) {
        if (s_oled_panel == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        system_state_t snap;
        portENTER_CRITICAL(&s_state_spinlock);
        snap = s_state;
        portEXIT_CRITICAL(&s_state_spinlock);

        bool is_connected = (s_conn_handle != BLE_HS_CONN_HANDLE_NONE);

        memset(fb, 0x00, sizeof(fb));

        // Page 0: Title & Mode Banner
        oled_draw_str(fb, 0, 0, "CAMMI MOUSE");
        oled_draw_str(fb, 0, 84, snap.active_mode ? "ACTIVE" : "LOCKED");

        // Top separator line
        for (int x = 0; x < OLED_WIDTH; x++) {
            fb[1 * OLED_WIDTH + x] |= 0x01;
        }

        // Page 2: Pitch gauge & value
        snprintf(str_buf, sizeof(str_buf), "P:%+05.1f", snap.pitch_deg);
        oled_draw_str(fb, 2, 0, str_buf);
        int px = 64 + (int)(snap.pitch_deg * 0.7f);
        if (px < 50) px = 50;
        if (px > 122) px = 122;
        for (int i = 0; i < 5; i++) {
            fb[2 * OLED_WIDTH + px + i] |= 0x7E;
        }

        // Page 4: Roll gauge & value
        snprintf(str_buf, sizeof(str_buf), "R:%+05.1f", snap.roll_deg);
        oled_draw_str(fb, 4, 0, str_buf);
        int rx = 64 + (int)(snap.roll_deg * 0.7f);
        if (rx < 50) rx = 50;
        if (rx > 122) rx = 122;
        for (int i = 0; i < 5; i++) {
            fb[4 * OLED_WIDTH + rx + i] |= 0x7E;
        }

        // Page 6: Scroll & Raw ADC & BLE Connection Status
        snprintf(str_buf, sizeof(str_buf), "S:%+d Y:%04d", snap.scroll_y, snap.raw_y);
        oled_draw_str(fb, 6, 0, str_buf);

        oled_draw_str(fb, 6, 84, is_connected ? "CONN" : "ADV");

        // Bottom status border
        for (int x = 0; x < OLED_WIDTH; x++) {
            fb[7 * OLED_WIDTH + x] |= (snap.active_mode ? 0x80 : ((x % 4 < 2) ? 0x80 : 0x00));
        }

        esp_lcd_panel_draw_bitmap(s_oled_panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, fb);

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz refresh
    }
}

/* =========================================================================
 * 9. CORE 0 BLE HID TX / BENCH TELEMETRY TASK (10 ms Cadence)
 * ========================================================================= */
static void ble_hid_tx_task(void *arg)
{
    uint32_t loop_count = 0;

    while (1) {
        int32_t dx = 0, dy = 0;
        int8_t scroll = 0;
        int raw_y_val = 2048;
        float pitch = 0.0f, roll = 0.0f;
        bool active = false;
        uint32_t drdy = 0;
        uint8_t who = 0;

        // Atomically consume accumulated cursor deltas at 100 Hz (10 ms)
        portENTER_CRITICAL(&s_state_spinlock);
        dx = s_state.acc_dx;
        dy = s_state.acc_dy;
        s_state.acc_dx = 0;
        s_state.acc_dy = 0;
        scroll = s_state.scroll_y;
        raw_y_val = s_state.raw_y;
        pitch  = s_state.pitch_deg;
        roll   = s_state.roll_deg;
        active = s_state.active_mode;
        drdy   = s_state.drdy_count;
        who    = s_state.who_am_i;
        portEXIT_CRITICAL(&s_state_spinlock);

        // Clamp cursor displacement to standard 8-bit signed range [-127, 127]
        int8_t report_dx = (dx > 127) ? 127 : ((dx < -127) ? -127 : (int8_t)dx);
        int8_t report_dy = (dy > 127) ? 127 : ((dy < -127) ? -127 : (int8_t)dy);

        // Send BLE HID report when connected and mouse is active
        if (active && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            if (report_dx != 0 || report_dy != 0 || scroll != 0) {
                ble_mouse_send_report(0, report_dx, report_dy, scroll);
            }
        }

        // Telemetry log output every 250 ms (25 loops * 10 ms) for bench video demo
        if ((++loop_count % 25) == 0) {
            ESP_LOGI(TAG, "[TELEMETRY] Mode:%s | BLE:%s | IMU:0x%02X DRDY:%lu | Pitch:%+6.1f Roll:%+6.1f | dX:%+ld dY:%+ld | Scroll:%+d (ADC_Y:%d)",
                     active ? "ACTIVE" : "LOCKED",
                     (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) ? "CONNECTED" : "ADVERTISING",
                     who, (unsigned long)drdy,
                     pitch, roll,
                     (long)dx, (long)dy, (int)scroll, (int)raw_y_val);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =========================================================================
 * 10. MAIN APPLICATION ENTRY POINT
 * ========================================================================= */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "CAMMI Wearable Air Mouse Hardware Bring-Up Demo");
    ESP_LOGI(TAG, "Target: ESP32-S3-N16R8 | ESP-IDF SPI Master & NimBLE");
    ESP_LOGI(TAG, "==================================================");

    // 0. Initialize NVS (Mandatory for NimBLE security keys and bonding)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Initialize SSD1306 OLED (I2C: SDA=4, SCL=5)
    oled_ui_init();

    // 2. Initialize Mode Switch (GPIO 41) & Joystick Y (GPIO 1 / ADC1_CH0)
    input_poll_init();

    // 3. Initialize MPU-6500 (SPI Master: MOSI=11, SCLK=12, MISO=13, nCS=10, INT=9)
    esp_err_t imu_err = imu_spi_init();
    if (imu_err != ESP_OK) {
        ESP_LOGE(TAG, "MPU-6500 initialization failed (%s). Continuing demo tasks.", esp_err_to_name(imu_err));
    }

    // 4. Initialize NimBLE BLE HID Mouse Stack
    ble_hid_init();

    // 5. Spawn Real-Time Tasks on Core 1
    xTaskCreatePinnedToCore(imu_sensor_task, "imu_sensor", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(input_poll_task, "input_poll", 3072, NULL,  6, NULL, 1);
    xTaskCreatePinnedToCore(oled_ui_task,    "oled_ui",    4096, NULL,  2, NULL, 1);

    // 6. Spawn Decoupled BLE Telemetry Task on Core 0
    xTaskCreatePinnedToCore(ble_hid_tx_task, "ble_hid_tx", 4096, NULL,  5, NULL, 0);

    ESP_LOGI(TAG, "All subsystem tasks launched across Core 0 and Core 1.");
}
