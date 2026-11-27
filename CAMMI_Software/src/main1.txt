#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "MPU6500_SPI";

// Pin definitions (Avoid GPIOs 26-37 on ESP32-S3 N16R8)
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

// MPU-6500 Register Definitions
#define MPU6500_REG_WHO_AM_I     0x75
#define MPU6500_READ_FLAG        0x80
#define MPU6500_EXPECTED_WHOAMI  0x70

static esp_err_t mpu6500_read_reg(spi_device_handle_t spi, uint8_t reg, uint8_t *val) {
    // 2-byte full-duplex transfer:
    // Byte 0: Register address with MSB (0x80) set for Read
    // Byte 1: Dummy byte (0x00) to clock out the sensor's response
    uint8_t tx_data[2] = { (uint8_t)(reg | MPU6500_READ_FLAG), 0x00 };
    uint8_t rx_data[2] = { 0 };


    spi_transaction_t t = {};
    t.length = 16;           // Total transaction length in bits (2 bytes)
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    if (ret == ESP_OK) {
        *val = rx_data[1];      // Valid data arrives during the second byte
    }
    return ret;
}


extern "C" void app_main() 
{
    spi_device_handle_t spi;

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 2. Attach the MPU-6500 Device to the Bus
    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 1 * 1000 * 1000;
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Give the IMU ~100ms after power-on before querying registers
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        uint8_t who_am_i = 0;
        esp_err_t err = mpu6500_read_reg(spi, MPU6500_REG_WHO_AM_I, &who_am_i);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (Expected: 0x%02X)", who_am_i, MPU6500_EXPECTED_WHOAMI);
            if (who_am_i == MPU6500_EXPECTED_WHOAMI) {
                ESP_LOGI(TAG, "MPU-6500 detected successfully over SPI!");
            } else {
                ESP_LOGW(TAG, "Unexpected ID! Check wiring, SPI mode, or chip variant.");
            }
        } else {
            ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}