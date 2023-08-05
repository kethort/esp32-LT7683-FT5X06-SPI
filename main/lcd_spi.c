#include "lcd_spi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "common.h"

spi_device_handle_t spi;

void pre_transfer_callback(spi_transaction_t *t) {
    // Pull CS low before starting the SPI transaction
    gpio_set_level(PIN_NUM_CS, 0);
}

void post_transfer_callback(spi_transaction_t *t) {
    // Pull CS high after completing the SPI transaction
    gpio_set_level(PIN_NUM_CS, 1);
}

static void lcd_spi_cs_pin_init(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_CS);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_CS, 1);
}

// Function to perform full-duplex SPI communication
esp_err_t lcd_spi_read_write(uint8_t* tx_data, uint8_t* rx_data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));   // Initialize transaction parameters to 0
    t.length = 16;              // Total transaction length in bits
    t.tx_buffer = tx_data;      // Pointer to the buffer containing the data to be transmitted
    t.rx_buffer = rx_data;      // Pointer to the buffer where received data will be stored
    //t.flags = SPI_DEVICE_NO_DUMMY;  // Set the SPI_DEVICE_NO_DUMMY flag to skip sending dummy data

    // Perform full-duplex SPI communication
    ret = spi_device_polling_transmit(spi, &t);

    // Check for errors and handle the received data
    if (ret == ESP_OK) {
        // Data transmission and reception were successful
        //printf("Received Data: 0x%02X\n", rx_data[1]);
    } else {
        // Handle error
        printf("Error during SPI communication: %d\n", ret);
    }

    return ret;
}

void lcd_spi_init(void) {
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_40M,    //Clock out at 40 MHz
        .mode = 0,                                // SPI mode 0
        .spics_io_num = -1,                       // Control CS pin manually in the pre/post callback functions
        .queue_size = 1,                          
        .pre_cb = pre_transfer_callback,
        .post_cb = post_transfer_callback,
    };

    lcd_spi_cs_pin_init();

    //Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}