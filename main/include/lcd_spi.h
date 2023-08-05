#ifndef LCD_SPI_H
#define LCD_SPI_H

#include "driver/spi_master.h"
#include "esp_err.h"

#define LCD_HOST     VSPI_HOST

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

esp_err_t lcd_spi_read_write(uint8_t* tx_data, uint8_t* rx_data);
//esp_err_t lcd_spi_read_write(uint8_t cmd, uint8_t *data);
void lcd_spi_init(void);

#endif