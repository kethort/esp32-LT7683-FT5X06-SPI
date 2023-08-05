#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "lcd_spi.h"
#include "LT7683.h"
#include "esp_task_wdt.h"
#include "math.h"
#include "time.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_touch_ft5x06.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MAIN";

SemaphoreHandle_t spi_mutex;
esp_lcd_touch_handle_t tp = NULL;

void roundGaugeTicker(uint16_t x, uint16_t y, uint16_t r, int from, int to, float dev, uint32_t color) {
  float dsec;
  int i;
  for (i = from; i <= to; i += 30) {
    dsec = i * (M_PI / 180);
    LT7683_DrawLine(
		x + (cos(dsec) * (r / dev)) + 1,
		y + (sin(dsec) * (r / dev)) + 1,
		x + (cos(dsec) * r) + 1,
		y + (sin(dsec) * r) + 1, 
		color);
  }
}

void drawGauge(uint16_t x, uint16_t y, uint16_t r) {
  LT7683_DrawCircle(x, y, r, WHITE); //draw instrument container
  roundGaugeTicker(x, y, r, 150, 390, 1.3, WHITE); //draw major ticks
  if (r > 15) roundGaugeTicker(x, y, r, 165, 375, 1.1, WHITE); //draw minor ticks

}

void drawPointerHelper(int16_t val, uint16_t x, uint16_t y, uint16_t r, uint32_t color) {
  float dsec, toSecX, toSecY;
  int16_t minValue = 0;
  int16_t maxValue = 255;
  float fromDegree = 150.0;     //start
  float toDegree = 240.0;       //end
  if (val > maxValue) val = maxValue;
  if (val < minValue) val = minValue;
  dsec = (((float)(val - minValue) / (float)(maxValue - minValue) * toDegree) + fromDegree) * (M_PI / 180.0);
  toSecX = cos(dsec) * (r / 1.35);
  toSecY = sin(dsec) * (r / 1.35);
  LT7683_DrawLine_Width(x, y, 1 + x + (int16_t)toSecX, 1 + y + (int16_t)toSecY, color, 4);
  LT7683_DrawCircle_Fill(x, y, 10, color);
}

void drawNeedle(int16_t val, int16_t oval, uint16_t x, uint16_t y, uint16_t r, uint32_t color, uint32_t bcolor) {
  uint16_t i;
  if (val > oval) {
    // Acquire the SPI bus mutex
    if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
        // Access the SPI bus here
        for (i = oval; i <= val; i++) {
            drawPointerHelper(i - 1, x, y, r, bcolor);
            drawPointerHelper(i, x, y, r, color);
            if ((val - oval) < (128)) vTaskDelay(1 / portTICK_PERIOD_MS); //ballistic
        }
        // Release the SPI bus mutex
        xSemaphoreGive(spi_mutex);
    }
  }
  else {
    if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
        // Access the SPI bus here
        for (i = oval; i > val; i--) {
            drawPointerHelper(i + 1, x, y, r, bcolor);
            drawPointerHelper(i, x, y, r, color);

            if ((oval - val) >= 128) {
                vTaskDelay(1 / portTICK_PERIOD_MS);
            } else {
                vTaskDelay(3 / portTICK_PERIOD_MS);
            }
        }
        // Release the SPI bus mutex
        xSemaphoreGive(spi_mutex);
    }
  }
}

void gaugeTask(void *pvParameters) {
    volatile int16_t curVal1 = 0;
    volatile int16_t oldVal1 = 0;

    drawGauge(512, 300, 250);

    uint32_t min_value = 1;
    uint32_t max_value = 254;
    
    srand(time(NULL));

    while (1) {
        curVal1 = min_value + (rand() % (max_value - min_value + 1));

        if (oldVal1 != curVal1) {
            drawNeedle(curVal1, oldVal1, 512, 300, 250, GREEN, BLACK);
            oldVal1 = curVal1;
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Delay for 1000 milliseconds (1 second)
    }
}

void uint16ToString(uint16_t num, char *buffer) {
    sprintf(buffer, ":%u ", num);
}

static void setupTouchPanelI2C(void) {
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    ESP_LOGI(TAG, "Initialize I2C");

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    /* Initialize I2C */
    ESP_ERROR_CHECK(i2c_param_config(0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(0, i2c_conf.mode, 0, 0, 0));

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");

    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)0, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LT7683_SCREEN_WIDTH,
        .y_max = LT7683_SCREEN_HEIGHT,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller FT5X06");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp));
}

void ft5x06Task(void *pvParameters) {
    setupTouchPanelI2C();

    char x_value[7];
    char y_value[7];

    // size, width, height, chroma, alignment
    LT7683_Select_Internal_Font_Init(24, 1, 1, 0, 0);
        
    while (1) {
        esp_lcd_touch_read_data(tp);

        uint16_t touch_x[1];
        uint16_t touch_y[1];
        uint16_t touch_strength[1];
        uint8_t touch_cnt = 0;

        bool touchpanel_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 1);

        if (touchpanel_pressed) {
            // Acquire the SPI bus mutex
            if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
                LT7683_DrawSquare_Fill(0, 0, 210, 50, BLACK);
                //printf("x:%d y:%d\n", touch_x[0], touch_y[0]);
                memset(x_value, 0, 7);
                memset(y_value, 0, 7);
            
                // Access the SPI bus here
                uint16ToString(touch_x[0], x_value);
                uint16ToString(touch_y[0], y_value);

                // cursor_x, cursor_y, font color, background color, text
                LT7683_Print_Internal_Font_String(10, 10, WHITE, BLACK, "x");
                LT7683_Print_Internal_Font_String(65534, 0, WHITE, BLACK, x_value);
                LT7683_Print_Internal_Font_String(65534, 0, WHITE, BLACK, "y");
                LT7683_Print_Internal_Font_String(65534, 0, WHITE, BLACK, y_value);

                // Release the SPI bus mutex
                xSemaphoreGive(spi_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Delay for 1000 milliseconds (1 second)
    }
}

void app_main(void)
{
    lcd_spi_init();  // initialize spi peripheral

    LT7683_init();
    PWM0_ON();      // turn on the lcd backlight
    
    // run color bar test
    Color_Bar_ON();
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    Color_Bar_OFF();

    LT7683_DrawSquare_Fill(0, 0, LT7683_SCREEN_WIDTH - 1, LT7683_SCREEN_HEIGHT - 1, BLACK);

    spi_mutex = xSemaphoreCreateMutex();

    xTaskCreate(&gaugeTask, "gaugeTask", 2048, NULL, 5, NULL);           
    xTaskCreate(&ft5x06Task, "touchScreenTask", 2048, NULL, 5, NULL);  
}
