#include "driver/twai.h"  // Include the TWAI (CAN) driver library
#include "esp_log.h"      // Include ESP logging library
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define OLED_I2C_ADDRESS 0x3C

#define OLED_CONTROL_BYTE_CMD_STREAM 0x00
#define OLED_CONTROL_BYTE_CMD_SINGLE 0x80
#define OLED_CONTROL_BYTE_DATA_STREAM 0x40

#define OLED_CMD_SET_CHARGE_PUMP 0x8D
#define OLED_CMD_SET_SEGMENT_REMAP 0xA1
#define OLED_CMD_SET_COM_SCAN_MODE 0xC8
#define OLED_CMD_DISPLAY_ON 0xAF
#define OLED_CMD_SET_CONTRAST 0x81


#define TAG "CAN_DRIVER"  // Define a tag for logging messages

static const char *TAG1 = "SSD1306";

void i2c_master_init()
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void ssd1306_init() {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); 
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); 

    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG1, "OLED configurado com sucesso");
    } else {
        ESP_LOGE(TAG1, "Erro ao configurar OLED: 0x%.2X", ret);
    }

    i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_clear(void *ignore) {
    i2c_cmd_handle_t cmd;
    uint8_t zero[128] = {0};

    for (uint8_t i = 0; i < 8; i++) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true);

        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, zero, sizeof(zero), true);

        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelete(NULL);
}

void ssd1306_display_text(const char *text) {
    uint8_t text_len = strlen(text);
    i2c_cmd_handle_t cmd;
    uint8_t cur_page = 0;

    // Clear screen
    uint8_t zero[128] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, zero, sizeof(zero), true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x00, true); 
    i2c_master_write_byte(cmd, 0x10, true); 
    i2c_master_write_byte(cmd, 0xB0 | cur_page, true); 
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    for (uint8_t i = 0; i < text_len; i++) {
        if (text[i] == '\n') {
            cur_page++;
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
            i2c_master_write_byte(cmd, 0x00, true);
            i2c_master_write_byte(cmd, 0x10, true);
            i2c_master_write_byte(cmd, 0xB0 | cur_page, true);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
        } else {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
            i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
        }
    }
}


void app_main() {

    i2c_master_init();
    ssd1306_init();

    int rpm_recebido = 0;
    int temperatura_recebida = 0;
    int umidade_recebida = 0;
    float temperatura_final = 0.0;

   char display_text[64];
    snprintf(display_text, sizeof(display_text),
         "Temp: %.1f C\nUmid: %d %%\nVel: %d m/s", temperatura_final, umidade_recebida, rpm_recebido);


    // Configure the CAN driver parameters (RX = 4; TX = 5)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Initialize the CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG, "TWAI driver installed successfully");
    } else {
        ESP_LOGE(TAG, "Failed to install TWAI driver");
        return; // Exit the function if driver installation fails
    }

    // Start the CAN driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "TWAI driver started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start TWAI driver");
        return; // Exit the function if driver start fails
    }

    while (1){

        // Receive a message
        twai_message_t rx_message;  // Declare a message structure for receiving
        if (twai_receive(&rx_message, pdMS_TO_TICKS(2000)) == ESP_OK) {
            printf("Message received -> ");
            if (rx_message.identifier == 0x001){
                    printf("Anemometro: \n");
                    ESP_LOGI(TAG, "DLC: %d, Data:", rx_message.data_length_code);
                for (int i = 0; i < rx_message.data_length_code; i++) {
                    ESP_LOGI(TAG, "Data[%d]: %d", i, rx_message.data[i]);
                }
                rpm_recebido =rx_message.data[0];
                ESP_LOGI(TAG, "RPM recebido: %d", rpm_recebido);

            } else if (rx_message.identifier == 0x002){
                    printf("Sensor de temperatura: \n");
                    ESP_LOGI(TAG, "DLC: %d, Data:", rx_message.data_length_code);
                for (int i = 0; i < rx_message.data_length_code; i++) {
                    ESP_LOGI(TAG, "Data[%d]: %d", i, rx_message.data[i]);
                }
                temperatura_recebida =rx_message.data[0];
                ESP_LOGI(TAG, "Temperatura recebida: %d", temperatura_recebida);
                temperatura_final = temperatura_recebida / 10.0f;
                ESP_LOGI(TAG, "Temperatura final: %.1f", temperatura_final);

            } else if (rx_message.identifier == 0x003){
                    printf("Sensor de umidade: \n");
                    ESP_LOGI(TAG, "DLC: %d, Data:", rx_message.data_length_code);
                for (int i = 0; i < rx_message.data_length_code; i++) {
                    ESP_LOGI(TAG, "Data[%d]: %d", i, rx_message.data[i]);
                }
                umidade_recebida =rx_message.data[0];
                ESP_LOGI(TAG, "Umidade recebida: %d%%", umidade_recebida);
            } 

        } else {
            ESP_LOGE(TAG, "Failed to receive message");
        }
        snprintf(display_text, sizeof(display_text),"Temp: %.1f C\nUmid: %d %%\nVel: %d m/s", temperatura_final, umidade_recebida, rpm_recebido);

        ssd1306_display_text(display_text);

        printf("\n");
    }

    // Stop and uninstall the CAN driver
    twai_stop();
    twai_driver_uninstall();
    ESP_LOGI(TAG, "TWAI driver uninstalled");
}