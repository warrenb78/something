#include "string.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_spiffs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdint.h>

#define TAG "main"

#define SELECTED_ADC        (ADC_UNIT_2)
#define SELECTED_CHANNEL    (ADC_CHANNEL_6)
#define UPPER_LIMIT         (2500) // 2500
#define LOWER_LIMIT         (1000) // 1000
#define SLEEP_IN_MS         (60000)

#define BUF_SIZE (1024)

void wait(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

adc_oneshot_unit_handle_t init_adc(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = SELECTED_ADC,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SELECTED_CHANNEL, &config));

    return adc1_handle;
}

int get_gpio_reading(adc_oneshot_unit_handle_t adc1_handle)
{
    int adc_raw;
    int voltage;

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(SELECTED_ADC, SELECTED_CHANNEL, ADC_ATTEN_DB_11, &adc1_cali_chan0_handle);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SELECTED_CHANNEL, &adc_raw));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", SELECTED_ADC + 1, SELECTED_CHANNEL, adc_raw);
    if (do_calibration1_chan0)
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", SELECTED_ADC + 1, SELECTED_CHANNEL, voltage);
    }

    // Tear Down
    // ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    // if (do_calibration1_chan0) {
    //     example_adc_calibration_deinit(adc1_cali_chan0_handle);
    // }
    // if (do_calibration1_chan1) {
    //     example_adc_calibration_deinit(adc1_cali_chan1_handle);
    // }

    return voltage;
}

size_t binary_search(const int *arr, size_t size)
{
    size_t low = 0, high = size, mid = 0;
    while (low <= high)
    {
        mid = low + (high - low) / 2;

        if (arr[mid] == 0xFFFFFFFF)
            high = mid - 1;

        else
            low = mid + 1;
    }
    return mid;
}

volatile FILE *data_file = NULL;

FILE *init_spiff()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return NULL;
    }

    FILE *data_file = fopen("/spiffs/result2.csv", "a");
    if (data_file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return NULL;
    }

    return data_file;
}

void write_value_to_file(FILE *f, int value, int trigger, int64_t time)
{
    fprintf(f, "[%lld,%d,%d]\n", time, value, trigger);
    fflush(f);
}

void configure_gpio_write(void)
{
    gpio_config_t io_conf = {
        // disable interrupt
        .intr_type = GPIO_INTR_DISABLE,
        // set as output mode
        .mode = GPIO_MODE_OUTPUT,
        // bit mask of the pins that you want to set, e.g.GPIO25
        .pin_bit_mask = (1ULL << GPIO_NUM_2),
        // enable pull-down mode
        .pull_down_en = 1,
        .pull_up_en = 0};

    // configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_2, 1);
}

bool read_trigger()
{
    FILE *f = fopen("/spiffs/event.txt", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return false;
    }

    char line[2];
    fgets(line, sizeof(line), f);
    fclose(f);
    char a = line[0];
    return (a == '1');
}

void write_trigger()
{
    FILE *f = fopen("/spiffs/event.txt", "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    fprintf(f, "%d", 1);
    fclose(f);
}

void uart_task(void *arg) {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(20);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(0, data, 5, pdMS_TO_TICKS(1000));
        if (len) {
            data[5] = '\0';
            if (strcmp("hello", (const char *)data) == 0) {
                FILE *f = fopen("/spiffs/result2.csv", "r");
                if (f == NULL)
                {
                    ESP_LOGE(TAG, "Failed to open file for writing");
                    return;
                }

                char line[40];
                while (fgets(line, sizeof(line), f) != NULL) {
                    uart_write_bytes(0, line, strlen(line));
                    memset(line, 0, 40);
                }
                fclose(f);
            }            
        }
    }
}

void app_main(void)
{
    adc_oneshot_unit_handle_t handle = init_adc();
    FILE *f = init_spiff();
    bool eventHappend = false;
    int trigger = 0;
    
    if (read_trigger()) {
        eventHappend = true;
        configure_gpio_write();
    }
    xTaskCreate(uart_task, "uart_command", 2048, NULL, 10, NULL);

    while (true) {
        trigger = 0;

        wait(SLEEP_IN_MS);
        int x = get_gpio_reading(handle);
        int64_t time = esp_timer_get_time();

        if (x > UPPER_LIMIT || x < LOWER_LIMIT)
        {
            // TODO: counter
            if (x > UPPER_LIMIT) {
                trigger = 1;
            } 
            if ( x < LOWER_LIMIT) {
                trigger = 2;
            }

            if (!eventHappend)
            {
                write_trigger();
                configure_gpio_write();
                eventHappend = true;
            }
        }

        write_value_to_file(f, x, trigger, time);
    }
}

