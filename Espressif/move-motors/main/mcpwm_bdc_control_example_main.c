/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

static const char *TAG = "example";

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                      // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                                                     // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A 16
#define BDC_MCPWM_GPIO_B 15

typedef struct
{
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

void app_main(void)
{
    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));

    while (1)
    {

        ESP_LOGI(TAG, "Forward motor");
        ESP_ERROR_CHECK(bdc_motor_forward(motor));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor, BDC_MCPWM_DUTY_TICK_MAX * (0.8)));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Motor Stop");
        ESP_ERROR_CHECK(bdc_motor_brake(motor));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Forward Backwards");
        ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor, BDC_MCPWM_DUTY_TICK_MAX * (0.8)));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Motor Stop");
        ESP_ERROR_CHECK(bdc_motor_brake(motor));
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // for (int i = 5; i > 0; i--)
        // {
        //     ESP_LOGI(TAG, "Forward motor");
        //     ESP_ERROR_CHECK(bdc_motor_forward(motor));
        //     ESP_ERROR_CHECK(bdc_motor_set_speed(motor, BDC_MCPWM_DUTY_TICK_MAX * ()));
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     ESP_LOGI(TAG, "Motor Stop");
        //     ESP_ERROR_CHECK(bdc_motor_brake(motor));
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     ESP_LOGI(TAG, "Forward Backwards");
        //     ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        //     ESP_ERROR_CHECK(bdc_motor_set_speed(motor, BDC_MCPWM_DUTY_TICK_MAX * (1 / i)));
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     ESP_LOGI(TAG, "Motor Stop");
        //     ESP_ERROR_CHECK(bdc_motor_brake(motor));
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // }
    }
}