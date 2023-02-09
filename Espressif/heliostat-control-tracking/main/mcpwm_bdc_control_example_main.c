// /*
//  * SPDdata-FileCopyrightTedatat: 2021-2022 Espressif Systems (Shanghai) CO LTD
//  *
//  * SPDdata-License-Identifier: Apache-2.0
//  */

// #include <stdio.h>
// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/timers.h"
// #include "freertos/event_groups.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "driver/pulse_cnt.h"
// #include "driver/i2c.h"
// #include "bdc_motor.h"
// #include "nvs_flash.h"
// #include "esp_netif.h"
// #include "esp_http_client.h"
// #include "esp_wifi.h"
// #include "esp_types.h"
// #include "my_data.h"
// #include "pid_ctrl.h"

// static const char *TAG = "edataample";

// // Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
// #define SERIAL_STUDIO_DEBUG CONFIG_SERIAL_STUDIO_DEBUG
// #define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                         // 10MHz, 1 tick = 0.1us
// #define BDC_MCPWM_FREQ_HZ 25000                                                        // 25KHz PWM
// #define BDC_MCPWM_DUTY_TICK_MAdata (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // madataimum value we can set for the duty cycle, in ticks
// #define BDC_MCPWM_GPIO_A 16
// #define BDC_MCPWM_GPIO_B 15
// #define MAG_ADDRESS 0X1d
// #define SCL_GPIO 22
// #define SDA_GPIO 21

// #define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
// #define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
// #define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
// #define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
// #define ACK_VAL 0x0                /*!< I2C ack value */
// #define NACK_VAL 0x1               /*!< I2C nack value */

// // HTTP Client - FreeRTOS ESP IDF - GET
// extern const uint8_t certificate_pem_start[] asm("_binary_certificate_pem_start");
// extern const uint8_t certificate_pem_end[] asm("_binary_certificate_pem_end");

// // Global Values used for motor control and PWM
// float pwm_x;
// float pwm_y;

// int16_t d[3];

// // Wifi handler function and status update function
// static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
// {
//     switch (event_id)
//     {
//     case WIFI_EVENT_STA_START:
//         printf("WiFi connecting ... \n");
//         break;
//     case WIFI_EVENT_STA_CONNECTED:
//         printf("WiFi connected ... \n");
//         break;
//     case WIFI_EVENT_STA_DISCONNECTED:
//         printf("WiFi lost connection ... \n");
//         break;
//     case IP_EVENT_STA_GOT_IP:
//         printf("WiFi got IP ... \n\n");
//         break;
//     default:
//         break;
//     }
// }

// // Function for connecting to the wifi with parameters defined in my_data.h
// void wifi_connection()
// {
//     // 1 - Wi-Fi/LwIP Init Phase
//     esp_netif_init();                    // TCP/IP initiation
//     esp_event_loop_create_default();     // event loop
//     esp_netif_create_default_wifi_sta(); // WiFi station
//     wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&wifi_initiation); // initialize wifi with reference
//     // 2 - Wi-Fi Configuration Phase
//     esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
//     esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
//     wifi_config_t wifi_configuration = {// create structure containing wifi connection parame
//                                         .sta = {
//                                             .ssid = SSID,
//                                             .password = PASS}};
//     esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
//     // 3 - Wi-Fi Start Phase
//     esp_wifi_start();
//     // 4- Wi-Fi Connect Phase
//     esp_wifi_connect();
// }

// esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
// {
//     switch (evt->event_id)
//     {
//     case HTTP_EVENT_ON_DATA:
//         printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
//         pwm_x = atof((char *)evt->data);
//         printf("data: %f\n", pwm_x);
//         break;

//     default:
//         break;
//     }
//     return ESP_OK;
// }

// static void rest_get(char *url)
// {
//     esp_http_client_config_t config_get = {

//         .url = url,
//         .method = HTTP_METHOD_GET,
//         .cert_pem = (const char *)certificate_pem_start,
//         .event_handler = client_event_get_handler};

//     esp_http_client_handle_t client = esp_http_client_init(&config_get);
//     esp_http_client_perform(client);
//     esp_http_client_cleanup(client);
// }

// esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
// {
//     switch (evt->event_id)
//     {
//     case HTTP_EVENT_ON_DATA:
//         printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
//         // if statement to handle HTTP request => if manual control, call new get request
//         break;

//     default:
//         break;
//     }
//     return ESP_OK;
// }

// static void post_rest_function()
// {
//     esp_http_client_config_t config_post = {
//         .url = "https://7w4yj6h87i.edataecute-api.us-east-2.amazonaws.com/default/update-database",
//         .method = HTTP_METHOD_POST,
//         .cert_pem = NULL,
//         .event_handler = client_event_post_handler};

//     esp_http_client_handle_t client = esp_http_client_init(&config_post);

//     char *post_data = "{\"sensor\":\"Magnetometer1\",\"data\":1.345}";

//     esp_http_client_set_post_field(client, post_data, strlen(post_data));
//     esp_http_client_set_header(client, "Content-Type", "application/json");
//     esp_http_client_perform(client);
//     esp_http_client_cleanup(client);
// }

// typedef struct
// {
//     bdc_motor_handle_t motor;
//     pcnt_unit_handle_t pcnt_encoder;
//     pid_ctrl_block_handle_t pid_ctrl;
//     int report_pulses;
// } motor_control_contedatat_t;

// static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_wr, size_t size)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     // first, send device address (indicating write) & register to be written
//     i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
//     // send register we want
//     i2c_master_write_byte(cmd, i2c_reg, true);
//     // write the data
//     i2c_master_write(cmd, data_wr, size, true);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_rd, size_t size)
// {
//     if (size == 0)
//     {
//         return ESP_OK;
//     }
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     // first, send device address (indicating write) & register to be read
//     i2c_master_write_byte(cmd, (i2c_addr << 1), ACK_CHECK_EN);
//     // send register we want
//     i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
//     // Send repeated start
//     i2c_master_start(cmd);
//     // now send device address (indicating read) & read data
//     i2c_master_write_byte(cmd, (i2c_addr << 1) | READ_BIT, ACK_CHECK_EN);
//     if (size > 1)
//     {
//         i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
//     }
//     i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// // void sensor_routine()
// // {
// //     int8_t raw[1];
// //     uint8_t towrite = 1;
// //     i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
// //     cmd_handle = i2c_cmd_link_create();
// //     i2c_master_start(cmd_handle);
// //     i2c_master_write_byte(cmd_handle, (MAG_ADDRESS << 1) | I2C_MASTER_READ, true);
// //     i2c_master_read(cmd_handle, (uint8_t *)&raw, 1, I2C_MASTER_ACK);
// //     i2c_master_stop(cmd_handle);
// //     i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
// // i2c_cmd_link_delete(cmd_handle); // freeing memory

// // int8_t x = ((raw[1] << 8) | raw[2]) >> 2;
// // int8_t y = ((raw[3] << 8) | raw[4]) >> 2;
// // int8_t z = ((raw[5] << 8) | raw[6]) >> 2;

// // for (int i = 0; i < 10; i++)
// // {
// //     printf("%i\n", raw[i]);
// // }

// // d[0] = ((raw[1] << 4) | raw[0]); // getting the first magnetometer data
// // printf("Recorded Data for Accelerometer x: %d , y: %d , z: %d \n", x, y, z);
// // printf("Recorded Data for Magnetometer x: %d  \n", d[0]);
// //}

// void app_main(void)
// {

//     nvs_flash_init();
//     wifi_connection();

//     printf("WIFI was initiated ...........\n\n");
//     i2c_config_t i2c_config = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = SDA_GPIO,
//         .scl_io_num = SCL_GPIO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLDOWN_ENABLE,
//         .master.clk_speed = 100000};
//     i2c_param_config(I2C_NUM_0, &i2c_config);
//     i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
//     printf("I2C was initiated ...........\n\n");

//     uint8_t data[2];
//     i2c_master_read_slave_reg(I2C_NUM_0, MAG_ADDRESS, 0x0d, data, 1);
//     printf("Data! %x, %x", data[0], data[1]);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     char *url = "https://capstone-database-c7175-default-rtdb.firebaseio.com/Flags/Motor/x.json";
//     static motor_control_contedatat_t motor_ctrl_ctdata = {
//         .pcnt_encoder = NULL,
//     };

//     ESP_LOGI(TAG, "Create DC motor");
//     bdc_motor_config_t motor_config = {
//         .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
//         .pwma_gpio_num = BDC_MCPWM_GPIO_A,
//         .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
//     };
//     bdc_motor_mcpwm_config_t mcpwm_config = {
//         .group_id = 0,
//         .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
//     };
//     bdc_motor_handle_t motor = NULL;
//     ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
//     motor_ctrl_ctdata.motor = motor;

//     ESP_LOGI(TAG, "Enable motor");
//     ESP_ERROR_CHECK(bdc_motor_enable(motor));

//     // while (1)
//     // {
//     //     sensor_routine();
//     //     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // rest_get(url);
//     // vTaskDelay(500 / portTICK_PERIOD_MS);
//     // ESP_ERROR_CHECK(bdc_motor_brake(motor));
//     // if (pwm_x < 0 && pwm_x >= -100)
//     // { // backward operation
//     //     ESP_LOGI(TAG, "Stopping motor");
//     //     ESP_LOGI(TAG, "Backward motor");
//     //     ESP_ERROR_CHECK(bdc_motor_reverse(motor));
//     //     ESP_ERROR_CHECK(bdc_motor_set_speed(motor, BDC_MCPWM_DUTY_TICK_MAdata * (pwm_x / -100)));
//     // }
//     // else if (pwm_x > 0 && pwm_x <= 100)
//     // { // forward operation
//     //     ESP_LOGI(TAG, "Stopping motor");
//     //     ESP_LOGI(TAG, "Forward motor");
//     //     ESP_ERROR_CHECK(bdc_motor_forward(motor));
//     //     ESP_ERROR_CHECK(bdc_motor_set_speed(motor, BDC_MCPWM_DUTY_TICK_MAdata * (pwm_x / 100)));
//     // }
//     //   }
// }

/* i2c_restart - Example
   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by configuring & reading a MMA8451
 * accelerometer. This device has a registered I2C interface, which requires the I2C
 * bus to issue a 'repeated start' between sending the register to be accessed, and
 * the subsequent read command.
 * (see: https://www.i2c-bus.org/repeated-start-condition/ )
 *
 * - read external i2c sensor, here we use a MMA8451 (accelerometer) for instance.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor (MMA8451) with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - i2c master(ESP32) will read data from i2c slave (MMA8451 accelerometer)
 */

#define SAMPLE_PERIOD_MS 200

#define I2C_SCL_IO 27          // 19               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO 26          // 18               /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ 100000     /*!< I2C master clock frequency */
#define I2C_PORT_NUM I2C_NUM_1 /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE 0   /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE 0   /*!< I2C master do not need buffer */

// I2C common protocol defines
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

// MMA8451 defines
#define MMA8451_I2C_ADDR 0x1D

#define MMA8451_OUT_X_MSB 0x01
#define WHO_AM_I_REG 0x0D
#define XYZ_DATA_CFG_REG 0x0E
#define CTRL_REG1 0x2A
#define CTRL_REG2 0x2B
#define CTRL_REG3 0x2C
#define CTRL_REG4 0x2D
#define CTRL_REG5 0x2E

#define ASLP_RATE_20MS 0x00
#define ACTIVE_MASK 0x01

#define DATA_RATE_80MS 0x28
#define FULL_SCALE_2G 0x00

#define MODS_MASK 0x03
#define MODS1_MASK 0x02
#define MODS0_MASK 0x01

#define PP_OD_MASK 0x01
#define INT_EN_DRDY_MASK 0x01

#define INT_CFG_DRDY_MASK 0x01

// Structure to hold accelerometer data
typedef struct ACCEL_DATA
{
    int16_t X;
    int16_t Y;
    int16_t Z;
} stACCEL_DATA_t;

static const char *TAG = "i2c_restart";

/**
 * @brief test code to read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, (i2c_addr << 1), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, (i2c_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write i2c slave device with registered interface
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 * ____________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, (i2c_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Read contents of a MMA8451 register
---------------------------------------------------------------------------*/
esp_err_t rdMMA845x(uint8_t reg, uint8_t *pdata, uint8_t count)
{
    return (i2c_master_read_slave_reg(I2C_PORT_NUM, MMA8451_I2C_ADDR, reg, pdata, count));
}

/* Write value to specified MMA8451 register
---------------------------------------------------------------------------*/
esp_err_t wrMMA845x(uint8_t reg, uint8_t *pdata, uint8_t count)
{
    return (i2c_master_write_slave_reg(I2C_PORT_NUM, MMA8451_I2C_ADDR, reg, pdata, count));
}

/**
 * @brief MMA8451 initialization
 */
static void mma8451_init()
{
    uint8_t val;

    // Before re-configuring, must enter 'standby' mode
    rdMMA845x(CTRL_REG1, &(val), 1);
    val &= ~(ACTIVE_MASK);
    wrMMA845x(CTRL_REG1, &(val), 1);

    rdMMA845x(WHO_AM_I_REG, &(val), 1);
    if (val == 0x1A)
    {
        ESP_LOGI(TAG, "MMA8245x ID:0x%X (ok)", val);
    }
    else
    {
        ESP_LOGE(TAG, "MMA8245x ID:0x%X !!!! (NOT correct; should be 0x1A)", val);
    }

    /*
    **  Configure accelerometer for:
    **    - Sleep Mode Poll Rate of 50Hz (20ms)
    **    - System Output Data Rate of 200Hz (5ms)
    **    - Full Scale of +/-2g
    */
    //	IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, ASLP_RATE_20MS+DATA_RATE_5MS);
    // val = (ASLP_RATE_20MS + DATA_RATE_80MS);
    // wrMMA845x(CTRL_REG1, &(val), 1);

    // // configure 2G full scale, High_Pass_Filter disabled
    // val = (FULL_SCALE_2G);
    // wrMMA845x(XYZ_DATA_CFG_REG, &(val), 1);

    // // Setup Hi-Res mode (14-bit)
    // rdMMA845x(CTRL_REG2, &(val), 1);
    // val &= ~(MODS_MASK);
    // val |= (MODS1_MASK);
    // wrMMA845x(CTRL_REG2, &(val), 1);

    // // Configure the INT pins for Open Drain and Active Low
    // val = (PP_OD_MASK);
    // wrMMA845x(CTRL_REG3, &(val), 1);

    // // Enable the DRDY Interrupt
    // val = (INT_EN_DRDY_MASK);
    // wrMMA845x(CTRL_REG4, &(val), 1);

    // // Set the DRDY Interrupt to INT1
    // val = (INT_CFG_DRDY_MASK);
    // wrMMA845x(CTRL_REG5, &(val), 1);

    // // reconfig done, make active
    // rdMMA845x(CTRL_REG1, &(val), 1);
    // val |= (ACTIVE_MASK);
    // wrMMA845x(CTRL_REG1, &(val), 1);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    int i2c_master_port = I2C_PORT_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    conf.clk_flags = 0;
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}

uint16_t byte_swap(uint16_t data)
{
    return ((data >> 8) | (data << 8));
}

static void i2c_test_task(void *arg)
{
    esp_err_t err;
    stACCEL_DATA_t acc;

    ESP_LOGI(TAG, "ESP I2C_RESTART Example - MMA8451 Accelerometer");

    while (1)
    {

        // Note: as configured, reading data from the output registers will start next acquisition
        err = rdMMA845x(MMA8451_OUT_X_MSB, (uint8_t *)&acc, sizeof(acc));

        // byte-swap values to make little-endian
        acc.X = byte_swap(acc.X);
        acc.Y = byte_swap(acc.Y);
        acc.Z = byte_swap(acc.Z);

        // shift each value to align 14-bits in 16-bit ints
        acc.X /= 4;
        acc.Y /= 4;
        acc.Z /= 4;

        ESP_LOGI(TAG, "Accelerometer err:%d  x:%5d  y:%5d  z:%5d", err, acc.X, acc.Y, acc.Z);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void app_main()
{
    i2c_master_init();

    // uint8_t val[2];
    // rdMMA845x(WHO_AM_I_REG, &(val), 1);
    // printf("%X\n", val[0]);

    mma8451_init();
    // xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}