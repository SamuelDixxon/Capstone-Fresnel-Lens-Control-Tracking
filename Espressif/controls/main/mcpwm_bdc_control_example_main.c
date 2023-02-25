// /*
//  * SPDdata-FileCopyrightTedatat: 2021-2022 Espressif Systems (Shanghai) CO LTD
//  *
//  * SPDdata-License-Identifier: Apache-2.0
//  */

#include <stdio.h>
#include <ctype.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "driver/i2c.h"
#include "bdc_motor.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_wifi.h"
#include "esp_types.h"
#include "my_data.h"
#include "pid_ctrl.h"
#include "math.h"

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

// MMA8451 defines ()
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

// MMC5603 defines (magnetometer sensor)
#define MMC560_I2C_ADDR 0x30 // data for address device as slave in i2c communication
#define DEVICE_ID 0x39       // data for distinct id correspond to device
#define ODR 0x1A             // address for setting the on data rate
#define TEMP 0x09            // address for getting the temperature
#define X_MSB 0x00           // most significant byte for the x reading
#define Y_MSB 0x02           // most significant byte for the y reading
#define Z_MSB 0x04           // most significant byte for the z reading
#define mCTRL_REG0 0x1B      // device address for control reg 0
#define mCTRL_REG1 0x1C      // device address for control reg 1
#define mCTRL_REG2 0x1D      // device address for control reg 2
#define mTEMP 0x09           // address for temperature data
#define STATUS 0x18          // address for the devices status

// MCP9808 defines (temperature sensor)
#define MCP9808_I2CADDR 0x18    ///< I2C address
#define MCP9808_REG_CONFIG 0x01 ///< MCP9808 config register

#define MCP9808_REG_CONFIG_SHUTDOWN 0x0100   ///< shutdown config
#define MCP9808_REG_CONFIG_CRITLOCKED 0x0080 ///< critical trip lock
#define MCP9808_REG_CONFIG_WINLOCKED 0x0040  ///< alarm window lock
#define MCP9808_REG_CONFIG_INTCLR 0x0020     ///< interrupt clear
#define MCP9808_REG_CONFIG_ALERTSTAT 0x0010  ///< alert output status
#define MCP9808_REG_CONFIG_ALERTCTRL 0x0008  ///< alert output control
#define MCP9808_REG_CONFIG_ALERTSEL 0x0004   ///< alert output select
#define MCP9808_REG_CONFIG_ALERTPOL 0x0002   ///< alert output polarity
#define MCP9808_REG_CONFIG_ALERTMODE 0x0001  ///< alert output mode

#define MCP9808_REG_UPPER_TEMP 0x02   ///< upper alert boundary
#define MCP9808_REG_LOWER_TEMP 0x03   ///< lower alert boundery
#define MCP9808_REG_CRIT_TEMP 0x04    ///< critical temperature
#define MCP9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature
#define MCP9808_REG_MANUF_ID 0x06     ///< manufacture ID
#define MCP9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP9808_REG_RESOLUTION 0x08   ///< resolutin

// L2987n driver defines
#define SERIAL_STUDIO_DEBUG CONFIG_SERIAL_STUDIO_DEBUG
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                         // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                                                        // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAdata (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // madataimum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A 16
#define BDC_MCPWM_GPIO_B 15

// HTTP Client - FreeRTOS ESP IDF - GET
extern const uint8_t certificate_pem_start[] asm("_binary_certificate_pem_start"); // binary certificate start for ssl in https request
extern const uint8_t certificate_pem_end[] asm("_binary_certificate_pem_end");     // binary certificate end for ssl in https request

static const char *url_x = "https://capstone-database-c7175-default-rtdb.firebaseio.com/Flags/Motor.json";
static const char *url_time = "https://capstone-database-c7175-default-rtdb.firebaseio.com/Flags/Angle/Time.json"; // api endpoint to get the time value

float pwm_x; // value to get and store the pwm value for x
float pwm_y; // value to get and store the pwm value for y

// Tag for getting the data with ESP32 logger function
static const char *TAG = "ESP Logger: ";

// struct to store motor driver parameters
typedef struct
{
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_contedatat_t;

// Structure to hold accelerometer data
typedef struct ACCEL_DATA
{
    int16_t X;
    int16_t Y;
    int16_t Z;
} stACCEL_DATA_t;

// Structure to hold magnetometer data
typedef struct MAG_DATA
{
    uint16_t X;
    uint16_t Y;
    uint16_t Z;
} stMAG_DATA_t;

// variables for storing the time variables to calculate the azimuth and elevation angle
uint8_t day;
uint8_t month;
uint16_t year;
uint8_t hour;
uint8_t minute;
uint8_t second;
uint8_t temp;

stMAG_DATA_t magd;
stACCEL_DATA_t accd;
float tempd;

// Wifi handler function and status update function
static void
wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

// Function for connecting to the wifi with parameters defined in my_data.h
void wifi_connection()
{
    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation
    esp_event_loop_create_default();     // event loop
    esp_netif_create_default_wifi_sta(); // WiFi station
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); // initialize wifi with reference
    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {// create structure containing wifi connection parame
                                        .sta = {
                                            .ssid = SSID,
                                            .password = PASS}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

// get request handler to obtain data from https request
esp_err_t client_event_get_handler_time(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:

        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data); // printing to serial monitor our queried data

        uint8_t count = 0;
        uint8_t length = evt->data_len; // length of the
        char *data = (char *)evt->data;

        char *tz;

        while (*data)

        { // While there are more characters to process...
            char c = *data;
            if (isdigit(c))
            {
                // Found a number
                if (count == 0)
                {
                    day = (int)strtol(data, &data, 10); // Read number
                }
                else if (count == 1)
                {
                    month = (int)strtol(data, &data, 10); // Read number
                }
                else if (count == 2)
                {
                    year = (int)strtol(data, &data, 10);
                    ESP_LOGI(TAG, "Year: %d", year);
                }
                else if (count == 3)
                {
                    hour = (int)strtol(data, &data, 10);
                }
                else if (count == 4)
                {
                    minute = (int)strtol(data, &data, 10);
                }
                else if (count == 5)
                {
                    second = (int)strtol(data, &data, 10);
                    data++;
                    break;
                }

                count++;
            }
            else
            {
                // Otherwise, move on to the next character.
                data++;
            }
        }

        if (strcmp(data, "PM") == 0)
        {
            // PM block
            printf("PM\n");
            tz = "PM";
            printf("Night\n");
        }
        else if (strcmp(data, "AM") == 0)
        {
            // AM block
            printf("AM\n");
            tz = "AM";
            printf("Day\n");
        }

        break;

    default:
        break;
    }

    return ESP_OK;
}

// rest get function to perform https get request from a given url char* input
static void rest_get_time(char *url)
{

    esp_http_client_config_t config_get = {

        .url = url,
        .method = HTTP_METHOD_GET,
        .cert_pem = (const char *)certificate_pem_start,
        .event_handler = client_event_get_handler_time};

    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

// post request handler to obtain data from https request
esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        // if statement to handle HTTP request => if manual control, call new get request
        break;

    default:
        break;
    }
    return ESP_OK;
}

// rest post function to perform https get request from a given url char* input
static void post_rest_function()
{
    esp_http_client_config_t config_post = {
        .url = "https://7w4yj6h87i.edataecute-api.us-east-2.amazonaws.com/default/update-database",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};

    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    char *post_data = "{\"sensor\":\"Magnetometer1\",\"data\":1.345}";

    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

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
static void MMA845_init()
{
    uint8_t val;

    // Before re-configuring, must enter 'standby' mode
    rdMMA845x(CTRL_REG1, &(val), 1);
    val &= ~(ACTIVE_MASK);
    wrMMA845x(CTRL_REG1, &(val), 1);

    rdMMA845x(WHO_AM_I_REG, &(val), 1);
    // verifying the correct device id and sending result to logger
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
    val = (ASLP_RATE_20MS + DATA_RATE_80MS);
    wrMMA845x(CTRL_REG1, &(val), 1);

    // configure 2G full scale, High_Pass_Filter disabled
    val = (FULL_SCALE_2G);
    wrMMA845x(XYZ_DATA_CFG_REG, &(val), 1);

    // Setup Hi-Res mode (14-bit)
    rdMMA845x(CTRL_REG2, &(val), 1);
    val &= ~(MODS_MASK);
    val |= (MODS1_MASK);
    wrMMA845x(CTRL_REG2, &(val), 1);

    // Configure the INT pins for Open Drain and Active Low
    val = (PP_OD_MASK);
    wrMMA845x(CTRL_REG3, &(val), 1);

    // Enable the DRDY Interrupt
    val = (INT_EN_DRDY_MASK);
    wrMMA845x(CTRL_REG4, &(val), 1);

    // Set the DRDY Interrupt to INT1
    val = (INT_CFG_DRDY_MASK);
    wrMMA845x(CTRL_REG5, &(val), 1);

    // reconfig done, make active
    rdMMA845x(CTRL_REG1, &(val), 1);
    val |= (ACTIVE_MASK);
    wrMMA845x(CTRL_REG1, &(val), 1);
}

/* Read contents of a MMC5603NJ register
---------------------------------------------------------------------------*/
esp_err_t rdMMC560x(uint8_t reg, uint8_t *pdata, uint8_t count)
{
    return (i2c_master_read_slave_reg(I2C_PORT_NUM, MMC560_I2C_ADDR, reg, pdata, count));
}

/* Write value to specified MMC5603NJ register
---------------------------------------------------------------------------*/
esp_err_t wrMMC560x(uint8_t reg, uint8_t *pdata, uint8_t count)
{
    return (i2c_master_write_slave_reg(I2C_PORT_NUM, MMC560_I2C_ADDR, reg, pdata, count));
}

static void MMC560_init()
{
    uint8_t val;
    uint8_t test_read = 0x00;
    rdMMC560x(DEVICE_ID, &(val), 1);
    // verifying the correct device id and sending result to logger
    if (val == 0x10)
    {
        ESP_LOGI(TAG, "MMC560x ID:0x%X (ok)", val);
    }
    else
    {
        ESP_LOGE(TAG, "MMC560x ID:0x%X !!!! (NOT correct; should be 0x0A)", val);
    }

    /*
    **  Configure mangetometer for:
    */

    val |= 0xFF;               // flip all bits to high
    wrMMC560x(ODR, &(val), 1); // write for maximum output data rate
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    val = 0xA0;
    wrMMC560x(mCTRL_REG0, &(val), 1); // writing to set the Cmm_freq_en and Auto_SR_en registers to active
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    rdMMC560x(mCTRL_REG0, &(test_read), 1); // write for maximum output data rate
    val = 0x03;
    wrMMC560x(mCTRL_REG1, &(val), 1); // writing to set the bandwidth corresponding to period of 1.2 ms
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    val = 0x10;
    wrMMC560x(mCTRL_REG2, &(val), 1); // writing to set the continous mode to active
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/* Read contents of a MMC5603NJ register
---------------------------------------------------------------------------*/
esp_err_t rdMCP980x(uint8_t reg, uint8_t *pdata, uint8_t count)
{
    return (i2c_master_read_slave_reg(I2C_PORT_NUM, MCP9808_I2CADDR, reg, pdata, count));
}

/* Write value to specified MCP980 register
---------------------------------------------------------------------------*/
esp_err_t wrMCP980x(uint8_t reg, uint8_t *pdata, uint8_t count)
{
    return (i2c_master_write_slave_reg(I2C_PORT_NUM, MCP9808_I2CADDR, reg, pdata, count));
}

static void MCP98_init()
{
    uint8_t val;
    uint8_t test_read = 0x00;
    // verifying the correct device id and sending result to logger
    rdMCP980x(MCP9808_REG_DEVICE_ID, &(val), 1);
    if (val == 0x04)
    {
        ESP_LOGI(TAG, "MCP98x ID:0x%X (ok)", val);
    }
    else
    {
        ESP_LOGE(TAG, "MCP98x ID:0x%X !!!! (NOT correct; should be 0x04)", val);
    }
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
    // swap the bits
    return ((data >> 8) | (data << 8));
}

esp_err_t i2c_acc_sample() // function to capture accelerometer data into struct and log
{
    esp_err_t err;
    // ESP_LOGI(TAG, "ESP I2C_RESTART Example - MMA8451 Accelerometer");
    // while (1)
    // {
    // Note: as configured, reading data from the output registers will start next acquisition
    err = rdMMA845x(MMA8451_OUT_X_MSB, (uint8_t *)&accd, sizeof(accd));

    // byte-swap values to make little-endian
    accd.X = byte_swap(accd.X);
    accd.Y = byte_swap(accd.Y);
    accd.Z = byte_swap(accd.Z);

    // shift each value to align 14-bits in 16-bit ints
    accd.X /= 4;
    accd.Y /= 4;
    accd.Z /= 4;

    float x = (accd.X * (9.80665 / 4096)); // converting acc.X reading to m/s^2
    float y = (accd.Y * (9.80665 / 4096)); // converting acc.Y reading to m/s^2
    float z = (accd.Z * (9.80665 / 4096)); // converting acc.Z reading to m/s^2

    ESP_LOGI(TAG, "Accelerometer err:%d  x:%5f  y:%5f  z:%5f", err, x, y, z);
    // printf("%f,%f,%f\n", x, y, z); // for CSV collection

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    return err;
}

esp_err_t i2c_mag_sample() // function to capture magnetomer data into struct and log
{
    esp_err_t err;        // error to log errors
    uint8_t temp_reading; // temperature reading
    uint8_t status;       // get the status of the i2c slave device
    uint8_t val;          // val is a placeholder for writing data ( need a l-value )

    ESP_LOGI(TAG, "ESP I2C_RESTART Example - MMC5603NJ Magnetometer");
    // while (1)
    // {
    // Note: as configured, reading data from the output registers will start next acquisition

    err = rdMMC560x(X_MSB, (uint8_t *)&magd, sizeof(magd)); // read the data into the magnetometer device

    // byte-swap values to make little-endian
    magd.X = byte_swap(magd.X); // byte swap the x value to convert from little endian to big endian
    magd.Y = byte_swap(magd.Y); // byte swap the x value to convert from little endian to big endian
    magd.Z = byte_swap(magd.Z); // byte swap the x value to convert from little endian to big endian

    double x = ((magd.X / 1024) - 30) / 10; // convert to microtesla
    double y = ((magd.Y / 1024) - 30) / 10; // convert to microtesla
    double z = ((magd.Z / 1024) - 30) / 10; // convert to microtesla

    // float bearing = atan2(x, y) * 180 / M_PI;

    ESP_LOGI(TAG, "Magnetometer Reading raw err:%d  x:%d  y:%d  z:%d", err, magd.X, magd.Y, magd.Z); // log results
    ESP_LOGI(TAG, "Magnetometer Reading raw err: x:%.3f y:%.3f z:%f", x, y, z);                      // log results

    val = 0xa2;
    wrMMC560x(mCTRL_REG0, &(val), 1); // writing to set the Cmm_freq_en and Auto_SR_en registers to active while simultaneously getting temperature reading
    err = rdMMC560x(0x09, (uint8_t *)&status, sizeof(status));
    ESP_LOGI(TAG, "err %d, Status %d", err, status);
    err = rdMMC560x(mTEMP, (uint8_t *)&temp_reading, 1);
    ESP_LOGI(TAG, "Temperature Reading raw err:%d  Temp:%d", err, temp_reading);
    float t_translated = (0.784 * temp_reading) - 75;
    ESP_LOGI(TAG, "Temperature Reading err:%d  Temp:%f", err, t_translated);

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    return err;
}

esp_err_t i2c_temp_sample()
{
    esp_err_t err = ESP_OK;                                // error to log errors
    uint16_t v;                                            // variable to read in the data from register
    rdMCP980x(MCP9808_REG_AMBIENT_TEMP, (uint8_t *)&v, 2); // casting v to an int
    ESP_LOGI(TAG, "RAW: %d", v);                           // debug print statement
    v = byte_swap(v);                                      // swap the v value from little endian to big endian
    ESP_LOGI(TAG, "BYTE SWAPPED: %d", v);                  // debug print statement

    if (v == 0xFFFF) // if all the values are one, erroneous data transfer has likely occured, return fail
    {
        err = ESP_FAIL; // return the failed value
        return err;
    }

    tempd = v & 0x0FFF; // mask upper 4 bits to get actual reading data
    tempd /= 16.0;      // divide by 16 to lose decimal precision bits

    if (v & 0x1000)
    {
        tempd -= 256;
    }

    // getting decimal places f
    if (v & 0x0008)
    {
        tempd += pow(2, -1);
    }
    else if (v & 0x0004)
    {
        tempd += pow(2, -2);
    }
    else if (v & 0x0002)
    {
        tempd += pow(2, -3);
    }
    else if (v & 0x0001)
    {
        tempd += pow(2, -4);
    }

    ESP_LOGI(TAG, "Temperature Reading err:%d  Temp:%f degrees C", err, tempd);
    return err;
}

static void sensor_routine()
{
    esp_err_t err;
    err = i2c_acc_sample(); // aggregating acceleration into data
    ESP_LOGI(TAG, "Accelerometer Reading Error Bit:%d", err);
    // esp_err_t i2c_mag_sample(); // aggreating magnetometer into data
    // ESP_LOGI(TAG, "Magnetometer Reading Error Bit:%d", err);
    err = i2c_temp_sample(); // aggregating temperature into data
    ESP_LOGI(TAG, "Temperature Reading Error Bit:%d", err);

    // TODO: Implement post routine to add data to database !
}

void app_main()
{
    nvs_flash_init();  // flash initialization
    wifi_connection(); // run routines to connect to the wifi
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // ESP_LOGI(TAG, "Caught key time parametrics with following parameters: Day: %d Month: %d Year: %d Hour: %d Minute: %d Second: %d", day, month, year, hour, minute, second);

    // I2C Serial Communication Commands Configuration
    i2c_master_init(); // initialize I2C serial commmunication with the esp32 and set internal pull-ups / SDA / SCL lines

    // Sensor initialization and control register configurations specific to device
    MMA845_init(); // initialize the accelerometer by adjusting the control registers to desired polling / resolution settings
    MMC560_init(); // initalize the magnetometer by adjusting the control registers to desired polling / resolution settings
    MCP98_init();  // initalize the temperature sensor by adjusting the control registers to desired polling / resolution settings

    // sensor_routine();

    while (1)
    {
        i2c_temp_sample();
        vTaskDelay(10000 / portTICK_PERIOD_MS); // take measurement every 10 seconds
    }
    // // Simple test for device id
    // uint8_t val[2];
    // rdMMA845x(WHO_AM_I_REG, &(val), 1); // simple test to verify the right device id of the accelerometer, MMA8451Q
    // printf("%X\n", val[0]);
    // rdMMC560x(DEVICE_ID, &(val), 1); // simple test to verify the right device id of the magnetometer, MMC5603NJ
    // printf("%X\n", val[0]);

    // /xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL); // use for when continuously polling data ! uncomment while loop in i2c_test_task

    // Jordan's Code for azimuth / elevation computation

    //     int count = 0;
    //     int day;
    //     int month;
    //     int year;
    //     int hour;
    //     int minute;
    //     int second;
    //     char* tz;
    //     char * data = "2/20/2023, 2:34:12 PM";

    //     while (*data) { // While there are more characters to process...
    //     if ( isdigit(*data) ) {
    //         // Found a number
    //         if (count == 0) {
    //             day = (int)strtol(data,&data, 10); // Read number
    //         } else if ( count == 1 ) {
    //             month = (int)strtol(data,&data, 10); // Read number
    //         } else if ( count == 2 ) {
    //             year = (int)strtol(data,&data, 10);
    //         } else if ( count == 3 ) {
    //             hour = (int)strtol(data,&data, 10);
    //         } else if ( count == 4 ) {
    //             minute = (int)strtol(data,&data, 10);
    //         } else if ( count == 5 ) {
    //             second = (int)strtol(data,&data, 10);
    //             data++;
    //             break;
    //         }

    //         count++;

    //     } else {
    //         // Otherwise, move on to the next character.
    //         data++;
    //     }
    // }

    //     if (strcmp(data, "PM") == 0) {
    //         // PM block
    //     } else if (strcmp(data, "AM") == 0) {
    //         AM block
    //     }
    // //}

    // get local time and day from library
    // double current_day;
    // int current_month;
    // int current_year;
    // float current_hour;
    // float current_minutes;
    // float longitude = -96.326; // longitude of college station
    // float latitude = 30.621;   // latitude of college station

    // // algorithm to get current time and convert the minutes into hours
    // time_t now;
    // time(&now);
    // struct tm *CST = localtime(&now); // this gets the current GMT time so hours and sometimes the day will need to be changed to match CST
    // current_hour = CST->tm_hour - 6;  // get hours since midnight (0-23), the minus 6 is to correct to CST
    // current_minutes = CST->tm_min;    // get minutes passed after the hour (0-59)

    // if (current_hour < 0)
    // { // to account for CST since the code gets the GMT
    //     current_hour += 24;
    // }
    // current_hour += (current_minutes / 60); // converts minutes into hours and gets added to the current hour
    // printf("The current time in hours is: %f \n", current_hour);

    // // algorithm to get day of year (example: Jan 1 is 1, Feb 1 is 32)
    // current_day = CST->tm_mday; // get day of month (1 to 31)
    // if (18.0 <= current_hour && current_hour < 24)
    // { // depending on the current hour of the GMT, the day might need to be adjusted back one
    //     current_day -= 1;
    // }
    // current_month = CST->tm_mon + 1;    // get month of year (0 to 11)
    // current_year = CST->tm_year + 1900; // get year from 1900
    // float doy = current_day;            // variable to store the current day of year
    // float days_in_feb = 28;             // in order to account for leap years
    // printf("The current month, day, and year are: %d:%f:%d \n", current_month, current_day, current_year);
    // if ((current_year % 4 == 0 && current_year % 100 != 0) || (current_year % 400 == 0))
    // { // check for leap year
    //     days_in_feb = 29;
    // }
    // // switch block to determine how many days based on which month it is
    // switch (current_month)
    // {
    // case 2:
    //     doy += 31;
    //     break;
    // case 3:
    //     doy += 31 + days_in_feb;
    //     break;
    // case 4:
    //     doy += 31 + days_in_feb + 31;
    //     break;
    // case 5:
    //     doy += 31 + days_in_feb + 31 + 30;
    //     break;
    // case 6:
    //     doy += 31 + days_in_feb + 31 + 30 + 31;
    //     break;
    // case 7:
    //     doy += 31 + days_in_feb + 31 + 30 + 31 + 30;
    //     break;
    // case 8:
    //     doy += 31 + days_in_feb + 31 + 30 + 31 + 30 + 31;
    //     break;
    // case 9:
    //     doy += 31 + days_in_feb + 31 + 30 + 31 + 30 + 31 + 31;
    //     break;
    // case 10:
    //     doy += 31 + days_in_feb + 31 + 30 + 31 + 30 + 31 + 31 + 30;
    //     break;
    // case 11:
    //     doy += 31 + days_in_feb + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31;
    //     break;
    // case 12:
    //     doy += 31 + days_in_feb + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30;
    //     break;
    // }
    // printf("The current day of year is %f \n", doy);

    // // algorithm to calculate azimuth and elevation angle
    // float LSTM = -90.0; // Local solar time meridian for CST in degrees
    // float B = (360.0 / 365.0) * (doy - 81.0);
    // float EoT = 9.87 * sin(2.0 * B * M_PI / 180.0) - 7.53 * cos(B * M_PI / 180.0) - 1.5 * sin(B * M_PI / 180.0); // equation of time formula
    // float TC = 4.0 * (longitude - LSTM) + EoT;                                                                   // time correction formula
    // float LST = current_hour + (TC / 60.0);                                                                      // local solar time formula
    // float HRA = 15.0 * (LST - 12.0);                                                                             // hour angle is in degrees
    // float dec_angle = 23.45 * sin(B * M_PI / 180.0);                                                             // declination angle
    // float elevation_angle = asin(sin(dec_angle * M_PI / 180.0) * sin(latitude * M_PI / 180.0) + cos(dec_angle * M_PI / 180.0) * cos(latitude * M_PI / 180.0) * cos(HRA * M_PI / 180.0));
    // elevation_angle = elevation_angle * 180.0 / M_PI;                                                                                                                                                                          // converts from radians to degrees
    // float azimuth_angle = acos((sin(dec_angle * M_PI / 180.0) * cos(latitude * M_PI / 180.0) - cos(dec_angle * M_PI / 180.0) * sin(latitude * M_PI / 180.0) * cos(HRA * M_PI / 180.0)) / cos(elevation_angle * M_PI / 180.0)); // numerator of the azimuth angle calculation
    // azimuth_angle = 360 - (azimuth_angle * 180.0 / M_PI);                                                                                                                                                                      // converts from radians to degrees
    // printf("The elevation angle is: %f \n", elevation_angle);
    // printf(" The azimuth angle is: %f \n", azimuth_angle);

    // // use azimuth angle and elevation angle to determine where to move motors
    // double tilt_angle = 90 - elevation_angle; // since the fresnel lens is pointing upwards in the home position, the motors will move the difference between home and the elevation angle
    // // code to move rotation motor according the azimuth angle and the tilt motor according to the elevation
}

// void app_main(void)
// {

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