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

// STNP

#include <time.h>
#include <sys/time.h>
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_sntp.h"

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
#define MCP9808_I2CADDR 0x18          ///< I2C address
#define MCP9808_REG_CONFIG 0x01       ///< MCP9808 config register
#define MCP9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature
#define MCP9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP9808_REG_RESOLUTION 0x08   ///< resolution

// L2987n driver defines
#define SERIAL_STUDIO_DEBUG CONFIG_SERIAL_STUDIO_DEBUG
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                         // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                                                        // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAdata (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // madataimum value we can set for the duty cycle, in ticks

// Motor one
#define BDC_MCPWM_GPIO_A_1 5
#define BDC_MCPWM_GPIO_B_1 18

// Motor two
#define BDC_MCPWM_GPIO_A_2 32
#define BDC_MCPWM_GPIO_B_2 33

// HTTP Client - FreeRTOS ESP IDF - GET
extern const uint8_t certificate_pem_start[] asm("_binary_certificate_pem_start"); // binary certificate start for ssl in https request
extern const uint8_t certificate_pem_end[] asm("_binary_certificate_pem_end");     // binary certificate end for ssl in https request

// HTTP Client - FreeRTOS ESP IDF - GET
// extern const uint8_t certificate_pem_start2[] asm("_binary_certificate_pem2_start"); // binary certificate start for ssl in https request
// extern const uint8_t certificate_pem_end2[] asm("_binary_certificate_pem2_end");     // binary certificate end for ssl in https request

int8_t pwm_x; // value to get and store the pwm value for x
int8_t pwm_y; // value to get and store the pwm value for y

// Tag for getting the data with ESP32 logger function
static const char *TAG = "ESP Logger: ";

// struct to store motor driver parameters
typedef struct
{
    bdc_motor_handle_t motor;         // motor object for control
    pcnt_unit_handle_t pcnt_encoder;  // pcnt encoder for QEP (quardature encoded pulse feedback)
    pid_ctrl_block_handle_t pid_ctrl; // pid control block (unused curently)
    int report_pulses;                // pulse reporter for pid control loop (unused curently)
} motor_control_contedatat_t;

// Structure to hold accelerometer data
typedef struct ACCEL_DATA
{
    int16_t x; // X stores the data for the raw X coordinate accelerometer data
    int16_t y; // Y stores the data for the raw Y coordinate accelerometer data
    int16_t z; // Z stores the data for the raw Z coordinate accelerometer data
} stACCEL_DATA_t;

// Structure to hold magnetometer data
typedef struct MAG_DATA
{
    uint16_t x; // X stores the data for the raw X coordinate magnetometer data
    uint16_t y; // Y stores the data for the raw Y coordinate mangetometer data
    uint16_t z; // Z stores the data for the raw Z coordinate magnetometer data
} stMAG_DATA_t;

typedef struct MAG_CAL
{
    double x; // X stores the data for the calibrated X coordinate magnetometer data (conversion made in sensor routine with hard coded offsets)
    double y; // Y stores the data for the calibrated Y coordinate mangetometer data (conversion made in sensor routine with hard coded offsets)
    double z; // Z stores the data for the calibrated Z coordinate magnetometer data (conversion made in sensor routine with hard coded offsets)
} stMAG_CAL_t;

typedef struct ANGLE_DATA
{
    double actual_azimuth;     // actual azimuth stores the azimuth angle as computed from magnetometer and accelerometer data ( current value )
    double computed_azimuth;   // computed azimuth is what the azimuth is based on the time and geographical latitude/longitude ( desired value )
    double actual_elevation;   // actual eleevation stores the elevation angle as computed from accelerometer data ( current value )
    double computed_elevation; // computed elevation stores the elevation angle as computed from the accelerometer data( desired value )
} stANGLE_t;

// global variables for storing the time from NTP servers (network time protocol)
double day;    // variable to get the day
double month;  // variable to get the month
double year;   // variable to get the year
double doy;    // variable to get day of the year
double hour;   // variable to get the hour
double minute; // variable to get the minute
double second; // variable to get the second
double t_a;    // ambient temperature from magnetometer on chip sensor

const float longitude = -96.326; // latitude of college station for computed elevation/azimuth angle
const float latitude = 30.3;     // longitude of college station for computed elevation/azimuth anglechange to actual latitude only used 30.3 for sun position calculator website

stMAG_DATA_t magd;   // structure for storing magnetometer data
stACCEL_DATA_t accd; // structure for storing accelerometer data
stANGLE_t angd;      // structure for storing angle data
stMAG_CAL_t mcal;    // structure for calibrating the magnetometer data
float t_m;           // variable for storing ambient temperature

// Hard-iron calibration settings (from calibration software, MotionCal)
const float hard_iron[3] = {7.98, 18.51, 86.05};

// Soft-iron calibration settings (from calibration software, MotionCal)
const float soft_iron[3][3] = {
    {0.995, -0.005, -0.009},
    {-0.005, 0.978, -0.047},
    {-0.009, -0.047, 1.030}};

// Wifi handler function and status update function
// adapted from online example youtube video
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
// adapted from online youtube video
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
    esp_wifi_set_mode(WIFI_MODE_STA);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

// get request handler to obtain data from https request
// adapted from online youtube video
esp_err_t client_event_get_handler_pwm(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:

        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data); // printing to serial monitor our queried data

        uint8_t count = 0;
        char *data = (char *)evt->data;

        while (*data)
        {
            char c = *(data);
            if ((c == '-') | isdigit(c))
            {
                if (count == 0)
                {
                    pwm_x = (int8_t)strtol(data, &data, 10);
                    ++count;
                }
                else if (count == 1)
                {
                    pwm_y = (int8_t)strtol(data, &data, 10);
                    ++count;
                }
            }
            ++data;
        }

        break;

    default:
        break;
    }

    return ESP_OK;
}

// rest get function to perform https get request from a given url char* input
// adapted from online youtube video
static void rest_get_pwm()
{
    static const char *url_pwm = "https://capstone-database-c7175-default-rtdb.firebaseio.com/Flags/Motor.json";
    esp_http_client_config_t config_get = {

        .url = url_pwm,
        .method = HTTP_METHOD_GET,
        .cert_pem = (const char *)certificate_pem_start,
        .event_handler = client_event_get_handler_pwm};

    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

// post request handler to obtain data from https request
// adapted from online youtube video
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
// adapted from online youtube video with some modification
static void post_rest_function(int sensor)
{
    esp_http_client_config_t config_post = {
        .url = "https://7w4yj6h87i.execute-api.us-east-2.amazonaws.com/default/update-database",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};

    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    // Post data names abbreviated to save space and work efficiently at transferring data

    // acronyms our shown in the comment below detailing structure of the JSON post data
    // {
    //     id : (Sensor ID String), (0)
    //     x  : (Reading from accelerometer / magnetometer units: m/s^2 or uT), (1)
    //     y  : (Reading from accelerometer / magnetometer units: m/s^2 or uT), (2)
    //     z  : (Reading from accelerometer / magnetometer units: m/s^2 or uT), (3)
    //     actual_azimuth_angle : (calculated azimuth angle of frame from sensors), (4)
    //     computed_azimuth_angle : (computed azimuth angle based on time of day and time), (5)
    //     actual_elevation_angle : (calculated elevation angle of frame from sensors), (6)
    //     computed_elevation_angle : (computed elevation angle based on time of day and time), (7)
    //     ambient_temperature : (temperature reading taken from on-chip magnetometer), (8)
    //     target_temperature : (temperature reading taken near target for reference) (9)
    // }

    // note time will be processed in the lambda script.

    char buffer[200]; // create space for the data to be formatted into a buffer string

    char *post_data = &buffer;
    // x / y / z values for the post req
    double x;
    double y;
    double z;
    // angles for the post req
    double c_e;
    double a_e;
    double c_a;
    double a_a;

    // sensor basically switches which data is being posted, in continuous sampling mode two calls are made to this function
    // one of the requests posts the acceleration data (input variable sensor=0) while the other posts the magnetometer data (input variable sensor=1)
    switch (sensor)
    {
    case (0):                                                                                                                                                                                             // for case 0 we get the acceleration data and write the struct data to post
        x = (accd.x * (9.80665 / 4096));                                                                                                                                                                  // converting acc.x reading to m/s^2
        y = (accd.y * (9.80665 / 4096));                                                                                                                                                                  // converting acc.y reading to m/s^2
        z = (accd.z * (9.80665 / 4096));                                                                                                                                                                  // convert the acc.z reading to m/s^2
        c_e = angd.computed_elevation;                                                                                                                                                                    // get the computed elevation angle into a double variable
        a_e = angd.actual_elevation;                                                                                                                                                                      // get the actual elevation agnle into a double variable                                                                                                                                               // converting acc.Z reading to m/s^2
        sprintf(buffer, "{\"sensor\":\"Accelerometer1\",\"x\":\"%.1f\",\"y\":\"%.1f\",\"z\":\"%.1f\",\"e_a\":\"%.1f\",\"e_c\":\"%.1f\",\"t_a\":\"%.1f\",\"t_m\":\"%.1f\"}", x, y, z, c_e, a_e, t_a, t_m); // formatting the data obtained from the accelerometer struct
        break;
    case (1):                                                                                                                                                                                            // for case 0 we get the magnetometer data and write the struct data to post
        x = magd.x;                                                                                                                                                                                      // converting acc.x reading to uTesla
        y = magd.y;                                                                                                                                                                                      // converting acc.y reading to uTesla
        z = magd.z;                                                                                                                                                                                      // converting magd.z reading to uTesla
        c_a = angd.computed_azimuth;                                                                                                                                                                     // get the computed elevation angle into a double variable
        a_a = angd.actual_azimuth;                                                                                                                                                                       // get the actual elevation agnle into a double variable                                                                                                                                                             // converting acc.Z reading to uTesla
        sprintf(buffer, "{\"sensor\":\"Magnetometer1\",\"x\":\"%.1f\",\"y\":\"%.1f\",\"z\":\"%.1f\",\"a_a\":\"%.1f\",\"c_a\":\"%.1f\",\"t_a\":\"%.1f\",\"t_m\":\"%.1f\"}", x, y, z, a_a, c_a, t_a, t_m); // formatting the data obtained from the magnetometer struct
        break;
    default:
        ESP_LOGI(TAG, "Error in the input parameter, input to post function should be integer, specifically: \n0 : Post Accelerometer data\n1: Post Magnetometer data \n2: Post mock data generated from random function");
    }

    esp_http_client_set_post_field(client, post_data, strlen(post_data));   // setting the corresponding fields of the post request
    esp_http_client_set_header(client, "Content-Type", "application/json"); // setting th header attributes of the request
    esp_http_client_perform(client);                                        // performing the clients request
    esp_http_client_cleanup(client);                                        // cleaning up the client structure
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
 * @brief MMA8451 initialization (Accelerometer)
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

/**
 * @brief MMC560 initialization (Magnetometer)
 */
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

    val |= 0xFF;                           // flip all bits to high
    wrMMC560x(ODR, &(val), 1);             // write for maximum output data rate
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for a second
    val = 0x80;                            // write the binary value 10100000 to control register 0
    wrMMC560x(mCTRL_REG0, &(val), 1);      // writing to set the Cmm_freq_en
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for a second
    val |= 0x20;
    ESP_LOGI(TAG, "updated val %d", val);
    wrMMC560x(mCTRL_REG0, &(val), 1);      // writing to set the to_SR_en registers to active
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for a second
    val = 0x03;                            // write the binary value 00000011 to control register 1
    wrMMC560x(mCTRL_REG1, &(val), 1);      // writing to set the bandwidth corresponding to period of 1.2 ms
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for a second
    val = 0x10;                            // writing the binary value 00010000 to control register 2
    wrMMC560x(mCTRL_REG2, &(val), 1);      // writing to set the continous mode to active
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
    // Note: as configured, reading data from the output registers will start next acquisition
    err = rdMMA845x(MMA8451_OUT_X_MSB, (uint8_t *)&accd, sizeof(accd));
    ESP_LOGI(TAG, "Error bit for reading the magnetometer data: %d", err);

    // byte-swap values to make little-endian
    accd.x = byte_swap(accd.x);
    accd.y = byte_swap(accd.y);
    accd.z = byte_swap(accd.z);

    // shift each value to align 14-bits in 16-bit ints
    accd.x /= 4;
    accd.y /= 4;
    accd.z /= 4;

    float x = (accd.x * (9.80665 / 4096)); // converting acc.x reading to m/s^2
    float y = (accd.y * (9.80665 / 4096)); // converting acc.y reading to m/s^2
    float z = (accd.z * (9.80665 / 4096)); // converting acc.z reading to m/s^2

    ESP_LOGI(TAG, "Accelerometer Reading Raw err:%d  x:%d  y:%d  z:%d", err, accd.x, accd.y, accd.z); // log the raw accelerometer readings
    ESP_LOGI(TAG, "Accelerometer Reading err:%d  x:%.3f  y:%.3f  z:%.3f", err, x, y, z);              // log the converted accelerometer readings
    // printf("%f,%f,%f\n", x, y, z); // for CSV collection in testing
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS)); // delay by the sample period
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

    // attempting a single read for the magnetometer x, y, z data register
    // val |= 0x03;                      // flip all bits to high
    // wrMMC560x(mCTRL_REG0, &(val), 1); // write for maximum output data rate
    uint16_t bits;
    uint32_t bits_cont;
    uint8_t d;
    uint32_t manipulated;

    // err = rdMMC560x(X_MSB, (uint16_t *)&magd, sizeof(magd)); // read the data into the magnetometer device

    err = rdMMC560x(X_MSB, (uint16_t *)&bits, sizeof(bits)); // reading x - values (upper)
    bits = byte_swap(bits);                                  // byte-swap values to make little-endian
    bits_cont = 0x0000 | bits;                               // containerize into larger container
    err = rdMMC560x(0x06, (uint8_t *)&d, sizeof(d));         // reading x - values (lower)
    manipulated = (0x0000) | (bits_cont << 4) | (d >> 4);    // bit wrangling to get the proper data structure
    ESP_LOGI(TAG, "After manipulation, the acquired value is %lu", manipulated);
    magd.x = manipulated;                                    // store x bit wrangled data into struct
    err = rdMMC560x(Y_MSB, (uint16_t *)&bits, sizeof(bits)); // read the y - values (upper)
    bits = byte_swap(bits);                                  // byte-swap values to make little-endian
    bits_cont = 0x0000 | bits;                               // containerize into larger container
    err = rdMMC560x(0x07, (uint8_t *)&d, sizeof(d));         // reading y - values (lower)
    manipulated = (0x0000) | (bits << 4) | (d >> 4);         // bit wrangling to get the proper data structure
    magd.y = manipulated;                                    // store y bit wrangled data into struct
    err = rdMMC560x(Z_MSB, (uint16_t *)&bits, sizeof(bits)); // read the z values (upper)
    bits = byte_swap(bits);                                  // byte-swap values to make little-endian
    bits_cont = 0x0000 | bits;                               // containerize into larger container
    err = rdMMC560x(0x08, (uint8_t *)&d, sizeof(d));         // reading z - values (lower)
    manipulated = (0x0000) | (bits_cont << 4) | (d >> 4);    // bit wrangling to get the proper data structure
    magd.z = manipulated;                                    // store z bit wrangled data into struct

    // double x = ((magd.x / 1024) - 30) / 10; // convert to microtesla
    // double y = ((magd.y / 1024) - 30) / 10; // convert to microtesla
    // double z = ((magd.z / 1024) - 30) / 10; // convert to microtesla

    double hi_cal[3] = {(((magd.x / 1024) - 30) / 10) - hard_iron[0], (((magd.y / 1024) - 30) / 10) - hard_iron[1], (((magd.z / 1024) - 30) / 10) - hard_iron[2]};

    for (int i = 0; i < 3; i++)
    {
        switch (i) // switch based of the index
        {
        case (0): // get the x value into the calibrated reading struct
            mcal.x = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
            break;
        case (1): // get the y value into the calibrated reading struct
            mcal.y = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
            break;
        case (2): // get the z value into the calibrated reading struct
            mcal.z = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
            break;
        default: // in the default case break the switch ( no use case here but for good practice included in statement )
            break;
        }
    }

    angd.actual_azimuth = atan2(mcal.y, mcal.x) * 180 / M_PI; // store actual aimuth into respective struct

    if (angd.actual_azimuth < 0)
    {
        angd.actual_azimuth = 360 + angd.actual_azimuth;
    }

    ESP_LOGI(TAG, "Magnetometer Reading raw err: %d  x: %d  y: %d  z: %d", err, magd.x, magd.y, magd.z);                  // log result for the raw reading
    ESP_LOGI(TAG, "Magnetometer Reading Calibrated and Scaled err: %d  x: %f  y: %f  z:%f", err, mcal.x, mcal.y, mcal.z); // log result for calibrated/scaled reading
    ESP_LOGI(TAG, "Magnetometer compass measurement reading: %f", angd.actual_azimuth);                                   // log result for compass reading

    // val = 0xa2;
    // wrMMC560x(mCTRL_REG0, &(val), 1); // writing to set the Cmm_freq_en and Auto_SR_en registers to active while simultaneously getting temperature reading
    // err = rdMMC560x(0x09, (uint8_t *)&status, sizeof(status));
    // ESP_LOGI(TAG, "err %d, Status %d", err, status);
    // err = rdMMC560x(mTEMP, (uint8_t *)&temp_reading, 1);
    // ESP_LOGI(TAG, "Temperature Reading raw err:%d  Temp:%d", err, temp_reading);
    // t_m = (0.784 * temp_reading) - 75;
    // ESP_LOGI(TAG, "Temperature Reading err:%d  Temp:%f", err, t_m);

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    return err;
}

esp_err_t i2c_temp_sample()
{
    esp_err_t err = ESP_OK;                                // error to log errors
    uint16_t v;                                            // variable to read in the data from register
    rdMCP980x(MCP9808_REG_AMBIENT_TEMP, (uint8_t *)&v, 2); // casting v to an int
    // ESP_LOGI(TAG, "RAW: %d", v);                           // debug print statement
    v = byte_swap(v); // swap the v value from little endian to big endian
    // ESP_LOGI(TAG, "BYTE SWAPPED: %d", v);                  // debug print statement

    if (v == 0xFFFF) // if all the values are one, erroneous data transfer has likely occured, return fail
    {
        err = ESP_FAIL; // return the failed value
        return err;
    }

    t_a = v & 0x0FFF; // mask upper 4 bits to get actual reading data
    t_a /= 16.0;      // divide by 16 to lose decimal precision bits

    // checking if the temperature is positive or negative
    if (v & 0x1000)
    {
        t_a -= 256;
    }

    // getting decimal places for increased precision
    if (v & 0x0008)
    {
        t_a += pow(2, -1);
    }
    else if (v & 0x0004)
    {
        t_a += pow(2, -2);
    }
    else if (v & 0x0002)
    {
        t_a += pow(2, -3);
    }
    else if (v & 0x0001)
    {
        t_a += pow(2, -4);
    }

    ESP_LOGI(TAG, "Temperature Reading err:%d  Temp:%f degrees C", err, t_a);
    return err;
}

static void sensor_routine()
{
    esp_err_t err;
    err = i2c_acc_sample(); // aggregating accelerometer data into data struct
    if (err != ESP_OK)
    { // logging error if exists for temperature sensor
        ESP_LOGI(TAG, "Accelerometer Reading Error Bit:%d", err);
    }
    err = i2c_mag_sample(); // aggregating magnetometer data into data struct
    if (err != ESP_OK)
    { // logging error if exists for temperature sensor
        ESP_LOGI(TAG, "Magnetometer Reading Error Bit:%d", err);
    }
    err = i2c_temp_sample(); // aggregating temperature data into data struct
    if (err != ESP_OK)
    { // logging error if exists for temperature sensor
        ESP_LOGI(TAG, "Temperature Reading Error Bit:%d", err);
    }
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

static void obtain_time(void)
{

    initialize_sntp();
    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
}
void Set_SystemTime_SNTP()
{
    // int test_hours;
    time_t now;
    struct tm timeinfo;

    time(&now);

    localtime_r(&now, &timeinfo);
    struct tm *local = localtime(&now);
    // test_hours = local->tm_hour;
    // printf("Current Hours: %d", test_hours);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900))
    {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
}

void Get_current_date_time()
{
    char strftime_buf[64];
    time_t now;
    struct tm timeinfo;
    int test_hours, test_minutes, test_seconds, test_month, test_day, test_year, test_doy;

    time(&now);
    localtime_r(&now, &timeinfo);

    // Set timezone to Texas time
    setenv("TZ", "UTC+6", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    struct tm *local = localtime(&now);
    test_hours = local->tm_hour;
    test_minutes = local->tm_min;
    test_seconds = local->tm_sec;
    test_month = local->tm_mon;
    test_day = local->tm_mday + 1;
    test_year = local->tm_year + 1900;
    test_doy = local->tm_yday + 1;
    // TIME TESTING FOR STNP TIME SYNC AND DATA
    // printf("Current Hours: %d\n", test_hours);
    // printf("Current Mins: %d\n", test_minutes);
    // printf("Current Seconds: %d\n", test_seconds);
    // printf("Current Month: %d\n", test_month);
    // printf("Current Day: %d\n", test_day);
    // printf("Current Year: %d\n", test_year);
    // printf("Current Day of Year: %d\n", test_doy);
    hour = test_hours;
    minute = test_minutes;
    second = test_seconds;
    month = test_month;
    day = test_day;
    year = test_year;
    doy = test_doy;

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in College Station is: %s", strftime_buf);
}

void app_main()
{

    // INITIALIZATIONS
    // includes initializing flash memory, connecting to wifi, configuring the 3 sensors via I2C, setting the system time, and the motors
    nvs_flash_init();  // flash initialization
    wifi_connection(); // run routines to connect to the wifi
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // i2c_master_init(); // initialize I2C serial commmunication with the esp32 and set internal pull-ups / SDA / SCL lines
    // MMA845_init();     // initialize the accelerometer by adjusting the control registers to desired polling / resolution settings
    // MMC560_init();     // initialize the magnetometer by adjusting the control registers to desired polling / resolution settings
    // // // MCP98_init();      // initialize the temperature sensor by adjusting the control registers to desired polling / resolution settings

    // while (1)
    // {
    //     esp_err_t mag_samp = i2c_mag_sample(); // collect the i2c magnetometer data
    //     esp_err_t acc_samp = i2c_acc_sample(); // collect the i2c accelerometer dara
    //     ESP_ERROR_CHECK(mag_samp);             // check for errors in the magnetometer
    //     ESP_ERROR_CHECK(acc_samp);             // check for errors in the accelerometer
    //     vTaskDelay(5000 / portTICK_PERIOD_MS); // delay by 5 seconds for another collection
    // }

    Set_SystemTime_SNTP(); // configuring the system time and sync with the system network time protocol
    // Get_current_date_time(); // get current time sets and syncs the values of current time on the device with expecation

    ESP_LOGI(TAG, "Create DC motor 1");
    bdc_motor_config_t motor1_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A_1,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B_1,
    };
    bdc_motor_config_t motor2_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A_2,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B_2,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    // Instantiating the motor handles
    bdc_motor_handle_t motor_1 = NULL;
    bdc_motor_handle_t motor_2 = NULL;
    // creating new motor control pwm to drive the motors
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor1_config, &mcpwm_config, &motor_1));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor2_config, &mcpwm_config, &motor_2));
    // Enableing the motor 1
    ESP_LOGI(TAG, "Enable motor 1 and 2 object handles");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_1));
    ESP_ERROR_CHECK(bdc_motor_enable(motor_2));

    // INFINITE PROGRAM LOOP

    // starting home position of frame
    float current_rotation = 0.0;
    float current_tilt = 90.0;

    // variables for desired motor position
    float next_rotation;
    float next_tilt;

    while (1)
    {
        // ANGLE CALCULATIONS
        Get_current_date_time(); // get current time sets and syncs the values of current time on the device with expecation
        // printf("current date and time is = %s\n", Current_Date_Time);
        // printf("Now month is: %.2f\n", month);
        // printf("Now day is: %.2f\n", day);
        // printf("Now year is: %.2f\n", year);
        // printf("The day of the year is: %.2f\n", doy);
        // printf("Now the hour is: %.2f\n", hour);
        // printf("Now the minute is: %.2f\n", minute);
        // printf("Now the second is: %.2f\n", second);
        hour += (minute / 60.0);
        // printf("Hour appended with minute is now: %f\n", hour);
        float LSTM = -90.0;
        float B = (360.0 / 365.0) * (doy - 81.0);
        float EoT = 9.87 * sin(2.0 * B * M_PI / 180.0) - 7.53 * cos(B * M_PI / 180.0) - 1.5 * sin(B * M_PI / 180.0);
        float TC = 4.0 * (longitude - LSTM) + EoT;
        float LST = hour + (TC / 60.0);
        float HRA = 15.0 * (LST - 12.0);
        float dec_angle = 23.45 * sin(B * M_PI / 180.0);
        float elevation_angle = asin(sin(dec_angle * M_PI / 180.0) * sin(latitude * M_PI / 180.0) + cos(dec_angle * M_PI / 180.0) * cos(latitude * M_PI / 180.0) * cos(HRA * M_PI / 180.0));
        elevation_angle = elevation_angle * 180.0 / M_PI;
        float azimuth_angle = acos((sin(dec_angle * M_PI / 180.0) * cos(latitude * M_PI / 180.0) - cos(dec_angle * M_PI / 180.0) * sin(latitude * M_PI / 180.0) * cos(HRA * M_PI / 180.0)) / cos(elevation_angle * M_PI / 180.0)); // numerator of the azimuth angle calculation
        azimuth_angle = (azimuth_angle * 180.0 / M_PI);                                                                                                                                                                            // converts from radians to degrees                                                                                                                                                                             // storing elevation angle into the data struct
        if (LST > 12 || HRA > 0)
        {
            printf("Past solar noon \n");
            azimuth_angle = 360 - azimuth_angle; // equation for azimuth angle after the local solar noon
        }
        printf("The current elevation angle is: %.2f \n", elevation_angle);
        printf("The current azimuth angle is: %.2f\n", azimuth_angle);
        angd.computed_azimuth = azimuth_angle;     // storing azimuth angle into the data struct
        angd.computed_elevation = elevation_angle; // storing elevation angle into the data struct

        // code to move motors

        next_rotation = azimuth_angle - current_rotation;
        next_tilt = elevation_angle - current_tilt;

        // if statements to determine if motors need to be moved forward or backward
        if (next_rotation >= 0.0)
        {
            ESP_LOGI(TAG, "ROTATING FORWARD %.2f degrees", next_rotation);
            // pwm code to move rotation motor forward
            bdc_motor_forward(motor_1);
            bdc_motor_set_speed(motor_1, 125);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            bdc_motor_brake(motor_1);
        }
        else
        {
            next_rotation = fabs(next_rotation);
            ESP_LOGI(TAG, "ROTATING BACKWARD %.2f degrees", next_rotation);
            bdc_motor_reverse(motor_1);
            bdc_motor_set_speed(motor_1, 125);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            bdc_motor_brake(motor_1);
        }
        //     vTaskDelay(3000 / portTICK_PERIOD_MS); // 5 second delay after rotating to desired position and then proceeds to tilt to desired position
        //                                            // if (next_tilt >= 0.0)
        //                                            // {
        //                                            //     ESP_LOGI(TAG, "TILTING UP %.2f degrees", next_tilt);
        //                                            //     // pwm code to move tilt motor up
        //                                            //     bdc_motor_reverse(motor_2);
        //                                            //     // bdc_motor_set_speed(motor_2, 125);
        //                                            //     // vTaskDelay(1000 / portTICK_PERIOD_MS);
        //                                            //     // bdc_motor_brake(motor_2);
        //                                            // }
        //                                            // else
        //                                            // {
        //                                            //     next_tilt = fabs(next_tilt);
        //                                            //     ESP_LOGI(TAG, "TILTING DOWN %.2f degrees", next_tilt);
        //                                            //     // pwm code to move tilt motor down
        //                                            //     bdc_motor_forward(motor_2);
        //                                            //     bdc_motor_set_speed(motor_2, 125);
        //                                            //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //                                            //     bdc_motor_brake(motor_2);
        //                                            // }

        // reset current position
        current_rotation = azimuth_angle;
        // current_tilt = elevation_angle;
        vTaskDelay(300000 / portTICK_PERIOD_MS); // delay for 5 min on the loop *SET TO 10 MIN DELAY WHEN READY
    }

    // // SENSOR ROUTINE
    // // sensor_routine();                                                          // this function samples all data into corresponding global variables and structs for processing
    // // angd.actual_elevation = (M_PI / 2 - (atan(accd.z / accd.x)) * 180 / M_PI); // compute the actual elevation angle
    // // angd.actual_elevation =  90;
    // // angd.actual_azimuth = ((accd.x / accd.y) * 180 / M_PI);                    // compute the actual azimuth angle

    // // POST ROUTINE
    // // for (int i = 0; i < 2; i++)
    // // {
    // //     post_rest_function(i); // posting two indices corresponding to each of the sensors
    // // }
    // // // vTaskDelay(600000 / portTICK_PERIOD_MS); // delay for 10 seconds on the loop
    // // MOTOR CONTROL BLOCK
}

// ORIENTATION TESTING FOR ACCELEROMETER

// for (int i = 0; i < 3; i++)
// {
//     vTaskDelay(1000);
//     i2c_acc_sample();
//     switch (i)
//     {
//     case (0):
//         ESP_LOGI(TAG, "X - Orientation Test x: %f y: %f z: %f", accd.X * (9.80665 / 4096), accd.Y * (9.80665 / 4096), accd.Z * (9.80665 / 4096));
//         break;
//     case (1):
//         ESP_LOGI(TAG, "Y - Orientation Test y: %f y: %f z: %f", accd.X * (9.80665 / 4096), accd.Y * (9.80665 / 4096), accd.Z * (9.80665 / 4096));
//         break;
//     case (2):
//         ESP_LOGI(TAG, "Y - Orientation Test z: %f y: %f z: %f", accd.X * (9.80665 / 4096), accd.Y * (9.80665 / 4096), accd.Z * (9.80665 / 4096));
//         break;
//     default:
//         break;
//     }
// }

// while (1)
// {
//     rest_get_pwm();
//     ESP_LOGI(TAG, "PWM Value Updates: x: %d y: %d", pwm_x, pwm_y);
//     double dc_x = (((double)pwm_x) / 100) * 400; // convert duty cycle to percent of 400
//     double dc_y = (((double)pwm_y) / 100) * 400;

//     if (pwm_x < 0 && pwm_x >= -100)
//     { // backward operation
//         ESP_LOGI(TAG, "Backward x motor");
//         ESP_ERROR_CHECK(bdc_motor_reverse(motor_1));
//         ESP_ERROR_CHECK(bdc_motor_set_speed(motor_1, -1 * dc_x));
//     }
//     else if (pwm_x > 0 && pwm_x <= 100)
//     { // forward operation
//         ESP_LOGI(TAG, "Forward x motor");
//         ESP_ERROR_CHECK(bdc_motor_forward(motor_1));
//         ESP_ERROR_CHECK(bdc_motor_set_speed(motor_1, dc_x));
//     }
//     else if (pwm_x == 0)
//     {
//         ESP_ERROR_CHECK(bdc_motor_brake(motor_1));
//     }
//     if (pwm_y < 0 && pwm_y >= -100)
//     { // backward operation
//         ESP_LOGI(TAG, "Backward y motor");
//         ESP_ERROR_CHECK(bdc_motor_reverse(motor_2));
//         ESP_ERROR_CHECK(bdc_motor_set_speed(motor_2, -1 * dc_y));
//     }
//     else if (pwm_y > 0 && pwm_y <= 100)
//     { // forward operation
//         ESP_LOGI(TAG, "Forward y motor");
//         ESP_ERROR_CHECK(bdc_motor_forward(motor_2));
//         ESP_ERROR_CHECK(bdc_motor_set_speed(motor_2, dc_y));
//     }
//     else if (pwm_y == 0)
//     {
//         ESP_ERROR_CHECK(bdc_motor_brake(motor_2));
//     }
//     vTaskDelay(50 / portTICK_PERIOD_MS); // delay half a second !
// }

// ESP_LOGI(TAG, "Caught key time parametrics with following parameters: Day: %d Month: %d Year: %d Hour: %d Minute: %d Second: %d", day, month, year, hour, minute, second);

// I2C Serial Communication Commands Configuration
// i2c_master_init(); // initialize I2C serial commmunication with the esp32 and set internal pull-ups / SDA / SCL lines

// Sensor initialization and control register configurations specific to device
// MMA845_init(); // initialize the accelerometer by adjusting the control registers to desired polling / resolution settings
// MMC560_init(); // initalize the magnetometer by adjusting the control registers to desired polling / resolution settings
// MCP98_init();  // initalize the temperature sensor by adjusting the control registers to desired polling / resolution settings

// sensor_routine(); // routine to sample all the data points and post the subsequent readings in the database

// while (1)
// {
//     i2c_temp_sample();
//     vTaskDelay(10000 / portTICK_PERIOD_MS); // take measurement every 10 seconds
// }
// }

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// useful but unused code for now
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// // rest get function to perform https get request to get the local time
// static void rest_get_time()
// {
//     static const char *url_time = "https://capstone-database-c7175-default-rtdb.firebaseio.com/Flags/Angle/Time.json"; // api endpoint to get the time value
//     esp_http_client_config_t config_get = {
//         .url = url_time, // url time is the
//         .method = HTTP_METHOD_GET, // method being get request data being received from the firebase query endpoint
//         .cert_pem = (const char *)certificate_pem_start, // get the pem certificate file for communication over ssl
//         .event_handler = client_event_get_handler_time};
//     esp_http_client_handle_t client = esp_http_client_init(&config_get);
//     esp_http_client_perform(client);
//     esp_http_client_cleanup(client);
// }
// get request handler to obtain data from https request for time
// esp_err_t client_event_get_handler_time(esp_http_client_event_handle_t evt)
// {
//     switch (evt->event_id)
//     {
//     case HTTP_EVENT_ON_DATA:
//         printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data); // printing to serial monitor our queried data
//         uint8_t count = 0;
//         uint8_t length = evt->data_len; // length of the
//         char *data = (char *)evt->data;
//         while (*data)
//         { // While there are more characters to process...
//             char c = *data;
//             if (isdigit(c))
//             {
//                 // Found a number
//                 if (count == 0)
//                 {
//                     month = (int)strtol(data, &data, 10); // Getting the month
//                 }
//                 else if (count == 1)
//                 {
//                     day = (int)strtol(data, &data, 10); // Getitng the day
//                 }
//                 else if (count == 2)
//                 {
//                     year = (int)strtol(data, &data, 10); // Getting the year
//                 }
//                 else if (count == 3)
//                 {
//                     hour = (int)strtol(data, &data, 10); // Getting the hour
//                 }
//                 else if (count == 4)
//                 {
//                     minute = (int)strtol(data, &data, 10); // Getting the minute
//                 }
//                 else if (count == 5)
//                 {
//                     second = (int)strtol(data, &data, 10); // Getting the second
//                     data++;                                // increment the pointer to get to the PM / AM
//                     break;
//                 }
//                 count++; // incrementing the counter to organize the data
//             }
//             else
//             {
//                 // Otherwise, move on to the next character.
//                 data++; // incrementing the pointer
//             }
//         }
//         char tz[3] = {*(data), *(data + 1), '\0'};
//         printf(tz);
//         if (strcmp(tz, "PM") == 0)
//         {
//             // PM block
//             printf("PM\n");
//             printf("Night\n");
//             hour += 12; // to convert to military time for calculation
//         }
//         else if (strcmp(tz, "AM") == 0)
//         {
//             // AM block
//             printf("AM\n");
//             printf("Day\n");
//         }
//         break;
//     default:
//         break;
//     }
//     // Debug TAG to see parsed variables and ensure correct times...
//     // ESP_LOGI(TAG, "Parsed data into data variables Month: %d , Day: %d , Year: %d , Hour: %d , Minute %d, Second %d", month, day, year, minute, second);
//     return ESP_OK;
// }

// // Simple test for device id
// uint8_t val[2];
// rdMMA845x(WHO_AM_I_REG, &(val), 1); // simple test to verify the right device id of the accelerometer, MMA8451Q
// printf("%X\n", val[0]);
// rdMMC560x(DEVICE_ID, &(val), 1); // simple test to verify the right device id of the magnetometer, MMC5603NJ
// printf("%X\n", val[0]);

// // Testing for sensor data
// // Populating magnetometer, temperature, and accelerometer data into the corresponding structures and variables,
// // then consequently calling the post_rest_function with input parameter sensor

// i = 0 corresponding to posting magnetometer parametric data (0)
// i = 1 corresponding to posting accelerometer parametric data (1)
// i = 2 testing data with invalide sensor input parameter (default i not equal to 0 or 1)

// for (int i = 0; i < 3; i++)
// { // iterating across values i=0,1,2 for testing post
//     uint16_t d = i * 3000;
//     int16_t a = i * 3000;

//     accd.x = a; // x acceleration data
//     accd.y = a; // y acceleration data
//     accd.z = a; // z acceleration data

//     magd.x = d; // x acceleration data
//     magd.y = d; // y magnetometer data
//     magd.z = d; // z magnetometer data

//     post_rest_function(i);                 // calling the post method for each case of the loop
//     vTaskDelay(1000 / portTICK_PERIOD_MS); // add a delay of one second
// }

// 2-degrees of motor testing (bringing one up in duty cycle and the other one taking the negative duty cycle)
// Occurs for 3 iterations and steps from 25 to 75 duty cycle, motor receives voltage ideally in range of
// 12 V * 0.25 = 3 V to 12 V * 0.75 = 9 V
// ESP_LOGI(TAG, "Create DC motor 1");
// bdc_motor_config_t motor1_config = {
//     .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
//     .pwma_gpio_num = BDC_MCPWM_GPIO_A_1,
//     .pwmb_gpio_num = BDC_MCPWM_GPIO_B_1,
// };
// bdc_motor_config_t motor2_config = {
//     .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
//     .pwma_gpio_num = BDC_MCPWM_GPIO_A_2,
//     .pwmb_gpio_num = BDC_MCPWM_GPIO_B_2,
// };
// bdc_motor_mcpwm_config_t mcpwm_config = {
//     .group_id = 0,
//     .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
// };
// // Instantiating the motor handles
// bdc_motor_handle_t motor_1 = NULL;
// bdc_motor_handle_t motor_2 = NULL;
// // creating new motor control pwm to drive the motors
// ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor1_config, &mcpwm_config, &motor_1));
// ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor2_config, &mcpwm_config, &motor_2));
// // Enableing the motor 1
// ESP_LOGI(TAG, "Enable motor 1 and 2 object handles");
// ESP_ERROR_CHECK(bdc_motor_enable(motor_1));
// ESP_ERROR_CHECK(bdc_motor_enable(motor_2));
// for (int i = 0; i < 3; i++)
// {
//     for (int i = 100; i <= 300; i += 20)
//     {
//         ESP_ERROR_CHECK(bdc_motor_forward(motor_1));
//         ESP_ERROR_CHECK(bdc_motor_forward(motor_2));
//         ESP_ERROR_CHECK(bdc_motor_set_speed(motor_1, i));
//         ESP_ERROR_CHECK(bdc_motor_set_speed(motor_2, 400 - i));
//         vTaskDelay(5000 / portTICK_PERIOD_MS);
//         ESP_LOGI(TAG, "Duty cycle is approximately %f", (((double)i) / 400) * 100);
//         ESP_LOGI(TAG, "Negative duty cycle is approximately %f", (1 - (((double)i) / 400)) * 100);
//     }
//     bdc_motor_brake(motor_1); // brake motor 1
//     bdc_motor_brake(motor_2); // brake motor 2
// }
