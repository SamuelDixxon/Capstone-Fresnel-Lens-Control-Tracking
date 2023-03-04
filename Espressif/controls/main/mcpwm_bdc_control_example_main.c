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
#define MCP9808_REG_RESOLUTION 0x08   ///< resolution

// L2987n driver defines
#define SERIAL_STUDIO_DEBUG CONFIG_SERIAL_STUDIO_DEBUG
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                         // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                                                        // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAdata (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // madataimum value we can set for the duty cycle, in ticks

// Motor one
#define BDC_MCPWM_GPIO_A_1 16
#define BDC_MCPWM_GPIO_B_1 15

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
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_contedatat_t;

// Structure to hold accelerometer data
typedef struct ACCEL_DATA
{
    int16_t X; // X stores the data for the raw X coordinate accelerometer data
    int16_t Y; // Y stores the data for the raw Y coordinate accelerometer data
    int16_t Z; // Z stores the data for the raw Z coordinate accelerometer data
} stACCEL_DATA_t;

// Structure to hold magnetometer data
typedef struct MAG_DATA
{
    uint16_t X; // X stores the data for the raw X coordinate magnetometer data
    uint16_t Y; // Y stores the data for the raw Y coordinate mangetometer data
    uint16_t Z; // Z stores the data for the raw Z coordinate magnetometer data
} stMAG_DATA_t;

typedef struct ANGLE_DATA
{
    float acutal_azimuth;     // actual azimuth stores the azimuth angle as computed from magnetometer and accelerometer data ( current value )
    float computed_azimuth;   // computed azimuth is what the azimuth is based on the time and geographical latitude/longitude ( desired value )
    float actual_elevation;   // actual eleevation stores the elevation angle as computed from accelerometer data ( current value )
    float computed_elevation; // computed elevation stores the elevation angle as computed from the accelerometer data( desired value )
} stANGLE_t;

// global variables for storing the time from NTP servers (network time protocol)
double day;    // variable to get the day
double month;  // variable to get the month
double year;   // variable to get the year
double doy;    // variable to get day of the year
double hour;   // variable to get the hour
double minute; // variable to get the minute
double second; // variable to get the second
uint8_t temp;  // variable to get the temperature

stMAG_DATA_t magd;   // structure for storing magnetometer data
stACCEL_DATA_t accd; // structure for storing accelerometer data
stANGLE_t angd;      // structure for storing angle data
float tempd;         // variable for storing ambient temperature

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
    //     id : (Sensor ID String),
    //     x  : (Reading from accelerometer / magnetometer units: m/s^2 or uT),
    //     y  : (Reading from accelerometer / magnetometer units: m/s^2 or uT),
    //     z  : (Reading from accelerometer / magnetometer units: m/s^2 or uT),
    //     actual_azimuth_angle : (calculated azimuth angle of frame from sensors),
    //     computed_azimuth_angle : (computed azimuth angle based on time of day and time),
    //     actual_elevation_angle : (calculated elevation angle of frame from sensors),
    //     computed_elevation_angle : (computed elevation angle based on time of day and time),
    //     ambient_temperature : (temperature reading taken from on-chip magnetometer),
    //     target_temperature : (temperature reading taken near target for reference)
    // }

    // note time will be processed in the lambda script.

    double f_a = (0.784 * temp_reading) - 75; // get the translated ambient temperature

    char buffer[200]; // create space for the data to be formatted into a buffer string

    char *post_data = &buffer;

    // sensor basically switches which data is being posted, in continuous sampling mode two calls are made to this function
    // one of the requests posts the acceleration data (input variable sensor=0) while the other posts the magnetometer data (input variable sensor=1)
    switch (sensor)
    {
    case (0):                                   // for case 0 we get the acceleration data and write the struct data
        double x = (accd.X * (9.80665 / 4096)); // converting acc.X reading to m/s^2
        double y = (accd.Y * (9.80665 / 4096)); // converting acc.Y reading to m/s^2
        double z = (accd.Z * (9.80665 / 4096)); // converting acc.Z reading to m/s^2
        sprintf(buffer, "{\"sensor\":\"Accelerometer1\",\"x\":\"%.1f\",\"y\":\"%.1f\",\"z\":\"%.1f\",\"e_a\":\"%.1f\",\"e_c\":\"%.1f\",\"t_a\":\"%.1f\"}", x, y, z, angd->computed_elevation, angd->actual_elevation, f_a); // formatting the data obtained from the accelerometer struct
        break;
    case (1):
        double x = (magd.X * (9.80665 / 4096)); // converting acc.X reading to uTesla
        double y = (magd.Y * (9.80665 / 4096)); // converting acc.Y reading to uTesla
        double z = (magd.Z * (9.80665 / 4096)); // converting acc.Z reading to uTesla
        sprintf(buffer, "{\"sensor\":\"Magnetometer1\",\"x\":\"%.1f\",\"y\":\"%.1f\",\"z\":\"%.1f\",\"a_a\":\"%.1f\",\"c_a\":\"%.1f\",\"t_a\":\"%.1f\"}", x, y, z, angd->computed_azimuth, angd->actual_azimuth, f_a); // formatting the data obtained from the magnetometer struct
        break;
    case (3):
        // Testing with arbitrary data to confirm functionality prior to sending real data with I2C int sensor = 3 as input parameter for this mode
        double arb = 1.1; // arbitrary data point
        sprintf(buffer, "{\"sensor\":\"Magnetometer1\",\"x\":\"%.1f\",\"y\":\"%.1f\",\"z\":\"%.1f\",\"a_a\":\"%.1f\",\"c_a\":\"%.1f\",\"e_a\":\"%.1f\",\"e_c\":\"%.1f\",\"t_a\":\"%.1f\"}", arb, arb, arb, arb, arb, arb);
        break;

    default:
        ESP_LOGE(TAG, "Error in the input parameter, input to post function should be integer, specifically: \n0 : Post Accelerometer data\n1: Post Magnetometer data \n2: Post mock data generated from random function");
    }

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

    // float x = (accd.X * (9.80665 / 4096)); // converting acc.X reading to m/s^2
    // float y = (accd.Y * (9.80665 / 4096)); // converting acc.Y reading to m/s^2
    // float z = (accd.Z * (9.80665 / 4096)); // converting acc.Z reading to m/s^2

    // ESP_LOGI(TAG, "Accelerometer err:%d  x:%5f  y:%5f  z:%5f", err, x, y, z);
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
    nvs_flash_init();  // flash initialization
    wifi_connection(); // run routines to connect to the wifi
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // i2c_master_init(); // initialize I2C serial commmunication with the esp32 and set internal pull-ups / SDA / SCL lines
    //  MMA845_init();     // initialize the accelerometer by adjusting the control registers to desired polling / resolution settings

    // Set_SystemTime_SNTP();
    float longitude = -96.326;
    float latitude = 30.3; // change to actual latitude only used 30.3 for sun position calculator website

    post_rest_function();

    // while (1)
    // {
    //     Get_current_date_time(Current_Date_Time);
    //     printf("current date and time is = %s\n", Current_Date_Time);
    //     printf("Now month is: %.2f\n", month);
    //     printf("Now day is: %.2f\n", day);
    //     printf("Now year is: %.2f\n", year);
    //     printf("The day of the year is: %.2f\n", doy);
    //     printf("Now the hour is: %.2f\n", hour);
    //     printf("Now the minute is: %.2f\n", minute);
    //     printf("Now the second is: %.2f\n", second);

    //     ANGLE CALCULATIONS

    //     hour += (minute / 60.0);
    //     //printf("Hour appended with minute is now: %f\n", hour);
    //     float LSTM = -90.0;
    //     float B = (360.0 / 365.0) * (doy - 81.0);
    //     float EoT = 9.87 * sin(2.0 * B * M_PI / 180.0) - 7.53 * cos(B * M_PI / 180.0) - 1.5 * sin(B * M_PI / 180.0);
    //     float TC = 4.0 * (longitude - LSTM) + EoT;
    //     float LST = hour + (TC / 60.0);
    //     float HRA = 15.0 * (LST - 12.0);
    //     float dec_angle = 23.45 * sin(B * M_PI / 180.0);
    //     float elevation_angle = asin(sin(dec_angle * M_PI / 180.0) * sin(latitude * M_PI / 180.0) + cos(dec_angle * M_PI / 180.0) * cos(latitude * M_PI / 180.0) * cos(HRA * M_PI / 180.0));
    //     elevation_angle = elevation_angle * 180.0 / M_PI;
    //     float azimuth_angle = acos((sin(dec_angle * M_PI / 180.0) * cos(latitude * M_PI / 180.0) - cos(dec_angle * M_PI / 180.0) * sin(latitude * M_PI / 180.0) * cos(HRA * M_PI / 180.0)) / cos(elevation_angle * M_PI / 180.0)); // numerator of the azimuth angle calculation
    //     azimuth_angle = (azimuth_angle * 180.0 / M_PI);
    //     angd->computed_azimuth = azimuth_angle;
    //     angd->computed_elevation = elevation_angle;                                                                                                                                                                        // converts from radians to degrees
    //
    //     if (LST > 12 || HRA > 0)
    //     {
    //         printf("Past solar noon \n");
    //         azimuth_angle = 360 - azimuth_angle; // equation for azimuth angle after the local solar noon
    //     }
    //     printf("The current elevation angle is: %.2f \n", elevation_angle);
    //     printf("The current azimuth angle is: %.2f\n", azimuth_angle);

    //     vTaskDelay(600000 / portTICK_PERIOD_MS);
    // }

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

    static motor_control_contedatat_t motor_ctrl_ctdata_1 = {
        .pcnt_encoder = NULL,
    };

    static motor_control_contedatat_t motor_ctrl_ctdata_2 = {
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config_1 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A_1,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B_1,
    };

    bdc_motor_mcpwm_config_t mcpwm_config_1 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config_2 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A_2,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B_2,
    };

    bdc_motor_mcpwm_config_t mcpwm_config_2 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor_1 = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_1, &mcpwm_config_1, &motor_1));
    motor_ctrl_ctdata_1.motor = motor_1;

    bdc_motor_handle_t motor_2 = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_2, &mcpwm_config_2, &motor_2));
    motor_ctrl_ctdata_2.motor = motor_2;

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_1));
    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_2));

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        rest_get_pwm();
        ESP_LOGI(TAG, "PWM Value Updates: x: %d y: %d", pwm_x, pwm_y);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        if (pwm_x < 0 && pwm_x >= -100)
        { // backward operation
            ESP_LOGI(TAG, "Stopping x motor");
            ESP_LOGI(TAG, "Backward x motor");
            ESP_ERROR_CHECK(bdc_motor_reverse(motor_1));
            ESP_ERROR_CHECK(bdc_motor_set_speed(motor_1, BDC_MCPWM_DUTY_TICK_MAdata * (double)(pwm_x / -100)));
        }
        else if (pwm_x > 0 && pwm_x <= 100)
        { // forward operation
            ESP_LOGI(TAG, "Stopping x motor");
            ESP_LOGI(TAG, "Forward x motor");
            ESP_ERROR_CHECK(bdc_motor_forward(motor_1));
            ESP_ERROR_CHECK(bdc_motor_set_speed(motor_1, BDC_MCPWM_DUTY_TICK_MAdata * (double)(pwm_x / 100)));
        }

        // if (pwm_y < 0 && pwm_y >= -100)
        // { // backward operation
        //     ESP_LOGI(TAG, "Stopping y motor");
        //     ESP_LOGI(TAG, "Backward y motor");
        //     ESP_ERROR_CHECK(bdc_motor_reverse(motor_2));
        //     ESP_ERROR_CHECK(bdc_motor_set_speed(motor_2, BDC_MCPWM_DUTY_TICK_MAdata * (pwm_y / -100)));
        // }
        // else if (pwm_y > 0 && pwm_y <= 100)
        // { // forward operation
        //     ESP_LOGI(TAG, "Stopping y motor");
        //     ESP_LOGI(TAG, "Forward y motor");
        //     ESP_ERROR_CHECK(bdc_motor_forward(motor_2));
        //     ESP_ERROR_CHECK(bdc_motor_set_speed(motor_2, BDC_MCPWM_DUTY_TICK_MAdata * (pwm_y / 100)));
        // }
    }

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
}

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
