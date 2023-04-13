/**************************************************************************/
/*!

    @file     Motor-Controls.imo
    @author   Samuel Dixon
    
    This code utilizies the adafruit libraries for 2 sensors, an accelerometer
    and a magnetometer to compute the azimuth angle and elevation angle at a 
    particular orientation. 

    The two angle measurements are then compared to expected value computed
    based on a current time and a corresponding geopgraphical latitude and
    longitude through a percent difference. This parameter is a percentage 
    we use to excite motors controllig the tilt and rotation of the motors
    for a short time instance to make an adjustment. This process is repeated
    until the percent difference is within an acceptable tolerance. 

    Once alligned, an internal timer is set before reallignment must be made.
    Currently, this internal timer will have a programmable value for how 
    frequently the  motors should be adjusted.

    v1.0  - First release
*/
/**************************************************************************/

#include <Wire.h> // I2C Serial Communications Library
#include <Adafruit_MMC56x3.h> // magnetometer library
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_MMC5603 mmc = Adafruit_MMC5603();

void setup(void) {
  Serial.begin(9600);
  
  Serial.println("Adafruit MMA8451 Accelerometer test!");
  
  if (! mma.begin()) { // beginning the device
  /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Couldnt start");
    while (1) delay(10);
  } else {
    Serial.println("Accelerometer Serial Communicaiton Initiated");
  }

  Serial.println("Adafruit_MMC5603 Magnetometer Test");

  if (!mmc.begin()) {  // beginning the device
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
    while (1) delay(10);
  } else {
    Serial.println("Magnetonemter Serial Communicaiton Initiated");
  }

  mma.setRange(MMA8451_RANGE_2_G);
  
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
  
}

void loop() {
  /* Get a new sensor event */ 
  sensors_event_t event_accel; 
  mma.getEvent(&event_accel);
  sensors_event_t event_mag; 
  mmc.getEvent(&event_mag);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: \t"); Serial.print(event_accel.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event_accel.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event_accel.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");

  Serial.print("X: \t"); Serial.print(event_mag.magnetic.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event_mag.magnetic.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event_mag.magnetic.z); Serial.print("\t");
  Serial.println("uT ");

    // Read and display temperature
  float temp_c = mmc.readTemperature();
  Serial.print("Temp: "); Serial.print(temp_c); Serial.println(" *C");
  // Delay before the next sample
  
  delay(500);
}