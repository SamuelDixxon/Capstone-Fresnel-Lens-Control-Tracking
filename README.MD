# ECEN Capstone Fresnel Lens Control Tracking Team #66

This is the repository that contains all the work done on the capstone project for Team 66.
The world is rapidly transitioning its change from renewable to non-renewable energy sources.
The idea behind this project was to help our sponsor develop an IoT based solution to use motors
for a fresnel lens to track the sun. In the broader scope, this lens focussing element will be 
utilized in a larger thermo-chemical conntrol system where energy can be captured from the sun 
and stored for future use.

## File structure and description

```
## Help

Any advise for common problems or issues.

command to run if program contains helper info
```

## Authors

Contributors names and contact info  
[@ Jordan George](jgeorge5@tamu.edu)
[@ Samuel Dixon](samueldixon@tamu.edu)

## Version History

* 1.0 (4/26/2023)
    * Project major revision 1 with two programs for controlling the motors found here
    *    [Manual Control]()
    *    [Automatic Control]()


## Resources utilized

Design Related Documents
* Magnetometer datasheet and resources
   * [Datasheet](https://www.espruino.com/files/MMC5603NJ.pdf)
* Accelerometer datahseet and resources
   * [Datasheet](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf

Code
* ESP-IDF HTTP Requests (Firebase)
*  [GET](https://www.youtube.com/watch?v=8uBIBfUBCt8)
*  * [POST](https://www.youtube.com/watch?v=99McYBmKcTw)
     * Note these are useful examples as you have to generate SSL pem certificate files for HTTPS requests to nodes in the NoSQL datbase in the firebase              software suite
* [ESP-IDF I2C Configuration]()
* [ESP-IDF Connecting to Wifi](https://www.youtube.com/@simsiotdevices6219)
* [ESP-IDF PWM and Motor Control](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html)
* [Joystick mmodule in java script](https://github.com/bobboteck/JoyStick)
* [System Network Time Protocol (STNP) Time Synchronization](https://techtutorialsx.com/2021/09/01/esp32-system-time-and-sntp/)
* [Configuring a lambda script in AWS and using API Gateway trigger](https://docs.aws.amazon.com/lambda/latest/dg/services-apigateway.html)
* [awesome-readme](https://github.com/matiassingers/awesome-readme)

Algorithm theory
* [Azimuth Angle Computation with system parametric data]
* [Elevation Angle Computation](https://www.cuemath.com/trigonometry/angle-of-elevation/)


## Acknowledgments
We would like to extend a warm thanks to our sponsor, Dr. Madsen, who works in the Electrical Engineering 
department as a professor at Texas A&M. Additionally, we would like to thank the supporting staff in the
electrical engineering department who coordinate the capstone course - specifically Dr. Lusher, Dr. Stavros,
Dr. Jang, and Dr. Nowka. Lastly, we would like to thank our TA, Dalton Cyr, and our peers who supported us on 
this journey. 

Additionally I would like to thank the creator behind the IoT Sims Devices youtube channel for providing extensive examples for various peripherals in the ESP32.

