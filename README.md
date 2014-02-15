RasPiTracker
============

This is the SW for my raspberry pi model a based high-altitude balloon tracker.

It operates the following sensors on i2c:

- ublox i2c gps
- bmp085 barometric sensor
- adxl345 accelerometer
- hmc5883 magnetometer

On the SPI:
- MCP 3002 A/D converter to measure battery voltage

On the pwm output:
- two DS18B20 one wire thermometers

On the UART:
- one NTX2B with 300 bd RTTY

On the PWM output:
- one NTX2B for DominoEX16 (driver adapted from https://github.com/m1ari/Sandals/blob/master/tests/dom16.c)

The picam is used to collect hi resolution images on the sd card as well as video snippets of 2 min length. Low resolution images are transmitted via ssdv. ssdv functions are from https://github.com/fsphil/ssdv

The i2c SW driver comes from here: http://www.byvac.com//downloads/Pi/bcm2835_i2cbb.zip and is needed since the pi hardware does not support clock stretching which is required for the gps chip






