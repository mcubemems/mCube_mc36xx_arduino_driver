Arduino Accelerometer sensor driver for mCube Product MC3600 series.
==============================================================

The MC3600 series are ultra-low power, low noise, integrated digital output 3-axis accelerometer family with a feature set optimized for wearables and consumer product motion sensing. Applications include wearable consumer products, IoT devices, user interface control, gaming motion input, electronic compass tilt compensation for cell phones, game controllers, remote controls and portable media products. Low noise and low power are inherent in the monolithic fabrication approach, where the MEMS accelerometer is integrated in a single-chip with the electronics integrated circuit.

This demo has been verified on Arduino MO Pro board with mCube's MC36XX evaluation board. If you want to connect a MC36XX chip to your own Auduino board, please pay attention to the following macros and const definition in file MC36XX.h:
1. MC36XX_CFG_BUS_I2C
   If you use I2C interface, please enable this macro, and disable MC36XX_CFG_BUS_SPI
2. MC36XX_CFG_BUS_SPI
   If you use SPI interface, please enable this macro, and disable MC36XX_CFG_BUS_I2C
3. chipSelectPin
   If you use SPI interface, please set the real number you're using to connect MC36XX's pin10.

MC3630
https://mcubemems.com/product/mc3630-3-axis-accelerometer/

MC3635
https://mcubemems.com/product/mc3635-3-axis-accelerometer/

MC3672
https://mcubemems.com/product/mc3672-3-axis-accelerometer/
