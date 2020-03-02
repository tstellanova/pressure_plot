## pressure_plot

A bare metal embedded rust application that reads an ambient air pressure sensor (the BMP280)
and plots the result to a small OLED display (the SSD1306).

It uses an i2c connection to both the sensor and the OLED display, with a common +3VDC
power supply for both.  

For an initial pass I used the 
[RobotyDyn STM32F303 mini](https://robotdyn.com/stm32f303cct6-256-kb-flash-stm32-arm-cortexr-m4-mini-system-dev-board-3326a9dd-3c19-11e9-910a-901b0ebb3621.html)
development board. 

Note that the RobotDyn board does not include pull-up resistors on the i2c SCL and SDA lines, 
however many BMP280 breakout boards (such as the "GY-BMP280") do include these. 
If your hardware does not include pull-up resistors, you may need to enable these in software. 

### Support for STM32F4 and STM32H7

There is some initial support for other STM32 families in this application.  
I tested briefly with:
- STM32F401CxBx dev board
- STM32H7 Nucleo (NUCLEO-H743ZI2)