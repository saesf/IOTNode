# Description
 IOT Sensor Node using stm's [B-L475E-IOT01A](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) (STM32L4 Discovery kit IoT node) and [x-nucleo-iks01a2](https://www.st.com/en/ecosystems/x-nucleo-iks01a2.html) (Motion MEMS and environmental sensor expansion board).
This code uses DMA for data transfer between microcontroller and pripherals to increase sampling rate of sensors and lower cpu overhead. DMA channel for I2C2 was already taken by DFSDM for microphones and becasue of that i couldn't use iota01's motions sensors. to solve this issue i used iks extension pack and connected it to I2C1 which had free DMA channel.

 This program read data from 2 microphones at 16 KHz with 24 bit resultion and extract accel/gyro data from LSM6DSL at 1.666 khz samlping rate (per axis, per sensor) and send them to a python program using wifi module and UDP protocol.
currently there is some problem with the wifi module on my board and it has some packet loss (around 4%) and sometimes it stops sending to computer. I raised an issue for this problem in stm community ([link](https://community.st.com/s/question/0D53W000022cREaSAM/bl475eiot01as-wifi-module-does-not-send-packet-after-a-while)).
## Sensors
 Currently i receive data from these sensors:
* 6-axis inertial measurement unit (LSM6DSL)
* 3-axis magnetometer (LIS3MDL)
* Microphone / audio sensor (MP34DT01)

# Development Tools
 Source code can be built and flashed using the official IDE of St-microelectronics:

* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) available for Linux, Mac and Windows

