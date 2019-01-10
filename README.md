# imu381-spidriver-c
A C-code based driver for IMU381 SPI Interface, tested on Raspberry Pi3. Using wiringPi library to implement the Pi3 SPI host Features: 
1. IMU381 module SPI read/write and data parse actions, user could fetch the calibrated IMU381 module data directly from register through SPI interface, and change the SPI ODR with the register output rate spi write action.
2. Coordinate remapping.
3. Typical packages parse, including standard package,S0,S1,A1,A2,A3. 

The feature 1 work normally with IMU381 firmware version:19.1.3 currently. Other features need to be perfected.
Before test the code on Pi3 board, user should connect pi3-b board with IMU381 module several wires: SPI(MOSI,MISO,CLK,NSS) lines, data ready line, 1PPS line, VCC,GND. Compile the code with command: gcc -o imu381_demo imu381_demo.c -lwiringPi, and sudo the executable file ./imu381_demo
