# Smart Stroke Paddling Unit Firmware
 
 Firmware for Paaddling Unit or Unit A. This firmware should collect data from the IMU and I2C/ADC (FSR) header and send it over ESP-NOW to the base station. 
 
## Componets

- ESP32-S3-WROOM-1-N4R2CT-ND MCU
- ICM-20602 IMU
- 124018612112A USBC

## Pinout

Leds
- Blue LED: IO16
- Green LED: IO9

I2C/ADC
- I2C SCL: IO4
- I2C SCA: IO5

IMU/SPI
- IMU CS:: IO18
- IMU SDI: IO19
- IMU SCK: IO20
- IMU SDO: IO21
- IMU INT: IO22

DIP (ON: Short to ground OFF: Disconnected) (Use internal pullup? I screwed up and made external pull downs not pull ups. Do not use R14/R15)
- BOOT: IO0
- JTAG: IO3
- IO15: IO15

## Boot Mode

To get into DFU mode, set dip switch 1 (BOOT) to ON. Then reset board. The board should be now able to be programmed via USB or UART0. After programming move dip switch 1 (BOOT) back to OFF and reset board. Board should now function.

![bootfipswitch](https://i.imgur.com/mhKzBrW.png)

## USB Auto Reset

If the board will not auto reset from USB then you might need to reprogram the bootloader.
To do so from the Ardunio IDE use the "burn bootloader" option as seen below. Errors include: Connnecting......, problem with the hardware...., 

![bootloader](https://i.imgur.com/1UKCGS1.png)

