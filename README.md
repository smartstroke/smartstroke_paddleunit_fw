# Smart Stroke Paddling Unit Firmware
 
 Firmware for Paaddling Unit or Unit A. This firmware should collect data from the IMU and I2C/ADC (FSR) header and send it over ESP-NOW to the base station. 
 
## Componets

- ESP32-S3-WROOM-1-N4R2CT-ND MCU
- ICM-20602 IMU
- 124018612112A USBC

## Pinout

Leds
- Blue LED: IO0
- Green LED: NO GREEN LED

I2C/ADC
- I2C SCL: NOT AVAILABLE
- I2C SCA: NOT AVAILABLE

IMU/SPI
- IMU CS:: IO10
- IMU SDI: IO7
- IMU SCK: IO2
- IMU SDO: IO6
- IMU INT: NOT AVAILABLE

FSR
- FSR1: IO5
- FSR2: IO4
- FSR3: IO3
- FSR4: IO1
- FSR5: IO0 (SHARED WITH LED)

## Boot Mode

To get into DFU mode, depress BOOT button. Then reset board. The board should be now able to be programmed via USB or UART0. Board should now function. This board should automatically be able to be programmed upon pluggin in USB. Errors include: Wrong boot mode detected (0x8)!

![board](https://i.imgur.com/qkQjWE7.png)

## USB Auto Reset

If the board will not auto reset from USB when programming then you might need to reprogram the bootloader.
To do so from the Ardunio IDE use the "burn bootloader" option as seen below. Errors include: Connecting......................................, problem with the hardware...., 

![bootloader](https://i.imgur.com/1UKCGS1.png)

