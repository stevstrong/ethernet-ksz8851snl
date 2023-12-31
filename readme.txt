This is a project based on Medion MD86562 to replace the main coontroller.
The STM32F407V is the main MCU (DevEBox), the ESP8266 provides Wifi and send the data to main MCU.

The MCU handles:
- ESP8266 Wifi (Serial?) - 2 pins: Rx, Tx
- KSZ8851SNL Ethernet (SPI1)
- VS1053 audio decoder (SPI2)
- CSC2314F audio switch (I2C1)
- ST7565 128*64 dots lyquid LCD (parallel interface Px+Py) - 12 pins
- key matrix (Pz) - 9 pins
- some LEDs (Pw) - 2 pins
- IR receiver


Pin mapping:
==============================
PA0
PA1
PA2
PA3  - Etheret IRQ
PA4  - SPI1 CS (Ethernet)
PA5  - SPI1_SCK
PA6  - SPI1_MISO
PA7  - SPI1_MOSI
PA8
PA9  - TX1
PA10 - Rx1
PA11
PA12
PA13
PA14
PA15

PB0
PB1
PB2
PB3
PB4
PB5
PB6
PB7
PB8
PB9
PB10 - I2C2_SCL
PB11 - I2C2_SDA
PB12 - SPI2_CS
PB13 - SPI2_SCK
PB14 - SPI2_MISO
PB15 - SPI2_MOSI