/**
 * @file board_pins.h
 * @brief Makerfabs ESP32 UWB Pro with Display pin map.
 */
#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

#define PIN_SPI_SCK     18
#define PIN_SPI_MISO    19
#define PIN_SPI_MOSI    23
#define PIN_UWB_SS      21
#define PIN_UWB_RST     27
#define PIN_UWB_IRQ     34

#define PIN_OLED_SDA    4
#define PIN_OLED_SCL    5
#define OLED_I2C_ADDR   0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT     64

#endif /* BOARD_PINS_H_ */
