#ifndef TEST_OLED_PERIPHERALS_H
#define TEST_OLED_PERIPHERALS_H
#define CS_GPIO_PORT ((void *)0)
#define DC_GPIO_PORT ((void *)0)
#define RST_GPIO_PORT ((void *)0)
#define SPIx_SCK_GPIO_PORT ((void *)0)
#define SPIx_MOSI_GPIO_PORT ((void *)0)
#define CS_PIN 0
#define DC_PIN 0
#define RST_PIN 0
#define SPIx_SCK_PIN 0
#define SPIx_MOSI_PIN 0
static inline void GPIO_SetBits(void *p,unsigned pin) { (void)p; (void)pin; }
static inline void GPIO_ResetBits(void *p,unsigned pin) { (void)p; (void)pin; }
static inline void GPIO_WriteBit(void *p,unsigned pin,unsigned value) { (void)p; (void)pin; (void)value; }
static inline void ssd1306_GPIO_Config(void) {}
static inline void ssd1306_SPI_Config(void) {}
#endif
