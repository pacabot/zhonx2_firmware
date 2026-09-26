#ifndef TEST_STM32_H
#define TEST_STM32_H
#include <stdint.h>
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
#define TIM5 ((void *)5)
#define ENABLE 1
#define DISABLE 0
typedef struct { uint32_t IDR, ODR; } GPIO_TypeDef;
typedef struct { uint32_t CR1; } USART_TypeDef;
typedef struct { uint32_t APB1ENR; } RCC_TypeDef;
typedef struct { uint32_t CR; } PWR_TypeDef;
typedef struct { uint32_t BKP0R; } RTC_TypeDef;
extern GPIO_TypeDef test_gpioc;
extern USART_TypeDef test_usarts[6];
extern RCC_TypeDef test_rcc;
extern PWR_TypeDef test_pwr;
extern RTC_TypeDef test_rtc;
#define GPIOC (&test_gpioc)
#define USART1 (&test_usarts[0])
#define USART2 (&test_usarts[1])
#define USART3 (&test_usarts[2])
#define UART4 (&test_usarts[3])
#define UART5 (&test_usarts[4])
#define USART6 (&test_usarts[5])
#define RCC (&test_rcc)
#define PWR (&test_pwr)
#define RTC (&test_rtc)
#define USART_CR1_UE 0x2000
#define RCC_APB1ENR_PWREN 0x10000000
#define PWR_CR_DBP 0x100
void fw_test_idle(void);
void NVIC_SystemReset(void);
static inline void __WFI(void) { fw_test_idle(); }
static inline void __DSB(void) {}
#endif
