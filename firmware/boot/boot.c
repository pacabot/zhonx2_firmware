#include "stm32f4xx.h"
#include "fw_layout.h"
#include "fw_protocol.h"
/* SPI3 slave, mode 0, half duplex on MISO (PB4). DMA1 channel 0.
 * PA13/PA14 remain SWD. No UART emulation and no radio work in the application.
 * The bridge must release DATA after each 1056-byte request and wait for READY.
 * NSS is high during processing, DATA low=BUSY, pulled high=READY.
 * NSS setup >=100 us, inter-transaction idle >=1 ms, DATA turnaround >=100 us.
 * No clocks during BUSY. See docs/FIRMWARE.md for the bridge contract. */
static fw_packet_t packet;
static fw_reply_t reply;
static fw_update_t update;
static int selected(void) { return !(GPIOA->IDR & (1u<<15)); }
static int elapsed(uint32_t start, uint32_t ms)
{ return (uint32_t)(DWT->CYCCNT-start) >= (SystemCoreClock/1000u)*ms; }
static void data_mode(unsigned mode)
{ GPIOB->MODER=(GPIOB->MODER & ~(3u<<8)) | mode<<8; }
static void dma_stop(void)
{
 DMA1_Stream0->CR=0; DMA1_Stream5->CR=0;
 while ((DMA1_Stream0->CR | DMA1_Stream5->CR) & DMA_SxCR_EN) {}
 DMA1->LIFCR=0x3d; DMA1->HIFCR=0xf40;
 SPI3->CR1=0; SPI3->CR2=0;
 RCC->APB1RSTR |= RCC_APB1RSTR_SPI3RST;
 RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI3RST;
}
static void arm(int transmit)
{
 dma_stop();
 DMA_Stream_TypeDef *s=transmit ? DMA1_Stream5 : DMA1_Stream0;
 s->PAR=(uint32_t)&SPI3->DR;
 s->M0AR=(uint32_t)(transmit ? (void *)&reply : (void *)&packet);
 s->NDTR=transmit ? sizeof reply : sizeof packet;
 s->FCR=0;
 s->CR=DMA_SxCR_MINC | DMA_SxCR_PL_1 | (transmit ? DMA_SxCR_DIR_0 : 0);
 SPI3->CR1=SPI_CR1_BIDIMODE | (transmit ? SPI_CR1_BIDIOE : 0);
 SPI3->CR2=transmit ? SPI_CR2_TXDMAEN : SPI_CR2_RXDMAEN;
 s->CR |= DMA_SxCR_EN;
 SPI3->CR1 |= SPI_CR1_SPE;
 data_mode(transmit ? 0 : 2); /* READY is always GPIO input until response CS falls. */
}
/* CS rising edge is authoritative (STM32F405 slave BSY erratum). */
static int transfer(int transmit)
{
 DMA_Stream_TypeDef *s=transmit ? DMA1_Stream5 : DMA1_Stream0;
 uint32_t start=DWT->CYCCNT;
 while (!selected()) if (elapsed(start,5000)) return -1;
 if (transmit) data_mode(2); /* Bridge guarantees >=100 us before first clock. */
 start=DWT->CYCCNT;
 while (selected()) if (elapsed(start,1000)) return -1;
 if (!transmit) { GPIOB->BSRR=1u<<(4+16); data_mode(1); }
 /* Allow the final peripheral-to-memory DMA beat to settle after NSS rises. */
 start=DWT->CYCCNT;
 while (s->NDTR && !elapsed(start,1)) {}
 __DSB();
 return s->NDTR==0 && !(SPI3->SR & (SPI_SR_OVR|SPI_SR_MODF)) ? 0 : -1;
}
static void service_init(void)
{
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
 RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
 __DSB();
 /* Deassert motor enable, assert sleep throughout loader operation. */
 GPIOA->BSRR=(1u<<8) | (1u<<(2+16)) | (1u<<(3+16));
 GPIOA->MODER=(GPIOA->MODER & ~((3u<<16)|(3u<<4)|(3u<<6))) | (1u<<16)|(1u<<4)|(1u<<6);
 GPIOA->AFR[1]=(GPIOA->AFR[1] & ~(15u<<28)) | 6u<<28;
 GPIOA->MODER=(GPIOA->MODER & ~(3u<<30)) | 2u<<30;
 GPIOA->PUPDR=(GPIOA->PUPDR & ~(3u<<30)) | 1u<<30;
 GPIOB->AFR[0]=(GPIOB->AFR[0] & ~((15u<<12)|(15u<<16))) | 6u<<12 | 6u<<16;
 GPIOB->MODER=(GPIOB->MODER & ~(3u<<6)) | 2u<<6;
 GPIOB->OSPEEDR |= 3u<<6 | 3u<<8;
 GPIOB->OTYPER &= ~(1u<<4);
 GPIOB->PUPDR=(GPIOB->PUPDR & ~(3u<<8)) | 1u<<8;
}
__attribute__((naked,noreturn)) static void enter_application(uint32_t sp __attribute__((unused)), uint32_t pc __attribute__((unused)))
{
 __asm volatile("msr msp, r0\n movs r2, #0\n msr control, r2\n isb\n cpsie i\n bx r1");
}
static void jump(void)
{
 __disable_irq();
 dma_stop();
 RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
 GPIOA->MODER=(GPIOA->MODER & ~(3u<<30));
 GPIOB->MODER &= ~((3u<<6)|(3u<<8)); /* All bridge pins high impedance. */
 SysTick->CTRL=0; SysTick->VAL=0;
 for (unsigned i=0;i<8;++i) { NVIC->ICER[i]=0xffffffff; NVIC->ICPR[i]=0xffffffff; }
 SCB->ICSR=SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
 SCB->VTOR=FW_APP_BASE; __DSB(); __ISB();
 const uint32_t *v=(const uint32_t *)FW_APP_BASE;
 enter_application(v[0],v[1]);
}
int boot_main(void)
{
 SystemInit();
 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
 DWT->CYCCNT=0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
 RCC->APB1ENR |= RCC_APB1ENR_PWREN;
 PWR->CR |= PWR_CR_DBP;
 int requested=RTC->BKP0R==FW_REQUEST_MAGIC;
 RTC->BKP0R=0;
 service_init();
 uint32_t start=DWT->CYCCNT;
 while (!elapsed(start,50)) {} /* Bridge may hold NSS low across reset for recovery. */
 requested |= selected();
 /* If install was interrupted, stage+manifest survive and installation restarts. */
 fw_update_install(&fw_stm32_flash);
 if (!requested && !fw_application_valid(&fw_stm32_flash)) jump();
 for (;;) {
   /* Wait for the bridge to release NSS after its reset request. */
   while (selected()) {}
   arm(0);
   if (transfer(0)) { dma_stop(); continue; }
   dma_stop();
   GPIOB->BSRR=1u<<(4+16); data_mode(1); /* BUSY; bridge is now high impedance. */
   int boot=fw_protocol(&fw_stm32_flash,&update,&packet,&reply);
   arm(1); /* GPIO input => external pull-up signals READY. */
   if (!transfer(1) && boot) jump();
   dma_stop(); data_mode(0);
 }
}
