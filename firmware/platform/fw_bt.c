#include "fw_bt.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "hal/hal_os.h"
#include <string.h>
/* USART1: RX DMA2 stream 2 channel 4 (circular), TX DMA2 stream 7 channel 4.
 * The ADC keeps DMA2 stream 0, the bootloader's SPI3 uses DMA1 only. */
#define RX_STREAM DMA2_Stream2
#define TX_STREAM DMA2_Stream7
#define RX_SIZE 1024u
#define TX_SIZE 4096u
#define CHUNK 64u
#define BUDGET_MAX (4u*CHUNK)   /* bytes the module may receive in one burst */
#define STALL_MS 200u
static uint8_t rx_dma[RX_SIZE];
static uint8_t tx_ring[TX_SIZE];
static uint8_t chunk[CHUNK];
static unsigned rx_tail,tx_head,tx_count,chunk_length;
static uint32_t chunk_started,budget_ms,budget_mbytes;
static unsigned tx_rate=FW_BT_TX_RATE_DEFAULT,current_baud;
static volatile int connected,quiet;
static int configuring;
static fw_hm18_t module;
static fw_bt_status_t stats;
static void set_baud(unsigned baud)
{
    RCC_ClocksTypeDef clocks;RCC_GetClocksFreq(&clocks);
    USART1->CR1&=~USART_CR1_UE;
    USART1->BRR=(uint16_t)((clocks.PCLK2_Frequency+baud/2)/baud);
    USART1->CR1|=USART_CR1_UE;
    current_baud=baud;
}
static int tx_busy(void)
{
    return DMA_GetCmdStatus(TX_STREAM)==ENABLE || !(USART1->SR&USART_SR_TC);
}
static void tx_start(const uint8_t *data,unsigned length)
{
    DMA_ClearFlag(TX_STREAM,DMA_FLAG_TCIF7|DMA_FLAG_HTIF7|DMA_FLAG_TEIF7|DMA_FLAG_DMEIF7|DMA_FLAG_FEIF7);
    TX_STREAM->M0AR=(uint32_t)data;TX_STREAM->NDTR=length;
    USART1->SR&=~USART_SR_TC;
    DMA_Cmd(TX_STREAM,ENABLE);
    chunk_started=hal_os_get_systicks();
}
static void rx_init(void)
{
    DMA_InitTypeDef d;
    DMA_DeInit(RX_STREAM);DMA_StructInit(&d);
    d.DMA_Channel=DMA_Channel_4;d.DMA_PeripheralBaseAddr=(uint32_t)&USART1->DR;
    d.DMA_Memory0BaseAddr=(uint32_t)rx_dma;d.DMA_DIR=DMA_DIR_PeripheralToMemory;
    d.DMA_BufferSize=RX_SIZE;d.DMA_MemoryInc=DMA_MemoryInc_Enable;d.DMA_Mode=DMA_Mode_Circular;
    d.DMA_Priority=DMA_Priority_Medium;
    DMA_Init(RX_STREAM,&d);DMA_Cmd(RX_STREAM,ENABLE);
    rx_tail=0;
}
static void tx_init(void)
{
    DMA_InitTypeDef d;
    DMA_DeInit(TX_STREAM);DMA_StructInit(&d);
    d.DMA_Channel=DMA_Channel_4;d.DMA_PeripheralBaseAddr=(uint32_t)&USART1->DR;
    d.DMA_Memory0BaseAddr=(uint32_t)chunk;d.DMA_DIR=DMA_DIR_MemoryToPeripheral;
    d.DMA_BufferSize=1;d.DMA_MemoryInc=DMA_MemoryInc_Enable;d.DMA_Mode=DMA_Mode_Normal;
    d.DMA_Priority=DMA_Priority_Low;
    DMA_Init(TX_STREAM,&d);
}
void fw_bt_init(uint32_t now)
{
    GPIO_InitTypeDef g;USART_InitTypeDef u;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
    GPIO_StructInit(&g);g.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;g.GPIO_Mode=GPIO_Mode_AF;
    g.GPIO_OType=GPIO_OType_PP;g.GPIO_PuPd=GPIO_PuPd_UP;g.GPIO_Speed=GPIO_Speed_25MHz;
    GPIO_Init(GPIOA,&g);
    USART_Cmd(USART1,DISABLE);USART_StructInit(&u);u.USART_BaudRate=FW_HM18_BAUD;
    USART_Init(USART1,&u);
    USART_DMACmd(USART1,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
    USART_Cmd(USART1,ENABLE);
    rx_init();tx_init();
    tx_head=tx_count=chunk_length=0;connected=quiet=0;memset(&stats,0,sizeof stats);
    fw_hm18_start(&module,now);configuring=1;current_baud=0;
    budget_ms=now;budget_mbytes=0;
}
/* Bytes received since the last call, in order; resynchronises after a lap. */
static int rx_next(uint8_t *byte)
{
    unsigned head=RX_SIZE-DMA_GetCurrDataCounter(RX_STREAM);
    if(head>=RX_SIZE || head==rx_tail)return 0;
    *byte=rx_dma[rx_tail];rx_tail=(rx_tail+1)%RX_SIZE;
    return 1;
}
static void configure(uint32_t now)
{
    uint8_t byte;
    while(rx_next(&byte))fw_hm18_rx(&module,byte,now);
    if(tx_busy())return;
    const char *send;
    if(fw_hm18_step(&module,now,&send)) {
        if(current_baud!=module.baud)set_baud(module.baud);
        rx_tail=RX_SIZE-DMA_GetCurrDataCounter(RX_STREAM);
        if(rx_tail>=RX_SIZE)rx_tail=0;
        unsigned n=(unsigned)strlen(send);if(n>CHUNK)n=CHUNK;
        memcpy(chunk,send,n);tx_start(chunk,n);
    }
    if(fw_hm18_ready(&module)) {configuring=0;tx_count=0;}
}
void fw_bt_poll(uint32_t now)
{
    uint32_t sr=USART1->SR;
    if(sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE))++stats.rx_errors;
    if(DMA_GetCmdStatus(TX_STREAM)==ENABLE && (uint32_t)(now-chunk_started)>STALL_MS) {
        /* A transfer that never completes would block the link for good. */
        DMA_Cmd(TX_STREAM,DISABLE);++stats.tx_stalls;
    }
    if(configuring) {configure(now);return;}
    uint32_t dt=now-budget_ms;budget_ms=now;
    if(dt>1000)dt=1000;
    budget_mbytes+=dt*tx_rate;
    if(budget_mbytes>BUDGET_MAX*1000u)budget_mbytes=BUDGET_MAX*1000u;
    if(DMA_GetCmdStatus(TX_STREAM)==ENABLE || !tx_count)return;
    if(quiet || !connected) {stats.tx_dropped+=tx_count;tx_count=0;return;}
    unsigned n=tx_count<CHUNK?tx_count:CHUNK;
    if(budget_mbytes<n*1000u)return;
    unsigned tail=(tx_head+TX_SIZE-tx_count)%TX_SIZE;
    for(unsigned i=0;i<n;++i)chunk[i]=tx_ring[(tail+i)%TX_SIZE];
    tx_count-=n;budget_mbytes-=n*1000u;stats.tx_bytes+=n;
    tx_start(chunk,n);
}
int fw_bt_read(uint8_t *byte)
{
    if(configuring || !rx_next(byte))return 0;
    ++stats.rx_bytes;return 1;
}
unsigned fw_bt_tx_free(void) {return configuring || quiet || !connected?0:TX_SIZE-tx_count;}
int fw_bt_send(const void *data,unsigned length)
{
    if(!length)return 1;
    if(length>fw_bt_tx_free())return 0;
    const uint8_t *p=data;
    for(unsigned i=0;i<length;++i) {tx_ring[tx_head]=p[i];tx_head=(tx_head+1)%TX_SIZE;}
    tx_count+=length;return 1;
}
void fw_bt_set_connected(int value)
{
    if(value && !connected)++stats.connects;
    if(!value && connected)++stats.losses;
    connected=value;
}
int fw_bt_connected(void) {return connected;}
int fw_bt_ready(void) {return !configuring;}
void fw_bt_quiet(int value) {quiet=value;}
void fw_bt_set_rate(unsigned rate)
{
    tx_rate=rate<FW_BT_TX_RATE_MIN?FW_BT_TX_RATE_MIN:rate>FW_BT_TX_RATE_MAX?FW_BT_TX_RATE_MAX:rate;
}
unsigned fw_bt_rate(void) {return tx_rate;}
void fw_bt_status(fw_bt_status_t *s)
{
    *s=stats;s->state=fw_hm18_state_name(&module);s->baud=current_baud;s->tx_rate=tx_rate;
    s->ready=!configuring;s->connected=connected;s->quiet=quiet;
    s->name=module.name[0]?module.name:FW_HM18_NAME;s->address=module.address;
}
