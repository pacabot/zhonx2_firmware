/**
 * hal_adc.c
 */

#include "config/config.h"
#include "config/basetypes.h"
#include "config/errors.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

#include "hal/hal_adc.h"

#include <string.h>

/* ADC1 DR register address */
#define ADC1_DR_Address (uint32_t)0x4001204C

#define MAX_ADC_HANDLES 2

/* Static functions */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);

/* Array of converted values */
volatile unsigned short convertedValues[MAX_ADC_HANDLES];


int hal_adc_init(void)
{
    memset((void *)convertedValues, 0, MAX_ADC_HANDLES * sizeof(unsigned short));

    RCC_Configuration();
    GPIO_Configuration();

    return HAL_ADC_E_SUCCESS;
}


int hal_adc_terminate(void)
{
    memset((void *)convertedValues, 0, MAX_ADC_HANDLES * sizeof(unsigned short));

    return HAL_ADC_E_SUCCESS;
}

int hal_adc_open(HAL_ADC_HANDLE *handle, void *params)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;

    UNUSED(params);

    /* DMA2_Stream0 channel0 configuration **************************************/
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)convertedValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = MAX_ADC_HANDLES;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    /* DMA2_Stream0 enable */
    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* ADC Common Init **********************************************************/
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC1 Init ****************************************************************/
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = MAX_ADC_HANDLES;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Battery divider on PA4 / ADC1 channel 4 (not internal VBAT). */
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 2, ADC_SampleTime_480Cycles);

    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    /* Enable ADC1 **************************************************************/
    ADC_Cmd(ADC1, ENABLE);

    ADC_SoftwareStartConv(ADC1);

    *handle = (void *)convertedValues;

    return HAL_ADC_E_SUCCESS;
}


int hal_adc_close(HAL_ADC_HANDLE handle)
{
    return HAL_ADC_E_SUCCESS;
}


int hal_adc_get_value(HAL_ADC_HANDLE handle,
                      unsigned char channel, unsigned short *value)
{
    volatile unsigned short *h;
    if (handle == null)
    {
        return HAL_ADC_E_BAD_HANDLE;
    }

    if (channel > (MAX_ADC_HANDLES - 1))
    {
        return HAL_ADC_E_ERROR;
    }

    h = (volatile unsigned short *)handle;

    *value = (h[channel]) & 0xFFF;

    return HAL_ADC_E_SUCCESS;
}


void RCC_Configuration(void)
{
	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}


void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure ADC Channel 4 pin as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
