#include "n32g031.h"
#include "n32g031_gpio.h"
#include "n32g031_adc.h"
#include "n32g031_rcc.h"

#define ADC_THRESHOLD 1000  // 1.5V if 12-bit ADC and Vref = 3.0V

extern uint32_t SystemCoreClock;

void delay_ms(uint32_t ms)
{
    SysTick->LOAD  = (SystemCoreClock / 1000) - 1;
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (ms--) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
    SysTick->CTRL = 0;
}

void gpio_init(void)
{
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);

    // PB1 เป็น output
    GPIOB->PMODE &= ~(3 << (1 * 2));
    GPIOB->PMODE |=  (1 << (1 * 2));
    GPIOB->POTYPE &= ~(1 << 1);  // push-pull

    // PA0 เป็น input analog
    GPIOA->PMODE &= ~(3 << (0 * 2));
}

void adc_init(void)
{
    ADC_InitType ADC_InitStructure;

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);

    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn = DISABLE;
    ADC_InitStructure.ContinueConvEn = ENABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = 1;
    ADC_Init(ADC, &ADC_InitStructure);

    ADC_ConfigRegularChannel(ADC, ADC_CH_0, 1, ADC_SAMP_TIME_56CYCLES5);

    ADC_Enable(ADC, ENABLE);
    delay_ms(1);
}


uint16_t read_adc(void)
{
    ADC_EnableSoftwareStartConv(ADC, ENABLE);  // เริ่ม convert
    while (!ADC_GetFlagStatus(ADC, ADC_FLAG_ENDC));
    ADC_ClearFlag(ADC, ADC_FLAG_ENDC);
    return ADC_GetDat(ADC);
}

int main(void)
{
    SystemInit();
    gpio_init();
    adc_init();

    while (1)
    {
         uint16_t adc_val = read_adc();
        if (adc_val > ADC_THRESHOLD)
            GPIOB->POD |= (1 << 1);
        else
            GPIOB->POD &= ~(1 << 1);
        delay_ms(200);
    }
}
