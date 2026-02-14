#include "interrupt.h"

volatile uint16_t adcBuf[4], adcInter;
volatile uint8_t adc_conversion_complete, adc_overrun_count;

uint16_t GlobalTickCount, adc_value[4];
float Temperature, Vsense;

int main(void)
{
    RCC_Init();
    GPIO_Init();
    SysTick_Init();
    TIM_PWM_Init();
    ADC_Init();
    DMA_Init();

    while (1)
    {
        if(adc_conversion_complete) {
            adc_conversion_complete = 0; // Сброс флага готовности данных
            adc_value[0] = adcBuf[0];
            adc_value[1] = adcBuf[1];
            adc_value[2] = adcBuf[2];
            adc_value[3] = adcBuf[3];
        }
        // Vsense = (3.1 * adcBuf[2]) / 4095U;
        TIM3->CCR2 = (8400 * adc_value[0]) / 4095;
        // Temperature = ((0.76 - Vsense) / 2.5) + 25;
    }
}