#include "interrupt.h"
#include "../FreeRTOS_Inc/FreeRTOS.h"

#define ADC_DIVISION 3.26 / 4095

volatile uint16_t adc_buf[ADC_NUM_CHANNEL - 1], adcInter, motor_freq;
volatile uint8_t adc_conversion_complete, adc_overrun_count, motor_direction;

uint16_t GlobalTickCount, adc_value[ADC_NUM_CHANNEL - 1];
float Temperature, Vsense, voltage;


int main(void)
{
    RCC_Init();
    GPIO_Init();
    SysTick_Init();
    /*Здесь важен порядок инициализации, так как АЦП должен быть настроен до DMA,
     *а DMA до таймера, который запускает преобразование
     */   
    ADC_Init();
    DMA_Init();
    TIM_PWM_Init();

    while (1)
    {
        if(adc_conversion_complete) {
            adc_conversion_complete = 0; // Сброс флага готовности данных
            adc_value[0] = adc_buf[0];
            // adc_value[1] = adc_buf[1];
            // adc_value[2] = adc_buf[2];
            // adc_value[3] = adc_buf[3];
        }
        voltage = adc_value[0] * ADC_DIVISION;

        if (motor_direction){
            SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS6);
        }
        else SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR6);
    
        // Vsense = (3.1 * adcBuf[2]) / 4095U;
        // TIM3->CCR2 = (4199 * adc_value[0]) / 4095;
        TIM3->CCR2 = motor_freq;
        // Temperature = ((0.76 - Vsense) / 2.5) + 25;
    }
}