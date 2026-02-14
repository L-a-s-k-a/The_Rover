#include "interrupt.h"

uint16_t ADC_Read_Inject(uint8_t channel);
uint16_t ADC_Read_Polling(uint8_t channel);

uint16_t adcBuf[3], adcInter;
uint16_t GlobalTickCount, MotorTickCount, potentiometr, adcTS;
uint8_t BtnNum;
float FLAG_Revolution, calculatePulseARR, Temperature, Vsense;

int main(void)
{
    RCC_Init();
    ITR_Init();
    GPIO_Init();
    SysTick_Init();
    TIM_PWM_Init();
    ADC_Init_Polling();
    // TIM_ENCODER_Init();
    // Enc_Trig_Init();

    while (1)
    {
        
        Vsense = (3.1 * adcBuf[2]) / 4095U;
        TIM3->CCR2 = (8400 * adcInter) / 4095; 
        Temperature = ((0.76 - Vsense) / 2.5) + 25;
    }
}