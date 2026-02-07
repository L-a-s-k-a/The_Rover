#include "interrupt.h"

uint16_t ADC_Read_Polling(uint8_t channel);

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
        // potentiometr = ADC_Read_Polling(0);
        adcTS = ADC_Read_Polling(16);
        Vsense = (3.1 * adcTS) / 4095U;
        // TIM3->CCR2 = (8400 * potentiometr) / 4095; 
        Temperature = ((0.76 - Vsense) / 2.5) + 25;
    }
}

uint16_t ADC_Read_Polling(uint8_t channel){
    // if (channel == 0){
        MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, (channel << ADC_SQR3_SQ1_Pos));
    // }
    // if (channel == 16){
    //     MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ16, (channel << ADC_SQR1_SQ16_Pos));
    // }
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); 

    while(READ_BIT(ADC1->SR, ADC_SR_EOC) == 0);

    return READ_REG(ADC1->DR);
}