#include "interrupt.h"

uint16_t ADC_Read_Polling(uint8_t channel);

uint16_t GlobalTickCount, MotorTickCount, adcValue;
uint8_t BtnNum;
float FLAG_Revolution, calculatePulseARR;

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
        adcValue = ADC_Read_Polling(0);
        TIM3->CCR2 = (8400 * adcValue) / 4095; 

    }
}

uint16_t ADC_Read_Polling(uint8_t channel){
    MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, (channel << ADC_SQR3_SQ1_Pos));
 
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); 

    while(READ_BIT(ADC1->SR, ADC_SR_EOC) == 0);

    return READ_REG(ADC1->DR);
}