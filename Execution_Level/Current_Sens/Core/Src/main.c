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

    // SET_BIT(ADC1->CR2, ADC_CR2_JSWSTART);
    // SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);

    while (1)
    {
        // adcTS = ADC_Read_Polling(0);
        // adcBuf[0] = ADC_Read_Polling(0);
        adcBuf[0] = READ_REG(ADC1->DR);
        // for(int i = 0; i < 10000000; i++);
        // adcTS = ADC_Read_Polling(1);
        // adcBuf[1] = ADC_Read_Polling(1);
        // for(int i = 0; i < 10000000; i++);
        // adcTS = ADC_Read_Polling(16);
        // adcBuf[2] = ADC_Read_Polling(16);
        // for(int i = 0; i < 10000000; i++);
        // adcBuf[3] = ADC_Read_Inject(2);
        adcBuf[3] = READ_REG(ADC1->JDR1);
        // for(int i = 0; i < 10000000; i++);
        Vsense = (3.1 * adcBuf[2]) / 4095U;
        TIM3->CCR2 = (8400 * adcBuf[0]) / 4095; 
        Temperature = ((0.76 - Vsense) / 2.5) + 25;
    }
}

uint16_t ADC_Read_Inject(uint8_t channel){
    MODIFY_REG(ADC1->JSQR, ADC_JSQR_JSQ4, (channel << ADC_JSQR_JSQ4_Pos));

    SET_BIT(ADC1->CR2, ADC_CR2_JSWSTART); 

    while(READ_BIT(ADC1->SR, ADC_SR_JEOC) == 0);

    return READ_REG(ADC1->JDR1);
}

uint16_t ADC_Read_Polling(uint8_t channel){
    MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, (channel << ADC_SQR3_SQ1_Pos));
    
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); 

    while(READ_BIT(ADC1->SR, ADC_SR_EOC) == 0);

    return READ_REG(ADC1->DR);
}