#include "interrupt.h"

uint16_t GlobalTickCount, MotorTickCount;
uint8_t BtnNum;
float FLAG_Revolution, calculatePulseARR;

int main(void)
{
    RCC_Init();
    ITR_Init();
    GPIO_Init();
    SysTick_Init();
    TIM_PWM_Init();
    TIM_ENCODER_Init();
    Enc_Trig_Init();

    while (1)
    {
        
    }
}
