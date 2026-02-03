#include "interrupt.h"

uint16_t GlobalTickCount, MotorTickCount;
uint16_t ENC_CNT, encItr;
uint8_t encDir, BtnNum;
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
