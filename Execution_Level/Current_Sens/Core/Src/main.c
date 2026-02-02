#include "interrupt.h"
#include "init.h"

float calculatePulseARR;

int main(void)
{
    RCC_Init();
    ITR_Init();
    GPIO_Init();
    SysTick_Init();

    while (1)
    {

    }
}
