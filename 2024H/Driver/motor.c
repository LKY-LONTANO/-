#include "motor.h"

void Motor_On(void)
{
//    DL_GPIO_setPins(GPIO_MOTOR_PIN_FSTBY_PORT, GPIO_MOTOR_PIN_FSTBY_PIN);
//    DL_GPIO_setPins(GPIO_MOTOR_PIN_BSTBY_PORT, GPIO_MOTOR_PIN_BSTBY_PIN);
}

void Motor_Off(void)
{
//    DL_GPIO_clearPins(GPIO_MOTOR_PIN_FSTBY_PORT, GPIO_MOTOR_PIN_FSTBY_PIN);
//    DL_GPIO_clearPins(GPIO_MOTOR_PIN_BSTBY_PORT, GPIO_MOTOR_PIN_BSTBY_PIN);

    DL_GPIO_clearPins(GPIO_MOTOR_PIN_FL1_PORT, GPIO_MOTOR_PIN_FL1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_FL2_PORT, GPIO_MOTOR_PIN_FL2_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_FR1_PORT, GPIO_MOTOR_PIN_FR1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_FR2_PORT, GPIO_MOTOR_PIN_FR2_PIN);

    DL_GPIO_clearPins(GPIO_MOTOR_PIN_BL1_PORT, GPIO_MOTOR_PIN_BL1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_BL2_PORT, GPIO_MOTOR_PIN_BL2_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_BR1_PORT, GPIO_MOTOR_PIN_BR1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_BR2_PORT, GPIO_MOTOR_PIN_BR2_PIN);
}

void Set_Speed(uint8_t side, int8_t duty)
{
    uint32_t compareValue = 0;
    if(side == 0)
    {
        if(duty < 0)
        {
            compareValue = 3199 - 3199 * (-duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_0_INDEX);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_FL1_PORT, GPIO_MOTOR_PIN_FL1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FL2_PORT, GPIO_MOTOR_PIN_FL2_PIN);

            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_2_INDEX);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_BL1_PORT, GPIO_MOTOR_PIN_BL1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BL2_PORT, GPIO_MOTOR_PIN_BL2_PIN);
        }
        else if(duty > 0)
        {
            compareValue = 3199 - 3199 * (duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_0_INDEX);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FL1_PORT, GPIO_MOTOR_PIN_FL1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_FL2_PORT, GPIO_MOTOR_PIN_FL2_PIN);

            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_2_INDEX);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BL1_PORT, GPIO_MOTOR_PIN_BL1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_BL2_PORT, GPIO_MOTOR_PIN_BL2_PIN);
        }
        else 
        {
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FL1_PORT, GPIO_MOTOR_PIN_FL1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FL2_PORT, GPIO_MOTOR_PIN_FL2_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BL1_PORT, GPIO_MOTOR_PIN_BL1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BL2_PORT, GPIO_MOTOR_PIN_BL2_PIN);
        }
    }
    else
    {
        if(duty < 0)
        {
            compareValue = 3199 - 3199 * (-duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_1_INDEX);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_FR1_PORT, GPIO_MOTOR_PIN_FR1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FR2_PORT, GPIO_MOTOR_PIN_FR2_PIN);

            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_3_INDEX);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_BR1_PORT, GPIO_MOTOR_PIN_BR1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BR2_PORT, GPIO_MOTOR_PIN_BR2_PIN);
        }
        else if(duty > 0)
        {
            compareValue = 3199 - 3199 * (duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_1_INDEX);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FR1_PORT, GPIO_MOTOR_PIN_FR1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_FR2_PORT, GPIO_MOTOR_PIN_FR2_PIN);

            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_3_INDEX);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BR1_PORT, GPIO_MOTOR_PIN_BR1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_BR2_PORT, GPIO_MOTOR_PIN_BR2_PIN);
        }
        else 
        {
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FR1_PORT, GPIO_MOTOR_PIN_FR1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_FR2_PORT, GPIO_MOTOR_PIN_FR2_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BR1_PORT, GPIO_MOTOR_PIN_BR1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_BR2_PORT, GPIO_MOTOR_PIN_BR2_PIN);
        }
    }
}
