/******************************************************************************
 * @file laser
 * @brief driver example a laser
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef LASER_H
#define LASER_H

#include "luos_engine.h"
#include "main.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PWM_PIN_CLK()                 \
    do                                \
    {                                 \
        __HAL_RCC_GPIOB_CLK_ENABLE(); \
    } while (0U)

#define PWM_TIMER_CLK()               \
    do                                \
    {                                 \
        __HAL_RCC_TIM17_CLK_ENABLE(); \
    } while (0U)

#define LASER_PWM_FREQ 5000

#define LASER_PWM_PIN  GPIO_PIN_9
#define LASER_PWM_PORT GPIOB
#define LASER_PWM_AF   GPIO_AF14_TIM17

#ifndef LASER_PWM_TIMER
    #define LASER_PWM_TIMER TIM17
#endif
#define LASER_PWM_CHANNEL TIM_CHANNEL_1

#ifndef DEFAULT_SAMPLE_FREQUENCY
    #define DEFAULT_SAMPLE_FREQUENCY 10000.0
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function
 ******************************************************************************/
void Laser_Init(void);
void Laser_Loop(void);

void LaserPeriod_Callback(TIM_HandleTypeDef *htim);

#endif /* LASER_H */
