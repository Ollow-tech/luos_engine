/******************************************************************************
 * @file laser
 * @brief driver example a laser
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#include "laser.h"
#include "product_config.h"
#include "main.h"
#include <stdio.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

ratio_t laser_stream_buf[4096];
streaming_channel_t laser_stream;
time_luos_t laser_period;
control_t laser_control;
buffer_mode_t laser_buffer_mode = SINGLE;
TIM_TypeDef *pwmtimer           = LASER_PWM_TIMER;
extern TIM_HandleTypeDef htim2;
volatile float nb_pattern_laser = 0;
float nb_pattern_laser_to_do = 0;
/*******************************************************************************
 * Function
 ******************************************************************************/
static void Laser_MsgHandler(service_t *service, const msg_t *msg);

/******************************************************************************
 * @brief DRV_DCMotorHWInit
 * @param None
 * @return None
 ******************************************************************************/
static void laser_pwmInit(void)
{
    ///////////////////////////////
    // Timer PWM Init
    ///////////////////////////////
    // Initialize clock
    PWM_TIMER_CLK();
    TIM_HandleTypeDef htim;
    TIM_ClockConfigTypeDef sClockSourceConfig           = {0};
    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim.Instance               = LASER_PWM_TIMER;
    htim.Init.Prescaler         = 0;
    htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim.Init.Period            = (uint32_t)(((80000000.0f) * (1/LASER_PWM_FREQ))-1+0.5);
    htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, LASER_PWM_CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Base_Init(&htim);
    HAL_TIM_Base_Start(&htim);
    HAL_TIM_PWM_Init(&htim);
    HAL_TIM_PWM_Start(&htim, LASER_PWM_CHANNEL);
    ///////////////////////////////
    // GPIO Init
    ///////////////////////////////
    PWM_PIN_CLK();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pin       = LASER_PWM_PIN;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = LASER_PWM_AF;
    HAL_GPIO_Init(LASER_PWM_PORT, &GPIO_InitStruct);
}

/******************************************************************************
 * @brief init must be call in project init
 * @param None
 * @return None
 ******************************************************************************/
static void laser_pwmSetPower(ratio_t power)
{
    LASER_PWM_TIMER->CCR1 = (uint32_t)((RatioOD_RatioTo_Percent(power) * (LASER_PWM_TIMER->ARR)) / 100.0);
}


/******************************************************************************
 * @brief modif_laser_period to send powers ratio
 * @param reading_period
 * @return None
 ******************************************************************************/
static void modif_laser_period(float reading_period)
{
    TIM2->ARR = (uint16_t)((((80000000.0f) * reading_period)/(htim2.Init.Prescaler + 1))-1+0.5);
}

/******************************************************************************
 * @brief Timer Init for sending powers ratio 
 * @param None
 * @return None
 ******************************************************************************/
static void sendingPowersTimer_Init(float reading_period)
{
    // Init TIM2 for reading powers ratio
    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim2); // Démarrage du TIM2 avec interruptions
    TIM2->DIER = TIM_DIER_UIE;
    modif_laser_period(reading_period);
}

/******************************************************************************
 * @brief init must be call in project init
 * @param None
 * @return None
 ******************************************************************************/
void Laser_Init(void)
{
    // Init the timer generating the PWM
    laser_pwmInit();
    LASER_PWM_TIMER->ARR = (uint16_t)(((80000000.0f) * 1/LASER_PWM_FREQ)-1+0.5);
    // Init the Luos service
    revision_t revision = {.major = 1, .minor = 0, .build = 0};
    Luos_CreateService(Laser_MsgHandler, POWER_TYPE, "laser", revision);
    laser_stream             = Streaming_CreateChannel(laser_stream_buf, sizeof(laser_stream_buf), sizeof(ratio_t));
    laser_period             = TimeOD_TimeFrom_s(1.0 / DEFAULT_SAMPLE_FREQUENCY); // Configure the trajectory samplerate at 100Hz
    laser_control.flux = STOP;

    // Init the timer for sending powers ratio
    sendingPowersTimer_Init(laser_period.raw);
}

/******************************************************************************
 * @brief loop must be call in project loop
 * @param None
 * @return None
 ******************************************************************************/
void Laser_Loop(void)
{
    // static time_luos_t last_time = {.raw = 0.0};
    // if (laser_control.flux == PLAY)
    // {
    //     // Check if it's time to update the trajectory
    //     if ((Luos_Timestamp().raw - last_time.raw) > laser_period.raw)
    //     {
    //         if ((Streaming_GetAvailableSampleNB(&laser_stream) > 0))
    //         {
    //             // We have available samples
    //             // update the last time
    //             last_time = Luos_Timestamp();
    //             // Get the next point
    //             ratio_t point;
    //             Streaming_GetSample(&laser_stream, (void *)&point, 1);
    //             // Send the point to the laser
    //             laser_pwmSetPower(point);
    //         }
    //         else
    //         {
    //             // We don't have any available samples
    //             switch (laser_buffer_mode)
    //             {
    //                 case SINGLE:
    //                     // We are in single mode, we have to loop on the ring buffer
    //                     // Put the read pointer at the begining of the buffer
    //                     laser_stream.sample_ptr = laser_stream.ring_buffer;
    //                     // stop the trjectory
    //                     laser_control.flux = STOP;
    //                     break;
    //                 case CONTINUOUS:
    //                     // We are in continuous mode, we have to loop on the ring buffer
    //                     // Put the read pointer at the begining of the buffer
    //                     laser_stream.sample_ptr = laser_stream.ring_buffer;
    //                     // Get the first sample
    //                     if (Streaming_GetAvailableSampleNB(&laser_stream) == 0)
    //                     {
    //                         // We don't have any new sample to compute
    //                         // stop the trajectory
    //                         laser_control.flux = STOP;
    //                         return;
    //                     }
    //                     // update the last time
    //                     last_time = Luos_Timestamp();
    //                     // Get the next point
    //                     ratio_t point;
    //                     Streaming_GetSample(&laser_stream, (void *)&point, 1);
    //                     // Send the point to the laser
    //                     laser_pwmSetPower(point);
    //                     break;
    //                 case STREAM:
    //                     // We are in stream mode, we don't have any new data so we stop the trajectory
    //                     laser_control.flux = STOP;
    //                     break;
    //                 default:
    //                     LUOS_ASSERT(0);
    //                     break;
    //             }
    //         }
    //     }
    // }
    // else
    // {
    //     // Stop the laser
    //     ratio_t power;
    //     power.raw = 0;
    //     laser_pwmSetPower(power);
    // }
}

/******************************************************************************
 * @brief Msg Handler call back when a msg receive for this service
 * @param Service destination
 * @param Msg receive
 * @return None
 ******************************************************************************/
static void Laser_MsgHandler(service_t *service, const msg_t *msg)
{
    if (msg->header.cmd == GET_CMD)
    {
        // The laser don't send anything back
        return;
    }
    if (msg->header.cmd == CONTROL)
    {
        // Get the laser_control value
        ControlOD_ControlFromMsg(&laser_control, msg);
        if (laser_control.flux == PLAY)
        {
            // Start the trajectory
            nb_pattern_laser = 0;
        }
    }
    if (msg->header.cmd == RATIO)
    {
        if (laser_buffer_mode == STREAM)
        {
            // The laser is in single mode, we can consider it as a streaming of point that will be consumed
            Luos_ReceiveStreaming(service, msg, &laser_stream);
        }
        else
        {
            // The laser is in SINGLE or CONTINUOUS mode, The buffer need to be loaded with the trajectory and the laser will play it from the begining of the buffer to the end.
            int size = Luos_ReceiveData(service, msg, (uint8_t *)&laser_stream_buf);
            if (size > 0)
            {
                LUOS_ASSERT(size <= sizeof(laser_stream_buf));
                Streaming_ResetChannel(&laser_stream);
                Streaming_AddAvailableSampleNB(&laser_stream, size / laser_stream.data_size);
            }
        }
    }
    if (msg->header.cmd == BUFFER_MODE)
    {
        laser_buffer_mode = msg->data[0];
    }
    if (msg->header.cmd == TIME)
    {
        // Get the time
        TimeOD_TimeFromMsg(&laser_period, msg);
        // laser_period = TimeOD_TimeFrom_s(0.000966);  
        modif_laser_period(laser_period.raw);      
    }
    if (msg->header.cmd == SET_NB_PATTERN)
    {
        // Get the number of pattern to do
       memcpy(&nb_pattern_laser_to_do, msg->data, msg->header.size);
    }
}

void LaserPeriod_Callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) // vérifie que l'interruption provient bien du TIM2
    {
        if (laser_control.flux == PLAY)
        {
            if ((Streaming_GetAvailableSampleNB(&laser_stream) > 0))
            {
                // We have available samples
                // Get the next point
                ratio_t point;
                Streaming_GetSample(&laser_stream, (void *)&point, 1);
                // Send the point to the laser
                laser_pwmSetPower(point);
            }
            else
            {
                // We don't have any available samples
                switch (laser_buffer_mode)
                {
                    case SINGLE:
                        // We are in single mode, we have to loop on the ring buffer
                        // Put the read pointer at the begining of the buffer
                        laser_stream.sample_ptr = laser_stream.ring_buffer;
                        // stop the trajectory
                        laser_control.flux = STOP;
                        break;
                    case CONTINUOUS:
                        nb_pattern_laser++;

                        // We are in continuous mode, we have to loop on the ring buffer
                        // Put the read pointer at the begining of the buffer
                        if (nb_pattern_laser == nb_pattern_laser_to_do)
                        {
                            // Put the read pointer at the begining of the buffer
                            laser_stream.sample_ptr = laser_stream.ring_buffer;
                            // stop the trajectory
                            laser_control.flux = STOP;
                        }
                        else
                        {
                            laser_stream.sample_ptr = laser_stream.ring_buffer;
                            // Get the first sample
                            if (Streaming_GetAvailableSampleNB(&laser_stream) == 0)
                            {
                                // We don't have any new sample to compute
                                // stop the trajectory
                                laser_control.flux = STOP;
                                return;
                            }
                            // Get the next point
                            ratio_t point;
                            Streaming_GetSample(&laser_stream, (void *)&point, 1);
                            // Send the point to the laser
                            laser_pwmSetPower(point);
                        }

                        break;
                    case STREAM:
                        // We are in stream mode, we don't have any new data so we stop the trajectory
                        laser_control.flux = STOP;
                        break;
                    default:
                        LUOS_ASSERT(0);
                        break;
                }
            }
        }
        else
        {
            // Stop the laser
            ratio_t power;
            power.raw = 0;
            laser_pwmSetPower(power);
        }
    }
}