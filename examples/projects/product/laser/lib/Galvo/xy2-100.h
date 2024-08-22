/******************************************************************************
 * @file xy2-100
 * @brief driver for the xy2-100 protocol
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef XY2_100_H
#define XY2_100_H

#include <stdint.h>
#include "luos_engine.h"
#include "product_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function
 ******************************************************************************/
void Xy_Init(void);
void Xy_Start(streaming_channel_t *stream, time_luos_t period);
void Xy_Stop(void);
void Xy_BufferMode(buffer_mode_t mode);
void Xy_SetPeriod(time_luos_t period);
void Xy_SetNbPatternToDo(uint32_t pattern_number);

#endif /* XY2_100_H */
