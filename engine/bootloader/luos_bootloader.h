/******************************************************************************
 * @file Bootloader
 * @brief Bootloader functionnalities for luos framework
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "struct_luos.h"

/*******************************************************************************
 * Function
 ******************************************************************************/

/******************************************************************************
 * @brief Initialize bootloader function
 ******************************************************************************/
void LuosBootloader_Init(void);

/******************************************************************************
 * @brief Main function used by the bootloader app
 ******************************************************************************/
void LuosBootloader_Loop(void);

/******************************************************************************
 * @brief function used by Luos to send message to the bootloader
 ******************************************************************************/

#endif /* BOOTLOADER_H */