/******************************************************************************
 * @file robus_config config
 * @brief config of the Robus protocol
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef _ROBUS_CONFIG_H_
#define _ROBUS_CONFIG_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TIMEOUT_VAL 2

#ifndef DEFAULTBAUDRATE
    #define DEFAULTBAUDRATE 1000000
#endif

#ifndef NBR_RETRY
    #define NBR_RETRY 10
#endif

#ifndef NBR_PORT
    #define NBR_PORT 2
#endif

#define CRC_SIZE 2

#endif /* _ROBUS_CONFIG_H_ */
