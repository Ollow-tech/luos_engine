/******************************************************************************
 * @file transmission
 * @brief transmission management.
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/

/******************************* Description of the TX process ************************************************************************
 * 
 *                                           |                             Luos_send
 *                                           |                                 |
 *                                           |                                 |
 *                       TX_status=NOK  +----v-----+  TX_status=DISABLE  +-----v-----+
 *                           +----------|  Timeout |-----------+         |    Msg    |         +-----------+     +-----------+
 *                           |          +----------+           |         |Preparation|         | Collision | x4  |  Prepare  |      ^
 *                           |               |TX_status=OK     |         +-----------+         | Reception |---->|  ACK RX   |------+
 *                           |               |                 |               |               +-----------+     +-----------+
 *      +-----------+   +----v-----+    +----v-----+    +------v----+    +-----v-----+               |
 * ^    | TX_status |   |  Delay   |    |   rm     |    | TX_status |    |    Tx     |     ^         |Collision
 * +----| =DISABLE  |<--| compute  |    | TX task  |    | =DISABLE  |    |Allocation |-----+         |
 *      +-----------+   +----------+    +----------+    +-----------+    +-----------+tx_lock  +-----v-----+
 *                           |               |             |                   |               |  Stop TX  |
 *                           |               |             |         +---------+-------+       +-----------+
 *                           |retry=10       +-------------+-------->|Transmit |       |             |
 *                           |               |                       |Process  |       |       +-----v-----+
 *                      +----v-----+         |                       |   +-----v-----+ |       | Pass data |
 *                      | Exclude  |         |                       |   |    Get    | |   ^   |   to RX   |
 *                      |container |---------+                       |   | TX_tasks  |-+---+   +-----------+
 *                      +----------+                                 |   +-----------+ No task       |
 *                                                                   |         |       |       +-----v-----+
 *                                                                   |   +-----v-----+ |       | TX_status |      ^
 *                                                                   |   |Send + set | |       |   =NOK    |------+
 *                                                                   |   | TX_status | |       +-----------+
 *                                                                   |   +-----------+ |
 *                                                                   |         |       |
 *                                      +----------+                 |   +-----v-----+ |
 *                      +----------+    |TX_status |     ^           |   |   Enable  | |   ^
 *                  --->|   RX[1]  |--->| =DISABLE |-----+           |   | Collision |-+---+
 *                      +----------+    +----------+                 |   +-----------+ |
 *                                                                   +-----------------+
 * 
 ************************************************************************************************************************************/

#include <transmission.h>

#include "luos_hal.h"
#include <string.h>
#include <stdbool.h>
#include "context.h"
#include "reception.h"
#include "msg_alloc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t nbrRetry = 0;

/*******************************************************************************
 * Function
 ******************************************************************************/
static uint8_t Transmit_GetLockStatus(void);

/******************************************************************************
 * @brief Transmit an ACK
 * @param None
 * @return None
 ******************************************************************************/
void Transmit_SendAck(void)
{
    // Info : We don't consider this transmission as a complete message transmission but as a complete message reception.
    LuosHAL_SetRxState(false);
    // Transmit Ack data
    LuosHAL_ComTransmit((unsigned char *)&ctx.rx.status.unmap, 1);
    // Reset Ack status
    ctx.rx.status.unmap = 0x0F;
}
/******************************************************************************
 * @brief transmission process
 * @param pointer data to send
 * @param size of data to send
 * @return None
 ******************************************************************************/
void Transmit_Process()
{
    uint8_t *data = 0;
    uint16_t size;
    uint8_t localhost;
    ll_container_t *ll_container_pt;

    if ((MsgAlloc_GetTxTask(&ll_container_pt, &data, &size, &localhost) == SUCCEED) && (Transmit_GetLockStatus() == false))
    {
        // We have a task available and we can use the bus
        // Check if we already try to send a message multiple times and put it on stats
        if (*ll_container_pt->ll_stat.max_retry < nbrRetry)
        {
            *ll_container_pt->ll_stat.max_retry = nbrRetry;
            if (*ll_container_pt->ll_stat.max_retry >= NBR_RETRY)
            {
                // We failed to transmit this message. We can't allow it, there is a issue on this target.
                // If it was an ACK issue, save the target as dead container into the sending ll_container
                if (ctx.tx.collision)
                {
                    ll_container_pt->dead_container_spotted = (uint16_t)(((msg_t *)data)->header.target);
                }
                ll_container_pt->ll_stat.fail_msg_nbr++;
                nbrRetry = 0;
                ctx.tx.collision = false;
                MsgAlloc_PullMsgFromTxTask(); //TODO remove all TX tasks of this target
                if (MsgAlloc_GetTxTask(&ll_container_pt, &data, &size, &localhost) == FAILED)
                {
                    return;
                }
            }
        }
        // We will prepare to transmit something enable tx status as OK
        ctx.tx.status = TX_OK;
        // Check if ACK needed
        if ((((msg_t *)data)->header.target_mode == IDACK) || (((msg_t *)data)->header.target_mode == NODEIDACK))
        {
            // Check if it is a localhost message
            if (!localhost || (((msg_t *)data)->header.target == DEFAULTID))
            {
                // We need ta validate the good reception af the ack, change the state of state machine after the end of collision detection to wait a ACK
                ctx.tx.status = TX_NOK;
            }
        }

        // switch reception in collision detection mode
        if (Transmit_GetLockStatus() == false)
        {
            ctx.tx.lock = true;
            LuosHAL_SetIrqState(false);
            ctx.rx.callback = Recep_GetCollision;
            ctx.tx.data = data;
            LuosHAL_SetIrqState(true);
            LuosHAL_ComTransmit(data, size);
        }
    }
}
/******************************************************************************
 * @brief Send ID to others container on network
 * @param None
 * @return lock status
 ******************************************************************************/
static uint8_t Transmit_GetLockStatus(void)
{
    if (ctx.tx.lock != true)
    {
        ctx.tx.lock |= LuosHAL_GetTxLockState();
    }
    return ctx.tx.lock;
}
/******************************************************************************
 * @brief finish transmit and try to launch a new one
 * @param None
 * @return None
 ******************************************************************************/
void Transmit_End(void)
{
    if (ctx.tx.status == TX_OK)
    {
        // A tx_task have been sucessfully transmitted
        nbrRetry = 0;
        ctx.tx.collision = false;
        // Remove the task
        MsgAlloc_PullMsgFromTxTask();
    }
    else if (ctx.tx.status == TX_NOK)
    {
        // A tx_task failed
        nbrRetry++;
        // compute a delay before retry
        LuosHAL_ResetTimeout(20 * nbrRetry * (ctx.node.node_id + 1));
        // Lock the trasmission to be sure no one can send something from this node.
        ctx.tx.lock = true;
        ctx.tx.status = TX_DISABLE;
        return;
    }
    // Try to send something if we need to.
    Transmit_Process();
}
