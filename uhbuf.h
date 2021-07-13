#ifndef __UHBUF_H__
#define __UHBUF_H__

#include "cyu3types.h"
#include "cyu3dma.h"

#define CY_FX_WAIT_TIMEOUT              (5000)          /* Timeout for transfers. */

/*
CyU3PReturnStatus_t
CyFxChanRecovery (
		uint8_t Ep,
		CyU3PDmaChannel *Ch);
*/

CyU3PReturnStatus_t
CyFxSendBuffer (
		uint8_t outEp,
		CyU3PDmaChannel *outCh,
        uint8_t *buffer,
        uint16_t count);

CyU3PReturnStatus_t
CyFxRecvBuffer (
		uint8_t inpEp,
		CyU3PDmaChannel *inpCh,
        uint8_t *buffer,
        uint16_t count,
        uint32_t *length);

#endif
