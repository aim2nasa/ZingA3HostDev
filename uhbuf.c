#include "uhbuf.h"
#include "cyu3error.h"
#include "cyu3usbhost.h"
#include "cyu3utils.h"
#include "cyu3system.h"
#include "host.h"

/*
CyU3PReturnStatus_t
CyFxChanRecovery (
		uint8_t Ep,
		CyU3PDmaChannel *Ch)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (Ep != 0)
    {
        CyU3PDmaChannelReset (Ch);
        CyU3PUsbHostEpAbort (Ep);

        status = CyFxSendSetupRqt (0x02, CY_U3P_USB_SC_CLEAR_FEATURE,
                0, Ep, 0, glEp0Buffer);
        if (status == CY_U3P_SUCCESS)
        {
            status = CyU3PUsbHostEpReset (Ep);
        }
    }
    return status;
}
*/

CyU3PReturnStatus_t
CyFxSendBuffer (
		uint8_t outEp,
		CyU3PDmaChannel *outCh,
        uint8_t *buffer,
        uint16_t count)
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PUsbHostEpStatus_t epStatus;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Setup the DMA for transfer. */
    buf_p.buffer = buffer;
    buf_p.count  = count;
    buf_p.size   = ((count + 0x0F) & ~0x0F);
    buf_p.status = 0;
    status = CyU3PDmaChannelSetupSendBuffer (outCh, &buf_p);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxSendBuffer] CyU3PDmaChannelSetupSendBuffer error=0x%x,count=%d,size=%d\r\n",status,buf_p.count,buf_p.size);
    	return status;
    }

    status = CyU3PUsbHostEpSetXfer (outEp,
            CY_U3P_USB_HOST_EPXFER_NORMAL, count);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxSendBuffer] CyU3PUsbHostEpSetXfer error=0x%x, ep=0x%x,count=%d\r\n",status,outEp,count);
    	return status;
    }

    status = CyU3PUsbHostEpWaitForCompletion (outEp, &epStatus,
    		CY_FX_WAIT_TIMEOUT);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxSendBuffer] CyU3PUsbHostEpWaitForCompletion error=0x%x,ep=0x%x,timeout=%d\r\n",status,outEp,CY_FX_WAIT_TIMEOUT);
    	return status;
    }

    status = CyU3PDmaChannelWaitForCompletion (outCh, CYU3P_NO_WAIT);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxSendBuffer] CyU3PDmaChannelWaitForCompletion error=0x%x\r\n",status);
    	return status;
    }
    return status;
}

CyU3PReturnStatus_t
CyFxRecvBuffer (
		uint8_t inpEp,
		CyU3PDmaChannel *inpCh,
        uint8_t *buffer,
        uint16_t count,
        uint32_t *length)
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PUsbHostEpStatus_t epStatus;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t prodXferCount = 0;
    uint32_t consXferCount = 0;
    CyU3PDmaState_t state = 0;

    *length = 0;
    /* Setup the DMA for transfer. */
    buf_p.buffer = buffer;
    buf_p.count  = 0;
    buf_p.size   = ((count + 0x0F) & ~0x0F);
    buf_p.status = 0;
    status = CyU3PDmaChannelSetupRecvBuffer (inpCh, &buf_p);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxRecvBuffer] CyU3PDmaChannelSetupRecvBuffer error=0x%x,count=%d,size=%d\r\n",status,buf_p.count,buf_p.size);
    	return status;
    }

    uint32_t *host_active_ep = (uint32_t*)0xE0032020;
    CyU3PDebugPrint (4,"host_active_ep=%x", *host_active_ep);

    status = CyU3PUsbHostEpWaitForCompletion (inpEp, &epStatus,
                 CYU3P_NO_WAIT);
    if (status != CY_U3P_ERROR_INVALID_SEQUENCE)
    {
     CyU3PDebugPrint (4,"hostepwaitstat=%x", status);
    }

    status = CyU3PUsbHostEpSetXfer (inpEp,
            CY_U3P_USB_HOST_EPXFER_NORMAL, count);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxRecvBuffer] CyU3PUsbHostEpSetXfer error=0x%x, ep=0x%x,count=%d\r\n",status,inpEp,count);
    	return status;
    }

    status = CyU3PUsbHostEpWaitForCompletion (inpEp, &epStatus,
    		CYU3P_WAIT_FOREVER);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxRecvBuffer] CyU3PUsbHostEpWaitForCompletion error=0x%x,ep=0x%x,timeout=%d\r\n",status,inpEp,CYU3P_WAIT_FOREVER);
    	return status;
    }

    status = CyU3PDmaChannelWaitForCompletion (inpCh, CYU3P_NO_WAIT);
    if(status!=CY_U3P_SUCCESS) {
    	CyU3PDebugPrint (4,"[CyFxRecvBuffer] CyU3PDmaChannelWaitForCompletion error=0x%x\r\n",status);
    	return status;
    }

	status = CyU3PDmaChannelGetStatus (inpCh, &state, &prodXferCount, &consXferCount);
    if (status == CY_U3P_SUCCESS) {
    	*length = prodXferCount;
    }else{
    	CyU3PDebugPrint (4,"[CyFxRecvBuffer] CyU3PDmaChannelGetStatus error=0x%x\r\n",status);
    	return status;
    }
    return status;
}
