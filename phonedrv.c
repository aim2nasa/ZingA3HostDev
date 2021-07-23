#include "phonedrv.h"
#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3usbhost.h"
#include "cyu3usbotg.h"
#include "cyu3utils.h"
#include "host.h"
#include "Dma.h"
#include "ZingToPhoneUsb.h"
#include "PhoneUsbToZing.h"
#include "ControlCh.h"
#include "setup.h"
#include "HOST-PPC\usbHostEpWrapper.h"

CyU3PDmaChannel glChHandlePhoneDataIn;      /* IN EP channel for ingress data. */
CyU3PDmaChannel glChHandlePhoneDataOut;     /* OUT EP channel for egress data. */

CyU3PReturnStatus_t
PhoneDriverInit ()
{
    uint16_t length, size, offset;
    CyU3PReturnStatus_t status;
    CyU3PUsbHostEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    uint8_t epSizeSet,inEpSet,outEpSet;
    memset(&Phone,sizeof(Phone),0);

    /* Read first four bytes of configuration descriptor to determine
     * the total length. */
    status = CyFxSendSetupRqt (0x80, CY_U3P_USB_SC_GET_DESCRIPTOR,
            (CY_U3P_USB_CONFIG_DESCR << 8), 0, 4, glEp0Buffer);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "[PhoneDriverInit] CY_U3P_USB_SC_GET_DESCRIPTOR config error\n");
        goto enum_error;
    }

    /* Identify the length of the data received. */
    length = CY_U3P_MAKEWORD(glEp0Buffer[3], glEp0Buffer[2]);
    if (length > 512)
    {
        CyU3PDebugPrint (4, "[PhoneDriverInit] length error\n");
        goto enum_error;
    }
    CyU3PDebugPrint (4, "[PhoneDriverInit] length=%d\n",length);

    /* Read the full configuration descriptor. */
    status = CyFxSendSetupRqt (0x80, CY_U3P_USB_SC_GET_DESCRIPTOR,
            (CY_U3P_USB_CONFIG_DESCR << 8), 0, length, glEp0Buffer);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "[PhoneDriverInit] CY_U3P_USB_SC_GET_DESCRIPTOR config error\n");
        goto enum_error;
    }

    CyU3PDebugPrint (4, "Identify the EP characteristics\n");
    for(int i=0;i<length;i++) CyU3PDebugPrint (4,"%x ", glEp0Buffer[i]);
    CyU3PDebugPrint(4, "\n");

    /* Identify the EP characteristics. */
    offset = epSizeSet = inEpSet = outEpSet =0;
    while (offset < length)
    {
        if (glEp0Buffer[offset + 1] == CY_U3P_USB_ENDPNT_DESCR)
        {
            if (glEp0Buffer[offset + 3] != CY_U3P_USB_EP_BULK)
            {
                CyU3PDebugPrint (4, "[PhoneDriverInit] glEp0Buffer[%d](%d)!= CY_U3P_USB_EP_BULK(%d)\n",
                		offset+3,glEp0Buffer[offset+3],CY_U3P_USB_EP_BULK);
            }else{
                /* Retreive the information. */
                if(!epSizeSet) { Phone.epSize = CY_U3P_MAKEWORD(glEp0Buffer[offset + 5],glEp0Buffer[offset + 4]); epSizeSet=1; }
                CyU3PDebugPrint (4, "[PhoneDriverInit] EpSize=%d\n", Phone.epSize);

                if (glEp0Buffer[offset + 2] & 0x80)
                {
                	if(!inEpSet) { Phone.inEp = glEp0Buffer[offset + 2]; inEpSet=1; }
                	CyU3PDebugPrint (4, "[PhoneDriverInit] inEp=%d(0x%x)\n", Phone.inEp,Phone.inEp);
                }
                else
                {
                	if(!outEpSet) { Phone.outEp = glEp0Buffer[offset + 2]; outEpSet=1; }
                	CyU3PDebugPrint (4, "[PhoneDriverInit] outEp=%d(0x%x)\n", Phone.outEp,Phone.outEp);
                }
            }
        }

        /* Advance to next descriptor. */
        offset += glEp0Buffer[offset];
    }

    /* Set the new configuration. */
    status = CyFxSendSetupRqt (0x00, CY_U3P_USB_SC_SET_CONFIGURATION,
            1, 0, 0, glEp0Buffer);
    if (status != CY_U3P_SUCCESS)
    {
    	CyU3PDebugPrint (4, "[PhoneDriverInit] CyFxSendSetupRqt, CY_U3P_USB_SC_SET_CONFIGURATION error(0x%x)\n",status);
        goto enum_error;
    }

    CyU3PDebugPrint (4, "[PhoneDriverInit] outEp=%d(0x%x), inEp=%d(0x%x), epSize=%d\n",Phone.outEp,Phone.outEp,Phone.inEp,Phone.inEp,Phone.epSize);

    /* Add the IN endpoint. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.type = CY_U3P_USB_EP_BULK;
    epCfg.mult = 1;
    epCfg.maxPktSize = Phone.epSize;
    epCfg.pollingRate = 0;
    /* Since DMA buffer sizes can only be multiple of 16 bytes and
     * also since this is an interrupt endpoint where the max data
     * packet size is same as the maxPktSize field, the fullPktSize
     * has to be a multiple of 16 bytes. */
    size = ((Phone.epSize + 0x0F) & ~0x0F);
    epCfg.fullPktSize = size;
    epCfg.isStreamMode = CyFalse;
    status = CyU3PUsbHostEpAdd (Phone.inEp, &epCfg);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "[PhoneDriverInit] CyU3PUsbHostEpAdd(glInEp) error\n");
        goto enum_error;
    }

    /* Add the OUT EP. */
    status = CyU3PUsbHostEpAdd (Phone.outEp, &epCfg);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "[PhoneDriverInit] CyU3PUsbHostEpAdd(glOutEp) error\n");
        goto enum_error;
    }

    /* Create a DMA channel for IN EP. */
    CyU3PMemSet ((uint8_t *)&dmaCfg, 0, sizeof(dmaCfg));
    dmaCfg.size = Phone.epSize;
    dmaCfg.count = 1;
    dmaCfg.prodSckId = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_PROD_0 + (0x0F & Phone.inEp));
    dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;
    status = CyU3PDmaChannelCreate (&glChHandlePhoneDataIn, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "[DriverInit] CyU3PDmaChannelCreate(glChHandlePhoneDataIn) error(%d)\r\n", status);
        goto app_error;
    }

    /* Create a DMA channel for OUT EP. */
    dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaCfg.consSckId = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0 + (0x0F & Phone.outEp));
    status = CyU3PDmaChannelCreate (&glChHandlePhoneDataOut, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "[DriverInit] CyU3PDmaChannelCreate(glChHandlePhoneDataOut) error(%d)\r\n", status);
        goto app_error;
    }

	if( (status=CreateZingToPhoneUsbThread ())!=CY_U3P_SUCCESS) {
		CyU3PDebugPrint(4,"[DriverInit] Zing To PhoneUsb Thread Creation failed, error(0x%x)\n",status);
        goto app_error;
	}

	if( (status=CreatePhoneUsbToZingThread ())!=CY_U3P_SUCCESS) {
		CyU3PDebugPrint(4,"[DriverInit] PhoneUsb To Zing Thread Creation failed, error(0x%x)\n",status);
        goto app_error;
	}

	if(ControlChTerminate) {
		CyU3PDebugPrint(4,"CyFxCreateCpuPibDmaChannels...\n");
		CyFxCreateCpuPibDmaChannels("[Phone]",CY_FX_DATA_BURST_LENGTH);
		CyU3PDebugPrint(4,"[Phone] DMA Channels for CPU-PIB Created\n");

		CyU3PDebugPrint(4,"CyFxCreateControlChannel...\n");
		CyFxCreateControlChannel();
		CyU3PDebugPrint(4,"[Phone] Control Channel Thread Created\n");
	}

    CyU3PDebugPrint (4, "PhoneDriverInit OK\n");
    return CY_U3P_SUCCESS;

app_error:
	CyU3PDmaChannelDestroy (&glChHandlePhoneDataIn);
	if (Phone.inEp != 0)
	{
		UsbHostEpRemove (Phone.inEp);
		Phone.inEp = 0;
	}
	CyU3PDmaChannelDestroy (&glChHandlePhoneDataOut);
	if (Phone.outEp != 0)
	{
		UsbHostEpRemove (Phone.outEp);
		Phone.outEp = 0;
	}

enum_error:
	CyU3PDebugPrint (4, "PhoneDriverInit failed\n");
    return CY_U3P_ERROR_FAILURE;
}

void
PhoneDriverDeInit ()
{
	CyU3PReturnStatus_t status;

	ControlChTerminate = phoneUsbToZingTerminate = zingToPhoneUsbTerminate = CyTrue;

    status = UsbHostPortDisable ();
    if(status==CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "host port disabled\r\n");
    }else{
    	CyU3PDebugPrint (4, "host port disable error(0x%x)\r\n",status);
    }

	CyU3PDmaChannelDestroy (&Dma.ControlIn_.Channel_);
	CyU3PDmaChannelDestroy (&Dma.ControlOut_.Channel_);
	CyU3PDmaChannelDestroy (&Dma.DataIn_.Channel_);
	CyU3PDmaChannelDestroy (&Dma.DataOut_.Channel_);

    CyU3PDmaChannelDestroy (&glChHandlePhoneDataIn);
    if (Phone.inEp != 0)
    {
        if((status=UsbHostEpRemove (Phone.inEp))==CY_U3P_SUCCESS) {
        	CyU3PDebugPrint(4,"[DriverDeInit] CyU3PUsbHostEpRemove(0x%x) ok\n",Phone.inEp);
            Phone.inEp = 0;
        }else{
        	CyU3PDebugPrint(4,"[DriverDeInit] CyU3PUsbHostEpRemove(0x%x) error(0x%x)\n",Phone.inEp,status);
        }
    }
    CyU3PDmaChannelDestroy (&glChHandlePhoneDataOut);
    if (Phone.outEp != 0)
    {
        if((status=UsbHostEpRemove (Phone.outEp))==CY_U3P_SUCCESS) {
        	CyU3PDebugPrint(4,"[DriverDeInit] CyU3PUsbHostEpRemove(0x%x) ok\n",Phone.outEp);
            Phone.outEp = 0;
        }else{
        	CyU3PDebugPrint(4,"[DriverDeInit] CyU3PUsbHostEpRemove(0x%x) error(0x%x)\n",Phone.outEp,status);
        }
    }

    CyU3PThreadSleep (10);

	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(zingToPhoneUsb.Handle_)...\n");
    status = tx_thread_terminate (&zingToPhoneUsb.Handle_);
    if(status==CY_U3P_SUCCESS) {
    	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(zingToPhoneUsb.Handle_) ok\n");
    	status = tx_thread_delete (&zingToPhoneUsb.Handle_);
        if(status==CY_U3P_SUCCESS) {
        	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_delete(zingToPhoneUsb.Handle_) ok\n");
        }else{
        	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_delete(zingToPhoneUsb.Handle_) error(0x%x)\n",status);
        }
    }else{
    	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(zingToPhoneUsb.Handle_) error(0x%x)\n",status);
    }

    CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(phoneUsbToZing.Handle_)...\n");
    status = tx_thread_terminate(&phoneUsbToZing.Handle_);
    if(status==CY_U3P_SUCCESS) {
    	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(phoneUsbToZing.Handle_) ok\n");
    	status = tx_thread_delete (&phoneUsbToZing.Handle_);
        if(status==CY_U3P_SUCCESS) {
        	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_delete(phoneUsbToZing.Handle_) ok\n");
        }else{
        	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_delete(phoneUsbToZing.Handle_) error(0x%x)\n",status);
        }
    }else{
    	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(phoneUsbToZing.Handle_) error(0x%x)\n",status);
    }

    CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(ControlCh.Handle_)...\n");
    status = tx_thread_terminate (&ControlCh.Handle_);
    if(status==CY_U3P_SUCCESS) {
    	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(ControlCh.Handle_) ok\n");
    	status = tx_thread_delete (&ControlCh.Handle_);
        if(status==CY_U3P_SUCCESS) {
        	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_delete(ControlCh.Handle_) ok\n");
        }else{
        	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_delete(ControlCh.Handle_) error(0x%x)\n",status);
        }
    }else{
    	CyU3PDebugPrint(4,"[DriverDeInit] tx_thread_terminate(ControlCh.Handle_) error(0x%x)\n",status);
    }

	CyU3PDebugPrint (4, "[DriverDeInit] PhoneDriverDeInit done\n");
}
