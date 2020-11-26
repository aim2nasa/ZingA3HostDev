#include "ZingToPhoneUsb.h"
#include "cyu3os.h"
#include "cyu3system.h"
#include "cyu3error.h"
#include "dma.h"
#include "Zing.h"
#include "phonedrv.h"
#include "cyu3usbhost.h"
#include "uhbuf.h"

CyU3PThread ZingToPhoneUsbThreadHandle;
extern CyU3PDmaChannel glChHandlePhoneDataOut;

CyU3PReturnStatus_t
CreateZingToPhoneUsbThread(
		void)
{
	void *StackPtr = NULL;
	CyU3PReturnStatus_t Status;

	StackPtr = CyU3PMemAlloc(ZINGTOPHONEUSB_THREAD_STACK);
	Status = CyU3PThreadCreate(&ZingToPhoneUsbThreadHandle,	// Handle to my Application Thread
				"202:ZingToPhoneUsb",						// Thread ID and name
				ZingToPhoneUsbThread,						// Thread entry function
				0,											// Parameter passed to Thread
				StackPtr,									// Pointer to the allocated thread stack
				ZINGTOPHONEUSB_THREAD_STACK,				// Allocated thread stack size
				ZINGTOPHONEUSB_THREAD_PRIORITY,				// Thread priority
				ZINGTOPHONEUSB_THREAD_PRIORITY,				// = Thread priority so no preemption
				CYU3P_NO_TIME_SLICE,						// Time slice no supported
				CYU3P_AUTO_START							// Start the thread immediately
	);
	return Status;
}

void
ZingToPhoneUsbThread(
		uint32_t Value)
{
	CyU3PReturnStatus_t Status;
	uint32_t rt_len;
	uint8_t *buf = (uint8_t *)CyU3PDmaBufferAlloc (Dma.DataIn_.Channel_.size);

	CyU3PDebugPrint(4,"[Z-P] Zing to Phone USB thread starts\n");
	CyU3PDebugPrint(4,"[Z-P] GpifDataIn.size=%d\n",Dma.DataIn_.Channel_.size);

	while(1){
		if((Status=Zing_Transfer_Recv(&Dma.DataIn_.Channel_,buf,&rt_len,CYU3P_WAIT_FOREVER))==CY_U3P_SUCCESS) {
			CyU3PDebugPrint(4,"[Z-P] %d bytes received from GpifDataIn\r\n",rt_len);

		    if ((Status=CyFxSendBuffer (Phone.outEp,&glChHandlePhoneDataOut,buf,rt_len)) != CY_U3P_SUCCESS)
				CyU3PDebugPrint(4,"[Z-P] sending %d bytes to PhoneDataOut failed error(0x%x),EP=0x%x\r\n",rt_len,Status,Phone.outEp);
		    else {

			    for(uint32_t i=0;i<rt_len;i++) CyU3PDebugPrint (6,"%x ", buf[i]);
			    CyU3PDebugPrint(6, "\n");

		    	CyU3PDebugPrint(4,"[Z-P] %d bytes sent to PhoneDataOut,EP=0x%x\r\n",rt_len,Phone.outEp);
		    }
		}else{
			CyU3PDebugPrint (4, "[Z-P] Zing_Transfer_Recv error(0x%x)\n",Status);
		}
	}
}
