#include "PhoneUsbToZing.h"
#include "cyu3os.h"
#include "cyu3system.h"
#include "cyu3error.h"
#include "Zing.h"
#include "phonedrv.h"
#include "cyu3usbhost.h"
#include "uhbuf.h"
#include "host.h"

extern CyU3PDmaChannel glChHandlePhoneDataIn;
extern CyU3PEvent  applnEvent;

CyU3PReturnStatus_t
CreatePhoneUsbToZingThread(
		void)
{
	CyU3PReturnStatus_t Status;

	CyU3PMemFree(phoneUsbToZing.StackPtr_);
	phoneUsbToZing.StackPtr_ = CyU3PMemAlloc(PHONEUSBTOZING_THREAD_STACK);

	CyU3PDmaBufferFree(phoneUsbToZing.pf_);
	if((phoneUsbToZing.pf_=(PacketFormat*)CyU3PDmaBufferAlloc(520))==0) return CY_U3P_ERROR_MEMORY_ERROR;

	Status = CyU3PThreadCreate(&phoneUsbToZing.Handle_,		// Handle to my Application Thread
				"201:PhoneUsbToZing",						// Thread ID and name
				PhoneUsbToZingThread,						// Thread entry function
				0,											// Parameter passed to Thread
				phoneUsbToZing.StackPtr_,					// Pointer to the allocated thread stack
				PHONEUSBTOZING_THREAD_STACK,				// Allocated thread stack size
				PHONEUSBTOZING_THREAD_PRIORITY,				// Thread priority
				PHONEUSBTOZING_THREAD_PRIORITY,				// = Thread priority so no preemption
				CYU3P_NO_TIME_SLICE,						// Time slice no supported
				CYU3P_AUTO_START							// Start the thread immediately
	);
	return Status;
}

static
CyBool_t
Receive(
		uint8_t ep,
		CyU3PDmaChannel *dmaCh,
		uint8_t *buffer,
		uint16_t count,
		uint32_t *length)
{
	CyU3PReturnStatus_t Status;

    if ((Status=CyFxRecvBuffer (ep,dmaCh,buffer,count,length)) != CY_U3P_SUCCESS) {
    	phoneUsbToZing.Count_.receiveErr++;
		CyU3PDebugPrint(4,"[P-Z] receiving from PhoneDataIn failed error(0x%x),EP=0x%x\r\n",Status,ep);
	    return CyFalse;
    }else{
    	phoneUsbToZing.Count_.receiveOk++;
        if(*length==0) {
            CyU3PDebugPrint(4,"[P-Z] Data size(%d) received from PhoneDataIn is zero, Skip further processing\r\n",*length);
        }else if(*length>512){
            CyU3PDebugPrint(4,"[P-Z] Data size(%d) received from PhoneDataIn is greater than 512\r\n",*length);
        }
#ifdef DEBUG_THREAD_LOOP
    	CyU3PDebugPrint(4,"[P-Z] %d bytes received from PhoneDataIn\r\n",rt_len);
#endif
    }
    return CyTrue;
}

static
CyBool_t
Send(
		PacketFormat *pf,
		uint32_t pfSize)
{
	CyU3PReturnStatus_t Status;

    phoneUsbToZing.pf_->size = pfSize;
	if((Status=Zing_DataWrite((uint8_t*)pf,pfSize+sizeof(uint32_t)))==CY_U3P_SUCCESS) {
		phoneUsbToZing.Count_.sendOk++;
#ifdef DEBUG_THREAD_LOOP
		CyU3PDebugPrint(4,"[P-Z] %d bytes sent to GpifDataOut\r\n",pfSize);
#endif
	}else{
		phoneUsbToZing.Count_.sendErr++;
		CyU3PDebugPrint (4, "[P-Z] Zing_DataWrite(%d) error(0x%x)\n",pfSize,Status);
		return CyFalse;
	}
	return CyTrue;
}

void
PhoneUsbToZingThread(
		uint32_t Value)
{
	uint32_t rt_len;
	CyU3PDmaBuffer_t Buf;

	CyU3PDebugPrint(4,"[P-Z] Phone USB to Zing thread starts\n");

	if(glChHandlePhoneDataIn.size==0) {
		CyU3PDebugPrint(4,"[P-Z] Waiting for PhoneDataIn.size=%d to be filled in...\n",glChHandlePhoneDataIn.size);
		while(glChHandlePhoneDataIn.size==0) CyU3PThreadSleep (10);
	}
	CyU3PDebugPrint(4,"[P-Z] PhoneDataIn.size=%d\n",glChHandlePhoneDataIn.size);

	Buf.buffer = phoneUsbToZing.pf_->data;
	Buf.count = 0;
	Buf.size = glChHandlePhoneDataIn.size;
	Buf.status = 0;
	memset(&phoneUsbToZing.Count_,0,sizeof(phoneUsbToZing.Count_));
	phoneUsbToZingTerminate = CyFalse;
	while(1){
		if(CyFalse==Receive(Phone.inEp,&glChHandlePhoneDataIn,Buf.buffer,Buf.size,&rt_len)) {
			if(phoneUsbToZingTerminate)
				break;
			else{
#ifndef INTENTIONALLY_CAUSE_RECEIVE_ERROR
			    CyU3PEventSet (&applnEvent, CY_FX_PHONEUSB_RECEIVE_ERR, CYU3P_EVENT_OR);
			    CyU3PDebugPrint(4,"Set PhoneUsb receive Error event\r\n");
#endif
				continue;
			}
		}

		if(CyFalse==Send(phoneUsbToZing.pf_,rt_len)) {
#ifndef INTENTIONALLY_CAUSE_RECEIVE_ERROR
			CyU3PEventSet (&applnEvent, CY_FX_PHONEUSB_RECEIVE_ERR, CYU3P_EVENT_OR);
			CyU3PDebugPrint(4,"Set PhoneUsb receive Error event\r\n");
#endif
			if(phoneUsbToZingTerminate) break;
		}
	}
	CyU3PDebugPrint (4, "[P-Z] PhoneUsbToZingThread ends\n");
}
