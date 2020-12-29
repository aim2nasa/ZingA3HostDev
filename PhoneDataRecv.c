#include "PhoneDataRecv.h"
#include "cyu3os.h"
#include "cyu3system.h"
#include "cyu3error.h"
#include "Zing.h"
#include "phonedrv.h"
#include "cyu3usbhost.h"
#include "uhbuf.h"

extern CyU3PDmaChannel glChHandlePhoneDataIn;

CyU3PReturnStatus_t
CreatePhoneDataRecvThread(
		void)
{
	void *StackPtr = NULL;
	CyU3PReturnStatus_t Status;

	StackPtr = CyU3PMemAlloc(PHONEDATARECV_THREAD_STACK);
	Status = CyU3PThreadCreate(&PhoneDataRecvThreadHandle,	// Handle to my Application Thread
				"301:PhoneDataRecv",						// Thread ID and name
				PhoneDataRecvThread,						// Thread entry function
				0,											// Parameter passed to Thread
				StackPtr,									// Pointer to the allocated thread stack
				PHONEDATARECV_THREAD_STACK,					// Allocated thread stack size
				PHONEDATARECV_THREAD_PRIORITY,				// Thread priority
				PHONEDATARECV_THREAD_PRIORITY,				// = Thread priority so no preemption
				CYU3P_NO_TIME_SLICE,						// Time slice no supported
				CYU3P_AUTO_START							// Start the thread immediately
	);
	return Status;
}

void
PhoneDataRecvThread(
		uint32_t Value)
{
	CyU3PReturnStatus_t Status;
	uint32_t rt_len;
	uint8_t *buffer;

	CyU3PDebugPrint(4,"[PhoneDataRecv] Phone Data Receiving thread starts\n");

	if(glChHandlePhoneDataIn.size==0) {
		CyU3PDebugPrint(4,"[PhoneDataRecv] Waiting for PhoneDataIn.size=%d to be filled in...\n",glChHandlePhoneDataIn.size);
		while(glChHandlePhoneDataIn.size==0) CyU3PThreadSleep (10);
	}
	CyU3PDebugPrint(4,"[PhoneDataRecv] PhoneDataIn.size=%d\n",glChHandlePhoneDataIn.size);

	buffer = CyU3PDmaBufferAlloc(glChHandlePhoneDataIn.size);
	memset(&phoneRecvCounter,0,sizeof(phoneRecvCounter));
	while(1){
	    if ((Status=CyFxRecvBuffer (Phone.inEp,&glChHandlePhoneDataIn,buffer,glChHandlePhoneDataIn.size,&rt_len)) != CY_U3P_SUCCESS) {
	    	phoneRecvCounter.Err++;
			CyU3PDebugPrint(4,"[PhoneDataRecv] receiving from PhoneDataIn failed error(0x%x),EP=0x%x\r\n",Status,Phone.inEp);
	    }else{
	    	phoneRecvCounter.Ok++;

			/* TO DO
			 * enqueue data received from phone */

#ifdef DEBUG_THREAD_LOOP
	    	CyU3PDebugPrint(4,"[PhoneDataRecv] %d bytes received from PhoneDataIn\r\n",rt_len);
#endif
	    }
	}
}
