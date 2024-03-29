#include "ZingToAutoUsb.h"
#include "cyu3os.h"
#include "cyu3system.h"
#include "cyu3error.h"
#include "dma.h"
#include "Zing.h"
#include "setup.h"

extern CyU3PDmaChannel glChHandleAutoDataOut;
extern CyBool_t glIsApplnActive;
CyBool_t AutoUsbToZing = CyFalse;

#ifndef PERSISTENT_USB
CyBool_t usbCon = CyFalse;
#endif

CyU3PReturnStatus_t
CreateZingToAutoUsbThread(
		void)
{
	CyU3PReturnStatus_t Status;

	CyU3PMemFree(zingToAutoUsb.StackPtr_);
	zingToAutoUsb.StackPtr_ = CyU3PMemAlloc(ZINGTOAUTOUSB_THREAD_STACK);

	CyU3PDmaBufferFree(zingToAutoUsb.pf_);
	if((zingToAutoUsb.pf_=(PacketFormat*)CyU3PDmaBufferAlloc(520))==0) return CY_U3P_ERROR_MEMORY_ERROR;

	Status = CyU3PThreadCreate(&zingToAutoUsb.Handle_,		// Handle to my Application Thread
				"102:ZingToAutoUsb",						// Thread ID and name
				ZingToAutoUsbThread,						// Thread entry function
				0,											// Parameter passed to Thread
				zingToAutoUsb.StackPtr_,					// Pointer to the allocated thread stack
				ZINGTOAUTOUSB_THREAD_STACK,					// Allocated thread stack size
				ZINGTOAUTOUSB_THREAD_PRIORITY,				// Thread priority
				ZINGTOAUTOUSB_THREAD_PRIORITY,				// = Thread priority so no preemption
				CYU3P_NO_TIME_SLICE,						// Time slice no supported
				CYU3P_AUTO_START							// Start the thread immediately
	);
	return Status;
}

#ifndef PERSISTENT_USB
CyBool_t
IsPingOn(
		uint8_t *buf)
{
	if (buf[0]==0x50 && buf[1]==0x49 && buf[2]==0x4E && buf[3]==0x47 && buf[4]==0x20 && buf[5]==0x4F && buf[6]==0x4E) return CyTrue;
	return CyFalse;
}

CyBool_t
IsPingOff(
		uint8_t *buf)
{
	if (buf[0]==0x50 && buf[1]==0x49 && buf[2]==0x4E && buf[3]==0x47 && buf[4]==0x20 && buf[5]==0x4F && buf[6]==0x46 && buf[7]==0x46) return CyTrue;
	return CyFalse;
}

void
onPingOn()
{
	if(!usbCon) {
		CyU3PDebugPrint(4,"PING ON received. Connecting USB...\r\n");
		CyFxUsbConnect();
		usbCon = CyTrue;
		CyU3PDebugPrint(4,"USB Connected\r\n");

		if(!AutoUsbToZing) {
			CyFxCreateAutoUsbToZingThread ();
			CyU3PDebugPrint(4,"[Auto] AutoUsb To Zing Thread Created\n");
			AutoUsbToZing = CyTrue;
		}
	}else{
#ifdef DEBUG
		CyU3PDebugPrint(4,"PING ON received. Already connected\r\n");
#endif
	}
}

void
onPingOff()
{
	CyU3PDebugPrint(4,"PING OFF received. Disconnecting USB...\r\n");
	CyFxUsbDisconnect();
	usbCon = CyFalse;
	CyU3PDebugPrint(4,"USB Disconnected\r\n");
}
#endif

static
CyBool_t
Receive(
		CyU3PDmaChannel *dmaCh,
		PacketFormat *pf,
		uint32_t *length
		)
{
	CyU3PReturnStatus_t Status;
	if((Status=Zing_Transfer_Recv(dmaCh,(uint8_t*)pf,length,CYU3P_WAIT_FOREVER))==CY_U3P_SUCCESS) {
		zingToAutoUsb.Count_.receiveOk++;
		if(zingToAutoUsb.pf_->size==0) {
			CyU3PDebugPrint(4,"[Z-A] Data size(%d) received from GpifDataIn is zero, Skip further processing\r\n",zingToAutoUsb.pf_->size);
		}else if(zingToAutoUsb.pf_->size>512){
			CyU3PDebugPrint(4,"[Z-A] Data size(%d) received from GpifDataIn is greater than 512\r\n",zingToAutoUsb.pf_->size);
		}
#ifdef DEBUG_THREAD_LOOP
		CyU3PDebugPrint(4,"[Z-A] %d->%d bytes received from GpifDataIn\r\n",rt_len,pf->size);
#endif
	}else{
		zingToAutoUsb.Count_.receiveErr++;
		CyU3PDebugPrint (4, "[Z-A] Zing_Transfer_Recv error(0x%x)\n",Status);
		return CyFalse;
	}
	return CyTrue;
}

static
void
Send(
		CyU3PDmaChannel *dmaCh,
		uint8_t *data,
		uint32_t length)
{
	CyU3PReturnStatus_t Status;

	if((Status=Zing_Transfer_Send(dmaCh,data,length))==CY_U3P_SUCCESS) {
		zingToAutoUsb.Count_.sendOk++;
#ifdef DEBUG_THREAD_LOOP
		CyU3PDebugPrint(4,"[A-Z] %d bytes sent to AutoDataOut\r\n",length);
#endif
	}else{
		zingToAutoUsb.Count_.sendErr++;
		CyU3PDebugPrint (4, "[Z-A] Zing_DataWrite(%d) error(0x%x)\n",length,Status);
		CyU3PThreadSleep (10);
		CyU3PDeviceReset (CyFalse);
	}
}

void
ZingToAutoUsbThread(
		uint32_t Value)
{
	uint32_t rt_len;

	CyU3PDebugPrint(4,"[Z-A] GpifDataIn.size=%d\n",Dma.DataIn_.Channel_.size);
	memset(&zingToAutoUsb.Count_,0,sizeof(zingToAutoUsb.Count_));
	while(1){
		if(CyFalse==Receive(&Dma.DataIn_.Channel_,zingToAutoUsb.pf_,&rt_len)) continue;
		if(zingToAutoUsb.pf_->size==0) continue;

#ifndef PERSISTENT_USB
		if (IsPingOn(zingToAutoUsb.pf_->data)) {
			onPingOn();
			continue;
		} else if (IsPingOff(zingToAutoUsb.pf_->data)) {
			onPingOff();
			continue;
		}

		/* If USB is connected and Application started properly, glIsApplnActive should become True.
		 * Otherwise, Zing_Transfer_Send should be delayed until it becomes True.
		 * */
		if(!glIsApplnActive) continue;
#endif
    	Send(&glChHandleAutoDataOut,zingToAutoUsb.pf_->data,zingToAutoUsb.pf_->size);
	}
}
