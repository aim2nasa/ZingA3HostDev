#ifndef __AUTOUSBTOZING_H__
#define __AUTOUSBTOZING_H__

#include "cyu3types.h"
#include "thread.h"

#define AUTOUSBTOZING_THREAD_STACK		(0x800)
#define AUTOUSBTOZING_THREAD_PRIORITY	(8)

CyU3PReturnStatus_t
CreateAutoUsbToZingThread(
		void);

typedef struct CyU3PDmaChannel CyU3PDmaChannel;

void
Receive(
		CyU3PDmaChannel *dmaCh,
		uint8_t *data,
		uint32_t *length);

void
Send(
		PacketFormat *pf,
		uint32_t pfSize);

void
AutoUsbToZingThread(
		uint32_t Value);

Thread_t autoUsbToZing;

#endif
