#ifndef __SETUP_H__
#define __SETUP_H__

#include "cyu3dma.h"

/* GPIO Setup */
void
CyFxAutoSetupGpio (
		void);

/* I2C Initialization */
void
CyFxAutoI2cInit (
		void);

/* PIB Initialization */
void
CyFxAutoPibInit (
		void);

/* Set values for DMA channel */
void
CyFxSetDmaChannelCfg(
		CyU3PDmaChannelConfig_t *pDmaCfg,
		uint16_t size,
		uint16_t count,
		CyU3PDmaSocketId_t prodSckId,
		CyU3PDmaSocketId_t consSckId,
		uint32_t notification,
		CyU3PDmaCallback_t cb);

/* Creating DMA Channel */
void
CyFxCreateChannel(
		uint16_t size,
		uint16_t count,
		CyU3PDmaSocketId_t prodSckId,
		CyU3PDmaSocketId_t consSckId,
		uint32_t notification,
		CyU3PDmaCallback_t cb,
		CyU3PDmaChannel *handle,
		CyU3PDmaType_t type);

/* Create DMA Channels between CPU-PIB */
void
CyFxCreateCpuPibDmaChannels (
		const char *tag,
		uint16_t dataBurstLength);

/* Create Control Channel Thread which receives control data generated from ZING */
void
CyFxCreateControlChannel (
        void);

/* ZING Initialization */
void
CyFxZingInit (
		void);

/* Set PPC/DEV */
void
CyFxSetHRCP (
		void);

/* Create AutoUSB to Zing Thread */
void
CyFxCreateAutoUsbToZingThread (
        void);

/* Create Zing to AutoUSB Thread */
void
CyFxCreateZingToAutoUsbThread (
        void);

/* Create PhoneUSB to Zing Thread */
void
CyFxCreatePhoneUsbToZingThread (
        void);

/* Create Zing to PhoneUSB Thread */
void
CyFxCreateZingToPhoneUsbThread (
        void);

/* Connect USB */
void
CyFxUsbConnect (
		void);

/* Disconnect USB */
void
CyFxUsbDisconnect (
		void);

/* Enable or Disable Golay code */
void
CyFxGolay (
		void);

#endif
