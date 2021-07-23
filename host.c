/*
 ## Cypress USB 3.0 Platform source file (cyfxusbhost.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2018,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file illustrates the usb host mode application example. */

/*
   This example illustrates the use of the FX3 firmware APIs to implement
   USB host mode operation. The example supports simple HID mouse class
   and simple MSC class devices.

   USB HID MOUSE DRIVER:

   A simple single interface USB HID mouse will be successfully enumerated
   and the current offset will be printed via the UART debug logs.

   We support only single interface with interface class = HID(0x03),
   interface sub class = Boot (0x01) and interface protocol = Mouse (0x02).
   This example supports only 4 byte input reports with the following format:
        BYTE0: Bitmask for each of the button present.
        BYTE1: Signed movement in X direction.
        BYTE2: Signed movement in Y direction.
        BYTE3: Signed movement in scroll wheel.
   Further types can be implemented by decoding the HID descriptor.

   Care should be taken so that an USB host is not connected to FX3 while
   host stack is active. This will mean that both sides will be driving
   the VBUS causing a hardware damage.

   Please refer to the FX3 DVK user manual for enabling VBUS control.
   The current example controls the VBUS supply VBUS_GPIO. The example assumes
   that a low on this line will turn on VBUS and allow remotely connected B-device
   to enumerate. A high / tri-state  on this line will turn off VBUS supply from
   the board. Also this requires that FX3 DVK is powered using external power supply.
   VBATT should also be enabled and connected. Depending upon the VBUS control,
   update the CY_FX_OTG_VBUS_ENABLE_VALUE and CY_FX_OTG_VBUS_DISABLE_VALUE definitions
   in cyfxusbhost.h file.

   USB MSC DRIVER:

   A simple single interface USB BOT MSC device will be successfully 
   enumerated and storage parameters will be queried. The example shall read
   the last sector of data, save it in local buffers and write a fixed pattern
   to location. It will  then read and verify the data written. Once this is
   done, the original data will be written back to the storage.
   
   NOTE: If MSC device is unplugged before all transactions have completed
   or if the example encounters an error, the data on the drive might be
   corrupted.
*/

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3usbhost.h"
#include "cyu3usbotg.h"
#include "cyu3uart.h"
#include "cyu3gpio.h"
#include "cyu3utils.h"
#include "gpio_regs.h"
#include "host.h"
#include "phonedrv.h"
#include "setup.h"
#include "gpio.h"
#include "Zing.h"
#include "ZingToPhoneUsb.h"
#include "PhoneUsbToZing.h"
#include "dma.h"
#include "ZingHw.h"
#include "ControlCh.h"
#include "helper.h"
#include "Host-PPC\usbHostEpWrapper.h"

CyU3PThread applnThread;                        /* Application thread structure */
CyU3PEvent  applnEvent;                         /* Event group used to signal the thread. */
CyBool_t glIsApplnActive = CyFalse;             /* Whether the application is active or not. */
volatile CyBool_t glIsPeripheralPresent = CyFalse;       /* Whether a remote peripheral is present or not. */

/* Setup packet buffer for host mode. */
uint8_t glSetupPkt[CY_FX_HOST_EP0_SETUP_SIZE] __attribute__ ((aligned (32)));

/* Buffer to send / receive data for EP0. */
uint8_t glEp0Buffer[CY_FX_HOST_EP0_BUFFER_SIZE] __attribute__ ((aligned (32)));

/* Buffer to hold the USB device descriptor. */
uint8_t glDeviceDesc[32] __attribute__ ((aligned (32)));

uint8_t glHostOwner = CY_FX_HOST_OWNER_NONE;    /* Current owner for the host controller. */

/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    CyU3PDebugPrint (4, "In CyFxAppErrorHandler\r\n");
    CyU3PThreadSleep (100);

    /* Loop Indefinitely */
    for (;;)
    {
    }
}

/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void
CyFxApplnDebugInit (void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set UART configuration */
    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set the UART transfer to a really large value. */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Initialize the debug module. */
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    CyU3PDebugPreamble (CyFalse);

    CyU3PDebugPrint (4, "UsbOtg Example\r\n");
    if (CyTrue == CY_FX_HOST_VBUS_ENABLE_VALUE)
    {
        CyU3PDebugPrint (4, "Assuming that driving CTL4 high turns VBus output ON\n");
    }
    else
    {
        CyU3PDebugPrint (4, "Assuming that driving CTL4 low turns VBus output OFF\n");
    }
}

/* USB host stack event callback function. */
void
CyFxHostEventCb (CyU3PUsbHostEventType_t evType, uint32_t evData)
{
    /* This is connect / disconnect event. Log it so that the
     * application thread can handle it. */
    if (evType == CY_U3P_USB_HOST_EVENT_CONNECT)
    {
        CyU3PDebugPrint (4, "Host connect event received\r\n");
        glIsPeripheralPresent = CyTrue;
    }
    else
    {
        CyU3PDebugPrint (4, "Host disconnect event received\r\n");
        glIsPeripheralPresent = CyFalse;
    }

    /* Signal the thread that a peripheral change has been detected. */
    CyU3PEventSet (&applnEvent, CY_FX_USB_CHANGE_EVENT, CYU3P_EVENT_OR);
}

/* Helper function to send EP0 setup packet. */
CyU3PReturnStatus_t
CyFxSendSetupRqt (
        uint8_t type,
        uint8_t request,
        uint16_t value,
        uint16_t index,
        uint16_t length,
        uint8_t *buffer_p)
{
    CyU3PUsbHostEpStatus_t epStatus;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    glSetupPkt[0] = type;
    glSetupPkt[1] = request;
    glSetupPkt[2] = CY_U3P_GET_LSB(value);
    glSetupPkt[3] = CY_U3P_GET_MSB(value);
    glSetupPkt[4] = CY_U3P_GET_LSB(index);
    glSetupPkt[5] = CY_U3P_GET_MSB(index);
    glSetupPkt[6] = CY_U3P_GET_LSB(length);
    glSetupPkt[7] = CY_U3P_GET_MSB(length);

    status = CyU3PUsbHostSendSetupRqt (glSetupPkt, buffer_p);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }
    status = CyU3PUsbHostEpWaitForCompletion (0, &epStatus,
            CY_FX_HOST_EP0_WAIT_TIMEOUT);

    return status;
}

void
DebugPrintBuffer(
		const char *msg)
{
    CyU3PDebugPrint (4, "%s\r\n",msg);
    for(int i=0;i<CY_FX_HOST_EP0_BUFFER_SIZE;i++) CyU3PDebugPrint (4,"%x ", glEp0Buffer[i]);
    CyU3PDebugPrint (4, "\r\n");
}

/* Ref
 * https://source.android.com/devices/accessories/aoa#attempt-to-start-in-accessory-mode */
CyU3PReturnStatus_t
AttemptToStartInAccessoryMode()
{
	CyU3PReturnStatus_t status;
	uint16_t version;
	CyU3PDebugPrint (4, "Attempt to start in accessory mode...\r\n");

	/* Step.1
	 * Send a 51 control request ("Get Protocol") to determine if the device supports the Android accessory protocol.
	 * requestType: USB_DIR_IN | USB_TYPE_VENDOR = 0xC0
	 * request: 51 (0x33)
	 * value: 0
	 * index: 0
	 * length: 2
	 * data: protocol version number (16 bits little endian sent from the device to the accessory) */
	if ((status = CyFxSendSetupRqt(0xC0, 0x33, 0, 0, 0x2, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.1 CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}
	version = (glEp0Buffer[1]<<8)|glEp0Buffer[0];
	CyU3PDebugPrint (4, "Supported protocol version:%d(0x%x)\r\n",version,version);

	if(version==0) {
		CyU3PDebugPrint (4, "Device does not support Android accessory protocol\r\n");
		return CY_U3P_ERROR_NOT_SUPPORTED;
	}

	/* Step.2 manufacturer name
	 * If the device returns a supported protocol version, send a control request with identifying string information to the device.
	 * requestType: USB_DIR_OUT | USB_TYPE_VENDOR = 0x40
	 * request: 52 (0x34)
	 * value: 0
	 * index: 0 (manufacturer name:0)
	 * length: 8
	 * data: 41 6E 64 72 6F 69 64 00 ("Android" zero terminated UTF8 string sent from accessory to device) */
	glEp0Buffer[0] = 0x41; glEp0Buffer[1] = 0x6E; glEp0Buffer[2] = 0x64; glEp0Buffer[3] = 0x72;
	glEp0Buffer[4] = 0x6F; glEp0Buffer[5] = 0x69; glEp0Buffer[6] = 0x64; glEp0Buffer[7] = 0x00;
	if ((status = CyFxSendSetupRqt(0x40, 0x34, 0, 0, 0x8, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.2 manufacturer name CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}

	/* Step.2 model name
	 * If the device returns a supported protocol version, send a control request with identifying string information to the device.
	 * requestType: USB_DIR_OUT | USB_TYPE_VENDOR = 0x40
	 * request: 52 (0x34)
	 * value: 0
	 * index: 1 (model name:1)
	 * length: 13
	 * data: 41 6E 64 72 6F 69 64 20 41 75 74 6F 00 ("Android Auto" zero terminated UTF8 string sent from accessory to device) */
	glEp0Buffer[0] = 0x41; glEp0Buffer[1] = 0x6E; glEp0Buffer[2] = 0x64; glEp0Buffer[3] = 0x72;
	glEp0Buffer[4] = 0x6F; glEp0Buffer[5] = 0x69; glEp0Buffer[6] = 0x64; glEp0Buffer[7] = 0x20;
	glEp0Buffer[8] = 0x41; glEp0Buffer[9] = 0x75; glEp0Buffer[10] = 0x74; glEp0Buffer[11] = 0x6F; glEp0Buffer[12] = 0x00;
	if ((status = CyFxSendSetupRqt(0x40, 0x34, 0, 1, 0xD, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.2 model name CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}

	/* Step.2 description
	 * If the device returns a supported protocol version, send a control request with identifying string information to the device.
	 * requestType: USB_DIR_OUT | USB_TYPE_VENDOR = 0x40
	 * request: 52 (0x34)
	 * value: 0
	 * index: 2 (description:2)
	 * length: 13
	 * data: 41 6E 64 72 6F 69 64 20 41 75 74 6F 00 ("Android Auto" zero terminated UTF8 string sent from accessory to device) */
	glEp0Buffer[0] = 0x41; glEp0Buffer[1] = 0x6E; glEp0Buffer[2] = 0x64; glEp0Buffer[3] = 0x72;
	glEp0Buffer[4] = 0x6F; glEp0Buffer[5] = 0x69; glEp0Buffer[6] = 0x64; glEp0Buffer[7] = 0x20;
	glEp0Buffer[8] = 0x41; glEp0Buffer[9] = 0x75; glEp0Buffer[10] = 0x74; glEp0Buffer[11] = 0x6F; glEp0Buffer[12] = 0x00;
	if ((status = CyFxSendSetupRqt(0x40, 0x34, 0, 2, 0xD, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.2 description CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}

	/* Step.2 version
	 * If the device returns a supported protocol version, send a control request with identifying string information to the device.
	 * requestType: USB_DIR_OUT | USB_TYPE_VENDOR = 0x40
	 * request: 52 (0x34)
	 * value: 0
	 * index: 3 (version:3)
	 * length: 4
	 * data: 31 2E 30 00 (zero terminated UTF8 string sent from accessory to device) */
	glEp0Buffer[0] = 0x31; glEp0Buffer[1] = 0x2E; glEp0Buffer[2] = 0x30; glEp0Buffer[3] = 0x00;
	if ((status = CyFxSendSetupRqt(0x40, 0x34, 0, 3, 0x4, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.2 version CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}

	/* Step.2 version
	 * If the device returns a supported protocol version, send a control request with identifying string information to the device.
	 * requestType: USB_DIR_OUT | USB_TYPE_VENDOR = 0x40
	 * request: 52 (0x34)
	 * value: 0
	 * index: 4 (URI:4)
	 * length: 1
	 * data: 00 (zero terminated UTF8 string sent from accessory to device) */
	glEp0Buffer[0] = 0x00;
	if ((status = CyFxSendSetupRqt(0x40, 0x34, 0, 4, 0x1, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.2 URI CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}

	/* Step.2 serial number
	 * If the device returns a supported protocol version, send a control request with identifying string information to the device.
	 * requestType: USB_DIR_OUT | USB_TYPE_VENDOR = 0x40
	 * request: 52 (0x34)
	 * value: 0
	 * index: 5 (serial number:5)
	 * length: 1
	 * data: 00 (zero terminated UTF8 string sent from accessory to device) */
	glEp0Buffer[0] = 0x00;
	if ((status = CyFxSendSetupRqt(0x40, 0x34, 0, 5, 0x1, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.2 serial number CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}

	/* Step.3
	 * Send a control request to ask the device to start in accessory mode.
	 * requestType: USB_DIR_OUT | USB_TYPE_VENDOR = 0x40
	 * request: 53 (0x35)
	 * value: 0
	 * index: 0
	 * length: 0
	 * data: none */
	glEp0Buffer[0] = 0x00;
	if ((status = CyFxSendSetupRqt(0x40, 0x35, 0, 0, 0, glEp0Buffer)) != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "AttemptToStartInAccessoryMode Step.3 CyFxSendSetupRqt Error(0x%x)\r\n",status);
		return status;
	}
	return CY_U3P_SUCCESS;
}

uint8_t
IsGoogleVendorID ()
{
	/* Google's ID (0x18D1) */
	if ((glEp0Buffer[8] == 0xd1) && (glEp0Buffer[9] == 0x18) ) return 1;
	return 0;
}

uint8_t IsAndroidPoweredDevice ()
{
	/* 0x2D00 is reserved for Android-powered devices that support accessory mode
	 * 0x2D01 is reserved for devices that support accessory mode as well as the
	 * Android Debug Bridge (ADB) protocol, which exposes a second interface with two bulk endpoints for ADB.*/
	if ((glEp0Buffer[10] == 0x00) && (glEp0Buffer[11] == 0x2d)) return 1;
	if ((glEp0Buffer[10] == 0x01) && (glEp0Buffer[11] == 0x2d)) return 1;
	return 0;
}

uint8_t
IsDeviceInAccessoryMode ()
{
	/* The vendor ID should match Google's ID (0x18D1).
	 * If the device is already in accessory mode,
	 * the product ID should be 0x2D00 or 0x2D01
	 * and the accessory can establish communication with the device
	 * through bulk transfer endpoints using its own communication protocol
	 *
	 * Ref. Android Open Accessory Protocol 1.0
	 * https://source.android.com/devices/accessories/aoa */
	if(IsGoogleVendorID () && IsAndroidPoweredDevice ()) return 1;
	return 0;
}

/* This function initializes the mouse driver application. */
void
CyFxApplnStart ()
{
    CyU3PReturnStatus_t status;
    CyU3PUsbHostEpConfig_t epCfg;
    CyU3PUsbHostPortStatus_t portStatus;
    CyU3PUsbHostOpSpeed_t    portSpeed;

    //CyU3PUsbHostPortStatus
    status = CyU3PUsbHostGetPortStatus(&portStatus, &portSpeed);
    CyU3PDebugPrint (4, "[CyFxApplnStart] call CyU3PUsbHostGetPortStatus, status=%d, portStatus=%d, portSpeed=%d\r\n", status, portStatus, portSpeed);

    /* Add EP0 to the scheduler. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.type = CY_U3P_USB_EP_CONTROL;
    epCfg.mult = 1;
    /* Start off with 8 byte EP0 packet size. */
    epCfg.maxPktSize = 64;//8;
    epCfg.pollingRate = 0;
    epCfg.fullPktSize = 64;//8;
    epCfg.isStreamMode = CyFalse;

    CyU3PDebugPrint (4, "[CyFxApplnStart] call CyU3PUsbHostEpAdd\r\n");
    status = CyU3PUsbHostEpAdd (0, &epCfg);
    if (status != CY_U3P_SUCCESS)
    {
    	CyU3PDebugPrint (4, "[CyFxApplnStart] CyU3PUsbHostEpAdd(0) error=0x%x\r\n",status);
        goto enum_error;
    }
    CyU3PDebugPrint (4, "[CyFxApplnStart] CyU3PUsbHostEpAdd returned\r\n");
	    CyU3PThreadSleep (100);

    CyU3PDebugPrint (4, "[CyFxApplnStart] call CyFxSendSetupRqt\r\n");

    /* Get the device descriptor. */
    status = CyFxSendSetupRqt (0x80, CY_U3P_USB_SC_GET_DESCRIPTOR,
            (CY_U3P_USB_DEVICE_DESCR << 8), 0, 8, glEp0Buffer);
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "[CyFxApplnStart] CyFxSendSetupRqt error\r\n");
        goto enum_error;
    }
    CyU3PDebugPrint (4, "Device descriptor received\r\n");

    /* Identify the EP0 packet size and update the scheduler. */
    if (glEp0Buffer[7] != 8)
    {
        UsbHostEpRemove (0);
        epCfg.maxPktSize = glEp0Buffer[7];
        epCfg.fullPktSize = glEp0Buffer[7];
        status = CyU3PUsbHostEpAdd (0, &epCfg);
        if (status != CY_U3P_SUCCESS)
        {
            goto enum_error;
        }
    }

    /* Read the full device descriptor. */
    status = CyFxSendSetupRqt (0x80, CY_U3P_USB_SC_GET_DESCRIPTOR,
            (CY_U3P_USB_DEVICE_DESCR << 8), 0, 18, glEp0Buffer);
    if (status != CY_U3P_SUCCESS)
    {
        goto enum_error;
    }

    /* Set the peripheral device address. */
    status = CyFxSendSetupRqt (0x00, CY_U3P_USB_SC_SET_ADDRESS,
            CY_FX_HOST_PERIPHERAL_ADDRESS, 0, 0, glEp0Buffer);
    if (status != CY_U3P_SUCCESS)
    {
        goto enum_error;
    }
    status = CyU3PUsbHostSetDeviceAddress (CY_FX_HOST_PERIPHERAL_ADDRESS);
    if (status != CY_U3P_SUCCESS)
    {
        goto enum_error;
    }

    status = CyFxSendSetupRqt (0x80, CY_U3P_USB_SC_GET_DESCRIPTOR,
            (CY_U3P_USB_DEVICE_DESCR << 8), 0, 18, glEp0Buffer);
    if (status != CY_U3P_SUCCESS)
    {
        goto enum_error;
    }

    CyU3PDebugPrint (4, "Device address set\r\n");

    status = CyFxSendSetupRqt(0x00, CY_U3P_USB_SC_SET_CONFIGURATION,
        1, 0, 0, glEp0Buffer);
	if (status != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "Set configured Error!!\r\n");
		goto enum_error;
	}

	CyU3PDebugPrint (4, "Current Vendor ID:0x%x%x, (cf. google=0x18D1)\r\n",glEp0Buffer[9],glEp0Buffer[8]);
	CyU3PDebugPrint (4, "Current Product ID:0x%x%x, (cf. Android-powered device=0x2D00 or 0x2D01)\r\n",glEp0Buffer[11],glEp0Buffer[10]);
	if ( IsDeviceInAccessoryMode () )
	{
		CyU3PDebugPrint (4, "Device is in Accessory Mode\r\n");
		status = PhoneDriverInit ();
		if (status == CY_U3P_SUCCESS)
		{
			glIsApplnActive = CyTrue;
			glHostOwner     = CY_FX_HOST_OWNER_PHONE_DRIVER;
			CyU3PDebugPrint (4, "Smart phone driver is initialized, OutEp=0x%x, InEp=0x%x, EpSize=%d\n",Phone.outEp,Phone.inEp,Phone.epSize);
			SendMessage("PING ON");
			return;
		}else{
			CyU3PDebugPrint (4, "Smart phone driver initialization failed, error: 0x%x\r\n",status);
		}
	}
	else
	{
		CyU3PDebugPrint (4, "Device is not in Accessory Mode\r\n");
		if((status = AttemptToStartInAccessoryMode())!=CY_U3P_SUCCESS)
			CyU3PDebugPrint (4, "Attempt to start in accessory mode failed, error: 0x%x\r\n",status);
	}

    /* We do not support this device. Fall-through to disable the USB port. */
    CyU3PDebugPrint (4, "Unknown device type\r\n");
    status = CY_U3P_ERROR_NOT_SUPPORTED;

enum_error:
    glIsApplnActive = CyFalse;
    /* Remove EP0. and disable the port. */
    UsbHostEpRemove (0);
    UsbHostPortDisable ();
    CyU3PDebugPrint (4, "Application start failed with error: 0x%x.\r\n", status);
}

/* This function disables the mouse driver application. */
void
CyFxApplnStop ()
{
    /* Call the device specific de-init routine. */
    switch (glHostOwner)
    {
        case CY_FX_HOST_OWNER_PHONE_DRIVER:
        	SendMessage("PING OFF");
        	PhoneDriverDeInit ();
        	break;
        default:
            break;
    }

    /* Remove EP0. and disable the port. */
    UsbHostEpRemove (0);
    UsbHostPortDisable ();

    glHostOwner = CY_FX_HOST_OWNER_NONE;

    /* Clear state variables. */
    glIsApplnActive = CyFalse;
}

/* USB host stack EP transfer completion callback. */
static void
CyFxHostXferCb (uint8_t ep, CyU3PUsbHostEpStatus_t epStatus)
{
	//CyU3PDebugPrint (4, "CyFxHostXferCb ep=0x%x,epStatus=0x%x\r\n", ep,epStatus);
}

/* This function initializes the USB host stack. */
void
CyFxUsbHostStart ()
{
    CyU3PUsbHostConfig_t hostCfg;
    CyU3PReturnStatus_t status;

    hostCfg.ep0LowLevelControl = CyFalse;
    hostCfg.eventCb = CyFxHostEventCb;
    hostCfg.xferCb = CyFxHostXferCb;
    status = CyU3PUsbHostStart (&hostCfg);
    if (status != CY_U3P_SUCCESS)
    {
    	CyU3PDebugPrint (4, "CyU3PUsbHostStart failed with error: %d.\r\n", status);
        return;
    }
}

/* This function disables the USB host stack. */
void
CyFxUsbHostStop ()
{
    /* Stop host mode application if running. */
    if (glIsApplnActive)
    {
        CyFxApplnStop ();
    }

    /* Stop the host module. */
    CyU3PUsbHostStop ();
}

/* This function enables / disables driving VBUS. */
void
CyFxUsbVBusControl (
        CyBool_t isEnable)
{
    if (isEnable)
    {
        /* Drive VBUS only if no-one else is driving. */
        if (!CyU3POtgIsVBusValid ())
        {
            CyU3PGpioSimpleSetValue (VBUS_GPIO, CY_FX_HOST_VBUS_ENABLE_VALUE);
        }
    }
    else
    {
        CyU3PGpioSimpleSetValue (VBUS_GPIO, CY_FX_HOST_VBUS_DISABLE_VALUE);
    }

    CyU3PDebugPrint (4, "VBUS enable:%d\r\n", isEnable);
}

/* OTG event handler. */
void
CyFxOtgEventCb (
        CyU3POtgEvent_t event,
        uint32_t input)
{
    CyU3PDebugPrint (4, "OTG Event: %d, Input: %d.\r\n", event, input);
    switch (event)
    {
        case CY_U3P_OTG_PERIPHERAL_CHANGE:
            if (input == CY_U3P_OTG_TYPE_A_CABLE)
            {
                CyFxUsbHostStart ();
                /* Enable the VBUS supply. */
                CyFxUsbVBusControl (CyTrue);
            }
            else
            {
                /* Make sure that the VBUS supply is disabled. */
                CyFxUsbVBusControl (CyFalse);

                /* Stop the previously started host stack. */
                if ((!CyU3POtgIsHostMode ()) && (CyU3PUsbHostIsStarted ()))
                {
                    CyFxUsbHostStop ();
                }
            }
            break;

        case CY_U3P_OTG_VBUS_VALID_CHANGE:
            if (input)
            {
                /* Start the host mode stack. */
                if (!CyU3PUsbHostIsStarted ())
                {
                    CyFxUsbHostStart ();
                }
            }
            else
            {
                /* If the OTG mode has changed, stop the previous stack. */
                if ((!CyU3POtgIsHostMode ()) && (CyU3PUsbHostIsStarted ()))
                {
                    /* Stop the previously started host stack. */
                    CyFxUsbHostStop ();
                }
            }
            break;

        default:
            /* do nothing */
            break;
    }
}

/* This function initializes the USB module. */
CyU3PReturnStatus_t
CyFxApplnInit (void)
{
    CyU3POtgConfig_t otgCfg;
    CyU3PGpioSimpleConfig_t simpleCfg;
    CyU3PReturnStatus_t status;

    /* Override VBUS_GPIO for VBUS control. */
    status = CyU3PDeviceGpioOverride (VBUS_GPIO, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Configure VBUS_GPIO as output for VBUS control. */
    simpleCfg.outValue = CY_FX_HOST_VBUS_DISABLE_VALUE;
    simpleCfg.driveLowEn = CyTrue;
    simpleCfg.driveHighEn = CyTrue;
    simpleCfg.inputEn = CyFalse;
    simpleCfg.intrMode = CY_U3P_GPIO_NO_INTR;
    status = CyU3PGpioSetSimpleConfig (VBUS_GPIO, &simpleCfg);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Wait until VBUS is stabilized. */
    CyU3PThreadSleep (100);

    /* Initialize the OTG module. */
    otgCfg.otgMode = CY_U3P_OTG_MODE_OTG;
    otgCfg.chargerMode = CY_U3P_OTG_CHARGER_DETECT_ACA_MODE;
    otgCfg.cb = CyFxOtgEventCb;
    status = CyU3POtgStart (&otgCfg);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Since VBATT or VBUS is required for OTG operation enable it. */
    status = CyU3PUsbVBattEnable (CyTrue);

    CyFxAutoSetupGpio();
    CyU3PDebugPrint(4,"[Phone] Setup GPIO OK\n");
    CyFxAutoI2cInit();
    CyU3PDebugPrint(4,"[Phone] I2C Init OK\n");
    CyFxAutoPibInit();
    CyU3PDebugPrint(4,"[Phone] PIB Init OK\n");

    ControlCh.StackPtr_ = 0;
    ControlCh.pf_ = 0;
    phoneUsbToZing.StackPtr_ = 0;
    phoneUsbToZing.pf_ = 0;
    zingToPhoneUsb.StackPtr_ = 0;
    zingToPhoneUsb.pf_ = 0;

    CyFxCreateCpuPibDmaChannels("[Phone]",CY_FX_DATA_BURST_LENGTH);
    CyU3PDebugPrint(4,"[Phone] DMA Channels for CPU-PIB Created\n");
    CyFxCreateControlChannel();
    CyU3PDebugPrint(4,"[Phone] Control Channel Thread Created\n");
	CyFxZingInit();
	CyU3PDebugPrint(4,"[Phone] ZING Init OK\n");
    CyFxSetHRCP();
    CyU3PDebugPrint(4,"[Phone] Set HRCP done\n");
    CyFxGolay();
    CyU3PDebugPrint(4,"[Phone] Golay code done\n");
    return status;
}

/* Entry function for the AppThread. */
void
ApplnThread_Entry (
        uint32_t input)
{
    CyU3PReturnStatus_t status    = CY_U3P_SUCCESS;
    CyU3PGpioClock_t    clkCfg;
    uint32_t            evStat;
    uint32_t			loop,iter;

    /* Initialize GPIO module. */
    clkCfg.fastClkDiv = 2;
    clkCfg.slowClkDiv = 0;
    clkCfg.halfDiv    = CyFalse;
    clkCfg.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    clkCfg.clkSrc     = CY_U3P_SYS_CLK;
    CyU3PGpioInit (&clkCfg, NULL);

    /* Wait until the board has been prepared for UART output. This code is only required when the FX3 is manually
     * being switched from SPI boot to UART enabled mode for debug output. */
    while ((GPIO->lpp_gpio_invalue1 & 0x04000000) == 0)
        CyU3PThreadSleep (1);

    /* Initialize the debug logger. */
    CyFxApplnDebugInit ();

    /* Initialize the example application. */
    status = CyFxApplnInit();
    if (status != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "Application initialization failed. Aborting.\r\n");
        CyFxAppErrorHandler (status);
    }

    loop = iter = 0;
    for (;;)
    {
        /* Wait until a peripheral change event has been detected, or until the poll interval has elapsed. */
        status = CyU3PEventGet (&applnEvent, CY_FX_USB_CHANGE_EVENT | CY_FX_PHONEUSB_RECEIVE_ERR, CYU3P_EVENT_OR_CLEAR,
                &evStat, CY_FX_HOST_POLL_INTERVAL);

        /* If a peripheral change has been detected, go through device discovery. */
        if (status == CY_U3P_SUCCESS)
        {
        	if((evStat & CY_FX_USB_CHANGE_EVENT) != 0){
                /* Add some delay for debouncing. */
                 CyU3PThreadSleep (100);

                 /* Clear the CHANGE event notification, so that we do not react to stale events. */
                 CyU3PEventGet (&applnEvent, CY_FX_USB_CHANGE_EVENT, CYU3P_EVENT_OR_CLEAR, &evStat, CYU3P_NO_WAIT);

                 /* Stop previously started application. */
                 if (glIsApplnActive)
                 {
                     CyFxApplnStop ();
                 }

                 /* If a peripheral got connected, then enumerate and start the application. */
                 if (glIsPeripheralPresent)
                 {
                     CyU3PDebugPrint (4, "Enable host port\r\n");
                     status = CyU3PUsbHostPortEnable ();
                     if (status == CY_U3P_SUCCESS)
                     {
                         CyFxApplnStart ();
                         CyU3PDebugPrint (4, "App start completed\r\n");
                     }
                     else
                     {
                         CyU3PDebugPrint (4, "HostPortEnable failed with code %d\r\n", status);
                     }
                 }
        	}else if((evStat & CY_FX_PHONEUSB_RECEIVE_ERR) != 0){
                CyU3PDebugPrint (4, "PhoneUsb receive error detected\r\n");
#if 1
                CyU3PThreadSleep (10);
                CyU3PDeviceReset (CyFalse);
#else
                CyU3PThreadSleep (100);

                CyU3PDebugPrint (4, "CyFxApplnStop...\r\n");
                CyFxApplnStop ();
                CyU3PDebugPrint (4, "CyFxApplnStop done\r\n");
                if((status = CyU3PUsbHostPortEnable ())==CY_U3P_SUCCESS) {
                	CyU3PDebugPrint (4, "CyFxApplnStart...\r\n");
                    CyFxApplnStart ();
                    CyU3PDebugPrint (4, "CyFxApplnStart done\r\n");
                }else{
                	CyU3PDebugPrint (4, "HostPortEnable failed error=0x%x\r\n", status);
                }
#endif
        	}
        }

        loop++;

        if(loop%10==0 && glIsApplnActive) SendMessage("PING ON");

        if(loop%100==0) {	//To print every 1 sec. cf. CY_FX_HOST_POLL_INTERVAL is 10ms
        	iter++;
#ifndef DEBUG_THREAD_LOOP
        	CyU3PDebugPrint (4, "%d [Z->P] Rcv(o:%d x:%d) Snd(o:%d x:%d) | [P->Z] Rcv(o:%d x:%d) Snd(o:%d x:%d)\r\n",iter,
        			zingToPhoneUsb.Count_.receiveOk,zingToPhoneUsb.Count_.receiveErr,zingToPhoneUsb.Count_.sendOk,zingToPhoneUsb.Count_.sendErr,
            		phoneUsbToZing.Count_.receiveOk,phoneUsbToZing.Count_.receiveErr,phoneUsbToZing.Count_.sendOk,phoneUsbToZing.Count_.sendErr);
#endif
        }

#ifdef INTENTIONALLY_CAUSE_RECEIVE_ERROR
		if(loop%5000==0) {
		    CyU3PEventSet (&applnEvent, CY_FX_PHONEUSB_RECEIVE_ERR, CYU3P_EVENT_OR);
		    CyU3PDebugPrint(4,"Set PhoneUsb receive Error event\r\n");
		}
#endif
    }
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    uint32_t status = CY_U3P_SUCCESS;
    void *ptr = NULL;

    /* Allocate the memory for the threads */
    ptr = CyU3PMemAlloc (CY_FX_APPLN_THREAD_STACK);
    if (ptr == 0)
        goto InitError;

    /* Create the thread for the application */
    status = CyU3PThreadCreate (&applnThread,               /* App Thread structure */
                          "21:USB_HOSTAPP",                 /* Thread ID and Thread name */
                          ApplnThread_Entry,                /* App Thread Entry function */
                          0,                                /* No input parameter to thread */
                          ptr,                              /* Pointer to the allocated thread stack */
                          CY_FX_APPLN_THREAD_STACK,         /* App Thread stack size */
                          CY_FX_APPLN_THREAD_PRIORITY,      /* App Thread priority */
                          CY_FX_APPLN_THREAD_PRIORITY,      /* App Thread pre-emption threshold */
                          CYU3P_NO_TIME_SLICE,              /* No time slice for the application thread */
                          CYU3P_AUTO_START                  /* Start the Thread immediately */
                          );

    if (status != 0)
        goto InitError;

    status = CyU3PEventCreate (&applnEvent);
    if (status != 0)
        goto InitError;

    return;

InitError:
    /* Add custom recovery or debug actions here */

    /* Loop indefinitely as the application cannot proceed. */
    while(1);
}

/*
 * Main function
 */
int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clkCfg;

	/* Initialize the device
	 * setSysClk400 clock configurations */
	clkCfg.setSysClk400 = CyTrue;   /* FX3 device's master clock is set to a frequency > 400 MHz */
	clkCfg.cpuClkDiv = 2;           /* CPU clock divider */
	clkCfg.dmaClkDiv = 2;           /* DMA clock divider */
	clkCfg.mmioClkDiv = 2;          /* MMIO clock divider */
	clkCfg.useStandbyClk = CyFalse; /* device has no 32KHz clock supplied */
	clkCfg.clkSrc = CY_U3P_SYS_CLK; /* Clock source for a peripheral block  */
	status = CyU3PDeviceInit(&clkCfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable both Instruction and Data Caches. */
    status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration. */
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 1<<(GPIO57-32); // TP2 in schematic
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

