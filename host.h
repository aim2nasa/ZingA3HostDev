/*
 ## Cypress USB 3.0 Platform header file (cyfxusbhost.h)
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

/* This file contains the externants used by the host application example */

#ifndef _INCLUDED_CYFXUSBHOST_H_
#define _INCLUDED_CYFXUSBHOST_H_

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3usbhost.h"
#include "cyu3externcstart.h"

#define CY_FX_APPLN_THREAD_STACK        (0x800)         /* Application thread stack size */
#define CY_FX_APPLN_THREAD_PRIORITY     (8)             /* Application thread priority */

#define CY_FX_HOST_POLL_INTERVAL        (10)            /* The polling interval for the DoWork function in ms. */

#define CY_FX_HOST_EP0_SETUP_SIZE       (32)            /* EP0 setup packet buffer size made multiple of 32 bytes. */
#define CY_FX_HOST_EP0_BUFFER_SIZE      (512)           /* EP0 data transfer buffer size. */
#define CY_FX_HOST_EP0_WAIT_TIMEOUT     (5000)          /* EP0 request timeout in clock ticks. */
#define CY_FX_HOST_PERIPHERAL_ADDRESS   (1)             /* USB host mode peripheral address to be used. */
#define CY_FX_HOST_DEFAULT_DEV_ADDRESS  (0)             /* Default device address to be used. */
#define CY_FX_HOST_DMA_BUF_COUNT        (4)             /* Number of buffers to be allocated for DMA channel. */

#define CY_FX_HOST_VBUS_ENABLE_VALUE    (CyTrue)        /* GPIO value for driving VBUS. */
#define CY_FX_HOST_VBUS_DISABLE_VALUE   (!CY_FX_HOST_VBUS_ENABLE_VALUE)

#define CY_FX_HOST_OWNER_NONE           (0)             /* Host controller is not associated with a class driver. */
#define CY_FX_HOST_OWNER_PHONE_DRIVER   (1)

#define CY_FX_USB_CHANGE_EVENT          (1 << 0)        /* Event signaling that a peripheral change is detected. */
#define CY_FX_PHONEUSB_RECEIVE_ERR		(1 << 1)

#define CY_FX_DATA_BURST_LENGTH			(8)				/* Number of Burst for the Data. USB 3.0 only, fix 8 in ZING */
#define VBUS_GPIO						(52)			/* Controls the VBUS supply via this GPIO port */

extern uint8_t glEp0Buffer[];                           /* Buffer to send / receive data for EP0. */

/* Summary
   Helper function to send the setup packet.

   Description
   This function takes in the various fields
   of the setup packet and formats it into
   the eight byte setup packet and then sends
   it the USB host. It also waits for the
   transfer to complete.

   Return Value
   CY_U3P_SUCCESS - The call was successful.
   Non zero value means error.

 */
extern CyU3PReturnStatus_t
CyFxSendSetupRqt (
        uint8_t type,           /* Attributes of setup request . */
        uint8_t request,        /* The request code. */
        uint16_t value,         /* Value field. */
        uint16_t index,         /* Index field. */
        uint16_t length,        /* Length of the data phase. */
        uint8_t *buffer_p       /* Buffer pointer for the data phase. */
        );

/* Summary
   Function initializes the mouse driver.

   Description
   The function is invoked by the host controller driver
   when a HID mouse class device is detected.

   Return Value
   CY_U3P_SUCCESS - The call was successful.
   CY_U3P_ERROR_FAILURE - The driver initialization failed.
 */
extern CyU3PReturnStatus_t
CyFxMouseDriverInit (
        void);

/* Summary
   Function disables the mouse driver.

   Description
   The function is invoked by the host controller driver
   when the previously attached mouse device is disconnected.

   Return Value
   None.
 */
extern void
CyFxMouseDriverDeInit (
        void);

/* Summary
   Function initializes the MSC driver.

   Description
   The function is invoked by the host controller driver
   when a MSC class device is detected.

   Return Value
   CY_U3P_SUCCESS - The call was successful.
   CY_U3P_ERROR_FAILURE - The driver initialization failed.
 */
extern CyU3PReturnStatus_t
CyFxMscDriverInit (
        void);

/* Summary
   Function disables the MSC driver.

   Description
   The function is invoked by the host controller driver
   when the previously attached MSC device is disconnected.

   Return Value
   None.
 */
extern void
CyFxMscDriverDeInit (
        void);

/* Summary
   Function runs the MSC driver periodic tasks.

   Description
   The function is invoked by the host controller driver
   thread at a fixed interval.

   Return Value
   None.
 */
extern void
CyFxMscDriverDoWork (
        void);

extern CyU3PReturnStatus_t
CyFxEchoDriverInit (
        void);

extern void
CyFxEchoDriverDeInit (
        void);

#include "cyu3externcend.h"

#endif /* _INCLUDED_CYFXUSBHOST_H_ */

/*[]*/
