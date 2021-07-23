#include "usbHostEpWrapper.h"
#include "cyu3usbhost.h"
#include "cyu3system.h"

CyU3PReturnStatus_t
UsbHostPortDisable (
        void)
{
	CyU3PDebugPrint(4,"CyU3PUsbHostPortDisable\r\n");
	return CyU3PUsbHostPortDisable();
}

CyU3PReturnStatus_t
UsbHostEpRemove (
        uint8_t ep                      /**< Endpoint to be removed from scheduler. */
        )
{
	CyU3PDebugPrint(4,"CyU3PUsbHostEpRemove\r\n");
	return CyU3PUsbHostEpRemove(ep);
}
