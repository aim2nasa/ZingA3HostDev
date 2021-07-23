#ifndef __USBHOSTEPWRAPPER_H__
#define __USBHOSTEPWRAPPER_H__

#include "cyu3types.h"

CyU3PReturnStatus_t
UsbHostPortDisable (
        void);

CyU3PReturnStatus_t
UsbHostEpRemove (
        uint8_t ep                      /**< Endpoint to be removed from scheduler. */
        );

#endif
