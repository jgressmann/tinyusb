#pragma once

// DFU_1.1.pdf, p. 22
enum dfu_state {
    DFU_STATE_APP_IDLE = 0,
    DFU_STATE_APP_DETACH,
    DFU_STATE_DFU_IDLE,
    DFU_STATE_DFU_DNLOAD_SYNC,
    DFU_STATE_DFU_DNBUSY,
    DFU_STATE_DFU_DNLOAD_IDLE,
    DFU_STATE_DFU_MANIFEST_SYNC,
    DFU_STATE_DFU_MANIFEST,
    DFU_STATE_DFU_MANIFEST_WAIT_RESET,
    DFU_STATE_DFU_UPLOAD_IDLE,
    DFU_STATE_DFU_ERROR
};

// DFU_1.1.pdf, p. 21
enum dfu_error {
    DFU_ERROR_OK = 0,        //  No error condition is present.
    DFU_ERROR_TARGET,        // File is not targeted for use by this device.
    DFU_ERROR_FILE,          // File is for this device but fails some vendor-specific verification test.
    DFU_ERROR_WRITE,         // Device is unable to write memory.
    DFU_ERROR_ERASE,         // Memory erase function failed.
    DFU_ERROR_CHECK_ERASED , // Memory erase check failed.
    DFU_ERROR_PROG ,         // Program memory function failed.
    DFU_ERROR_VERIFY ,       // Programmed memory failed verification.
    DFU_ERROR_ADDRESS ,      // Cannot program memory due to received address that is out of range.
    DFU_ERROR_NOTDONE ,    // Received DFU_DNLOAD with wLength = 0, but device does not think it has all of the data yet.
    DFU_ERROR_FIRMWARE ,   // Device’s firmware is corrupt. It cannot return to run-time (non-DFU) operations.
    DFU_ERROR_VENDOR ,     // iString indicates a vendor-specific error.
    DFU_ERROR_USBR ,       // Device detected unexpected USB reset signaling.
    DFU_ERROR_POR ,        // Device detected unexpected power on reset.
    DFU_ERROR_UNKNOWN ,    // Something went wrong, but the device does not know what it was.
    DFU_ERROR_STALLEDPKT , // Device stalled an unexpected request.
};

// DFU_1.1.pdf, p. 10
enum dfu_request {
    DFU_REQUEST_DETACH,
    DFU_REQUEST_DNLOAD,
    DFU_REQUEST_UPLOAD,
    DFU_REQUEST_GETSTATUS,
    DFU_REQUEST_CLRSTATUS,
    DFU_REQUEST_GETSTATE,
    DFU_REQUEST_ABORT
};