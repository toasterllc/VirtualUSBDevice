## VirtualUSBDevice

`VirtualUSBDevice` is a C++ class that creates a virtual USB device on Linux. The virtual device is visible to the rest of the system as any real USB device is.

The implementation relies on the `usbip` subsystem, but is self-contained and doesn't require the `usbip` utility to be manually invoked.

### Usage

To create a virtual USB device, instantiate a `VirtualUSBDevice` (supplying standard USB descriptors to the constructor), and call `start()`.

To handle USB transfers, call `read()`. The returned `VirtualUSBDevice::Xfer` object represents the USB transfer to be performed. Before `read()` returns, `VirtualUSBDevice()` will automatically handle standard USB requests (such as `GET_STATUS`, `GET_DESCRIPTOR`, `SET_CONFIGURATION`), and will only return from `read()` when there's a USB transfer that it can't handle itself.

To tear down the virtual device, call `stop()`.

### Example

The included example (`main.cpp`) implements a simple ACM/CDC USB serial device that can be accessed at `/dev/ttyACM0` (or `ttyACM1`, etc). The example outputs periodic messages while consuming input sent to the device.
