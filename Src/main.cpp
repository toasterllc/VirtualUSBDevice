#include <climits>
#include "VirtualUSBDevice.h"
#include "Descriptor.h"
#include "RuntimeError.h"

static USB::CDC::LineCoding LineCoding = {};

static struct {
    std::mutex lock;
    std::condition_variable signal;
    bool dtePresent = false;
} _State;

static void _handleXferEP0(VirtualUSBDevice& dev, VirtualUSBDevice::Xfer&& xfer) {
    const USB::SetupRequest req = xfer.setupReq;
    const uint8_t* payload = xfer.data.get();
    const size_t payloadLen = xfer.len;
    
    // Verify that this is a Class request intended for an Interface
    if (req.bmRequestType != (USB::RequestType::TypeClass|USB::RequestType::RecipientInterface))
        throw RuntimeError("invalid request");
    
    switch (req.bRequest) {
    case USB::CDC::Request::SET_LINE_CODING: {
        if (payloadLen != sizeof(LineCoding))
            throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
        
        memcpy(&LineCoding, payload, sizeof(LineCoding));
        LineCoding = {
            .dwDTERate      = Endian::HFL_U32(LineCoding.dwDTERate),
            .bCharFormat    = Endian::HFL_U8(LineCoding.bCharFormat),
            .bParityType    = Endian::HFL_U8(LineCoding.bParityType),
            .bDataBits      = Endian::HFL_U8(LineCoding.bDataBits),
        };
        
        printf("SET_LINE_CODING:\n");
        printf("  dwDTERate: %08x\n", LineCoding.dwDTERate);
        printf("  bCharFormat: %08x\n", LineCoding.bCharFormat);
        printf("  bParityType: %08x\n", LineCoding.bParityType);
        printf("  bDataBits: %08x\n", LineCoding.bDataBits);
        return dev.write(USB::Endpoint::DefaultIn, nullptr, 0);
    }
    
    case USB::CDC::Request::GET_LINE_CODING: {
        printf("GET_LINE_CODING\n");
        if (payloadLen != sizeof(LineCoding))
            throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
        return dev.write(USB::Endpoint::DefaultIn, &LineCoding, sizeof(LineCoding));
    }
    
    case USB::CDC::Request::SET_CONTROL_LINE_STATE: {
        const bool dtePresent = req.wValue&1;
        printf("SET_CONTROL_LINE_STATE:\n");
        printf("  dtePresent=%d\n", dtePresent);
        auto lock = std::unique_lock(_State.lock);
        _State.dtePresent = dtePresent;
        _State.signal.notify_all();
        return dev.write(USB::Endpoint::DefaultIn, nullptr, 0);
    }
    
    default:
        throw RuntimeError("invalid request");
    }
}

static void _handleXferEPX(VirtualUSBDevice& dev, VirtualUSBDevice::Xfer&& xfer) {
    switch (xfer.ep) {
    case Endpoint::Out2: {
        printf("Endpoint::Out2: <");
        for (size_t i=0; i<xfer.len; i++) {
            printf(" %02x", xfer.data[i]);
        }
        printf(" >\n\n");
        break;
    }
    
    default:
        throw RuntimeError("invalid endpoint: 0x%02x", xfer.ep);
    }
}

static void _handleXfer(VirtualUSBDevice& dev, VirtualUSBDevice::Xfer&& xfer) {
    // Endpoint 0
    if (xfer.ep == 0) _handleXferEP0(dev, std::move(xfer));
    // Other endpoints
    else _handleXferEPX(dev, std::move(xfer));
}

static void _threadResponse(VirtualUSBDevice& dev) {
    for (;;) {
        auto lock = std::unique_lock(_State.lock);
        while (!_State.dtePresent) {
            _State.signal.wait(lock);
        }
        lock.unlock();
        
        const char text[] = "Testing 123\r\n";
        dev.write(Endpoint::In2, text, strlen(text));
        usleep(500000);
    }
}

int main(int argc, const char* argv[]) {
    const VirtualUSBDevice::Info deviceInfo = {
        .deviceDesc             = &Descriptor::Device,
        .deviceQualifierDesc    = &Descriptor::DeviceQualifier,
        .configDescs            = Descriptor::Configurations,
        .configDescsCount       = std::size(Descriptor::Configurations),
        .stringDescs            = Descriptor::Strings,
        .stringDescsCount       = std::size(Descriptor::Strings),
        .throwOnErr             = true,
    };
    
    VirtualUSBDevice dev(deviceInfo);
    try {
        try {
            dev.start();
        } catch (std::exception& e) {
            throw RuntimeError(
                "Failed to start VirtualUSBDevice: %s"                                  "\n"
                "Make sure:"                                                            "\n"
                "  - you're root"                                                       "\n"
                "  - the vhci-hcd kernel module is loaded: `sudo modprobe vhci-hcd`"    "\n",
                e.what()
            );
        }
        
        printf("Started\n");
        
        std::thread([&] {
            _threadResponse(dev);
        }).detach();
        
//        // Test device teardown (after 5 seconds)
//        std::thread([&] {
//            sleep(5);
//            printf("Stopping device...\n");
//            device.stop();
//        }).detach();
        
        for (;;) {
            VirtualUSBDevice::Xfer data = dev.read();
            _handleXfer(dev, std::move(data));
        }
        
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        // Using _exit to avoid calling destructors for static vars, since that hangs
        // in __pthread_cond_destroy, because our thread is sitting in _State.signal.wait()
        _exit(1);
    }
    
    return 0;
}
