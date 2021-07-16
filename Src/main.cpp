#include <climits>
#include "VirtualUSBDevice.h"
#include "Descriptor.h"
#include "RuntimeError.h"

static USB::CDC::LineCoding LineCoding = {};

static struct {
    std::mutex lock;
    std::condition_variable signal;
    std::deque<VirtualUSBDevice::Xfer> queue;
    bool dtePresent = false;
} _Xfers;

static void _handleRequestEP0(VirtualUSBDevice& d, VirtualUSBDevice::Xfer&& xfer) {
    const USB::SetupRequest req = xfer.getSetupRequest();
    const uint8_t* payload = xfer.payload.get();
    const size_t payloadLen = xfer.payloadLen;
    
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
        return d.reply(xfer, nullptr, 0);
    }
    
    case USB::CDC::Request::GET_LINE_CODING: {
        printf("GET_LINE_CODING\n");
        if (payloadLen != sizeof(LineCoding))
            throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
        return d.reply(xfer, &LineCoding, sizeof(LineCoding));
    }
    
    case USB::CDC::Request::SET_CONTROL_LINE_STATE: {
        const bool dtePresent = req.wValue&1;
        printf("SET_CONTROL_LINE_STATE:\n");
        printf("  dtePresent=%d\n", dtePresent);
        auto lock = std::unique_lock(_Xfers.lock);
        _Xfers.dtePresent = dtePresent;
        if (!_Xfers.dtePresent) {
            _Xfers.queue.clear();
        }
        _Xfers.signal.notify_all();
        return d.reply(xfer, nullptr, 0);
    }
    
    default:
        throw RuntimeError("invalid request");
    }
}

static bool _handleRequestEPX(VirtualUSBDevice& d, VirtualUSBDevice::Xfer&& xfer) {
    const uint8_t ep = xfer.getEndpointAddress();
//    const uint8_t* payload = xfer.payload.get();
//    const size_t payloadLen = xfer.payloadLen;
    
    switch (ep) {
    case Endpoint::In1: {
        printf("Endpoint::In1\n");
        auto lock = std::unique_lock(_Xfers.lock);
        _Xfers.queue.push_back(std::move(xfer));
        _Xfers.signal.notify_all();
        break;
    }
    
    case Endpoint::In2: {
        printf("Endpoint::In2\n");
        auto lock = std::unique_lock(_Xfers.lock);
        _Xfers.queue.push_back(std::move(xfer));
        _Xfers.signal.notify_all();
        break;
    }
    
    case Endpoint::Out2: {
        printf("Endpoint::Out2\n");
        d.reply(xfer, nullptr, 0);
        break;
    }
    
    default:
        throw RuntimeError("invalid endpoint: %02x", ep);
    }
    
    return true;
}

static void _handleMessage(VirtualUSBDevice& d, VirtualUSBDevice::Xfer&& xfer) {
    // Endpoint 0
    if (!xfer.cmd.base.ep) _handleRequestEP0(d, std::move(xfer));
    // Other endpoints
    else _handleRequestEPX(d, std::move(xfer));
}

static void _threadResponse(VirtualUSBDevice& d) {
    for (;;) {
        auto lock = std::unique_lock(_Xfers.lock);
        while (!_Xfers.dtePresent || _Xfers.queue.empty()) {
            _Xfers.signal.wait(lock);
        }
        
        VirtualUSBDevice::Xfer xfer = std::move(_Xfers.queue.front());
        _Xfers.queue.pop_front();
        lock.unlock();
        
        const uint8_t ep = xfer.getEndpointAddress();
        switch (ep) {
        case Endpoint::In1: {
            printf("_threadResponse: replied to Endpoint::In1\n");
            d.reply(xfer, nullptr, 0);
            break;
        }
        
        case Endpoint::In2: {
            printf("_threadResponse: replied to Endpoint::In2\n");
            char text[] = "Testing\r\n";
            d.reply(xfer, text, strlen(text));
            break;
        }
        
        default:
            abort();
        }
        
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
    
    VirtualUSBDevice device(deviceInfo);
    try {
        try {
            device.start();
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
            _threadResponse(device);
        }).detach();
        
//        // Test device teardown (after 5 seconds)
//        std::thread([&] {
//            sleep(5);
//            printf("Stopping device...\n");
//            device.stop();
//        }).detach();
        
        for (;;) {
            VirtualUSBDevice::Xfer xfer = device.read();
            _handleMessage(device, std::move(xfer));
        }
        
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        // Using _exit to avoid calling destructors for static vars, since that hangs
        // in __pthread_cond_destroy, because our thread is sitting in _Xfers.signal.wait()
        _exit(1);
    }
    
    return 0;
}
