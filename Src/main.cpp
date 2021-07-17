#include <climits>
#include "VirtualUSBDevice.h"
#include "Descriptor.h"
#include "RuntimeError.h"

static USB::CDC::LineCoding LineCoding = {};

static struct {
    std::mutex lock;
    std::condition_variable signal;
    std::deque<VirtualUSBDevice::Data> queue;
    bool dtePresent = false;
} _Datas;

static void _handleRequestEP0(VirtualUSBDevice& dev, VirtualUSBDevice::Data&& data) {
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
        auto lock = std::unique_lock(_Datas.lock);
        _Datas.dtePresent = dtePresent;
        if (!_Datas.dtePresent) {
            _Datas.queue.clear();
        }
        _Datas.signal.notify_all();
        return d.reply(xfer, nullptr, 0);
    }
    
    default:
        throw RuntimeError("invalid request");
    }
}

static bool _handleRequestEPX(VirtualUSBDevice& dev, VirtualUSBDevice::Data&& data) {
    const uint8_t ep = xfer.getEndpointAddress();
//    const uint8_t* payload = xfer.payload.get();
//    const size_t payloadLen = xfer.payloadLen;
    
    switch (ep) {
    case Endpoint::In1: {
        printf("Endpoint::In1\n");
        auto lock = std::unique_lock(_Datas.lock);
        _Datas.queue.push_back(std::move(xfer));
        _Datas.signal.notify_all();
        break;
    }
    
    case Endpoint::In2: {
        printf("Endpoint::In2\n");
        auto lock = std::unique_lock(_Datas.lock);
        _Datas.queue.push_back(std::move(xfer));
        _Datas.signal.notify_all();
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

static void _handleData(VirtualUSBDevice& dev, VirtualUSBDevice::Data&& data) {
    // Endpoint 0
    if (!xfer.cmd.base.ep) _handleRequestEP0(d, std::move(data));
    // Other endpoints
    else _handleRequestEPX(d, std::move(data));
}

static void _threadResponse(VirtualUSBDevice& dev) {
    for (;;) {
        auto lock = std::unique_lock(_Datas.lock);
        while (!_Datas.dtePresent || _Datas.queue.empty()) {
            _Datas.signal.wait(lock);
        }
        
        VirtualUSBDevice::Data xfer = std::move(_Datas.queue.front());
        _Datas.queue.pop_front();
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
            VirtualUSBDevice::Data data = dev.read();
            _handleData(dev, std::move(data));
        }
        
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        // Using _exit to avoid calling destructors for static vars, since that hangs
        // in __pthread_cond_destroy, because our thread is sitting in _Datas.signal.wait()
        _exit(1);
    }
    
    return 0;
}
