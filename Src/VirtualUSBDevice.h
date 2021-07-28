#pragma once
#include <thread>
#include <cerrno>
#include <functional>
#include <optional>
#include <cassert>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <set>
#include <chrono>
#include <sys/socket.h>
#include "USBIP.h"
#include "USBIPLib.h"
#include "Toastbox/Endian.h"
#include "Toastbox/USB.h"
#include "Toastbox/RuntimeError.h"
using namespace std::chrono_literals;

class VirtualUSBDevice {
public:
    struct Info {
        const USB::DeviceDescriptor* deviceDesc = nullptr;
        const USB::DeviceQualifierDescriptor* deviceQualifierDesc = nullptr;
        const USB::ConfigurationDescriptor*const* configDescs = nullptr;
        size_t configDescsCount = 0;
        const USB::StringDescriptor*const* stringDescs = nullptr;
        size_t stringDescsCount = 0;
        bool throwOnErr = false;
    };
    
    using Err = std::exception_ptr;
    static const inline Err ErrStopped = std::make_exception_ptr(std::runtime_error("VirtualUSBDevice stopped"));
    
    struct Xfer {
        uint8_t ep = 0;
        USB::SetupRequest setupReq = {};
        std::unique_ptr<uint8_t[]> data;
        size_t len = 0;
    };
    
    struct _Cmd {
        USBIP::HEADER header = {};
        std::unique_ptr<uint8_t[]> payload = {};
        size_t payloadLen = 0;
    };
    
    using _Rep = _Cmd;
    
    static const std::exception& ErrExtract(Err err) {
        assert(err);
        try {
            std::rethrow_exception(err);
        } catch (const std::exception& e) {
            return e;
        }
    }
    
    VirtualUSBDevice(const Info& info) : _info(info) {}
    
    ~VirtualUSBDevice() {
        auto lock = std::unique_lock(_s.lock);
        _reset(lock, ErrStopped);
    }
    
    void start() {
        auto lock = std::unique_lock(_s.lock);
        try {
            assert(_s.state == _State::Idle);
            _s.state |= _State::Started;
            
            const uint32_t speed = _SpeedFromBCDUSB(_info.deviceDesc->bcdDevice);
            int sockets[2] = {-1,-1};
            int ir = socketpair(AF_UNIX, SOCK_STREAM, 0, sockets);
            if (ir) throw RuntimeError("socketpair failed: %s", strerror(errno));
            _s.socket = sockets[0];
            _s.usbipSocket = sockets[1];
            
            ir = USBIPLib::usbip_vhci_driver_open();
            if (ir) throw RuntimeError("usbip_vhci_driver_open failed: %s", strerror(errno));
            
            for (;;) {
                int usbipPort = USBIPLib::usbip_vhci_get_free_port(speed);
                if (usbipPort < 0) throw RuntimeError("usbip_vhci_get_free_port failed: %s", strerror(errno));
                
                ir = USBIPLib::usbip_vhci_attach_device2(usbipPort, _s.usbipSocket, _DeviceID, speed);
                if (ir < 0) {
                    if (errno == EBUSY) continue;
                    else throw RuntimeError("usbip_vhci_attach_device2 failed: %s", strerror(errno));
                }
                break;
            }
            
            USBIPLib::usbip_vhci_driver_close();
            
            close(_s.usbipSocket);
            _s.usbipSocket = -1;
            
            _s.state |= (_State::ReadThreadRunning|_State::WriteThreadRunning);
            std::thread([this] { _readThread(); }).detach();
            std::thread([this] { _writeThread(); }).detach();
        
        } catch (const std::exception& e) {
            _reset(lock, std::current_exception());
            if (_info.throwOnErr) {
                // Throw `_s.err`, not `e`, so that we throw the original cause (eg ErrStopped)
                std::rethrow_exception(_s.err);
            }
        }
    }
    
    void stop() {
        auto lock = std::unique_lock(_s.lock);
        _reset(lock, ErrStopped);
    }
    
    std::optional<Xfer> read(std::chrono::milliseconds timeout=std::chrono::milliseconds::max()) {
        auto lock = std::unique_lock(_s.lock);
        try {
            for (;;) {
                // Wait for a command or an error
                for (;;) {
                    // Bail if there's an error (and therefore we're stopped)
                    if (_s.err) std::rethrow_exception(_s.err);
                    // Break if a command is available
                    if (!_s.cmds.empty()) break;
                    // Otherwise wait to get signalled
                    if (timeout == std::chrono::milliseconds::zero()) {
                        // Check once, which we already did, so bail
                        return std::nullopt;
                    } else if (timeout == std::chrono::milliseconds::max()) {
                        // Wait forever
                        _s.signal.wait(lock);
                    } else {
                        // Wait a specific amount of time
                        const std::cv_status cr = _s.signal.wait_for(lock, timeout);
                        if (cr == std::cv_status::timeout) return std::nullopt;
                    }
                }
                
                _Cmd cmd = std::move(_s.cmds.front());
                _s.cmds.pop_front();
                
                auto xfer = _handleCmd(cmd);
                if (xfer) return std::move(*xfer);
            }
        
        } catch (const std::exception& e) {
            _reset(lock, std::current_exception());
            if (_info.throwOnErr) {
                // Throw `_s.err`, not `e`, so that we throw the original cause (eg ErrStopped)
                std::rethrow_exception(_s.err);
            }
            return std::nullopt;
        }
    }
    
    void write(uint8_t ep, const void* data, size_t len) {
        // Must be an IN endpoint
        assert((ep & USB::Endpoint::DirMask) == USB::Endpoint::DirIn);
        const uint8_t epIdx = ep&USB::Endpoint::IdxMask;
        assert(epIdx < std::size(_s.inCmds));
        
        auto lock = std::unique_lock(_s.lock);
        try {
            // Bail if there's an error (and therefore we're stopped)
            if (_s.err) std::rethrow_exception(_s.err);
            
            // Enqueue the data into `epInData`
            auto& epInData = _s.inData[epIdx];
            _Data d = {
                .data = std::make_unique<uint8_t[]>(len),
                .len = len,
            };
            memcpy(d.data.get(), data, len);
            epInData.push_back(std::move(d));
            // Send the data if there are existing IN transfers
            _sendDataForInEndpoint(epIdx);
        
        } catch (const std::exception& e) {
            _reset(lock, std::current_exception());
            if (_info.throwOnErr) {
                // Throw `_s.err`, not `e`, so that we throw the original cause (eg ErrStopped)
                std::rethrow_exception(_s.err);
            }
        }
    }
    
    Err err() {
        auto lock = std::unique_lock(_s.lock);
        return _s.err;
    }
    
private:
    static constexpr uint8_t _DeviceID = 1;
    
    struct _Data {
        std::unique_ptr<uint8_t[]> data;
        size_t len = 0;
        size_t off = 0;
    };
    
    struct _State {
        static constexpr uint8_t Idle               = 0;
        static constexpr uint8_t Started            = 1<<0;
        static constexpr uint8_t ReadThreadRunning  = 1<<1;
        static constexpr uint8_t WriteThreadRunning = 1<<2;
        static constexpr uint8_t Reset              = 1<<3;
    };
    
    USB::SetupRequest _GetSetupRequest(const _Cmd& cmd) const {
        using namespace Endian;
        USB::SetupRequest req;
        memcpy(&req, cmd.header.cmd_submit.setup.u8, sizeof(req));
        req = {
            .bmRequestType  = HFL_U8(req.bmRequestType),
            .bRequest       = HFL_U8(req.bRequest),
            .wValue         = HFL_U16(req.wValue),
            .wIndex         = HFL_U16(req.wIndex),
            .wLength        = HFL_U16(req.wLength),
        };
        return req;
    }
    
    uint8_t _GetEndpointAddr(const _Cmd& cmd) {
        const bool dirIn = cmd.header.base.direction==USBIPLib::USBIP_DIR_IN;
        const uint8_t ep = cmd.header.base.ep | (dirIn ? 0x80 : 0x00);
        return ep;
    }
    
    static void _Read(int socket, void* data, size_t len) {
        size_t off = 0;
        while (off < len) {
            ssize_t sr = recv(socket, (uint8_t*)data+off, len-off, 0);
            if (!sr) throw RuntimeError("recv returned 0");
            if (sr < 0) {
                if (errno == EINTR) continue;
                else throw RuntimeError("recv failed: %s", strerror(errno));
            }
            off += sr;
        }
    }
    
    static void _Write(int socket, const void* data, size_t len) {
        size_t off = 0;
        while (off < len) {
            ssize_t sr = send(socket, (uint8_t*)data+off, len-off, MSG_NOSIGNAL);
            if (!sr) throw RuntimeError("send returned 0");
            if (sr < 0) {
                if (errno == EINTR) continue;
                else throw RuntimeError("send failed: %s", strerror(errno));
            }
            off += sr;
        }
    }
    
    static _Cmd _ReadCmd(int socket) {
        using namespace Endian;
        _Cmd cmd;
        _Read(socket, &cmd.header, sizeof(cmd.header));
        
        // Big endian -> host endian
        cmd.header.base = {
            .command    = HFB_U32(cmd.header.base.command),
            .seqnum     = HFB_U32(cmd.header.base.seqnum),
            .devid      = HFB_U32(cmd.header.base.devid),
            .direction  = HFB_U32(cmd.header.base.direction),
            .ep         = HFB_U32(cmd.header.base.ep),
        };
        
        switch (cmd.header.base.command) {
        case USBIPLib::USBIP_CMD_SUBMIT:
            cmd.header.cmd_submit = {
                .transfer_flags             = HFB_U32(cmd.header.cmd_submit.transfer_flags),
                .transfer_buffer_length     = HFB_S32(cmd.header.cmd_submit.transfer_buffer_length),
                .start_frame                = HFB_S32(cmd.header.cmd_submit.start_frame),
                .number_of_packets          = HFB_S32(cmd.header.cmd_submit.number_of_packets),
                .interval                   = HFB_S32(cmd.header.cmd_submit.interval),
                .setup                      = {.u64 = cmd.header.cmd_submit.setup.u64 }, // stream of bytes -- don't change
            };
            break;
        
        case USBIPLib::USBIP_CMD_UNLINK:
            cmd.header.cmd_unlink = {
                .seqnum = HFB_U32(cmd.header.cmd_unlink.seqnum),
            };
            break;
        
        default:
            throw RuntimeError("unknown USBIP command: %u", (uint32_t)cmd.header.base.command);
        }
        
        if (cmd.header.base.command==USBIPLib::USBIP_CMD_SUBMIT &&
            cmd.header.base.direction==USBIPLib::USBIP_DIR_OUT) {
            // Set payload length if this is data coming from the host
            cmd.payloadLen = cmd.header.cmd_submit.transfer_buffer_length;
        }
        
        if (cmd.payloadLen) {
            cmd.payload = std::make_unique<uint8_t[]>(cmd.payloadLen);
            _Read(socket, cmd.payload.get(), cmd.payloadLen);
        }
        
        return cmd;
    }
    
    static uint32_t _SpeedFromBCDUSB(uint16_t bcdUSB) {
        bcdUSB = Endian::HFL_U16(bcdUSB);
        switch (bcdUSB) {
        case 0x0100: return USBIPLib::USB_SPEED_FULL;
        case 0x0110: return USBIPLib::USB_SPEED_FULL;
        case 0x0200: return USBIPLib::USB_SPEED_HIGH;
        case 0x0300: return USBIPLib::USB_SPEED_SUPER;
        case 0x0310: return USBIPLib::USB_SPEED_SUPER_PLUS;
        case 0x0320: return USBIPLib::USB_SPEED_SUPER_PLUS;
        }
        throw RuntimeError("invalid bcdUSB: %x", bcdUSB);
    }
    
    static size_t _DescLen(const USB::DeviceDescriptor& d) {
        return Endian::HFL_U8(d.bLength);
    }
    
    static size_t _DescLen(const USB::ConfigurationDescriptor& d) {
        return Endian::HFL_U16(d.wTotalLength);
    }
    
    static size_t _DescLen(const USB::DeviceQualifierDescriptor& d) {
        return Endian::HFL_U8(d.bLength);
    }
    
    static size_t _DescLen(const USB::StringDescriptor& d) {
        return Endian::HFL_U8(d.bLength);
    }
    
    static uint8_t _ConfigVal(const USB::ConfigurationDescriptor& d) {
        return Endian::HFL_U8(d.bConfigurationValue);
    }
    
    static bool _SelfPowered(const USB::ConfigurationDescriptor& d) {
        const uint8_t bmAttributes = Endian::HFL_U8(d.bmAttributes);
        return bmAttributes & USB::ConfigurationCharacteristics::SelfPowered;
    }
    
    void _readThread() {
        int socket = -1;
        // Copy the socket so we can reference it without the lock.
        // The _reset() logic ensures that the socket won't be closed until this thread exits.
        auto lock = std::unique_lock(_s.lock);
        socket = _s.socket;
        lock.unlock();
        
        try {
            for (;;) {
                _Cmd cmd = _ReadCmd(socket);
                
                lock.lock();
                _s.cmds.push_back(std::move(cmd));
                _s.signal.notify_all();
                lock.unlock();
            }
        
        } catch (const std::exception& e) {
            lock.lock();
            _reset(lock, std::current_exception());
            lock.unlock();
        }
        
        lock.lock();
        _s.state &= ~_State::ReadThreadRunning;
        _s.signal.notify_all();
        lock.unlock();
        
        printf("VirtualUSBDevice: _readThread() exiting\n");
    }
    
    void _writeThread() {
        int socket = -1;
        // Copy the socket so we can reference it without the lock.
        // The _reset() logic ensures that the socket won't be closed until this thread exits.
        auto lock = std::unique_lock(_s.lock);
        socket = _s.socket;
        lock.unlock();
        
        try {
            for (;;) {
                auto lock = std::unique_lock(_s.lock);
                for (;;) {
                    // Bail if there's an error (and therefore we're stopped)
                    if (_s.err) std::rethrow_exception(_s.err);
                    // Break if a reply is available
                    if (!_s.reps.empty()) break;
                    // Otherwise wait to get signalled
                    _s.signal.wait(lock);
                }
                
                // Dequeue the reply
                _Rep rep = std::move(_s.reps.front());
                _s.reps.pop_front();
                lock.unlock();
                
                _Write(socket, &rep.header, sizeof(rep.header));
                _Write(socket, rep.payload.get(), rep.payloadLen);
            }
        
        } catch (const std::exception& e) {
            lock.lock();
            _reset(lock, std::current_exception());
            lock.unlock();
        }
        
        lock.lock();
        _s.state &= ~_State::WriteThreadRunning;
        _s.signal.notify_all();
        lock.unlock();
        
        printf("VirtualUSBDevice: _writeThread() exiting\n");
    }
    
    // _s.lock must be held
    void _reply(const _Cmd& cmd, const void* data, size_t len, int32_t status=0) {
        using namespace Endian;
        
        _Rep rep;
        switch (cmd.header.base.command) {
        case USBIPLib::USBIP_CMD_SUBMIT: {
            // Validate our arguments for SUBMIT replies:
            //   - For IN transfers, either we're sending data (len>0) and have a valid data pointer
            //     (data!=null), or we're not sending data (len==0)
            //   - For OUT transfers, we can't respond with any data, but the `len` argument is used
            //     to populate `actual_length` -- the amount of data sent to the device
            assert(
                (cmd.header.base.direction==USBIPLib::USBIP_DIR_IN && ((len && data) || !len)) ||
                (cmd.header.base.direction==USBIPLib::USBIP_DIR_OUT && !data)
            );
            
            std::unique_ptr<uint8_t[]> payload;
            size_t payloadLen = 0;
            if (cmd.header.base.direction==USBIPLib::USBIP_DIR_IN && len) {
                payloadLen = len;
                payload = std::make_unique<uint8_t[]>(payloadLen);
                memcpy(payload.get(), data, payloadLen);
            }
            
            rep = {
                .header = {
                    .base = {
                        .command            = BFH_U32(USBIPLib::USBIP_RET_SUBMIT),
                        .seqnum             = BFH_U32(cmd.header.base.seqnum),
                        .devid              = BFH_U32(cmd.header.base.devid),
                        .direction          = BFH_U32(cmd.header.base.direction),
                        .ep                 = BFH_U32(cmd.header.base.ep),
                    },
                    
                    .ret_submit = {
                        .status             = BFH_S32(0),
                        .actual_length      = BFH_S32(len),
                        .start_frame        = BFH_S32(0),
                        .number_of_packets  = BFH_S32(0),
                        .error_count        = BFH_S32(0),
                    },
                },
                
                .payload = std::move(payload),
                .payloadLen = payloadLen,
            };
            
            break;
        }
        
        case USBIPLib::USBIP_CMD_UNLINK: {
            rep = {
                .header = {
                    .base = {
                        .command            = BFH_U32(USBIPLib::USBIP_RET_UNLINK),
                        .seqnum             = BFH_U32(cmd.header.base.seqnum),
                        .devid              = BFH_U32(cmd.header.base.devid),
                        .direction          = BFH_U32(cmd.header.base.direction),
                        .ep                 = BFH_U32(cmd.header.base.ep),
                    },
                    
                    .ret_unlink = {
                        .status             = BFH_S32(status),
                    },
                },
            };
            break;
        }
        
        default:
            throw RuntimeError("invalid cmd.header.base.command: %u", cmd.header.base.command);
        }
        
        _s.reps.push_back(std::move(rep));
        _s.signal.notify_all();
    }
    
    std::optional<Xfer> _handleCmd(_Cmd& cmd) {
        switch (cmd.header.base.command) {
        case USBIPLib::USBIP_CMD_SUBMIT:
            if (cmd.header.base.ep == 0) {
                return _handleCmdSubmitEP0(cmd);
            } else {
                return _handleCmdSubmitEPX(cmd);
            }
        
        case USBIPLib::USBIP_CMD_UNLINK:
            _handleCmdUnlink(cmd);
            return std::nullopt;
        
        default:
            throw RuntimeError("invalid USBIP command: %u", (uint32_t)cmd.header.base.command);
        }
    }
    
    std::optional<Xfer> _handleCmdSubmitEP0(_Cmd& cmd) {
        printf("_handleCmdSubmitOut\n");
        const USB::SetupRequest setupReq = _GetSetupRequest(cmd);
        const bool standardType =
            (setupReq.bmRequestType & USB::RequestType::TypeMask) == USB::RequestType::TypeStandard;
        
        // Handle standard requests
        if (standardType) {
            _handleCmdSubmitEP0StandardRequest(cmd, setupReq);
            return std::nullopt;
        
        // Otherwise, handle as a regular endpoint command
        } else {
            auto xfer = _handleCmdSubmitEPX(cmd);
            // Populate the setupReq member, since it's always expected for ep==0
            if (xfer) xfer->setupReq = setupReq;
            return xfer;
        }
    }
    
    std::optional<Xfer> _handleCmdSubmitEPX(_Cmd& cmd) {
        switch (cmd.header.base.direction) {
        // OUT command (data from host->device)
        case USBIPLib::USBIP_DIR_OUT:
            return _handleCmdSubmitEPXOut(cmd);
        
        // IN command (data from device->host)
        case USBIPLib::USBIP_DIR_IN:
            _handleCmdSubmitEPXIn(cmd);
            return std::nullopt;
        
        default:
            throw RuntimeError("invalid Cmd direction");
        }
    }
    
    Xfer _handleCmdSubmitEPXOut(_Cmd& cmd) {
//        printf("_handleCmdSubmitEPXOut\n");
        const uint8_t epIdx = cmd.header.base.ep;
        if (epIdx >= USB::Endpoint::MaxCount) throw RuntimeError("invalid epIdx");
        
        // Let host know that we received the data
        _reply(cmd, nullptr, cmd.payloadLen);
        return Xfer{
            .ep     = _GetEndpointAddr(cmd),
            .data   = std::move(cmd.payload),
            .len    = cmd.payloadLen,
        };
    }
    
    void _handleCmdSubmitEPXIn(_Cmd& cmd) {
//        printf("_handleCmdSubmitEPXIn\n");
        const uint8_t epIdx = cmd.header.base.ep;
        if (epIdx >= USB::Endpoint::MaxCount) throw RuntimeError("invalid epIdx");
        auto& epInCmds = _s.inCmds[epIdx];
        epInCmds.push_back(std::move(cmd));
        _sendDataForInEndpoint(epIdx);
    }
    
    void _sendDataForInEndpoint(uint8_t epIdx) {
        auto& epInCmds = _s.inCmds[epIdx];
        auto& epInData = _s.inData[epIdx];
        
        // Send data while there's data requested and data available
        while (!epInCmds.empty() && !epInData.empty()) {
            const _Cmd& cmd = epInCmds.front();
            _Data& d = epInData.front();
            // Limit the length of data to send by the length requested (transfer_buffer_length),
            // or the length available, whichever is smaller
            const size_t len = std::min((size_t)cmd.header.cmd_submit.transfer_buffer_length, d.len-d.off);
//            printf("_sendDataForInEndpoint for seqnum=%u\n", cmd.header.base.seqnum);
            _reply(cmd, &d.data[d.off], len);
            d.off += len;
            // Pop the command unconditionally
            epInCmds.pop_front();
            // Pop the data if we sent it all
            if (d.off == d.len) {
                epInData.pop_front();
            }
        }
    }
    
    void _handleCmdUnlink(const _Cmd& cmd) {
        printf("_handleCmdUnlink\n");
        const uint8_t epIdx = cmd.header.base.ep;
        if (epIdx >= USB::Endpoint::MaxCount) throw RuntimeError("invalid epIdx");
        
        // Remove the IN cmd from the endpoint's inCmds deque
        bool found = false;
        for (std::deque<_Cmd>& deq : _s.inCmds) {
            for (auto it=deq.begin(); it!=deq.end(); it++) {
                const _Cmd& inCmd = *it;
                if (inCmd.header.base.seqnum == cmd.header.cmd_unlink.seqnum) {
                    deq.erase(it);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        
//        printf("UNLINK seqnum=%u: %d\n", cmd.header.cmd_unlink.seqnum, found);
        
        // status = -ECONNRESET on success
        const int32_t status = (found ? -ECONNRESET : 0);
        _reply(cmd, nullptr, 0, status);
    }
    
    // _s.lock must be held
    void _handleCmdSubmitEP0StandardRequest(const _Cmd& cmd, const USB::SetupRequest& req) {
        using namespace Endian;
        printf("_handleCmdSubmitEP0StandardRequest\n");
        
        // We only support requests to the device for now
        const uint8_t recipient = req.bmRequestType & USB::RequestType::RecipientMask;
        if (recipient != USB::RequestType::RecipientDevice)
            throw RuntimeError("invalid recipient: %u", recipient);
        
        switch (cmd.header.base.direction) {
        // IN command (data from device->host)
        case USBIPLib::USBIP_DIR_IN: {
            switch (req.bRequest) {
            case USB::Request::GetStatus: {
                printf("USB::Request::GetStatus\n");
                if (!_s.configDesc) throw RuntimeError("no active configuration");
                uint16_t reply = 0;
                // If self-powered, bit 0 is 1
                if (_SelfPowered(*_s.configDesc)) reply |= 1;
                reply = LFH_U16(reply);
                _reply(cmd, &reply, sizeof(reply));
                return;
            }
            
            case USB::Request::GetDescriptor: {
                const uint8_t descType = (req.wValue&0xFF00)>>8;
                const uint8_t descIdx = (req.wValue&0x00FF)>>0;
                const void* replyData = nullptr;
                size_t replyDataLen = 0;
                
                switch (descType) {
                case USB::DescriptorType::Device:
                    printf("USB::Request::GetDescriptor::Device\n");
                    replyData = _info.deviceDesc;
                    replyDataLen = _DescLen(*_info.configDescs[descIdx]);
                    break;
                
                case USB::DescriptorType::Configuration:
                    printf("USB::Request::GetDescriptor::Configuration\n");
                    if (descIdx >= _info.configDescsCount)
                        throw RuntimeError("invalid Configuration descriptor index: %u", descIdx);
                    replyData = _info.configDescs[descIdx];
                    replyDataLen = _DescLen(*_info.configDescs[descIdx]);
                    break;
                
                case USB::DescriptorType::String:
                    printf("USB::Request::GetDescriptor::String\n");
                    if (_info.stringDescs) {
                        if (descIdx >= _info.stringDescsCount)
                            throw RuntimeError("invalid String descriptor index: %u", descIdx);
                        replyData = _info.stringDescs[descIdx];
                        replyDataLen = _DescLen(*_info.stringDescs[descIdx]);
                    }
                    break;
                
                case USB::DescriptorType::DeviceQualifier:
                    printf("USB::Request::GetDescriptor::DeviceQualifier\n");
                    if (_info.deviceQualifierDesc) {
                        replyData = _info.deviceQualifierDesc;
                        replyDataLen = _DescLen(*_info.deviceQualifierDesc);
                    }
                    break;
                
                default:
                    // Unsupported descriptor type
                    break;
                }
                
                const int32_t status = (replyData ? 0 : 1);
                // Cap reply length to `wLength` in the original request
                replyDataLen = std::min(replyDataLen, (size_t)req.wLength);
                _reply(cmd, replyData, replyDataLen, status);
                return;
            }
            
            case USB::Request::SetConfiguration: {
                printf("USB::Request::SetConfiguration\n");
                const uint8_t configVal = (req.wValue&0x00FF)>>0;
                
                bool ok = false;
                for (size_t i=0; i<_info.configDescsCount && !ok; i++) {
                    const USB::ConfigurationDescriptor& configDesc = *_info.configDescs[i];
                    if (_ConfigVal(configDesc) == configVal) {
                        _s.configDesc = &configDesc;
                        ok = true;
                    }
                }
                
                if (!ok) throw RuntimeError("invalid Configuration value: %u", configVal);
                _reply(cmd, nullptr, 0);
                return;
            }
            
            default:
                throw RuntimeError("invalid device->host standard request: %u", req.bRequest);
            }
        }
        
        // OUT command (data from host->device)
        case USBIPLib::USBIP_DIR_OUT: {
            const size_t payloadLen = cmd.header.cmd_submit.transfer_buffer_length;
            if (payloadLen) throw RuntimeError("unexpected payload for EP0 standard request");
            
            switch (req.bRequest) {
            case USB::Request::SetConfiguration: {
                printf("USB::Request::SetConfiguration\n");
                const uint8_t configVal = (req.wValue&0x00FF)>>0;
                
                bool ok = false;
                for (size_t i=0; i<_info.configDescsCount && !ok; i++) {
                    const USB::ConfigurationDescriptor& configDesc = *_info.configDescs[i];
                    if (_ConfigVal(configDesc) == configVal) {
                        _s.configDesc = &configDesc;
                        ok = true;
                    }
                }
                
                if (!ok) throw RuntimeError("invalid Configuration value: %u", configVal);
                _reply(cmd, nullptr, payloadLen);
                return;
            }
            
            default:
                throw RuntimeError("invalid host->device standard request: %u", req.bRequest);
            }
        }
        
        default:
            throw RuntimeError("invalid Cmd direction");
        }
    }
    
    // _s.lock must be held
    void _reset(std::unique_lock<std::mutex>& lock, Err err) {
        // Short-circuit if we've already been reset
        if (_s.state & _State::Reset) return;
        _s.state |= _State::Reset;
        _s.err = err;
        _s.signal.notify_all(); // Wake threads so they can observe `_s.err`
        
        auto sockets = {std::ref(_s.socket), std::ref(_s.usbipSocket)};
        
        // Shutdown sockets
        // read() will exit when it sees that the socket is shutdown
        for (const int& s : sockets) {
            if (s >= 0) {
                shutdown(s, SHUT_RDWR);
            }
        }
        
        // Wait until the threads exit
        while (_s.state & (_State::ReadThreadRunning|_State::WriteThreadRunning)) {
            _s.signal.wait(lock);
        }
        
        // Close sockets now that the thread has exited
        for (int& s : sockets) {
            if (s >= 0) {
                close(s);
                s = -1;
            }
        }
    }
    
    const Info _info = {};
    
    struct {
        std::mutex lock; // Struct should only be accessed while holding lock
        std::condition_variable signal;
        uint8_t state = _State::Idle;
        Err err;
        int socket = -1;
        int usbipSocket = -1;
        const USB::ConfigurationDescriptor* configDesc = nullptr;
        
        std::deque<_Cmd> cmds;
        std::deque<_Cmd> reps;
        
        std::deque<_Cmd> inCmds[USB::Endpoint::MaxCount];
        std::deque<_Data> inData[USB::Endpoint::MaxCount];
    } _s = {};
};
