#pragma once
#include <cstdint>
#include "Endian.h"

namespace USB {

// Universal Serial Bus Specification
// Revision 2.0

// Descriptor Type (bDescriptorType, wValue [high byte])
namespace DescriptorType {
    static constexpr uint8_t Device                     = 1;
    static constexpr uint8_t Configuration              = 2;
    static constexpr uint8_t String                     = 3;
    static constexpr uint8_t Interface                  = 4;
    static constexpr uint8_t Endpoint                   = 5;
    static constexpr uint8_t DeviceQualifier            = 6;
    static constexpr uint8_t OtherSpeedConfiguration    = 7;
    static constexpr uint8_t InterfacePower             = 8;
};

// Request (bRequest)
namespace Request {
    static constexpr uint8_t GetStatus                  = 0;
    static constexpr uint8_t ClearFeature               = 1;
    static constexpr uint8_t _Reserved0                 = 2;
    static constexpr uint8_t SetFeature                 = 3;
    static constexpr uint8_t _Reserved1                 = 4;
    static constexpr uint8_t SetAddress                 = 5;
    static constexpr uint8_t GetDescriptor              = 6;
    static constexpr uint8_t SetDescriptor              = 7;
    static constexpr uint8_t GetConfiguration           = 8;
    static constexpr uint8_t SetConfiguration           = 9;
    static constexpr uint8_t GetInterface               = 10;
    static constexpr uint8_t SetInterface               = 11;
    static constexpr uint8_t SynchFrame                 = 12;
};

// Request Type (bmRequestType)
namespace RequestType {
    static constexpr uint8_t DirectionHostToDevice      = 0x00;
    static constexpr uint8_t DirectionDeviceToHost      = 0x80;
    static constexpr uint8_t DirectionMask              = 0x80;
    
    static constexpr uint8_t TypeStandard               = 0x00;
    static constexpr uint8_t TypeClass                  = 0x20;
    static constexpr uint8_t TypeVendor                 = 0x40;
    static constexpr uint8_t TypeReserved               = 0x60;
    static constexpr uint8_t TypeMask                   = 0x60;
    
    static constexpr uint8_t RecipientDevice            = 0x00;
    static constexpr uint8_t RecipientInterface         = 0x01;
    static constexpr uint8_t RecipientEndpoint          = 0x02;
    static constexpr uint8_t RecipientOther             = 0x03;
    static constexpr uint8_t RecipientMask              = 0x1F;
};

namespace Endpoint {
    static constexpr uint8_t MaxCount                   = 16; // Max number of endpoints
    static constexpr uint8_t DefaultOut                 = 0x00;
    static constexpr uint8_t DefaultIn                  = 0x80;
    
    static constexpr uint8_t DirIn                      = 0x80;
    static constexpr uint8_t DirOut                     = 0x00;
    static constexpr uint8_t DirMask                    = 0x80;
    
    static constexpr uint8_t IdxMask                    = 0x0F;
};

struct DeviceDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} __attribute__((packed));

namespace ConfigurationCharacteristics {
    static constexpr uint8_t RemoteWakeup   = 1<<5;
    static constexpr uint8_t SelfPowered    = 1<<6;
};

struct ConfigurationDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} __attribute__((packed));

struct InterfaceDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __attribute__((packed));

struct EndpointDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} __attribute__((packed));

struct DeviceQualifierDescriptor {
    uint8_t bLength;
    uint8_t bType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint8_t bNumConfigurations;
    uint8_t bReserved;
} __attribute__((packed));

struct StringDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
} __attribute__((packed));

template <size_t N>
struct StringDescriptorN : StringDescriptor {
    uint16_t str[N] = {};
    
    constexpr StringDescriptorN(const char (&s)[N+1]) :
    StringDescriptor({.bLength=(2*N+2), .bDescriptorType=DescriptorType::String}) {
        for (size_t i=0; i<N; i++) {
            str[i] = Endian::LFH_U16(s[i]);
        }
    }
} __attribute__((packed));

template <size_t N>
constexpr auto StringDescriptorMake(const char (&str)[N]) {
    return StringDescriptorN<N-1>(str);
}

template <size_t N>
struct SupportedLanguagesDescriptorN : StringDescriptor {
    uint16_t langs[N] = {};
    
    constexpr SupportedLanguagesDescriptorN(const uint16_t (&l)[N]) :
    StringDescriptor({.bLength=(2*N+2), .bDescriptorType=DescriptorType::String}) {
        for (size_t i=0; i<N; i++) {
            langs[i] = Endian::LFH_U16(l[i]);
        }
    }
} __attribute__((packed));

template <size_t N>
constexpr auto SupportedLanguagesDescriptorMake(const uint16_t (&langs)[N]) {
    return SupportedLanguagesDescriptorN<N>(langs);
}

struct SetupRequest {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed));

} // namespace USB

namespace USB::CDC {

// Universal Serial Bus Class Definitions for Communication Devices
// Version 1.1

namespace Request {
    static constexpr uint8_t SEND_ENCAPSULATED_COMMAND   = 0x00;
    static constexpr uint8_t GET_ENCAPSULATED_RESPONSE   = 0x01;
    static constexpr uint8_t SET_COMM_FEATURE            = 0x02;
    static constexpr uint8_t GET_COMM_FEATURE            = 0x03;
    static constexpr uint8_t CLEAR_COMM_FEATURE          = 0x04;
    static constexpr uint8_t SET_LINE_CODING             = 0x20;
    static constexpr uint8_t GET_LINE_CODING             = 0x21;
    static constexpr uint8_t SET_CONTROL_LINE_STATE      = 0x22;
    static constexpr uint8_t SEND_BREAK                  = 0x23;
};

struct HeaderFunctionalDescriptor {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint16_t bcdCDC;
} __attribute__((packed));

struct AbstractControlManagementFunctionalDescriptor {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmCapabilities;
} __attribute__((packed));

struct UnionFunctionalDescriptor {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bMasterInterface;
    uint8_t bSlaveInterface0;
} __attribute__((packed));

struct CallManagementFunctionalDescriptor {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmCapabilities;
    uint8_t bDataInterface;
} __attribute__((packed));

struct LineCoding {
    uint32_t dwDTERate;
    uint8_t bCharFormat;
    uint8_t bParityType;
    uint8_t bDataBits;
} __attribute__((packed));

} // namespace USB::CDC
