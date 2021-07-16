#pragma once
#include <cstdint>

namespace USBIP {

struct OP_REQ_DEVLIST {
    uint16_t version;
    uint16_t command;
    uint32_t status;
} __attribute__((packed));

struct OP_REP_DEVLIST_HEADER {
    uint16_t version;
    uint16_t reply;
    uint32_t status;
    uint32_t deviceCount;
} __attribute__((packed));

struct OP_REP_DEVLIST_DEVICE {
    uint8_t path[256];
    uint8_t busid[32];
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bConfigurationValue;
    uint8_t bNumConfigurations; 
    uint8_t bNumInterfaces;
} __attribute__((packed));

struct OP_REP_DEVLIST_INTERFACE {
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t pad;
} __attribute__((packed));

struct OP_REQ_IMPORT {
    uint16_t version;
    uint16_t command;
    uint32_t status;
    uint8_t busid[32];
} __attribute__((packed));

struct OP_REP_IMPORT_HEADER {
    uint16_t version;
    uint16_t reply;
    uint32_t status;
} __attribute__((packed));

struct OP_REP_IMPORT_PAYLOAD {
    uint8_t path[256];
    uint8_t busid[32];
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bConfigurationValue;
    uint8_t bNumConfigurations;
    uint8_t bNumInterfaces;
} __attribute__((packed));

// usbip_header_basic
struct HEADER_BASIC {
    uint32_t command;
    uint32_t seqnum;
    uint32_t devid;
    uint32_t direction;
    uint32_t ep;
} __attribute__((packed));

// usbip_header_cmd_submit
struct HEADER_CMD_SUBMIT {
    uint32_t transfer_flags;
    int32_t transfer_buffer_length;
    int32_t start_frame;
    int32_t number_of_packets;
    int32_t interval;
    union {
        uint8_t u8[8];
        uint64_t u64;
    } setup;
} __attribute__((packed));

// usbip_header_ret_submit
struct HEADER_RET_SUBMIT {
    int32_t status;
    int32_t actual_length;
    int32_t start_frame;
    int32_t number_of_packets;
    int32_t error_count;
} __attribute__((packed));

// usbip_header_cmd_unlink
struct HEADER_CMD_UNLINK {
    uint32_t seqnum;
} __attribute__((packed));

// usbip_header_ret_unlink
struct HEADER_RET_UNLINK {
    int32_t status;
} __attribute__((packed));

// usbip_header
struct HEADER {
    struct HEADER_BASIC base;
    union {
        HEADER_CMD_SUBMIT cmd_submit;
        HEADER_RET_SUBMIT ret_submit;
        HEADER_CMD_UNLINK cmd_unlink;
        HEADER_RET_UNLINK ret_unlink;
    };
} __attribute__((packed));
static_assert(sizeof(HEADER) == 48);

} // namespace USBIP
