// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005-2007 Takahiro Hirofuchi
 */

#pragma once
#include <cstdint>
#include <libudev.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

namespace USBIPLib {

constexpr uint32_t USBIP_CMD_SUBMIT = 1;
constexpr uint32_t USBIP_CMD_UNLINK = 2;
constexpr uint32_t USBIP_RET_SUBMIT = 3;
constexpr uint32_t USBIP_RET_UNLINK = 4;

constexpr uint32_t USBIP_DIR_OUT    = 0;
constexpr uint32_t USBIP_DIR_IN     = 1;

constexpr size_t SYSFS_PATH_MAX = 256;
constexpr size_t SYSFS_BUS_ID_SIZE = 32;
constexpr size_t MAX_STATUS_NAME = 18;
constexpr char USBIP_VHCI_BUS_TYPE[] = "platform";
constexpr char USBIP_VHCI_DEVICE_NAME[] = "vhci_hcd.0";

static void err(const char* str) {}

template <typename ...Args>
static void err(const char* fmt, Args&& ...args) {}

template <typename ...Args>
static void dbg(const char* fmt, Args&& ...args) {
}

static void BUG() {
    abort();
}

enum hub_speed {
    HUB_SPEED_HIGH = 0,
    HUB_SPEED_SUPER,
};

enum usbip_status {
    /* sdev is available. */
    SDEV_ST_AVAILABLE = 0x01,
    /* sdev is now used. */
    SDEV_ST_USED,
    /* sdev is unusable because of a fatal error. */
    SDEV_ST_ERROR,

    /* vdev does not connect a remote device. */
    VDEV_ST_NULL,
    /* vdev is used, but the USB address is not assigned yet */
    VDEV_ST_NOTASSIGNED,
    VDEV_ST_USED,
    VDEV_ST_ERROR
};

struct usbip_usb_device {
    char path[SYSFS_PATH_MAX];
    char busid[SYSFS_BUS_ID_SIZE];

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

struct usbip_imported_device {
    enum hub_speed hub;
    uint8_t port;
    uint32_t status;

    uint32_t devid;

    uint8_t busnum;
    uint8_t devnum;

    /* usbip_class_device list */
    struct usbip_usb_device udev;
};

struct usbip_vhci_driver {

    /* /sys/devices/platform/vhci_hcd */
    struct udev_device *hc_device;

    int ncontrollers;
    int nports;
    struct usbip_imported_device idev[];
};

enum usb_device_speed {
    USB_SPEED_UNKNOWN = 0,            /* enumerating */
    USB_SPEED_LOW, USB_SPEED_FULL,        /* usb 1.1 */
    USB_SPEED_HIGH,                /* usb 2.0 */
    USB_SPEED_WIRELESS,            /* wireless (usb 2.5) */
    USB_SPEED_SUPER,            /* usb 3.0 */
    USB_SPEED_SUPER_PLUS,            /* usb 3.1 */
};

struct speed_string {
    int num;
    const char *speed;
    const char *desc;
};

static const struct speed_string speed_strings[] = {
    { USB_SPEED_UNKNOWN, "unknown", "Unknown Speed"},
    { USB_SPEED_LOW,  "1.5", "Low Speed(1.5Mbps)"  },
    { USB_SPEED_FULL, "12",  "Full Speed(12Mbps)" },
    { USB_SPEED_HIGH, "480", "High Speed(480Mbps)" },
    { USB_SPEED_WIRELESS, "53.3-480", "Wireless"},
    { USB_SPEED_SUPER, "5000", "Super Speed(5000Mbps)" },
    { 0, NULL, NULL }
};

struct usbip_vhci_driver *vhci_driver;
struct udev *udev_context;

int usbip_net_set_keepalive(int sockfd)
{
    const int val = 1;
    int ret;

    ret = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val));
    if (ret < 0)
        dbg("setsockopt: SO_KEEPALIVE");

    return ret;
}

int write_sysfs_attribute(const char *attr_path, const char *new_value, size_t len)
{
    int fd;
    int length;

    fd = open(attr_path, O_WRONLY);
    if (fd < 0) {
        dbg("error opening attribute %s", attr_path);
        return -1;
    }

    length = write(fd, new_value, len);
    if (length < 0) {
        dbg("error writing to attribute %s", attr_path);
        close(fd);
        return -1;
    }

    close(fd);

    return 0;
}


int read_attr_speed(struct udev_device *dev)
{
    const char *speed;

    speed = udev_device_get_sysattr_value(dev, "speed");
    if (!speed) {
        err("udev_device_get_sysattr_value failed");
        goto err;
    }

    for (int i = 0; speed_strings[i].speed != NULL; i++) {
        if (!strcmp(speed, speed_strings[i].speed))
            return speed_strings[i].num;
    }

err:

    return USB_SPEED_UNKNOWN;
}

int read_attr_value(struct udev_device *dev, const char *name,
            const char *format)
{
    const char *attr;
    int num = 0;
    int ret;

    attr = udev_device_get_sysattr_value(dev, name);
    if (!attr) {
        err("udev_device_get_sysattr_value failed");
        goto err;
    }

    /* The client chooses the device configuration
     * when attaching it so right after being bound
     * to usbip-host on the server the device will
     * have no configuration.
     * Therefore, attributes such as bConfigurationValue
     * and bNumInterfaces will not exist and sscanf will
     * fail. Check for these cases and don't treat them
     * as errors.
     */

    ret = sscanf(attr, format, &num);
    if (ret < 1) {
        if (strcmp(name, "bConfigurationValue") &&
                strcmp(name, "bNumInterfaces")) {
            err("sscanf failed for attribute %s", name);
            goto err;
        }
    }

err:

    return num;
}

int read_usb_device(struct udev_device *sdev, struct usbip_usb_device *udev)
{
    uint32_t busnum, devnum;
    const char *path, *name;
    
#define to_string(s) #s
#define READ_ATTR(object, type, dev, name, format)                                \
    do {                                                                        \
        (object)->name = (type) read_attr_value(dev, to_string(name), format);  \
    } while (0)
    
    READ_ATTR(udev, uint8_t, sdev, bDeviceClass, "%02x\n");
    READ_ATTR(udev, uint8_t, sdev, bDeviceSubClass, "%02x\n");
    READ_ATTR(udev, uint8_t, sdev, bDeviceProtocol, "%02x\n");

    READ_ATTR(udev, uint16_t, sdev, idVendor, "%04x\n");
    READ_ATTR(udev, uint16_t, sdev, idProduct, "%04x\n");
    READ_ATTR(udev, uint16_t, sdev, bcdDevice, "%04x\n");

    READ_ATTR(udev, uint8_t, sdev, bConfigurationValue, "%02x\n");
    READ_ATTR(udev, uint8_t, sdev, bNumConfigurations,    "%02x\n");
    READ_ATTR(udev, uint8_t, sdev, bNumInterfaces, "%02x\n");

    READ_ATTR(udev, uint8_t, sdev, devnum, "%d\n");

#undef READ_ATTR
#undef to_string
   
    udev->speed = read_attr_speed(sdev);

    path = udev_device_get_syspath(sdev);
    name = udev_device_get_sysname(sdev);

    strncpy(udev->path,  path,  SYSFS_PATH_MAX - 1);
    udev->path[SYSFS_PATH_MAX - 1] = '\0';
    strncpy(udev->busid, name, SYSFS_BUS_ID_SIZE - 1);
    udev->busid[SYSFS_BUS_ID_SIZE - 1] = '\0';

    sscanf(name, "%u-%u", &busnum, &devnum);
    udev->busnum = busnum;

    return 0;
}

static struct usbip_imported_device *
imported_device_init(struct usbip_imported_device *idev, char *busid)
{
    struct udev_device *sudev;

    sudev = udev_device_new_from_subsystem_sysname(udev_context,
                               "usb", busid);
    if (!sudev) {
        dbg("udev_device_new_from_subsystem_sysname failed: %s", busid);
        goto err;
    }
    read_usb_device(sudev, &idev->udev);
    udev_device_unref(sudev);

    return idev;

err:
    return NULL;
}

static int get_nports(struct udev_device *hc_device)
{
    const char *attr_nports;

    attr_nports = udev_device_get_sysattr_value(hc_device, "nports");
    if (!attr_nports) {
        err("udev_device_get_sysattr_value nports failed");
        return -1;
    }

    return (int)strtoul(attr_nports, NULL, 10);
}

static int vhci_hcd_filter(const struct dirent *dirent)
{
    return !strncmp(dirent->d_name, "vhci_hcd.", 9);
}

static int get_ncontrollers(void)
{
    struct dirent **namelist;
    struct udev_device *platform;
    int n;

    platform = udev_device_get_parent(vhci_driver->hc_device);
    if (platform == NULL)
        return -1;

    n = scandir(udev_device_get_syspath(platform), &namelist, vhci_hcd_filter, NULL);
    if (n < 0)
        err("scandir failed");
    else {
        for (int i = 0; i < n; i++)
            free(namelist[i]);
        free(namelist);
    }

    return n;
}

static int parse_status(const char *value)
{
    int ret = 0;
    char *c;

    /* skip a header line */
    c = strchr((char*)value, '\n');
    if (!c)
        return -1;
    c++;

    while (*c != '\0') {
        int port, status, speed, devid;
        int sockfd;
        char lbusid[SYSFS_BUS_ID_SIZE];
        struct usbip_imported_device *idev;
        char hub[3];

        ret = sscanf(c, "%2s  %d %d %d %x %u %31s\n",
                hub, &port, &status, &speed,
                &devid, &sockfd, lbusid);

        if (ret < 5) {
            dbg("sscanf failed: %d", ret);
            BUG();
        }

        dbg("hub %s port %d status %d speed %d devid %x",
                hub, port, status, speed, devid);
        dbg("sockfd %u lbusid %s", sockfd, lbusid);

        /* if a device is connected, look at it */
        idev = &vhci_driver->idev[port];
        memset(idev, 0, sizeof(*idev));

        if (strncmp("hs", hub, 2) == 0)
            idev->hub = HUB_SPEED_HIGH;
        else /* strncmp("ss", hub, 2) == 0 */
            idev->hub = HUB_SPEED_SUPER;

        idev->port    = port;
        idev->status    = status;

        idev->devid    = devid;

        idev->busnum    = (devid >> 16);
        idev->devnum    = (devid & 0x0000ffff);

        if (idev->status != VDEV_ST_NULL
            && idev->status != VDEV_ST_NOTASSIGNED) {
            idev = imported_device_init(idev, lbusid);
            if (!idev) {
                dbg("imported_device_init failed");
                return -1;
            }
        }

        /* go to the next line */
        c = strchr(c, '\n');
        if (!c)
            break;
        c++;
    }

    dbg("exit");

    return 0;
}

static int refresh_imported_device_list(void)
{
    const char *attr_status;
    char status[MAX_STATUS_NAME+1] = "status";
    int i, ret;

    for (i = 0; i < vhci_driver->ncontrollers; i++) {
        if (i > 0)
            snprintf(status, sizeof(status), "status.%d", i);

        attr_status = udev_device_get_sysattr_value(vhci_driver->hc_device,
                                status);
        if (!attr_status) {
            err("udev_device_get_sysattr_value failed");
            return -1;
        }

        dbg("controller %d", i);

        ret = parse_status(attr_status);
        if (ret != 0)
            return ret;
    }

    return 0;
}

int usbip_vhci_driver_open(void)
{
    int nports;
    struct udev_device *hc_device;

    udev_context = udev_new();
    if (!udev_context) {
        err("udev_new failed");
        return -1;
    }

    /* will be freed in usbip_driver_close() */
    hc_device =
        udev_device_new_from_subsystem_sysname(udev_context,
                               USBIP_VHCI_BUS_TYPE,
                               USBIP_VHCI_DEVICE_NAME);
    if (!hc_device) {
        err("udev_device_new_from_subsystem_sysname failed");
        goto err;
    }

    nports = get_nports(hc_device);
    if (nports <= 0) {
        err("no available ports");
        goto err;
    }
    dbg("available ports: %d", nports);

    vhci_driver = (USBIPLib::usbip_vhci_driver*)calloc(1, sizeof(struct usbip_vhci_driver) +
            nports * sizeof(struct usbip_imported_device));
    if (!vhci_driver) {
        err("vhci_driver allocation failed");
        goto err;
    }

    vhci_driver->nports = nports;
    vhci_driver->hc_device = hc_device;
    vhci_driver->ncontrollers = get_ncontrollers();
    dbg("available controllers: %d", vhci_driver->ncontrollers);

    if (vhci_driver->ncontrollers <=0) {
        err("no available usb controllers");
        goto err;
    }

    if (refresh_imported_device_list())
        goto err;

    return 0;

err:
    udev_device_unref(hc_device);

    if (vhci_driver)
        free(vhci_driver);

    vhci_driver = NULL;

    udev_unref(udev_context);

    return -1;
}

int usbip_vhci_get_free_port(uint32_t speed)
{
    for (int i = 0; i < vhci_driver->nports; i++) {

        switch (speed) {
        case    USB_SPEED_SUPER:
            if (vhci_driver->idev[i].hub != HUB_SPEED_SUPER)
                continue;
        break;
        default:
            if (vhci_driver->idev[i].hub != HUB_SPEED_HIGH)
                continue;
        break;
        }

        if (vhci_driver->idev[i].status == VDEV_ST_NULL)
            return vhci_driver->idev[i].port;
    }

    return -1;
}

int usbip_vhci_attach_device2(uint8_t port, int sockfd, uint32_t devid, uint32_t speed) {
    char buff[200]; /* what size should be ? */
    char attach_attr_path[SYSFS_PATH_MAX];
    char attr_attach[] = "attach";
    const char *path;
    int ret;

    snprintf(buff, sizeof(buff), "%u %d %u %u",
            port, sockfd, devid, speed);
    dbg("writing: %s", buff);

    path = udev_device_get_syspath(vhci_driver->hc_device);
    snprintf(attach_attr_path, sizeof(attach_attr_path), "%s/%s",
         path, attr_attach);
    dbg("attach attribute path: %s", attach_attr_path);

    ret = write_sysfs_attribute(attach_attr_path, buff, strlen(buff));
    if (ret < 0) {
        dbg("write_sysfs_attribute failed");
        return -1;
    }

    dbg("attached port: %d", port);

    return 0;
}

void usbip_vhci_driver_close(void)
{
	if (!vhci_driver)
		return;

	udev_device_unref(vhci_driver->hc_device);

	free(vhci_driver);

	vhci_driver = NULL;

	udev_unref(udev_context);
}

} // namespace USBIPLib
