/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

/*
 * USB descriptors for the bridge: an ncm ethernet port and a cdc serial
 * port, both always present. The host may use either or both; the serial
 * port needs no setup at all, which is what makes the network side of the
 * bridge optional rather than mandatory.
 *
 * The product id differs from the cdc-only one the bootloader still uses,
 * so a host never has to tell the two apart by their product string, and
 * never tries a cached driver binding from the other.
 */

#include "tusb.h"
#include "app_main.h"

#define USB_VID             0x2e3c
#define USB_PID             0x5742  // 0x5740 is the bootloader, 0x5741 ncm only
#define USB_BCD_DEVICE      0x0200

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_INTERFACE,
    STRID_MAC,
    STRID_CDC,
    STRID_COUNT
};

enum {
    ITF_NUM_NCM = 0,    // control
    ITF_NUM_NCM_DATA,
    ITF_NUM_CDC,        // control
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_NET_NOTIF     0x81
#define EPNUM_NET_OUT       0x02
#define EPNUM_NET_IN        0x82
#define EPNUM_CDC_NOTIF     0x83
#define EPNUM_CDC_OUT       0x04
#define EPNUM_CDC_IN        0x84


//--------------------------------------------------------------------
// device descriptor
//--------------------------------------------------------------------

// both functions are described by an interface association descriptor, so
// the device has to be declared as a miscellaneous / interface association
// device. windows rejects the descriptor set otherwise.
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = USB_BCD_DEVICE,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = 1
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&desc_device;
}


//--------------------------------------------------------------------
// configuration descriptor
//--------------------------------------------------------------------

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_NCM_DESC_LEN + \
                             TUD_CDC_DESC_LEN)

// the same set for both speeds, only the bulk packet size differs: 512 is
// the only legal size at high speed and 64 the largest at full speed, which
// is what the port gets behind a usb 1.1 hub or on a full speed host
#define CONFIG_DESCRIPTOR(_bulk_size) \
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100), \
    TUD_CDC_NCM_DESCRIPTOR(ITF_NUM_NCM, STRID_INTERFACE, STRID_MAC, \
            EPNUM_NET_NOTIF, 64, EPNUM_NET_OUT, EPNUM_NET_IN, _bulk_size, \
            CFG_TUD_NET_MTU, 9, \
            NCM_NETWORK_CAPS_ETH_FILTER | NCM_NETWORK_CAPS_NTB_INPUT_SIZE), \
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 64, \
            EPNUM_CDC_OUT, EPNUM_CDC_IN, _bulk_size)

static const uint8_t desc_hs_configuration[] = { CONFIG_DESCRIPTOR(512) };
static const uint8_t desc_fs_configuration[] = { CONFIG_DESCRIPTOR(64) };

TU_VERIFY_STATIC(sizeof(desc_hs_configuration) == CONFIG_TOTAL_LEN, "bad config size");

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return tud_speed_get() == TUSB_SPEED_HIGH ?
            desc_hs_configuration : desc_fs_configuration;
}

// a high speed capable device is asked for these two; without them a host
// that has to fall back to full speed is left with 512 byte bulk endpoints
// it cannot use
static const tusb_desc_device_qualifier_t desc_device_qualifier = {
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved          = 0
};

const uint8_t *tud_descriptor_device_qualifier_cb(void)
{
    return (const uint8_t *)&desc_device_qualifier;
}

static uint8_t desc_other_speed[CONFIG_TOTAL_LEN];

const uint8_t *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    (void)index;
    // the set for the speed we are not running at, with the type the host
    // asked for; tinyusb hands the buffer over as is
    memcpy(desc_other_speed, tud_speed_get() == TUSB_SPEED_HIGH ?
            desc_fs_configuration : desc_hs_configuration, CONFIG_TOTAL_LEN);
    desc_other_speed[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
    return desc_other_speed;
}


//--------------------------------------------------------------------
// bos / microsoft os 2.0 descriptor
//--------------------------------------------------------------------

/*
 * Windows 11 has an in-box ncm driver but only binds it automatically when
 * the device reports the WINNCM compatible id here. Without this the port
 * shows up as an unknown device and has to be bound by hand.
 */

#define MS_OS_20_DESC_LEN   0xb2
#define BOS_TOTAL_LEN       (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define MS_OS_20_VENDOR_REQ 7

static const uint8_t desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, MS_OS_20_VENDOR_REQ)
};

const uint8_t *tud_descriptor_bos_cb(void)
{
    return desc_bos;
}

static const uint8_t desc_ms_os_20[] = {
    // set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000a), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // configuration subset header: length, type, config index, reserved, length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0a),

    // function subset header: length, type, first interface, reserved, length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    ITF_NUM_NCM, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0a - 0x08),

    // compatible id: length, type, compatible id, sub compatible id
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'N', 'C', 'M', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // registry property: DeviceInterfaceGUIDs
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0a - 0x08 - 0x08 - 0x14),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002a),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050),
    // {c1c0a5e4-3a7f-4a1d-9f2e-6b0d9a8f4c11}
    '{', 0x00, 'c', 0x00, '1', 0x00, 'c', 0x00, '0', 0x00, 'a', 0x00,
    '5', 0x00, 'e', 0x00, '4', 0x00, '-', 0x00, '3', 0x00, 'a', 0x00,
    '7', 0x00, 'f', 0x00, '-', 0x00, '4', 0x00, 'a', 0x00, '1', 0x00,
    'd', 0x00, '-', 0x00, '9', 0x00, 'f', 0x00, '2', 0x00, 'e', 0x00,
    '-', 0x00, '6', 0x00, 'b', 0x00, '0', 0x00, 'd', 0x00, '9', 0x00,
    'a', 0x00, '8', 0x00, 'f', 0x00, '4', 0x00, 'c', 0x00, '1', 0x00,
    '1', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "bad ms os 2.0 size");

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
        const tusb_control_request_t *request)
{
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
            request->bRequest == MS_OS_20_VENDOR_REQ && request->wIndex == 7) {
        uint16_t total_len;
        memcpy(&total_len, desc_ms_os_20 + 8, 2);
        return tud_control_xfer(rhport, request,
                (void *)(uintptr_t)desc_ms_os_20, total_len);
    }
    return false;
}


//--------------------------------------------------------------------
// string descriptors
//--------------------------------------------------------------------

static const char *const string_desc_arr[STRID_COUNT] = {
    [STRID_LANGID]       = (const char[]){0x09, 0x04},
    [STRID_MANUFACTURER] = "DUKELEC",
    [STRID_PRODUCT]      = "CDBUS Bridge",
    [STRID_SERIAL]       = NULL, // from the cpu id
    [STRID_INTERFACE]    = "CDBUS Bridge Network",
    [STRID_MAC]          = NULL, // from tud_network_mac_address
    [STRID_CDC]          = "CDBUS Bridge Serial"
};

static uint16_t desc_str[32 + 1];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    const char hex[] = "0123456789ABCDEF";
    unsigned chr_count = 0;

    switch (index) {
    case STRID_LANGID:
        memcpy(&desc_str[1], string_desc_arr[STRID_LANGID], 2);
        chr_count = 1;
        break;

    case STRID_SERIAL:
        for (unsigned i = 0; i < 12; i++) {
            uint8_t val = *((const uint8_t *)CPU_UID_ADDR + i);
            desc_str[1 + chr_count++] = hex[val >> 4];
            desc_str[1 + chr_count++] = hex[val & 0xf];
        }
        break;

    case STRID_MAC:
        // the host uses this as the mac of its end of the link
        for (unsigned i = 0; i < 6; i++) {
            desc_str[1 + chr_count++] = hex[tud_network_mac_address[i] >> 4];
            desc_str[1 + chr_count++] = hex[tud_network_mac_address[i] & 0xf];
        }
        break;

    default: {
        if (index >= STRID_COUNT || !string_desc_arr[index])
            return NULL;
        const char *str = string_desc_arr[index];
        chr_count = min(strlen(str), (sizeof(desc_str) / sizeof(desc_str[0])) - 1);
        for (unsigned i = 0; i < chr_count; i++)
            desc_str[1 + i] = str[i];
        break;
    }
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
