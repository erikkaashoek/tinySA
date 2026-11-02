/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"
#include "nanovna.h"
#ifdef __USE_USB_MSC__
#include "usb_msc.h"
#endif

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*
 * Endpoints to be used for USBD1.
 */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1
#define USBD1_INTERRUPT_REQUEST_EP      2

/*
 * USB vendor and product ID: the tinySA ULTRA has its own, the tinySA keeps
 * the ST virtual COM port pair.
 */
#ifdef TINYSA4
#define USB_VID                         0x1781
#define USB_PID                         0x0A44
#else
#define USB_VID                         0x0483  /* ST */
#define USB_PID                         0x5740
#endif

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
#ifdef __USE_USB_MSC__
  /* Composite device: CDC (serial) + mass storage (SD card).*/
  USB_DESC_DEVICE       (0x0200,        /* bcdUSB (2.0), for the IAD.       */
                         0xEF,          /* bDeviceClass (Miscellaneous).    */
                         0x02,          /* bDeviceSubClass (Common Class).  */
                         0x01,          /* bDeviceProtocol (IAD).           */
                         0x40,          /* bMaxPacketSize.                  */
                         USB_VID,       /* idVendor.                        */
                         USB_PID,       /* idProduct.                       */
                         0x0201,        /* bcdDevice, changed with the
                                           composite layout so that Windows
                                           selects the drivers again.       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
#else
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         USB_VID,       /* idVendor.                        */
                         USB_PID,       /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
#endif
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC (and mass storage).*/
#ifdef __USE_USB_MSC__
#define VCOM_CONFIGURATION_SIZE 98
#define VCOM_INTERFACES         3
#else
#define VCOM_CONFIGURATION_SIZE 67
#define VCOM_INTERFACES         2
#endif
static const uint8_t vcom_configuration_descriptor_data[VCOM_CONFIGURATION_SIZE] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(VCOM_CONFIGURATION_SIZE, /* wTotalLength.          */
                         VCOM_INTERFACES, /* bNumInterfaces.                */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
#ifdef __USE_USB_MSC__
  /* Interface Association Descriptor: interfaces 0 and 1 are the CDC
     function (protocol 0 as in the interface descriptor).*/
  USB_DESC_INTERFACE_ASSOCIATION(0x00,  /* bFirstInterface.                 */
                         0x02,          /* bInterfaceCount.                 */
                         0x02,          /* bFunctionClass (CDC).            */
                         0x02,          /* bFunctionSubClass (ACM).         */
                         0x00,          /* bFunctionProtocol.               */
                         0),            /* iInterface.                      */
#endif
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x00,          /* bInterfaceProtocol (No protocol,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
#ifdef __USE_USB_MSC__
  /* Mass storage Interface Descriptor.*/
  USB_DESC_INTERFACE    (MSC_IF,        /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x08,          /* bInterfaceClass (Mass Storage).  */
                         0x06,          /* bInterfaceSubClass (SCSI
                                           transparent command set).        */
                         0x50,          /* bInterfaceProtocol (Bulk-Only
                                           Transport).                      */
                         0),            /* iInterface.                      */
  /* Mass storage bulk OUT Endpoint Descriptor.*/
  USB_DESC_ENDPOINT     (MSC_EP,        /* bEndpointAddress.                */
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Mass storage bulk IN Endpoint Descriptor.*/
  USB_DESC_ENDPOINT     (MSC_EP|0x80,   /* bEndpointAddress.                */
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
#endif
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(22),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  't', 0, 'i', 0, 'n', 0, 'y', 0, 's', 0, 'a', 0, '.', 0, 'o', 0, 'r', 0, 'g', 0
};

#ifdef TINYSA4
/*
 * Device Description string, use "tinySA4" as in "VERSION"
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(16),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  't', 0, 'i', 0, 'n', 0, 'y', 0, 'S', 0, 'A', 0, '4', 0
};
#else
/*
 * Device Description string, use "tinySA"
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(14),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  't', 0, 'i', 0, 'n', 0, 'y', 0, 'S', 0, 'A', 0
};
#endif

/*
 * Serial Number string: 12 hex digits made from the MCU unique ID the way the STM32F303 DFU
 * bootloader makes its serial number (UID0 + UID2, then the upper half of UID2), so every tinySA
 * has its own. It was the kernel version "400" on all of them: Windows could not tell two units
 * apart and gave one unit's COM port to the other. Filled at every USB reset, before it is requested.
 */
static uint8_t vcom_string3[2 + 2 * 12];       // the upper bytes of the UTF-16 characters stay 0

static void fill_serial_string(void)
{
  const uint32_t *uid = (const uint32_t *)UID_BASE;
  uint32_t v = uid[0] + uid[2];
  vcom_string3[0] = sizeof vcom_string3;
  vcom_string3[1] = USB_DESCRIPTOR_STRING;
  for (int i = 2; i < (int)sizeof vcom_string3; i += 2) {
    if (i == 2 + 2 * 8)                        // 8 digits done: then the upper half of UID2
      v = uid[2];
    uint8_t c = v >> 28;
    v <<= 4;
    vcom_string3[i] = c + (c < 10 ? '0' : 'A' - 10);
  }
}

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  0x0040,
  0x0040,
  &ep1instate,
  &ep1outstate,
};

/**
 * @brief   IN EP2 state.
 */
static USBInEndpointState ep2instate;

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  &ep2instate,
  NULL,
};

#ifdef __USE_USB_MSC__
/* First free packet memory address after the buffer table and EP0.*/
static uint32_t pm_after_ep0;
#endif

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  extern SerialUSBDriver SDU1;

  switch (event) {
  case USB_EVENT_RESET:
    fill_serial_string();
#ifdef __USE_USB_MSC__
    {
      /* usb_lld_reset() has just allocated the EP0 buffers.*/
      pm_after_ep0 = usbp->pmnext;
      /* Also invoked from usbStart() with the system locked.*/
      syssts_t sts = chSysGetStatusAndLockX();
      msc_usb_reset_I();
      chSysRestoreStatusX(sts);
    }
#endif
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();
#ifdef __USE_USB_MSC__
    /* Fork-specific (re-check after a ChibiOS upgrade): SET_CONFIGURATION does
       not free the endpoint packet memory, a repeated one without bus reset
       would overflow the 512 byte packet memory with the mass storage
       endpoints. It is also raised for configuration 0.*/
    usbp->pmnext = pm_after_ep0;
    usbp->epc[USBD1_DATA_REQUEST_EP] = NULL;
    usbp->epc[USBD1_INTERRUPT_REQUEST_EP] = NULL;
    usbp->epc[MSC_EP] = NULL;
    usbp->transmitting &= 1U;
    usbp->receiving &= 1U;
    if (usbp->state != USB_ACTIVE) {
      /* SET_CONFIGURATION(0): disable the endpoints.*/
      for (usbep_t ep = 1; ep <= MSC_EP; ep++) {
        STM32_USB->EPR[ep] = STM32_USB->EPR[ep];  /* Clears the toggle bits.*/
        STM32_USB->EPR[ep] = 0;
      }
      sduDisconnectI(&SDU1);
      msc_configured_I(false);
      chSysUnlockFromISR();
      return;
    }
#endif

    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);
#ifdef __USE_USB_MSC__
    usbInitEndpointI(usbp, MSC_EP, &msc_ep_config);
#endif

    /* Resetting the state of the CDC subsystem.*/
    sduConfigureHookI(&SDU1);
#ifdef __USE_USB_MSC__
    msc_configured_I(true);
#endif

    chSysUnlockFromISR();
    return;
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();

    /* Disconnection event on suspend.*/
    sduDisconnectI(&SDU1);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp) {

  (void)usbp;

  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  osalSysUnlockFromISR();
}

#ifdef __USE_USB_MSC__
/*
 * Handles the mass storage requests, the others go to the CDC.
 */
static bool requests_hook(USBDriver *usbp) {
  if (msc_requests_hook(usbp))
    return true;
  /* Never let the CDC handler answer a class request for the mass storage interface.*/
  if ((usbp->setup.bmRequestType & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS &&
      (usbp->setup.wIndex & 0xFF) == MSC_IF)
    return false;
  return sduRequestsHook(usbp);
}
#else
#define requests_hook sduRequestsHook
#endif

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  requests_hook,
  sof_handler
};

/*
 * Serial over USB driver configuration.
 */
const SerialUSBConfig serusbcfg = {
  &USBD1,
  USBD1_DATA_REQUEST_EP,
  USBD1_DATA_AVAILABLE_EP,
  USBD1_INTERRUPT_REQUEST_EP
};
