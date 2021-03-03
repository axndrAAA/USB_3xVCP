/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *
  *  @verbatim
  *
  *          ===================================================================
  *                                CDC Class Driver Description
  *          ===================================================================
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  *
  *           These aspects may be enriched or modified for a specific user application.
  *
  *            This driver doesn't implement the following aspects of the specification
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);

static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Variables
  * @{
  */


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC =
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_CDC_GetHSCfgDesc,
  USBD_CDC_GetFSCfgDesc,
  USBD_CDC_GetOtherSpeedCfgDesc,
  USBD_CDC_GetDeviceQualifierDescriptor,
};


/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
		/* Configuration Descriptor */
			  0x09,                                       /* bLength: Configuration Descriptor size */
			  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
			  USB_CDC_CONFIG_DESC_SIZ,                    /* wTotalLength:no of returned bytes */
			  0x00,
			  0x06,                                       /* bNumInterfaces: 2 interface */
			  0x01,                                       /* bConfigurationValue: Configuration value */
			  0x00,                                       /* iConfiguration: Index of string descriptor describing the configuration */
			  0xC0,                                       /* bmAttributes: self powered */
			  0x32,                                       /* MaxPower 0 mA */



							/******** IAD0 should be positioned just before the CDC interfaces ******
									 IAD to associate the two CDC interfaces */

							0x08, /* bLength */
							0x0B, /* bDescriptorType */
							CDC0_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
							0x02, /* bInterfaceCount */
							0x02, /* bFunctionClass */
							0x02, /* bFunctionSubClass */
							0x01, /* bFunctionProtocol */
							0x02, /* iFunction (Index of string descriptor describing this function) */
							/* 08 bytes */

						  /*---------------------------------------------------------------------------*/

			  /* Interface Descriptor */
			  0x09,                                       /* bLength: Interface Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
			  /* Interface descriptor type */
			  CDC0_DESCRIPTOR_INTERFACE_ID,                          /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x01,                                       /* bNumEndpoints: One endpoints used */
			  0x02,                                       /* bInterfaceClass: Communication Interface Class */
			  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
			  0x01,                                       /* bInterfaceProtocol: Common AT commands */
			  0x00,                                       /* iInterface: */

			  /* Header Functional Descriptor */
			  0x05,                                       /* bLength: Endpoint Descriptor size */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
			  0x10,                                       /* bcdCDC: spec release number */
			  0x01,

			  /* Call Management Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
			  0x00,                                       /* bmCapabilities: D0+D1 */
			  CDC0_DESCRIPTOR_INTERFACE_ID+1,                        /* bDataInterface: 1 */

			  /* ACM Functional Descriptor */
			  0x04,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
			  0x02,                                       /* bmCapabilities */

			  /* Union Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x06,                                       /* bDescriptorSubtype: Union func desc */
			  CDC0_DESCRIPTOR_INTERFACE_ID,                          /* bMasterInterface: Communication class interface */
			  CDC0_DESCRIPTOR_INTERFACE_ID+1,                        /* bSlaveInterface0: Data Class Interface */

			  /* Endpoint 2 Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC0_CMD_EP,                                 /* bEndpointAddress */
			  0x03,                                       /* bmAttributes: Interrupt */
			  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
			  HIBYTE(CDC_CMD_PACKET_SIZE),
			  CDC_HS_BINTERVAL,                           /* bInterval: */
			  /*---------------------------------------------------------------------------*/

			  /* Data class interface descriptor */
			  0x09,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
			  CDC0_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x02,                                       /* bNumEndpoints: Two endpoints used */
			  0x0A,                                       /* bInterfaceClass: CDC */
			  0x00,                                       /* bInterfaceSubClass: */
			  0x00,                                       /* bInterfaceProtocol: */
			  0x00,                                       /* iInterface: */

			  /* Endpoint OUT Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC0_OUT_EP,                                 /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
			  0x00,                                       /* bInterval: ignore for Bulk transfer */

			  /* Endpoint IN Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC0_IN_EP,                                  /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
			  0x00,                                       /* bInterval: ignore for Bulk transfer */

						/******** IAD1 should be positioned just before the CDC interfaces ******
								 IAD to associate the two CDC interfaces */

						0x08, /* bLength */
						0x0B, /* bDescriptorType */
						CDC1_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
						0x02, /* bInterfaceCount */
						0x02, /* bFunctionClass */
						0x02, /* bFunctionSubClass */
						0x01, /* bFunctionProtocol */
						0x02, /* iFunction (Index of string descriptor describing this function) */
						/* 08 bytes */

						/*---------------------------------------------------------------------------*/

				/* Interface Descriptor */
				0x09,                                       /* bLength: Interface Descriptor size */
				USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
															/* Interface descriptor type */
				CDC1_DESCRIPTOR_INTERFACE_ID,                                       /* bInterfaceNumber: Number of Interface */
				0x00,                                       /* bAlternateSetting: Alternate setting */
				0x01,                                       /* bNumEndpoints: One endpoints used */
				0x02,                                       /* bInterfaceClass: Communication Interface Class */
				0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
				0x01,                                       /* bInterfaceProtocol: Common AT commands */
				0x00,                                       /* iInterface: */

				/* Header Functional Descriptor */
				0x05,                                       /* bLength: Endpoint Descriptor size */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x00,                                       /* bDescriptorSubtype: Header Func Desc */
				0x10,                                       /* bcdCDC: spec release number */
				0x01,

				/* Call Management Functional Descriptor */
				0x05,                                       /* bFunctionLength */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
				0x00,                                       /* bmCapabilities: D0+D1 */
				CDC1_DESCRIPTOR_INTERFACE_ID+1,                        /* bDataInterface: 1 */

				/* ACM Functional Descriptor */
				0x04,                                       /* bFunctionLength */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
				0x02,                                       /* bmCapabilities */

				/* Union Functional Descriptor */
				0x05,                                       /* bFunctionLength */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x06,                                       /* bDescriptorSubtype: Union func desc */
				CDC1_DESCRIPTOR_INTERFACE_ID,                          /* bMasterInterface: Communication class interface */
				CDC1_DESCRIPTOR_INTERFACE_ID+1,                        /* bSlaveInterface0: Data Class Interface */

				/* Endpoint 2 Descriptor */
				0x07,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
				CDC1_CMD_EP,                                 /* bEndpointAddress */
				0x03,                                       /* bmAttributes: Interrupt */
				LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
				HIBYTE(CDC_CMD_PACKET_SIZE),
				CDC_HS_BINTERVAL,                           /* bInterval: */
				/*---------------------------------------------------------------------------*/

				/* Data class interface descriptor */
				0x09,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
				CDC1_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
				0x00,                                       /* bAlternateSetting: Alternate setting */
				0x02,                                       /* bNumEndpoints: Two endpoints used */
				0x0A,                                       /* bInterfaceClass: CDC */
				0x00,                                       /* bInterfaceSubClass: */
				0x00,                                       /* bInterfaceProtocol: */
				0x00,                                       /* iInterface: */

				/* Endpoint OUT Descriptor */
				0x07,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
				CDC1_OUT_EP,                                 /* bEndpointAddress */
				0x02,                                       /* bmAttributes: Bulk */
				LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
				HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
				0x00,                                       /* bInterval: ignore for Bulk transfer */

				/* Endpoint IN Descriptor */
				0x07,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
				CDC1_IN_EP,                                  /* bEndpointAddress */
				0x02,                                       /* bmAttributes: Bulk */
				LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
				HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
				0x00,

						/******** IAD2 should be positioned just before the CDC interfaces ******
								 IAD to associate the two CDC interfaces */

						0x08, /* bLength */
						0x0B, /* bDescriptorType */
						CDC2_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
						0x02, /* bInterfaceCount */
						0x02, /* bFunctionClass */
						0x02, /* bFunctionSubClass */
						0x01, /* bFunctionProtocol */
						0x00, /* iFunction (Index of string descriptor describing this function) */
						/* 08 bytes */

					  /*---------------------------------------------------------------------------*/

			  /* Interface Descriptor */
			  0x09,                                       /* bLength: Interface Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
														  /* Interface descriptor type */
			  CDC2_DESCRIPTOR_INTERFACE_ID,                          /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x01,                                       /* bNumEndpoints: One endpoints used */
			  0x02,                                       /* bInterfaceClass: Communication Interface Class */
			  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
			  0x01,                                       /* bInterfaceProtocol: Common AT commands */
			  0x00,                                       /* iInterface: */

			  /* Header Functional Descriptor */
			  0x05,                                       /* bLength: Endpoint Descriptor size */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
			  0x10,                                       /* bcdCDC: spec release number */
			  0x01,

			  /* Call Management Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
			  0x00,                                       /* bmCapabilities: D0+D1 */
			  CDC2_DESCRIPTOR_INTERFACE_ID+1,                                       /* bDataInterface: 1 */

			  /* ACM Functional Descriptor */
			  0x04,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
			  0x02,                                       /* bmCapabilities */

			  /* Union Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x06,                                       /* bDescriptorSubtype: Union func desc */
			  CDC2_DESCRIPTOR_INTERFACE_ID,   		                  /* bMasterInterface: Communication class interface */
			  CDC2_DESCRIPTOR_INTERFACE_ID+1,      				  /* bSlaveInterface0: Data Class Interface */

			  /* Endpoint 2 Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC2_CMD_EP,                                 /* bEndpointAddress */
			  0x03,                                       /* bmAttributes: Interrupt */
			  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
			  HIBYTE(CDC_CMD_PACKET_SIZE),
			  CDC_HS_BINTERVAL,                           /* bInterval: */
			  /*---------------------------------------------------------------------------*/

			  /* Data class interface descriptor */
			  0x09,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
			  CDC2_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x02,                                       /* bNumEndpoints: Two endpoints used */
			  0x0A,                                       /* bInterfaceClass: CDC */
			  0x00,                                       /* bInterfaceSubClass: */
			  0x00,                                       /* bInterfaceProtocol: */
			  0x00,                                       /* iInterface: */

			  /* Endpoint OUT Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC2_OUT_EP,                                 /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
			  0x00,                                       /* bInterval: ignore for Bulk transfer */

			  /* Endpoint IN Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC2_IN_EP,                                  /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
			  0x00                                      /* bInterval: ignore for Bulk transfer */
};

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
	 /* Configuration Descriptor */
	  0x09,                                       /* bLength: Configuration Descriptor size */
	  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
	  USB_CDC_CONFIG_DESC_SIZ,                    /* wTotalLength:no of returned bytes */
	  0x00,
	  0x06,                                       /* bNumInterfaces: 2 interface */
	  0x01,                                       /* bConfigurationValue: Configuration value */
	  0x00,                                       /* iConfiguration: Index of string descriptor describing the configuration */
	  0xC0,                                       /* bmAttributes: self powered */
	  0x32,                                       /* MaxPower 0 mA */



					/******** IAD0 should be positioned just before the CDC interfaces ******
							 IAD to associate the two CDC interfaces */

					0x08, /* bLength */
					0x0B, /* bDescriptorType */
					CDC0_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
					0x02, /* bInterfaceCount */
					0x02, /* bFunctionClass */
					0x02, /* bFunctionSubClass */
					0x01, /* bFunctionProtocol */
					0x02, /* iFunction (Index of string descriptor describing this function) */
					/* 08 bytes */

				  /*---------------------------------------------------------------------------*/

	  /* Interface Descriptor */
	  0x09,                                       /* bLength: Interface Descriptor size */
	  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
	  /* Interface descriptor type */
	  CDC0_DESCRIPTOR_INTERFACE_ID,                          /* bInterfaceNumber: Number of Interface */
	  0x00,                                       /* bAlternateSetting: Alternate setting */
	  0x01,                                       /* bNumEndpoints: One endpoints used */
	  0x02,                                       /* bInterfaceClass: Communication Interface Class */
	  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
	  0x01,                                       /* bInterfaceProtocol: Common AT commands */
	  0x00,                                       /* iInterface: */

	  /* Header Functional Descriptor */
	  0x05,                                       /* bLength: Endpoint Descriptor size */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
	  0x10,                                       /* bcdCDC: spec release number */
	  0x01,

	  /* Call Management Functional Descriptor */
	  0x05,                                       /* bFunctionLength */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
	  0x00,                                       /* bmCapabilities: D0+D1 */
	  CDC0_DESCRIPTOR_INTERFACE_ID+1,                        /* bDataInterface: 1 */

	  /* ACM Functional Descriptor */
	  0x04,                                       /* bFunctionLength */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
	  0x02,                                       /* bmCapabilities */

	  /* Union Functional Descriptor */
	  0x05,                                       /* bFunctionLength */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x06,                                       /* bDescriptorSubtype: Union func desc */
	  CDC0_DESCRIPTOR_INTERFACE_ID,                          /* bMasterInterface: Communication class interface */
	  CDC0_DESCRIPTOR_INTERFACE_ID+1,                        /* bSlaveInterface0: Data Class Interface */

	  /* Endpoint 2 Descriptor */
	  0x07,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	  CDC0_CMD_EP,                                 /* bEndpointAddress */
	  0x03,                                       /* bmAttributes: Interrupt */
	  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
	  HIBYTE(CDC_CMD_PACKET_SIZE),
	  CDC_FS_BINTERVAL,                           /* bInterval: */
	  /*---------------------------------------------------------------------------*/

	  /* Data class interface descriptor */
	  0x09,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
	  CDC0_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
	  0x00,                                       /* bAlternateSetting: Alternate setting */
	  0x02,                                       /* bNumEndpoints: Two endpoints used */
	  0x0A,                                       /* bInterfaceClass: CDC */
	  0x00,                                       /* bInterfaceSubClass: */
	  0x00,                                       /* bInterfaceProtocol: */
	  0x00,                                       /* iInterface: */

	  /* Endpoint OUT Descriptor */
	  0x07,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	  CDC0_OUT_EP,                                 /* bEndpointAddress */
	  0x02,                                       /* bmAttributes: Bulk */
	  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
	  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	  0x00,                                       /* bInterval: ignore for Bulk transfer */

	  /* Endpoint IN Descriptor */
	  0x07,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	  CDC0_IN_EP,                                  /* bEndpointAddress */
	  0x02,                                       /* bmAttributes: Bulk */
	  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
	  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	  0x00,                                       /* bInterval: ignore for Bulk transfer */

				/******** IAD1 should be positioned just before the CDC interfaces ******
						 IAD to associate the two CDC interfaces */

				0x08, /* bLength */
				0x0B, /* bDescriptorType */
				CDC1_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
				0x02, /* bInterfaceCount */
				0x02, /* bFunctionClass */
				0x02, /* bFunctionSubClass */
				0x01, /* bFunctionProtocol */
				0x02, /* iFunction (Index of string descriptor describing this function) */
				/* 08 bytes */

				/*---------------------------------------------------------------------------*/

		/* Interface Descriptor */
		0x09,                                       /* bLength: Interface Descriptor size */
		USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
													/* Interface descriptor type */
		CDC1_DESCRIPTOR_INTERFACE_ID,                                       /* bInterfaceNumber: Number of Interface */
		0x00,                                       /* bAlternateSetting: Alternate setting */
		0x01,                                       /* bNumEndpoints: One endpoints used */
		0x02,                                       /* bInterfaceClass: Communication Interface Class */
		0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
		0x01,                                       /* bInterfaceProtocol: Common AT commands */
		0x00,                                       /* iInterface: */

		/* Header Functional Descriptor */
		0x05,                                       /* bLength: Endpoint Descriptor size */
		0x24,                                       /* bDescriptorType: CS_INTERFACE */
		0x00,                                       /* bDescriptorSubtype: Header Func Desc */
		0x10,                                       /* bcdCDC: spec release number */
		0x01,

		/* Call Management Functional Descriptor */
		0x05,                                       /* bFunctionLength */
		0x24,                                       /* bDescriptorType: CS_INTERFACE */
		0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
		0x00,                                       /* bmCapabilities: D0+D1 */
		CDC1_DESCRIPTOR_INTERFACE_ID+1,                        /* bDataInterface: 1 */

		/* ACM Functional Descriptor */
		0x04,                                       /* bFunctionLength */
		0x24,                                       /* bDescriptorType: CS_INTERFACE */
		0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
		0x02,                                       /* bmCapabilities */

		/* Union Functional Descriptor */
		0x05,                                       /* bFunctionLength */
		0x24,                                       /* bDescriptorType: CS_INTERFACE */
		0x06,                                       /* bDescriptorSubtype: Union func desc */
		CDC1_DESCRIPTOR_INTERFACE_ID,                          /* bMasterInterface: Communication class interface */
		CDC1_DESCRIPTOR_INTERFACE_ID+1,                        /* bSlaveInterface0: Data Class Interface */

		/* Endpoint 2 Descriptor */
		0x07,                                       /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
		CDC1_CMD_EP,                                 /* bEndpointAddress */
		0x03,                                       /* bmAttributes: Interrupt */
		LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
		HIBYTE(CDC_CMD_PACKET_SIZE),
		CDC_FS_BINTERVAL,                           /* bInterval: */
		/*---------------------------------------------------------------------------*/

		/* Data class interface descriptor */
		0x09,                                       /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
		CDC1_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
		0x00,                                       /* bAlternateSetting: Alternate setting */
		0x02,                                       /* bNumEndpoints: Two endpoints used */
		0x0A,                                       /* bInterfaceClass: CDC */
		0x00,                                       /* bInterfaceSubClass: */
		0x00,                                       /* bInterfaceProtocol: */
		0x00,                                       /* iInterface: */

		/* Endpoint OUT Descriptor */
		0x07,                                       /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
		CDC1_OUT_EP,                                 /* bEndpointAddress */
		0x02,                                       /* bmAttributes: Bulk */
		LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
		HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
		0x00,                                       /* bInterval: ignore for Bulk transfer */

		/* Endpoint IN Descriptor */
		0x07,                                       /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
		CDC1_IN_EP,                                  /* bEndpointAddress */
		0x02,                                       /* bmAttributes: Bulk */
		LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
		HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
		0x00,

				/******** IAD2 should be positioned just before the CDC interfaces ******
						 IAD to associate the two CDC interfaces */

				0x08, /* bLength */
				0x0B, /* bDescriptorType */
				CDC2_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
				0x02, /* bInterfaceCount */
				0x02, /* bFunctionClass */
				0x02, /* bFunctionSubClass */
				0x01, /* bFunctionProtocol */
				0x00, /* iFunction (Index of string descriptor describing this function) */
				/* 08 bytes */

			  /*---------------------------------------------------------------------------*/

	  /* Interface Descriptor */
	  0x09,                                       /* bLength: Interface Descriptor size */
	  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
												  /* Interface descriptor type */
	  CDC2_DESCRIPTOR_INTERFACE_ID,                          /* bInterfaceNumber: Number of Interface */
	  0x00,                                       /* bAlternateSetting: Alternate setting */
	  0x01,                                       /* bNumEndpoints: One endpoints used */
	  0x02,                                       /* bInterfaceClass: Communication Interface Class */
	  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
	  0x01,                                       /* bInterfaceProtocol: Common AT commands */
	  0x00,                                       /* iInterface: */

	  /* Header Functional Descriptor */
	  0x05,                                       /* bLength: Endpoint Descriptor size */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
	  0x10,                                       /* bcdCDC: spec release number */
	  0x01,

	  /* Call Management Functional Descriptor */
	  0x05,                                       /* bFunctionLength */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
	  0x00,                                       /* bmCapabilities: D0+D1 */
	  CDC2_DESCRIPTOR_INTERFACE_ID+1,                                       /* bDataInterface: 1 */

	  /* ACM Functional Descriptor */
	  0x04,                                       /* bFunctionLength */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
	  0x02,                                       /* bmCapabilities */

	  /* Union Functional Descriptor */
	  0x05,                                       /* bFunctionLength */
	  0x24,                                       /* bDescriptorType: CS_INTERFACE */
	  0x06,                                       /* bDescriptorSubtype: Union func desc */
	  CDC2_DESCRIPTOR_INTERFACE_ID,   		                  /* bMasterInterface: Communication class interface */
	  CDC2_DESCRIPTOR_INTERFACE_ID+1,      				  /* bSlaveInterface0: Data Class Interface */

	  /* Endpoint 2 Descriptor */
	  0x07,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	  CDC2_CMD_EP,                                 /* bEndpointAddress */
	  0x03,                                       /* bmAttributes: Interrupt */
	  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
	  HIBYTE(CDC_CMD_PACKET_SIZE),
	  CDC_FS_BINTERVAL,                           /* bInterval: */
	  /*---------------------------------------------------------------------------*/

	  /* Data class interface descriptor */
	  0x09,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
	  CDC2_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
	  0x00,                                       /* bAlternateSetting: Alternate setting */
	  0x02,                                       /* bNumEndpoints: Two endpoints used */
	  0x0A,                                       /* bInterfaceClass: CDC */
	  0x00,                                       /* bInterfaceSubClass: */
	  0x00,                                       /* bInterfaceProtocol: */
	  0x00,                                       /* iInterface: */

	  /* Endpoint OUT Descriptor */
	  0x07,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	  CDC2_OUT_EP,                                 /* bEndpointAddress */
	  0x02,                                       /* bmAttributes: Bulk */
	  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
	  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	  0x00,                                       /* bInterval: ignore for Bulk transfer */

	  /* Endpoint IN Descriptor */
	  0x07,                                       /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	  CDC2_IN_EP,                                  /* bEndpointAddress */
	  0x02,                                       /* bmAttributes: Bulk */
	  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
	  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	  0x00                                      /* bInterval: ignore for Bulk transfer */
};

__ALIGN_BEGIN static uint8_t USBD_CDC_OtherSpeedCfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
		/* Configuration Descriptor */
			  0x09,                                       /* bLength: Configuration Descriptor size */
			  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
			  USB_CDC_CONFIG_DESC_SIZ,                    /* wTotalLength:no of returned bytes */
			  0x00,
			  0x06,                                       /* bNumInterfaces: 2 interface */
			  0x01,                                       /* bConfigurationValue: Configuration value */
			  0x00,                                       /* iConfiguration: Index of string descriptor describing the configuration */
			  0xC0,                                       /* bmAttributes: self powered */
			  0x32,                                       /* MaxPower 0 mA */



							/******** IAD0 should be positioned just before the CDC interfaces ******
									 IAD to associate the two CDC interfaces */

							0x08, /* bLength */
							0x0B, /* bDescriptorType */
							CDC0_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
							0x02, /* bInterfaceCount */
							0x02, /* bFunctionClass */
							0x02, /* bFunctionSubClass */
							0x01, /* bFunctionProtocol */
							0x02, /* iFunction (Index of string descriptor describing this function) */
							/* 08 bytes */

						  /*---------------------------------------------------------------------------*/

			  /* Interface Descriptor */
			  0x09,                                       /* bLength: Interface Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
			  /* Interface descriptor type */
			  CDC0_DESCRIPTOR_INTERFACE_ID,                          /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x01,                                       /* bNumEndpoints: One endpoints used */
			  0x02,                                       /* bInterfaceClass: Communication Interface Class */
			  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
			  0x01,                                       /* bInterfaceProtocol: Common AT commands */
			  0x00,                                       /* iInterface: */

			  /* Header Functional Descriptor */
			  0x05,                                       /* bLength: Endpoint Descriptor size */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
			  0x10,                                       /* bcdCDC: spec release number */
			  0x01,

			  /* Call Management Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
			  0x00,                                       /* bmCapabilities: D0+D1 */
			  CDC0_DESCRIPTOR_INTERFACE_ID+1,                        /* bDataInterface: 1 */

			  /* ACM Functional Descriptor */
			  0x04,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
			  0x02,                                       /* bmCapabilities */

			  /* Union Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x06,                                       /* bDescriptorSubtype: Union func desc */
			  CDC0_DESCRIPTOR_INTERFACE_ID,                          /* bMasterInterface: Communication class interface */
			  CDC0_DESCRIPTOR_INTERFACE_ID+1,                        /* bSlaveInterface0: Data Class Interface */

			  /* Endpoint 2 Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC0_CMD_EP,                                 /* bEndpointAddress */
			  0x03,                                       /* bmAttributes: Interrupt */
			  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
			  HIBYTE(CDC_CMD_PACKET_SIZE),
			  CDC_HS_BINTERVAL,                           /* bInterval: */
			  /*---------------------------------------------------------------------------*/

			  /* Data class interface descriptor */
			  0x09,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
			  CDC0_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x02,                                       /* bNumEndpoints: Two endpoints used */
			  0x0A,                                       /* bInterfaceClass: CDC */
			  0x00,                                       /* bInterfaceSubClass: */
			  0x00,                                       /* bInterfaceProtocol: */
			  0x00,                                       /* iInterface: */

			  /* Endpoint OUT Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC0_OUT_EP,                                 /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
			  0x00,                                       /* bInterval: ignore for Bulk transfer */

			  /* Endpoint IN Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC0_IN_EP,                                  /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
			  0x00,                                       /* bInterval: ignore for Bulk transfer */

						/******** IAD1 should be positioned just before the CDC interfaces ******
								 IAD to associate the two CDC interfaces */

						0x08, /* bLength */
						0x0B, /* bDescriptorType */
						CDC1_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
						0x02, /* bInterfaceCount */
						0x02, /* bFunctionClass */
						0x02, /* bFunctionSubClass */
						0x01, /* bFunctionProtocol */
						0x02, /* iFunction (Index of string descriptor describing this function) */
						/* 08 bytes */

						/*---------------------------------------------------------------------------*/

				/* Interface Descriptor */
				0x09,                                       /* bLength: Interface Descriptor size */
				USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
															/* Interface descriptor type */
				CDC1_DESCRIPTOR_INTERFACE_ID,                                       /* bInterfaceNumber: Number of Interface */
				0x00,                                       /* bAlternateSetting: Alternate setting */
				0x01,                                       /* bNumEndpoints: One endpoints used */
				0x02,                                       /* bInterfaceClass: Communication Interface Class */
				0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
				0x01,                                       /* bInterfaceProtocol: Common AT commands */
				0x00,                                       /* iInterface: */

				/* Header Functional Descriptor */
				0x05,                                       /* bLength: Endpoint Descriptor size */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x00,                                       /* bDescriptorSubtype: Header Func Desc */
				0x10,                                       /* bcdCDC: spec release number */
				0x01,

				/* Call Management Functional Descriptor */
				0x05,                                       /* bFunctionLength */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
				0x00,                                       /* bmCapabilities: D0+D1 */
				CDC1_DESCRIPTOR_INTERFACE_ID+1,                        /* bDataInterface: 1 */

				/* ACM Functional Descriptor */
				0x04,                                       /* bFunctionLength */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
				0x02,                                       /* bmCapabilities */

				/* Union Functional Descriptor */
				0x05,                                       /* bFunctionLength */
				0x24,                                       /* bDescriptorType: CS_INTERFACE */
				0x06,                                       /* bDescriptorSubtype: Union func desc */
				CDC1_DESCRIPTOR_INTERFACE_ID,                          /* bMasterInterface: Communication class interface */
				CDC1_DESCRIPTOR_INTERFACE_ID+1,                        /* bSlaveInterface0: Data Class Interface */

				/* Endpoint 2 Descriptor */
				0x07,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
				CDC1_CMD_EP,                                 /* bEndpointAddress */
				0x03,                                       /* bmAttributes: Interrupt */
				LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
				HIBYTE(CDC_CMD_PACKET_SIZE),
				CDC_FS_BINTERVAL,                           /* bInterval: */
				/*---------------------------------------------------------------------------*/

				/* Data class interface descriptor */
				0x09,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
				CDC1_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
				0x00,                                       /* bAlternateSetting: Alternate setting */
				0x02,                                       /* bNumEndpoints: Two endpoints used */
				0x0A,                                       /* bInterfaceClass: CDC */
				0x00,                                       /* bInterfaceSubClass: */
				0x00,                                       /* bInterfaceProtocol: */
				0x00,                                       /* iInterface: */

				/* Endpoint OUT Descriptor */
				0x07,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
				CDC1_OUT_EP,                                 /* bEndpointAddress */
				0x02,                                       /* bmAttributes: Bulk */
				LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
				HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
				0x00,                                       /* bInterval: ignore for Bulk transfer */

				/* Endpoint IN Descriptor */
				0x07,                                       /* bLength: Endpoint Descriptor size */
				USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
				CDC1_IN_EP,                                  /* bEndpointAddress */
				0x02,                                       /* bmAttributes: Bulk */
				LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
				HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
				0x00,

						/******** IAD2 should be positioned just before the CDC interfaces ******
								 IAD to associate the two CDC interfaces */

						0x08, /* bLength */
						0x0B, /* bDescriptorType */
						CDC2_DESCRIPTOR_INTERFACE_ID, /* bFirstInterface */
						0x02, /* bInterfaceCount */
						0x02, /* bFunctionClass */
						0x02, /* bFunctionSubClass */
						0x01, /* bFunctionProtocol */
						0x00, /* iFunction (Index of string descriptor describing this function) */
						/* 08 bytes */

					  /*---------------------------------------------------------------------------*/

			  /* Interface Descriptor */
			  0x09,                                       /* bLength: Interface Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
														  /* Interface descriptor type */
			  CDC2_DESCRIPTOR_INTERFACE_ID,                          /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x01,                                       /* bNumEndpoints: One endpoints used */
			  0x02,                                       /* bInterfaceClass: Communication Interface Class */
			  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
			  0x01,                                       /* bInterfaceProtocol: Common AT commands */
			  0x00,                                       /* iInterface: */

			  /* Header Functional Descriptor */
			  0x05,                                       /* bLength: Endpoint Descriptor size */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
			  0x10,                                       /* bcdCDC: spec release number */
			  0x01,

			  /* Call Management Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
			  0x00,                                       /* bmCapabilities: D0+D1 */
			  CDC2_DESCRIPTOR_INTERFACE_ID+1,                                       /* bDataInterface: 1 */

			  /* ACM Functional Descriptor */
			  0x04,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
			  0x02,                                       /* bmCapabilities */

			  /* Union Functional Descriptor */
			  0x05,                                       /* bFunctionLength */
			  0x24,                                       /* bDescriptorType: CS_INTERFACE */
			  0x06,                                       /* bDescriptorSubtype: Union func desc */
			  CDC2_DESCRIPTOR_INTERFACE_ID,   		                  /* bMasterInterface: Communication class interface */
			  CDC2_DESCRIPTOR_INTERFACE_ID+1,      				  /* bSlaveInterface0: Data Class Interface */

			  /* Endpoint 2 Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC2_CMD_EP,                                 /* bEndpointAddress */
			  0x03,                                       /* bmAttributes: Interrupt */
			  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize: */
			  HIBYTE(CDC_CMD_PACKET_SIZE),
			  CDC_FS_BINTERVAL,                           /* bInterval: */
			  /*---------------------------------------------------------------------------*/

			  /* Data class interface descriptor */
			  0x09,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
			  CDC2_DESCRIPTOR_INTERFACE_ID + 1,                      /* bInterfaceNumber: Number of Interface */
			  0x00,                                       /* bAlternateSetting: Alternate setting */
			  0x02,                                       /* bNumEndpoints: Two endpoints used */
			  0x0A,                                       /* bInterfaceClass: CDC */
			  0x00,                                       /* bInterfaceSubClass: */
			  0x00,                                       /* bInterfaceProtocol: */
			  0x00,                                       /* iInterface: */

			  /* Endpoint OUT Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC2_OUT_EP,                                 /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
			  0x00,                                       /* bInterval: ignore for Bulk transfer */

			  /* Endpoint IN Descriptor */
			  0x07,                                       /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
			  CDC2_IN_EP,                                  /* bEndpointAddress */
			  0x02,                                       /* bmAttributes: Bulk */
			  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
			  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
			  0x00                                      /* bInterval: ignore for Bulk transfer */
};

uint8_t getIn_EPN_by_IntrfID(uint8_t intrfIdx){
	uint8_t in_epn = 0;

	if(intrfIdx == USB_CDC0){
		in_epn = CDC0_IN_EP;

	}else if(intrfIdx == USB_CDC1){
		in_epn = CDC1_IN_EP;

	}else if(intrfIdx == USB_CDC2){
		in_epn = CDC2_IN_EP;
	}
	return in_epn;
}

uint8_t getOut_EPN_by_IntrfID(uint8_t intrfIdx){
	uint8_t out_epn = 0;

	if(intrfIdx == USB_CDC0){
		out_epn = CDC0_OUT_EP;

	}else if(intrfIdx == USB_CDC1){
		out_epn = CDC1_OUT_EP;

	}else if(intrfIdx == USB_CDC2){
		out_epn = CDC2_OUT_EP;
	}
	return out_epn;
}

uint8_t getIntrfIdx_by_EPN(uint8_t epn){
	uint8_t ret = USB_CDC0;
	if((epn == CDC0_OUT_EP) || (epn == CDC0_IN_EP)){
		ret = USB_CDC0;
	}else if((epn == CDC1_OUT_EP) || (epn == CDC1_IN_EP)){
		ret = USB_CDC1;
	}else if((epn == CDC2_OUT_EP) || (epn == CDC2_IN_EP)){
		ret = USB_CDC2;
	}

	return ret;
}


/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Functions
  * @{
  */

/**
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_CDC_HandleTypeDef *hcdc;

  static USBD_CDC_HandleTypeDef cdc_handle;
  hcdc = &cdc_handle;

  if (hcdc == NULL)
  {
    pdev->pClassData = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassData = (void *)hcdc;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev,
    					 CDC0_IN_EP,
						 USBD_EP_TYPE_BULK,
                         CDC_DATA_HS_IN_PACKET_SIZE);
     pdev->ep_in[CDC0_IN_EP & 0xFU].is_used = 1U;

	 /* Open EP IN */
	 (void)USBD_LL_OpenEP(pdev,
						 CDC1_IN_EP,
						 USBD_EP_TYPE_BULK,
						  CDC_DATA_HS_IN_PACKET_SIZE);
	  pdev->ep_in[CDC1_IN_EP & 0xFU].is_used = 1U;


	  /* Open EP IN */
	  (void)USBD_LL_OpenEP(pdev,
						 CDC2_IN_EP,
						 USBD_EP_TYPE_BULK,
						   CDC_DATA_HS_IN_PACKET_SIZE);
	   pdev->ep_in[CDC2_IN_EP & 0xFU].is_used = 1U;

	   /*---------------------------------------------*/

     /* Open EP OUT */
     (void)USBD_LL_OpenEP(pdev,
    		 	 	 	  CDC0_OUT_EP,
						  USBD_EP_TYPE_BULK,
                          CDC_DATA_HS_OUT_PACKET_SIZE);
      pdev->ep_out[CDC0_OUT_EP & 0xFU].is_used = 1U;

      /* Open EP OUT */
	   (void)USBD_LL_OpenEP(pdev,
						  CDC1_OUT_EP,
						  USBD_EP_TYPE_BULK,
							CDC_DATA_HS_OUT_PACKET_SIZE);
		pdev->ep_out[CDC1_OUT_EP & 0xFU].is_used = 1U;

		/* Open EP OUT */
	   (void)USBD_LL_OpenEP(pdev,
						  CDC2_OUT_EP,
						  USBD_EP_TYPE_BULK,
							CDC_DATA_HS_OUT_PACKET_SIZE);
		pdev->ep_out[CDC2_OUT_EP & 0xFU].is_used = 1U;

      /* Set bInterval for CDC CMD Endpoint */
      pdev->ep_in[CDC0_CMD_EP & 0xFU].bInterval = CDC_HS_BINTERVAL;
      pdev->ep_in[CDC1_CMD_EP & 0xFU].bInterval = CDC_HS_BINTERVAL;
      pdev->ep_in[CDC2_CMD_EP & 0xFU].bInterval = CDC_HS_BINTERVAL;
  }
  else
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev,
    					 CDC0_IN_EP,
						 USBD_EP_TYPE_BULK,
                         CDC_DATA_FS_IN_PACKET_SIZE);
     pdev->ep_in[CDC0_IN_EP & 0xFU].is_used = 1U;

     /* Open EP IN */
	 (void)USBD_LL_OpenEP(pdev,
						 CDC1_IN_EP,
						 USBD_EP_TYPE_BULK,
						  CDC_DATA_FS_IN_PACKET_SIZE);
	  pdev->ep_in[CDC1_IN_EP & 0xFU].is_used = 1U;

	  /* Open EP IN */
	  (void)USBD_LL_OpenEP(pdev,
						 CDC2_IN_EP,
						 USBD_EP_TYPE_BULK,
						   CDC_DATA_FS_IN_PACKET_SIZE);
	   pdev->ep_in[CDC2_IN_EP & 0xFU].is_used = 1U;


	   /*---------------------------------------------------*/

     /* Open EP OUT */
		 (void)USBD_LL_OpenEP(pdev,
							  CDC0_OUT_EP,
							  USBD_EP_TYPE_BULK,
							  CDC_DATA_FS_OUT_PACKET_SIZE);
		  pdev->ep_out[CDC0_OUT_EP & 0xFU].is_used = 1U;

      /* Open EP OUT */
           (void)USBD_LL_OpenEP(pdev,
          		 	 	 	  CDC1_OUT_EP,
      						  USBD_EP_TYPE_BULK,
                                CDC_DATA_FS_OUT_PACKET_SIZE);
            pdev->ep_out[CDC1_OUT_EP & 0xFU].is_used = 1U;

		/* Open EP OUT */
			 (void)USBD_LL_OpenEP(pdev,
								  CDC2_OUT_EP,
								  USBD_EP_TYPE_BULK,
								  CDC_DATA_FS_OUT_PACKET_SIZE);
			  pdev->ep_out[CDC2_OUT_EP & 0xFU].is_used = 1U;


      /* Set bInterval for CMD Endpoint */
      pdev->ep_in[CDC0_CMD_EP & 0xFU].bInterval = CDC_FS_BINTERVAL;
      pdev->ep_in[CDC1_CMD_EP & 0xFU].bInterval = CDC_FS_BINTERVAL;
      pdev->ep_in[CDC2_CMD_EP & 0xFU].bInterval = CDC_FS_BINTERVAL;
  }

  /* Open Command IN EP */
  (void)USBD_LL_OpenEP(pdev, CDC0_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[CDC0_CMD_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_OpenEP(pdev, CDC1_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[CDC1_CMD_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_OpenEP(pdev, CDC2_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[CDC2_CMD_EP & 0xFU].is_used = 1U;

  /* Init  physical Interface components */
  ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();

  /* Init Xfer states */

  for(uint8_t i = 0; i < CDC_PORTS_NUMBER;i++){
	  hcdc->TxState[i] = 0U;
	  hcdc->RxState[i] = 0U;
  }

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
		(void)USBD_LL_PrepareReceive(pdev,
									 CDC0_OUT_EP,
									 hcdc->RxBuffer[USB_CDC0],
									 CDC_DATA_HS_OUT_PACKET_SIZE);
    /* Prepare Out endpoint to receive next packet */
        (void)USBD_LL_PrepareReceive(pdev,
        							 CDC1_OUT_EP,
    								 hcdc->RxBuffer[USB_CDC1],
                                     CDC_DATA_HS_OUT_PACKET_SIZE);
        /* Prepare Out endpoint to receive next packet */
		(void)USBD_LL_PrepareReceive(pdev,
									 CDC2_OUT_EP,
									 hcdc->RxBuffer[USB_CDC2],
									 CDC_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev,
    							 CDC0_OUT_EP,
								 hcdc->RxBuffer[USB_CDC0],
                                 CDC_DATA_FS_OUT_PACKET_SIZE);
    /* Prepare Out endpoint to receive next packet */
	(void)USBD_LL_PrepareReceive(pdev,
								 CDC1_OUT_EP,
								 hcdc->RxBuffer[USB_CDC1],
								 CDC_DATA_FS_OUT_PACKET_SIZE);
	/* Prepare Out endpoint to receive next packet */
	(void)USBD_LL_PrepareReceive(pdev,
								 CDC2_OUT_EP,
								 hcdc->RxBuffer[USB_CDC2],
								 CDC_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  uint8_t ret = 0U;

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, CDC0_IN_EP);
  pdev->ep_in[CDC0_IN_EP & 0xFU].is_used = 0U;

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, CDC1_IN_EP);
  pdev->ep_in[CDC1_IN_EP & 0xFU].is_used = 0U;

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, CDC2_IN_EP);
  pdev->ep_in[CDC2_IN_EP & 0xFU].is_used = 0U;

  /*-----------------------------------------------------------*/

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, CDC0_OUT_EP);
  pdev->ep_out[CDC0_OUT_EP & 0xFU].is_used = 0U;

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, CDC1_OUT_EP);
  pdev->ep_out[CDC1_OUT_EP & 0xFU].is_used = 0U;

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, CDC1_OUT_EP);
  pdev->ep_out[CDC1_OUT_EP & 0xFU].is_used = 0U;

  /*-------------------------------------------------------------*/

  /* Close Command IN EP */
  (void)USBD_LL_CloseEP(pdev, CDC0_CMD_EP);
  pdev->ep_in[CDC0_CMD_EP & 0xFU].is_used = 0U;
  pdev->ep_in[CDC0_CMD_EP & 0xFU].bInterval = 0U;

  /* Close Command IN EP */
  (void)USBD_LL_CloseEP(pdev, CDC1_CMD_EP);
  pdev->ep_in[CDC1_CMD_EP & 0xFU].is_used = 0U;
  pdev->ep_in[CDC1_CMD_EP & 0xFU].bInterval = 0U;

  /* Close Command IN EP */
  (void)USBD_LL_CloseEP(pdev, CDC2_CMD_EP);
  pdev->ep_in[CDC2_CMD_EP & 0xFU].is_used = 0U;
  pdev->ep_in[CDC2_CMD_EP & 0xFU].bInterval = 0U;


  /* DeInit  physical Interface components */
  if (pdev->pClassData != NULL)
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->DeInit();
    pdev->pClassData = NULL;
  }

  return ret;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev,
                              USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:
    if (req->wLength != 0U)
    {
      if ((req->bmRequest & 0x80U) != 0U)
      {
        ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                          (uint8_t *)hcdc->data,
                                                          req->wLength,
														  req->wIndex/2);

          (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, req->wLength);
      }
      else
      {
        hcdc->CmdOpCode = req->bRequest;
        hcdc->CmdLength = (uint8_t)req->wLength;

        (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, req->wLength);
      }
    }
    else
    {
      ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                        (uint8_t *)req, 0U, req->wIndex/2);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_GET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, &ifalt, 1U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
      if (pdev->dev_state != USBD_STATE_CONFIGURED)
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_CLEAR_FEATURE:
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  default:
    USBD_CtlError(pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc;
  PCD_HandleTypeDef *hpcd = pdev->pData;

  uint8_t intrfIdx = getIntrfIdx_by_EPN(epnum);

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if ((pdev->ep_in[epnum].total_length > 0U) &&
      ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
  {
    /* Update the packet total length */
    pdev->ep_in[epnum].total_length = 0U;

    /* Send ZLP */
    (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
  }
  else
  {
    hcdc->TxState[intrfIdx] = 0U;
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->TransmitCplt(hcdc->TxBuffer[intrfIdx], &hcdc->TxLength[intrfIdx], epnum);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  uint8_t intrfIdx = getIntrfIdx_by_EPN(epnum);

  /* Get the received data length */
  hcdc->RxLength[intrfIdx] = USBD_LL_GetRxDataSize(pdev, epnum);


  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */

  ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Receive(hcdc->RxBuffer[intrfIdx], &hcdc->RxLength[intrfIdx], intrfIdx);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if ((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFFU))
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(hcdc->CmdOpCode,
                                                      (uint8_t *)hcdc->data,
                                                      (uint16_t)hcdc->CmdLength,
													  pdev->request.wIndex/2);
    hcdc->CmdOpCode = 0xFFU;

  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_CfgFSDesc);

  return USBD_CDC_CfgFSDesc;
}

/**
  * @brief  USBD_CDC_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_CfgHSDesc);

  return USBD_CDC_CfgHSDesc;
}

/**
  * @brief  USBD_CDC_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_OtherSpeedCfgDesc);

  return USBD_CDC_OtherSpeedCfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_DeviceQualifierDesc);

  return USBD_CDC_DeviceQualifierDesc;
}

/**
* @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev,
                                   USBD_CDC_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData = fops;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t intrfIdx,
                             uint8_t *pbuff, uint32_t length)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  hcdc->TxBuffer[intrfIdx] = pbuff;
  hcdc->TxLength[intrfIdx] = length;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t intrfIdx, uint8_t *pbuff)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  hcdc->RxBuffer[intrfIdx] = pbuff;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t intrfIdx)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
  USBD_StatusTypeDef ret = USBD_BUSY;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (hcdc->TxState[intrfIdx] == 0U)
  {
    /* Tx Transfer in progress */
    hcdc->TxState[intrfIdx] = 1U;

    uint8_t in_epn = getIn_EPN_by_IntrfID(intrfIdx);

    /* Update the packet total length */
    pdev->ep_in[in_epn & 0xFU].total_length = hcdc->TxLength[intrfIdx];

    /* Transmit next packet */
    (void)USBD_LL_Transmit(pdev,
    						in_epn,
							hcdc->TxBuffer[intrfIdx],
							hcdc->TxLength[intrfIdx]);

    ret = USBD_OK;
  }

  return (uint8_t)ret;
}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev, uint8_t intrfIdx)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  uint8_t out_ep = getOut_EPN_by_IntrfID(intrfIdx);

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, out_ep, hcdc->RxBuffer[intrfIdx],
                                 CDC_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, out_ep, hcdc->RxBuffer[intrfIdx],
                                 CDC_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
