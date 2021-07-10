/** \file
 *
 *  Header file for Descriptors.c.
 */

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

	/* Includes: */
		#include <LUFA/Drivers/USB/USB.h>

		#include <avr/pgmspace.h>

	/* Macros: */
		/** Endpoint address of the Generic HID reporting IN endpoint. */
		#define GENERIC_IN_EPADDR         (ENDPOINT_DIR_IN | 1)

		/** Endpoint address of the Generic HID reporting OUT endpoint. */
		#define GENERIC_OUT_EPADDR        (ENDPOINT_DIR_OUT | 2)

		/** Size in bytes of the Generic HID reporting endpoint. */
		#define GENERIC_EPSIZE            8

		/** Endpoint address of the CDC device-to-host notification IN endpoint. */
		#define CDC_NOTIFICATION_EPADDR   (ENDPOINT_DIR_IN  | 3)

		/** Endpoint address of the CDC device-to-host data IN endpoint. */
		#define CDC_TX_EPADDR             (ENDPOINT_DIR_IN  | 4)

		/** Endpoint address of the CDC host-to-device data OUT endpoint. */
		#define CDC_RX_EPADDR             (ENDPOINT_DIR_OUT | 5)

		/** Size in bytes of the CDC device-to-host notification IN endpoint. */
		#define CDC_NOTIFICATION_EPSIZE   8

		/** Size in bytes of the CDC data IN and OUT endpoints. */
		#define CDC_TXRX_EPSIZE           16

	/* Type Defines: */
		/** Type define for the device configuration descriptor structure. This must be defined in the
		 *  application code, as the configuration descriptor contains several sub-descriptors which
		 *  vary between devices, and which describe the device's usage to the host.
		 */
		typedef struct
		{
			USB_Descriptor_Configuration_Header_t  Config;

			// Generic HID Interface
			USB_Descriptor_Interface_t             HID_Interface;
			USB_HID_Descriptor_HID_t               HID_GenericHID;
			USB_Descriptor_Endpoint_t              HID_ReportINEndpoint;
			USB_Descriptor_Endpoint_t              HID_ReportOUTEndpoint;

			// CDC Control Interface
			USB_Descriptor_Interface_Association_t CDC_IAD;
			USB_Descriptor_Interface_t             CDC_CCI_Interface;
			USB_CDC_Descriptor_FunctionalHeader_t  CDC_Functional_Header;
			USB_CDC_Descriptor_FunctionalACM_t     CDC_Functional_ACM;
			USB_CDC_Descriptor_FunctionalUnion_t   CDC_Functional_Union;
			USB_Descriptor_Endpoint_t              CDC_NotificationEndpoint;

			// CDC Data Interface
			USB_Descriptor_Interface_t             CDC_DCI_Interface;
			USB_Descriptor_Endpoint_t              CDC_DataOutEndpoint;
			USB_Descriptor_Endpoint_t              CDC_DataInEndpoint;
		} USB_Descriptor_Configuration_t;

		/** Enum for the device interface descriptor IDs within the device. Each interface descriptor
		 *  should have a unique ID index associated with it, which can be used to refer to the
		 *  interface from other descriptors.
		 */
		enum InterfaceDescriptors_t
		{
			INTERFACE_ID_GenericHID   = 0, /**< GenericHID interface descriptor ID */
			INTERFACE_ID_CDC_CCI      = 1, /**< CDC CCI interface descriptor ID */
			INTERFACE_ID_CDC_DCI      = 2, /**< CDC DCI interface descriptor ID */
		};

		/** Enum for the device string descriptor IDs within the device. Each string descriptor should
		 *  have a unique ID index associated with it, which can be used to refer to the string from
		 *  other descriptors.
		 */
		enum StringDescriptors_t
		{
			STRING_ID_Language     = 0, /**< Supported Languages string descriptor ID (must be zero) */
			STRING_ID_Manufacturer = 1, /**< Manufacturer string ID */
			STRING_ID_Product      = 2, /**< Product string ID */
		};

	/* Function Prototypes: */
		uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
		                                    const uint16_t wIndex,
		                                    const void** const DescriptorAddress)
		                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

#endif

