/*
* Copyright(C) NXP Semiconductors, 2011
* All rights reserved.
*
* Copyright (C) Dean Camera, 2011.
*
* LUFA Library is licensed from Dean Camera by NXP for NXP customers 
* for use with NXP�s LPC microcontrollers.
*
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* LPC products.  This software is supplied "AS IS" without any warranties of
* any kind, and NXP Semiconductors and its licensor disclaim any and 
* all warranties, express or implied, including all implied warranties of 
* merchantability, fitness for a particular purpose and non-infringement of 
* intellectual property rights.  NXP Semiconductors assumes no responsibility
* or liability for the use of the software, conveys no license or rights under any
* patent, copyright, mask work right, or any other intellectual property rights in 
* or to any products. NXP Semiconductors reserves the right to make changes
* in the software without notification. NXP Semiconductors also makes no 
* representation or warranty that such application will be suitable for the
* specified use without further testing or modification.
* 
* Permission to use, copy, modify, and distribute this software and its 
* documentation is hereby granted, under NXP Semiconductors� and its 
* licensor�s relevant copyrights in the software, without fee, provided that it 
* is used in conjunction with NXP Semiconductors microcontrollers.  This 
* copyright, permission, and disclaimer notice must appear in all copies of 
* this code.
*/



/** \file
 *
 *  Header file for ConfigDescriptor.c.
 */

#ifndef _CONFIGDESCRIPTOR_H_
#define _CONFIGDESCRIPTOR_H_

	/* Includes: */
		#include "LUFA/Drivers/USB/USB.h"

		#include "HIDReport.h"

	/* Macros: */
		/** Pipe number for the mouse report data pipe. */
		#define MOUSE_DATA_IN_PIPE          1

	/* Enums: */
		/** Enum for the possible return codes of the \ref ProcessConfigurationDescriptor() function. */
		enum MouseHostWithParser_GetConfigDescriptorDataCodes_t
		{
			SuccessfulConfigRead            = 0, /**< Configuration Descriptor was processed successfully */
			ControlError                    = 1, /**< A control request to the device failed to complete successfully */
			DescriptorTooLarge              = 2, /**< The device's Configuration Descriptor is too large to process */
			InvalidConfigDataReturned       = 3, /**< The device returned an invalid Configuration Descriptor */
			NoCompatibleInterfaceFound      = 4, /**< A compatible interface with the required endpoints was not found */
		};

	/* Function Prototypes: */
		uint8_t ProcessConfigurationDescriptor(void);

		uint8_t DComp_NextMouseInterface(void* CurrentDescriptor);
		uint8_t DComp_NextMouseInterfaceDataEndpoint(void* CurrentDescriptor);
		uint8_t DComp_NextHID(void* CurrentDescriptor);

#endif

