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


#ifndef NXPUSBLIB_CONFIG_H_
#define NXPUSBLIB_CONFIG_H_

/* ARCH=ARCH_LPC is not configurable */
#define ARCH							ARCH_LPC

/* Define NXPUSBLIB_DEBUG to allow the lib print out diagnostic message */
//#define NXPUSBLIB_DEBUG

#define FIXED_NUM_CONFIGURATIONS		1
#define FIXED_CONTROL_ENDPOINT_SIZE		64

//#define __TEST__			/* Test development */
#define USBRAM_BUFFER_SIZE  (2*1024)

#endif /* NXPUSBLIB_CONFIG_H_ */
