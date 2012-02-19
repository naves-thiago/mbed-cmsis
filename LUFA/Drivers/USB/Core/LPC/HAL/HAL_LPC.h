/*
* Copyright(C) NXP Semiconductors, 2011
* All rights reserved.
*
* Copyright (C) Dean Camera, 2011.
*
* LUFA Library is licensed from Dean Camera by NXP for NXP customers 
* for use with NXP’s LPC microcontrollers.
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
* documentation is hereby granted, under NXP Semiconductors’ and its 
* licensor’s relevant copyrights in the software, without fee, provided that it 
* is used in conjunction with NXP Semiconductors microcontrollers.  This 
* copyright, permission, and disclaimer notice must appear in all copies of 
* this code.
*/



#ifndef __LPC_HAL_H__
#define __LPC_HAL_H__

#ifdef __CODE_RED
	#include <cr_section_macros.h>
#elif defined(__CC_ARM) // FIXME temporarily fix for Keil (work only for lpc17xx)
	#define __DATA(x)
	#define __BSS(x)
#endif

/* Includes ------------------------------------------------------------------- */
#if defined(__LPC18XX__)
	#include "LPC18XX/HAL_LPC18xx.h"
#elif defined(__LPC17XX__)
	#include "LPC17XX/HAL_LPC17xx.h"
#elif defined(__LPC11UXX__)
	#include "LPC11UXX/HAL_LPC11Uxx.h"
#endif

void HAL_USBInit(void);
void HAL_USBDeInit(void);
void HAL_EnableUSBInterrupt(void);
void HAL_DisableUSBInterrupt(void);

static __INLINE void HAL_USBConnect (uint32_t con);
static __INLINE void HAL_USBConnect (uint32_t con)
{
#if defined(__LPC18XX__)
	HAL18XX_USBConnect(con);
#elif defined(__LPC17XX__)
	HAL17XX_USBConnect(con);
#elif defined(__LPC11UXX__)
	HAL11UXX_USBConnect(con);
#endif
}

/**
 * @}
 */
#endif /*__LPC_HAL_H__*/
/* --------------------------------- End Of File ------------------------------ */
