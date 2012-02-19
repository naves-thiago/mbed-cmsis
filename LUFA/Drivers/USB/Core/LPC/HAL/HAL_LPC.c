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


/* Includes ------------------------------------------------------------------- */
#include "HAL_LPC.h"

/********************************************************************//**
 * @brief
 * @param
 * @return
 *********************************************************************/
 void HAL_USBInit(void)
 {
	#if defined(__LPC18XX__)
		HAL18XX_USBInit();
	#elif defined(__LPC17XX__)
		HAL17XX_USBInit();
	#elif defined(__LPC11UXX__)
		HAL11UXX_USBInit();
	#endif
 }
/********************************************************************//**
 * @brief
 * @param
 * @return
 *********************************************************************/
 void HAL_USBDeInit(void)
 {
	#if defined(__LPC18XX__)
		HAL18XX_USBDeInit();
	#elif defined(__LPC17XX__)
		HAL17XX_USBDeInit();
	#elif defined(__LPC11UXX__)
		HAL11UXX_USBDeInit();
	#endif
 }
/********************************************************************//**
 * @brief
 * @param
 * @return
 *********************************************************************/
void HAL_EnableUSBInterrupt(void)
{
#if defined(__LPC18XX__)
	HAL18XX_EnableUSBInterrupt();
#elif defined(__LPC17XX__)
	HAL17XX_EnableUSBInterrupt();
#elif defined(__LPC11UXX__)
	HAL11UXX_EnableUSBInterrupt();
#endif
}
/********************************************************************//**
 * @brief
 * @param
 * @return
 *********************************************************************/
void HAL_DisableUSBInterrupt(void)
{
#if defined(__LPC18XX__)
	HAL18XX_DisableUSBInterrupt();
#elif defined(__LPC17XX__)
	HAL17XX_DisableUSBInterrupt();
#elif defined(__LPC11UXX__)
	HAL11UXX_DisableUSBInterrupt();
#endif
}

/* --------------------------------- End Of File ------------------------------ */
