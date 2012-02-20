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


#include "HIDReport.h"

/** Size in bytes of the attached device's HID report descriptor */
uint16_t         HIDReportSize;

/** Processed HID report descriptor items structure, containing information on each HID report element */
HID_ReportInfo_t HIDReportInfo;


/** Function to read in the HID report descriptor from the attached device, and process it into easy-to-read
 *  structures via the HID parser routines in the LUFA library.
 *
 *  \return  A value from the MouseHostWithParser_GetHIDReportDataCodes_t enum
 */
uint8_t GetHIDReportData(void)
{
	/* Create a buffer big enough to hold the entire returned HID report */
	uint8_t HIDReportData[HIDReportSize];

	USB_ControlRequest = (USB_Request_Header_t)
		{
			.bmRequestType = (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE),
			.bRequest      = REQ_GetDescriptor,
			.wValue        = (HID_DTYPE_Report << 8),
			.wIndex        = 0,
			.wLength       = HIDReportSize,
		};

	/* Select the control pipe for the request transfer */
	Pipe_SelectPipe(PIPE_CONTROLPIPE);

	/* Send control request to retrieve the HID report from the attached device */
	if (USB_Host_SendControlRequest(HIDReportData) != HOST_SENDCONTROL_Successful)
	  return ParseControlError;

	/* Send the HID report to the parser for processing */
	if (USB_ProcessHIDReport(HIDReportData, HIDReportSize, &HIDReportInfo) != HID_PARSE_Successful)
	  return ParseError;

	return ParseSuccessful;
}

/** Callback for the HID Report Parser. This function is called each time the HID report parser is about to store
 *  an IN, OUT or FEATURE item into the HIDReportInfo structure. To save on RAM, we are able to filter out items
 *  we aren't interested in (preventing us from being able to extract them later on, but saving on the RAM they would
 *  have occupied).
 *
 *  \param[in] CurrentItem  Pointer to the item the HID report parser is currently working with
 *
 *  \return Boolean true if the item should be stored into the HID report structure, false if it should be discarded
 */
bool CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t* const CurrentItem)
{
	bool IsMouse = false;

	/* Iterate through the item's collection path, until either the root collection node or a collection with the
	 * Mouse Usage is found - this prevents Joysticks, which use identical descriptors except for the Joystick usage
	 * parent node, from being erroneously treated as a mouse
	 */
	for (HID_CollectionPath_t* CurrPath = CurrentItem->CollectionPath; CurrPath != NULL; CurrPath = CurrPath->Parent)
	{
		if ((CurrPath->Usage.Page  == USAGE_PAGE_GENERIC_DCTRL) &&
		    (CurrPath->Usage.Usage == USAGE_MOUSE))
		{
			IsMouse = true;
			break;
		}
	}

	/* If a collection with the mouse usage was not found, indicate that we are not interested in this item */
	if (!IsMouse)
	  return false;

	/* Check the attributes of the current mouse item - see if we are interested in it or not;
	 * only store BUTTON and GENERIC_DESKTOP_CONTROL items into the Processed HID Report
	 * structure to save RAM and ignore the rest
	 */
	return ((CurrentItem->Attributes.Usage.Page == USAGE_PAGE_BUTTON) ||
	        (CurrentItem->Attributes.Usage.Page == USAGE_PAGE_GENERIC_DCTRL));
}

