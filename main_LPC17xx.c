/******************************************************************************
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#define __LPC17XX__
#include "LPC17xx.h"
#include "MouseHost.h"


volatile uint32_t msTicks;                            /* counts 1ms timeTicks */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}

/*------------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *------------------------------------------------------------------------------*/
__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
  msTicks = 0;
}

/*------------------------------------------------------------------------------
  configer LED pins
 *------------------------------------------------------------------------------*/
__INLINE static void LED_Config(void) {

  LPC_GPIO1->FIODIR = 0xFFFFFFFF;               /* LEDs PORT1 are Output */
}

/*------------------------------------------------------------------------------
  Switch on LEDs
 *------------------------------------------------------------------------------*/
__INLINE static void LED_On (uint32_t led) {

  LPC_GPIO1->FIOPIN |=  (led);                  /* Turn On  LED */
}


/*------------------------------------------------------------------------------
  Switch off LEDs
 *------------------------------------------------------------------------------*/
__INLINE static void LED_Off (uint32_t led) {

  LPC_GPIO1->FIOPIN &= ~(led);                  /* Turn Off LED */
}

/*----------------------------------------------------------------------------
UART0 configuration
*----------------------------------------------------------------------------*/
uint32_t UARTInit( uint32_t PortNum, uint32_t baudrate )
{
  uint32_t Fdiv;
  uint32_t pclkdiv, pclk;

  if ( PortNum == 0 )
  {
    LPC_PINCON->PINSEL0 &= ~0x000000F0;
    LPC_PINCON->PINSEL0 |= 0x00000050; /* RxD0 is P0.3 and TxD0 is P0.2 */
    /* By default, the PCLKSELx value is zero, thus, the PCLK for
* all the peripherals is 1/4 of the SystemCoreClock. */
    /* Bit 6~7 is for UART0 */
    pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
    switch ( pclkdiv )
    {
      case 0x00:
      default:
        pclk = SystemCoreClock/4;
        break;
      case 0x01:
        pclk = SystemCoreClock;
        break;
      case 0x02:
        pclk = SystemCoreClock/2;
        break;
      case 0x03:
        pclk = SystemCoreClock/8;
        break;
    }

    LPC_UART0->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
    Fdiv = ( pclk / 16 ) / baudrate ; /*baud rate */
    LPC_UART0->DLM = Fdiv / 256;
    LPC_UART0->DLL = Fdiv % 256;
    LPC_UART0->LCR = 0x03; /* DLAB = 0 */
// LPC_UART0->FCR = 0x07; /* Enable and reset TX and RX FIFO. */
    LPC_UART0->IER = 1; /* Enable Receive Data Avaliable interrupt */
    NVIC_EnableIRQ( UART0_IRQn);

    return (1);
  }
  else if ( PortNum == 1 )
  {
    LPC_PINCON->PINSEL4 &= ~0x0000000F;
    LPC_PINCON->PINSEL4 |= 0x0000000A; /* Enable RxD1 P2.1, TxD1 P2.0 */

    /* By default, the PCLKSELx value is zero, thus, the PCLK for
* all the peripherals is 1/4 of the SystemCoreClock. */
    /* Bit 8,9 are for UART1 */
    pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
    switch ( pclkdiv )
    {
      case 0x00:
      default:
        pclk = SystemCoreClock/4;
        break;
      case 0x01:
        pclk = SystemCoreClock;
        break;
      case 0x02:
        pclk = SystemCoreClock/2;
        break;
      case 0x03:
        pclk = SystemCoreClock/8;
        break;
    }

    LPC_UART1->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
    Fdiv = ( pclk / 16 ) / baudrate ; /*baud rate */
    LPC_UART1->DLM = Fdiv / 256;
    LPC_UART1->DLL = Fdiv % 256;
    LPC_UART1->LCR = 0x03; /* DLAB = 0 */
// LPC_UART1->FCR = 0x07; /* Enable and reset TX and RX FIFO. */
    LPC_UART1->IER = 1; /* Enable Receive Data Avaliable interrupt */
    NVIC_EnableIRQ( UART1_IRQn);

    return 1;
  }
  return 0;
}

/*****************************************************************************
* ** Function name: UARTSend
* **
* ** Descriptions: Send a block of data to the UART 0 port based
* ** on the data length
* **
* ** parameters: portNum, buffer pointer, and data length
* ** Returned value: None
* **
* *****************************************************************************/
void UARTSend( uint32_t portNum, uint8_t *BufferPtr, uint32_t Length )
{
  if ( portNum == 0 )
  {
    while ( Length != 0 )
    {
      /* THRE status, contain valid data */
      while ( !(LPC_UART0->LSR & 1<<5) );
      LPC_UART0->THR = *BufferPtr;
      BufferPtr++;
      Length--;
    }
  }
  else
  {
    while ( Length != 0 )
    {
      /* THRE status, contain valid data */
      while ( !(LPC_UART1->LSR & 1<<5) );
      LPC_UART1->THR = *BufferPtr;
      BufferPtr++;
      Length--;
    }
  }
  return;
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
#define HOST_STATE_WaitForDeviceRemoval HOST_STATE_Default

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  *  passed to all HID Class driver functions, so that multiple instances of the same class
 *   *  within a device can be differentiated from one another.
 *    */
USB_ClassInfo_HID_Host_t Mouse_HID_Interface =
{
  .Config =
  {
    .DataINPipeNumber       = 1,
    .DataINPipeDoubleBank   = false,

    .DataOUTPipeNumber      = 2,
    .DataOUTPipeDoubleBank  = false,

    .HIDInterfaceProtocol   = HID_CSCP_MouseBootProtocol,
  },
};


int main (void) {

  SystemCoreClockUpdate();

  if (SysTick_Config(SystemCoreClock / 1000)) { /* Setup SysTick Timer for 1 msec interrupts  */
    while (1);                                  /* Capture error */
  }

  __enable_irq();
  LED_Config();                             
  UARTInit( 0, 9600 );

  UARTSend( 0, "Mouse Host Demo running.\r", 25 );

  for (;;)
	{
		switch (USB_HostState)
		{
			case HOST_STATE_Addressed:
      {
				uint16_t ConfigDescriptorSize;
				uint8_t  ConfigDescriptorData[512];

				if (USB_Host_GetDeviceConfigDescriptor(1, &ConfigDescriptorSize, ConfigDescriptorData,
				                                       sizeof(ConfigDescriptorData)) != HOST_GETCONFIG_Successful)
				{
          UARTSend( 0, "Error Retrieving Configuration Descriptor.\r", 43 );
					USB_HostState = HOST_STATE_WaitForDeviceRemoval;
					break;
				}

				if (HID_Host_ConfigurePipes(&Mouse_HID_Interface,
				                            ConfigDescriptorSize, ConfigDescriptorData) != HID_ENUMERROR_NoError)
				{
          UARTSend( 0,"Attached Device Not a Valid Mouse.\r", 35 );
					USB_HostState = HOST_STATE_WaitForDeviceRemoval;
					break;
				}

				if (USB_Host_SetDeviceConfiguration(1) != HOST_SENDCONTROL_Successful)
				{
          UARTSend( 0,"Error Setting Device Configuration.\r", 36 ); 
					USB_HostState = HOST_STATE_WaitForDeviceRemoval;
					break;
				}

				if (HID_Host_SetBootProtocol(&Mouse_HID_Interface) != 0)
				{
          UARTSend( 0,"Could not Set Boot Protocol Mode.\r", 34 ); 
					USB_HostState = HOST_STATE_WaitForDeviceRemoval;
					break;
				}

        UARTSend( 0,"Mouse Enumerated.\r", 18 ); 
				USB_HostState = HOST_STATE_Configured;
				break;
      }
			case HOST_STATE_Configured:
      {
				if (HID_Host_IsReportReceived(&Mouse_HID_Interface))
				{
					USB_MouseReport_Data_t MouseReport;
					HID_Host_ReceiveReport(&Mouse_HID_Interface, &MouseReport);

          UARTSend( 0, "Mouse Buttons: ", 15 );
          UARTSend( 0, MouseReport.Button, 1 );
          UARTSend( 0, "\r", 1 );
				}

				break;
      }
		}

		HID_Host_USBTask(&Mouse_HID_Interface);
		USB_USBTask();
	}
}

/** Event handler for the USB_DeviceAttached event. This indicates that a device has been attached to the host, and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Host_DeviceAttached(void)
{
 	UARTSend( 0, "Device Attached\r", 16 ); 
}

/** Event handler for the USB_DeviceUnattached event. This indicates that a device has been removed from the host, and
 *  stops the library USB task management process.
 */
void EVENT_USB_Host_DeviceUnattached(void)
{
 	UARTSend( 0, "Device Unattached\r", 18 ); 
}

/** Event handler for the USB_DeviceEnumerationComplete event. This indicates that a device has been successfully
 *  enumerated by the host and is now ready to be used by the application.
 */
void EVENT_USB_Host_DeviceEnumerationComplete(void)
{
 	UARTSend( 0, "Device Enumeration Complete\r", 28 ); 
}

/** Event handler for the USB_HostError event. This indicates that a hardware error occurred while in host mode. */
void EVENT_USB_Host_HostError(const uint8_t ErrorCode)
{
//	USB_Disable();

 	UARTSend( 0, "Host Mode Error\r", 16 ); 

	for(;;);
}

/** Event handler for the USB_DeviceEnumerationFailed event. This indicates that a problem occurred while
 *  enumerating an attached USB device.
 */
void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t ErrorCode,
                                            const uint8_t SubErrorCode)
{
 	UARTSend( 0, "Device Enumeration Failed\r", 26 ); 
}

