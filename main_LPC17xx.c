/**************************************************************************//**
 * @file     main.c
 * @brief    CMSIS Cortex-M3 Blinky example
 *           Blink a LED using CM3 SysTick
 * @version  V1.03
 * @date     24. September 2009
 *
 * @note
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#include "LPC17xx.h"

/*
 * LCD prototypes
 */
void lcd_write(uint8_t data);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init(void);



volatile uint32_t msTicks;                            /* counts 1ms timeTicks */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in delay() */
}

/*------------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *------------------------------------------------------------------------------*/
__INLINE static void delay (uint32_t dlyTicks) {
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
    LPC_PINCON->PINSEL0 |= 0x00000050;  /* RxD0 is P0.3 and TxD0 is P0.2 */
    /* By default, the PCLKSELx value is zero, thus, the PCLK for
     *  all the peripherals is 1/4 of the SystemCoreClock. */
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

    LPC_UART0->LCR = 0x83;    /* 8 bits, no Parity, 1 Stop bit */
    Fdiv = ( pclk / 16 ) / baudrate ; /*baud rate */
    LPC_UART0->DLM = Fdiv / 256;              
    LPC_UART0->DLL = Fdiv % 256;
    LPC_UART0->LCR = 0x03;    /* DLAB = 0 */
//    LPC_UART0->FCR = 0x07;    /* Enable and reset TX and RX FIFO. */
    LPC_UART0->IER = 1; /* Enable Receive Data Avaliable interrupt */
    NVIC_EnableIRQ( UART0_IRQn);

    return (1);
  }
  else if ( PortNum == 1 )
  {
    LPC_PINCON->PINSEL4 &= ~0x0000000F;
    LPC_PINCON->PINSEL4 |= 0x0000000A;  /* Enable RxD1 P2.1, TxD1 P2.0 */

    /* By default, the PCLKSELx value is zero, thus, the PCLK for
     *  all the peripherals is 1/4 of the SystemCoreClock. */
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

    LPC_UART1->LCR = 0x83;    /* 8 bits, no Parity, 1 Stop bit */
    Fdiv = ( pclk / 16 ) / baudrate ; /*baud rate */
    LPC_UART1->DLM = Fdiv / 256;              
    LPC_UART1->DLL = Fdiv % 256;
    LPC_UART1->LCR = 0x03;    /* DLAB = 0 */
//    LPC_UART1->FCR = 0x07;    /* Enable and reset TX and RX FIFO. */
    LPC_UART1->IER = 1; /* Enable Receive Data Avaliable interrupt */
    NVIC_EnableIRQ( UART1_IRQn);

    return 1; 
  }
  return 0; 
}

/*****************************************************************************
 * ** Function name:    UARTSend
 * **
 * ** Descriptions:   Send a block of data to the UART 0 port based
 * **           on the data length
 * **
 * ** parameters:     portNum, buffer pointer, and data length
 * ** Returned value:   None
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

/* --------------------------------------------------------------------------
  LCD functions
 * --------------------------------------------------------------------------*/

#define LCD_DELAY 10 // ms
#define RS (1<<15)
#define RW (1<<9)
#define EN (1<<8)

/*
 * p0.0 - b0
 * p0.1 - b1
 * p2.0 ~ p2.5 - b2~7
 * p0.8 - EN (active high)
 * p0.9 - R/W (low = write)
 * p0.15 - RS (low = Command)
 */

void lcd_init(void)
{
  /* Set write mode first */
  LPC_GPIO0->FIODIR |= RW;
  LPC_GPIO0->FIOPIN &= ~RW;
  delay(LCD_DELAY);

  /* Set all pins as output */
  LPC_GPIO0->FIODIR |= 3 | (3<<8) | (1<<15);
  LPC_GPIO2->FIODIR |= 0x3F;

  LPC_GPIO0->FIOPIN &= ~EN;

  // Initialize lcd (HD44780)
  lcd_send_cmd(0x38);   // function set:
                        // 8-bit interface, 2 display lines, 5x7 font
  lcd_send_cmd(0x06);   // entry mode set:
                        // increment automatically, no display shift
  lcd_send_cmd(0x0E);   // display control:
                        // turn display on, cursor on, no blinking
  lcd_send_cmd(0x01);   // clear display, set cursor position to zero
}

void lcd_send_cmd(uint8_t cmd)
{
  LPC_GPIO0->FIOPIN &= ~RS; // CMD mode
  LPC_GPIO0->FIOPIN |= EN;
  lcd_write(cmd);
  delay(LCD_DELAY);
  LPC_GPIO0->FIOPIN |= ~EN;
}

void lcd_send_data(uint8_t data)
{
  LPC_GPIO0->FIOPIN |= RS; // Data mode
  LPC_GPIO0->FIOPIN |= EN;
  lcd_write(data);
  delay(LCD_DELAY);
  LPC_GPIO0->FIOPIN |= ~EN;
}

void lcd_write(uint8_t data)
{
  LPC_GPIO0->FIOPIN &= ~3;
  LPC_GPIO0->FIOPIN |= data & 3;

  LPC_GPIO2->FIOPIN &= ~0x3F;
  LPC_GPIO2->FIOPIN |= (data >> 2) & 0x3F;
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

  SystemCoreClockUpdate();

  if (SysTick_Config(SystemCoreClock / 1000)) { /* Setup SysTick Timer for 1 msec interrupts  */
    while (1);                                  /* Capture error */
  }

  __enable_irq();
  LED_Config();                             
  UARTInit( 0, 9600 );

#if 1
  lcd_init();
  lcd_send_data('Y');
  lcd_send_data('a');
  lcd_send_data('y');
#endif

  LPC_GPIO0->FIODIR |= 3 | (3<<8) | (1<<15);
  LPC_GPIO2->FIODIR |= 0x3F;

  uint32_t led = 1<<18;
  while(1) {
//    LPC_GPIO0->FIOPIN |= 3 | (3<<8) | (1<<15);
//    LPC_GPIO2->FIOPIN |= 0x3F;

    LED_On (led);                               /* Turn on the LED. */
    delay (100);                                /* delay  100 Msec */
    LED_Off (led);                              /* Turn off the LED. */

//    LPC_GPIO0->FIOPIN &= ~(3 | (3<<8) | (1<<15));
//    LPC_GPIO2->FIOPIN &= ~(0x3F);

    delay (100);                                /* delay  100 Msec */
    UARTSend( 0, "1234567890---", 13 );
  }

}

/*
 * UART0 interrupt handler
 */
void UART0_IRQHandler(void) {
//  switch(LPC_UART0->IIR & 0x0E) {
//    case RDA_INTERRUPT:
//    case CTI_INTERRUPT:
      // Loopback to send
      LPC_UART0->THR = LPC_UART0->RBR;
//      break;
//  }
}
