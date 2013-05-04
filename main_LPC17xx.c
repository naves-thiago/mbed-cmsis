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

/*****************************************************************************
 * ** Function name:   I2C Init 
 * **
 * ** Descriptions:  Initializes the I2C port as a master controller
 * **
 * ** parameters:     portNum (0,1,2), clockDiv (1,2,4,8)
 * ** Returned value:   None
 * ** 
 * *****************************************************************************/
void I2CInit( int8_t portNum, uint8_t clockDiv)
{
  // Power on the peripheral
  const uint32_t i2c_pconp[3] = {1<<7, 1<<19, 1<<26};
  LPC_SC->PCONP |= i2c_pconp[portNum];

  // Set the clock
  uint32_t * i2c_clk_reg[3] = {&(LPC_SC->PCLKSEL0), &(LPC_SC->PCLKSEL01, &(LPC_SC->PCLKSEL1)};
  uint8_t i2c_clk_bits[3] = {14, 6, 20};

  // Clear clock bits
  *(i2c_clk_reg[portNum]) &= ~(3<<i2c_clk_bits[portNum]);

  // Set the right ones
  switch (clockDiv) {
    case 1: *(i2c_clk_reg[portNum]) |= 1<<i2c_clk_bits[portNum]; break;
    case 2: *(i2c_clk_reg[portNum]) |= 2<<i2c_clk_bits[portNum]; break;
    case 8: *(i2c_clk_reg[portNum]) |= 3<<i2c_clk_bits[portNum]; break;
  }
  
  // Change pin function
  switch (porNum) {
    case 0: 
      LPC_PINCON->PINSEL1 &= ~(3<<22 | 3<<24);
      LPC_PINCON->PINSEL1 |= 1<<22 | 1<<24;
      break;

    case 1: LPC_PINCON->PINSEL0 |= 3 | 3<<2;
            break;

    case 2: LPC_PINCON->PINSEL0 &= ~(3<<20 | 3<<22);
            LPC_PINCON->PINSEL0 |= 1<<21 | 1<<23; 
            break;
  }
}

/******************************************************************************
 * **                            End Of File
 * ******************************************************************************/

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


  uint32_t led = 1<<18;
  while(1) {
    LED_On (led);                               /* Turn on the LED. */
    Delay (100);                                /* delay  100 Msec */
    LED_Off (led);                              /* Turn off the LED. */
    Delay (100);                                /* delay  100 Msec */
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
