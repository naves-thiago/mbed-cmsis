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
  SPI functions 
 *----------------------------------------------------------------------------*/

/* Param: clk = desired clock
 * Retun: actual clock
 */
unsigned int spi_init_master( unsigned int clk )
{
  uint32_t SPCR = 0; // SPI Control Register
  uint32_t clk_tmp;  // Used to calculate the SPI Clock Counter
  uint8_t SPCCR;     // SPI clock counter register

  SPCR |= 0<<2;  // 8 bits mode
  SPCR |= 1<<3;  /* Data is sampled on the second clock edge of the SCK. 
                    A transfer starts with the first clock edge, and ends
                    with the last sampling edge when the SSEL signal is active.*/
  SPCR |= 0<<4;  // Clock polarity: active high
  SPCR |= 1<<5;  // Master Mode
  SPCR |= 1<<6;  // LSB (bit 0) first mode
  SPCR |= 1<<7;  // Enable SPI interrupts

  LPC_SPI->SPCR = SPCR;

  switch ((LPC_SC->PCLKSEL0 >>16) & 3)
  {
    case 0: clk_tmp = SystemCoreClock / 4;
    case 1: clk_tmp = SystemCoreClock;
    case 2: clk_tmp = SystemCoreClock / 2;
    case 3: clk_tmp = SystemCoreClock / 8;
  };

  // Calculate the clock counter ( note: PCCR must be even >= 8 ):
  SPCCR = clk_tmp / clk;
  if ( (SPCCR & 1) != 0)
    SPCCR--;

  if (SPCCR >= 8)
  {
    LPC_SPI->SPCCR = SPCCR;
    return SPCCR;
  }

  return 0;
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


  uint32_t led = 1<<18;
  while(1) {
    LED_On (led);                               /* Turn on the LED. */
    Delay (100);                                /* delay  100 Msec */
    LED_Off (led);                              /* Turn off the LED. */
    Delay (100);                                /* delay  100 Msec */
  }

}

