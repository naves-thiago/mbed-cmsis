#include "context.h" 
/*-----------------------------------------------------------------------*/
/* Context changing Functions                                            */
/*-----------------------------------------------------------------------*/

Context tasks[2];

void context_next( void )
{
  asm(
      "stmfd   sp!, {r0-r12}\n\r"              // Push registers
      "mrs     r0, cpsr\n\r"
      "stmfd   sp!, {r0, lr}\n\r"
      "ldr     r0, =other_stack\n\r"           // Get other_stack pointer
      "str     sp, [r0]\n\r"                   // Store SP into other_stack


void context_mmc( void )
{
  asm(
      "stmfd   sp!, {r0-r12}\n\r"              // Push registers
      "mrs     r0, cpsr\n\r"
      "stmfd   sp!, {r0, lr}\n\r"
      "ldr     r0, =other_stack\n\r"           // Get other_stack pointer
      "str     sp, [r0]\n\r"                   // Store SP into other_stack
      "ldr     sp, =other_stack\n\r"           // Swap stack
      "ldr     sp, [sp]\n\r"                   // Offset to USER_STACK 
      "ldr     r0, =mmc_pc\n\r"                // Load thread Function address
      "ldr     r0, [r0]\n\r"
      "bx      r0\n\r"                         // Start thread
      );
}

void context_elua( void )
{
  asm(
      "stmfd   sp!, {r0-r12, lr}\n\r"          // Push registers
      "mov     lr, sp\n\r"                     // Store SP into LR
      "ldr     r0, =other_stack\n\r"           // Get stack pointer pointer
      "ldr     sp, [r0]\n\r"                   // Get stack pointer (swap stack)
      "str     lr, [r0]\n\r"                   // Store the other SP into other_stack
      "ldmfd   sp!, {r0, lr}\n\r"              // Pop registers
      "msr     SPSR_cxsf, R0\n\r"
      "ldmfd   sp!, {r0-r12}\n\r"              // Pop registers
      "stmfd   sp!, {r0}\n\r"                  // Push r0
      "ldr     r0, =mmc_pc\n\r"                // Store PC into user_pc
      "str     pc, [r0]\n\r"
      "ldmfd   sp!, {r0}\n\r"                  // Pop r0
      "bx      lr\n\r"                         // Change thread
      );
}

void context_end_mmc( void )
{
//  mmc_pc = userThread;
  asm(
      "stmfd   sp!, {r0-r12, lr}\n\r"          // Push registers
      "mov     lr, sp\n\r"                     // Store SP into LR
      "ldr     r0, =other_stack\n\r"           // Get stack pointer pointer
      "ldr     sp, [r0]\n\r"                   // Get stack pointer (swap stack)
      "str     lr, [r0]\n\r"                   // Store the other SP into other_stack
      "ldmfd   sp!, {r0, lr}\n\r"              // Pop registers
      "msr     SPSR_cxsf, R0\n\r"
      "ldmfd   sp!, {r0-r12}\n\r"              // Pop registers
      "bx      lr\n\r"                         // Change thread
      );

}


