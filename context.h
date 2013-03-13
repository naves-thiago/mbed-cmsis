#define TASK_STACK_SIZE 512

void context_mmc( void );     // Goto MMC Thread
void context_elua( void );    // Goto eLua Thread
void context_end_mmc( void ); // End MMC Thread and go back to eLua

typedef struct context {
  void * sp;        // Stack Pointer
  void * pc;        // Program pointer
  char active;      // If 0 scheduler will skip this task
  
} Context;
