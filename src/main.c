/*
  FX-RTOS demo application.
  Copyright (C) 2008-2015 Eremex Ltd. www.fxrtos.ru
*/

#include <MDR32Fx.h>
#include <FXRTOS.h>


void fx_intr_handler(void)
{
    ;
}

//
// LED-blinking thread 1.
//
static void thread_1(void* arg)
{
  while(1)
  {
    MDR_PORTC->RXTX |= 1<<2;
    fx_thread_sleep(20);
    MDR_PORTC->RXTX &= ~(1<<2);
    fx_thread_sleep(20);
  }
}

//
// LED-blinking thread 2.
//
static void thread_2(void* arg)
{
  while(1)
  {
    fx_thread_sleep(40);
  }
}

//
// LED-blinking thread 3.
//
static void thread_3(void* arg)
{
  while(1)
  {
    fx_thread_sleep(80);
  }
}

//
// Application definition.
//
void fx_app_init(void)
{
  static fx_thread_t t1,t2,t3; 
  
  static uint32_t stack0[ 0x200 / sizeof(uint32_t) ];
  static uint32_t stack1[ 0x200 / sizeof(uint32_t) ];
  static uint32_t stack2[ 0x200 / sizeof(uint32_t) ];

  fx_thread_init(&t1, thread_1, NULL, 10, (void*) stack0, sizeof(stack0), false);
  fx_thread_init(&t2, thread_2, NULL, 11, (void*) stack1, sizeof(stack1), false);
  fx_thread_init(&t3, thread_3, NULL, 12, (void*) stack2, sizeof(stack2), false);
}

//
// Exception handler just stop the system and sets LED7.
//
void system_stop(void)
{
    while(1); 
}

//
// Application entry point.
//
int main(void)
{
  SystemInit();
  //
  // Start system tick source.
  //
  SysTick->LOAD  = 100000 - 1;
  SysTick->VAL   = 0;
  SysTick->CTRL  = (1 << 2) | (1 << 0) | (1 << 1);

  //
  // Enable the peripheral clock
  //
  MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;
  //
  // Configure GPIOs for led driving
  //
  MDR_PORTC->OE     = 7;
  MDR_PORTC->FUNC   = 0x00000000;
  MDR_PORTC->ANALOG = 0x0000FFFF;
  MDR_PORTC->PULL   = 7 << 16;
  MDR_PORTC->PD     = ~7;
  MDR_PORTC->PWR    = 0x55555555;
  MDR_PORTC->GFEN   = 0;
  MDR_PORTC->RXTX   = 0xFFFF;

  //
  // Set the lowest possible priority for PendSV interrupt.
  //
  NVIC_SetPriority(PendSV_IRQn, 0xFF);

  //
  // Kernel start. This function must be called with interrupts disabled.
  //
  fx_kernel_entry();
  return 0;
}
