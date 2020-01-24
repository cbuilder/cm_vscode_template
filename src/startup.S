.syntax unified
.cpu cortex-m3
.thumb

.extern main
.extern hal_svc_entry
.extern hal_swi_entry
.extern hal_tick_entry
.extern hal_intr_entry
.extern hal_trap_entry

.section .vectors, "ax"
.code 16
.align 0
.global _vectors

_vectors:
.word     0x20008000                  // Top of interrupts stack
.word     RESET_Handler               // Reset Handler
.word     system_stop                 // NMI
.word     system_stop                 // HardFault
.word     system_stop                 // MPU Fault Handler
.word     system_stop                 // Bus Fault Handler
.word     system_stop                 // Usage Fault Handler
.word     0                           // Reserved
.word     0                           // Reserved
.word     0                           // Reserved
.word     0                           // Reserved
.word     system_stop                 // SVCall Handler
.word     system_stop                 // Debug Monitor Handler
.word     0                           // Reserved
.word     hal_swi_entry               // PendSV Handler
.word     hal_tick_entry              // SysTick Handler
.word     hal_intr_entry              // IRQ0
.word     hal_intr_entry              // IRQ1
.word     hal_intr_entry              // IRQ2
.word     hal_intr_entry              // IRQ3
.word     hal_intr_entry              // IRQ4
.word     hal_intr_entry              // IRQ5
.word     hal_intr_entry              // IRQ6
.word     hal_intr_entry              // IRQ7
.word     hal_intr_entry              // IRQ8
.word     hal_intr_entry              // IRQ9
.word     hal_intr_entry              // IRQ10
.word     hal_intr_entry              // IRQ11
.word     hal_intr_entry              // IRQ12
.word     hal_intr_entry              // IRQ13
.word     hal_intr_entry              // IRQ14
.word     hal_intr_entry              // IRQ15
.word     hal_intr_entry              // IRQ16
.word     hal_intr_entry              // IRQ17
.word     hal_intr_entry              // IRQ18
.word     hal_intr_entry              // IRQ19
.word     hal_intr_entry              // IRQ20
.word     hal_intr_entry              // IRQ21
.word     hal_intr_entry              // IRQ22
.word     hal_intr_entry              // IRQ23
.word     hal_intr_entry              // IRQ24
.word     hal_intr_entry              // IRQ25
.word     hal_intr_entry              // IRQ26
.word     hal_intr_entry              // IRQ27
.word     hal_intr_entry              // IRQ28
.word     hal_intr_entry              // IRQ29
.word     hal_intr_entry              // IRQ30
.word     hal_intr_entry              // IRQ31
.word     hal_intr_entry              // IRQ32

.section .text
.global RESET_Handler
.thumb_func
RESET_Handler:

copy_data_start:
  ldr   r0, =_etext
  ldr   r1, =_data
  ldr   r2, =_data_sz
  cmp   r2, $0
  beq   init_bss_start

copy_data:
  ldrb  r4, [r0]
  strb  r4, [r1]
  adds  r0, r0, $1
  adds  r1, r1, $1
  subs  r2, r2, $1
  bne   copy_data

init_bss_start:
  ldr   r0, =0
  ldr   r1, =_bss
  ldr   r2, =_bss_sz 
  cmp   r2, $0
  beq   image_init_done
        
init_bss:
  strb  r0, [r1]
  adds  r1, r1, $1
  subs  r2, r2, $1
  bne   init_bss
        
image_init_done:

  cpsid i
  b main                              // Jump to C entry.
