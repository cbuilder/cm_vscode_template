#define FX_METADATA(data) 
#ifndef _HAL_INIT_ARMv7M_LIB_HEADER_
#define _HAL_INIT_ARMv7M_LIB_HEADER_
/** 
  ****************************************************************************************************
  *  @file   CortexM/init/hal_init.h
  *  @brief  HAL initialization.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//!
//! User application.
//! It is called by HAL after system fully initialized at SPL = LOW level in context of first thread.
//!
void fx_app_init(void);
//!
//! Non-returning function used from user code in order to start scheduling. 
//!
void fx_kernel_entry(void);
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [HAL_INIT, ARMv7M_LIB] }))
FX_METADATA(({ options: [                                               
    HAL_INIT_INTR_STACK_SIZE: {                                                        
        type: int, range: [0x400, 0xffffffff], default: 0x1000,                     
        description: "Size of the interrupt stack (in bytes)."}]}))
//-----------------------------------------------------------------------------------------------------
#endif
#ifndef _CFG_OPTIONS_STD_CM3_GNU_HEADER_
#define _CFG_OPTIONS_STD_CM3_GNU_HEADER_
#define FX_SCHED_ALG_PRIO_NUM 32
#define FX_TIMER_THREAD_PRIO 1                              
#define FX_TIMER_THREAD_STACK_SIZE 0x400
#define HAL_INIT_INTR_STACK_SIZE 0x400
#define RTL_MEM_POOL_MAX_CHUNK 15
#ifndef __IAR_SYSTEMS_ASM__
FX_METADATA(({ interface: [CFG_OPTIONS, STANDARD_CORTEX_M3_GNU] }))
#endif
#endif
#ifndef _LANG_TYPES_OS_186_HEADER_
#define _LANG_TYPES_OS_186_HEADER_
/** 
  ****************************************************************************************************
  *  @file   lang_types.h
  *  @brief  Common language extensions, default headers and useful macros.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//
// These standard headers are required for all FX-RTOS distributions.
//
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
//----------------------------------------------------------------------------------------------------
//!
//! Error checking is disabled.
//!
#if (!defined LANG_ASSERT_ERROR_CHECKING_TYPE) || (LANG_ASSERT_ERROR_CHECKING_TYPE == 0)
# define lang_param_assert(assert, errcode) 
//----------------------------------------------------------------------------------------------------
//!
//! Classic error checking policy.
//!
#elif (LANG_ASSERT_ERROR_CHECKING_TYPE == 1)
# define lang_param_assert(assert, err_code) if (!(assert)) { return (err_code); }
//----------------------------------------------------------------------------------------------------
//!
//! Centralized error checkin policy: user function call on error.
//!
#elif (LANG_ASSERT_ERROR_CHECKING_TYPE == 2)
//!
//! User-supplied function for error handling.
//!
void fx_error_catch(const char* func, const char* err_code_str, int err_code);
//!
//! Helper macro to get error code as a string. 
//!
# define lang_param_assert_return_str(cond, err_code_str, err_code) \
    if (!(cond)) { fx_error_catch(__func__, err_code_str, err_code); }
//!
//! Parameter checking. 
//! @param assert Condition.
//! @param err_code Unique identifier of assert in target module (i.e. error code).
//! @warning This macro should be used inside functions only.
//!
# define lang_param_assert(assert, err_code) \
    lang_param_assert_return_str(assert, #err_code, err_code)
//----------------------------------------------------------------------------------------------------
#else
#error Unknown error checking type!
#endif
//----------------------------------------------------------------------------------------------------
#define FX_STATUS_OK 0
//----------------------------------------------------------------------------------------------------
#define lang_type_to_bits(type) (sizeof(type)*8)
#define lang_bits_to_words(n) (((n) + lang_type_to_bits(unsigned) - 1)/(lang_type_to_bits(unsigned)))
//----------------------------------------------------------------------------------------------------
#define lang_containing_record(address, type, field) \
    ((type*)(void*)((char*)(void*)(address) - (unsigned)(&((type*)0)->field)))
//----------------------------------------------------------------------------------------------------
#define __lang_static_assert(cond, line) typedef int static_assertion_##line[((!(!(cond))) * 2) - 1]
#define ___lang_static_assert(cond, line) __lang_static_assert(cond, line)
#define lang_static_assert(cond) ___lang_static_assert(cond, __LINE__)
//----------------------------------------------------------------------------------------------------
#define lang_min(a, b) ((a) < (b) ? (a) : (b))
#define lang_max(a, b) ((a) > (b) ? (a) : (b))
//----------------------------------------------------------------------------------------------------
    
FX_METADATA(({ interface: [LANG_TYPES, OS_186] }))
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ options: [                                               
    LANG_ASSERT_ERROR_CHECKING_TYPE: {                                            
        type: enum, values: [Disabled: 0, Classic: 1, Centralized: 2], default: 0,  
        description: "Default error checking policy." }]}))
#endif 
#ifndef _HAL_CPU_INTR_ARMv7M_V1_HEADER_
#define _HAL_CPU_INTR_ARMv7M_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   CortexM/intr_v7m/hal_cpu_intr.h
  *  @brief  HAL interrupt implementation for CortexM.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//!
//! Getting of current interrupt vector.
//! @return Vector of current ISR.
//! @warning This function may be called only in interrupt handler environment.
//!
unsigned int hal_intr_get_current_vect(void);
//!
//! User-supplied function which is called on every OS-managed interrupt.
//!
void fx_intr_handler(void);
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [HAL_CPU_INTR, ARMv7M_V1] }))
//-----------------------------------------------------------------------------------------------------
#endif
#ifndef _RTL_LIST_V1_HEADER_
#define _RTL_LIST_V1_HEADER_
/*
  * Copyright (c) 2005-2007, Kohsuke Ohtani
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions
  * are met:
  * 1. Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright
  *    notice, this list of conditions and the following disclaimer in the
  *    documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the author nor the names of any co-contributors
  *    may be used to endorse or promote products derived from this software
  *    without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
  * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
  * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
  * SUCH DAMAGE.
  */
typedef struct _rtl_list_t 
{
    struct _rtl_list_t  *next;
    struct _rtl_list_t  *prev;
}
rtl_list_t, rtl_list_linkage_t;
//----------------------------------------------------------------------------------------------------
#define rtl_list_init(head) ((head)->next = (head)->prev = (head))
#define rtl_list_next(node) ((node)->next)
#define rtl_list_prev(node) ((node)->prev)
#define rtl_list_empty(head) ((head)->next == (head))
#define rtl_list_first(head) ((head)->next)
#define rtl_list_last(head) ((head)->prev)
#define rtl_list_end(head, node) ((node) == (head))
#define rtl_list_is_node_linked(node) (((node)->next) && ((node)->prev))
#define rtl_list_entry(p, type, member) ((type*)((char*)(p) - (unsigned)(&((type*)0)->member)))
//----------------------------------------------------------------------------------------------------
static inline void
rtl_list_insert(rtl_list_t* prev, rtl_list_t* node)
{
    node->next = prev->next;
    node->prev = prev;
    prev->next->prev = node;
    prev->next = node;
}
//----------------------------------------------------------------------------------------------------
static inline void
rtl_list_remove(rtl_list_t* node)
{
    node->prev->next = node->next;
    node->next->prev = node->prev;
    node->next = node->prev = (rtl_list_linkage_t*)0;
}
//----------------------------------------------------------------------------------------------------
static inline void 
rtl_list_insert_range(rtl_list_t* dst, rtl_list_t* src)
{
    dst->prev->next = src->next;
    src->next->prev = dst->prev;
    dst->prev = src->prev;
    src->prev->next = dst;
}
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [RTL_LIST, V1] }))
#endif
#ifndef _HAL_BARRIER_UP_HEADER_
#define _HAL_BARRIER_UP_HEADER_
/** 
  ****************************************************************************************************
  *  @file   up/hal_barrier.h
  *  @brief  Stub implementation of low-level barriers.
  *  Barrier functions may only be called from uninterruptible SPL levels, so, on uniprocessor
  *  systems no other synchronization is needed.
  ****************************************************************************************************
  *  Copyright (C) 2008-2014 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//
// Barrier representation.
//
typedef struct _hal_barrier_t { int dummy; } hal_barrier_t;
#define hal_barrier_set(barrier, newval) 
#define hal_barrier_add(barrier, addend) ((void)(barrier), (void)(addend))
#define hal_barrier_wait(barrier, key)
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [HAL_BARRIER, UP] }))
#endif
#ifndef _FX_TIMER_THREADED_HEADER_
#define _FX_TIMER_THREADED_HEADER_
/**
  ****************************************************************************************************
  *  @file   threaded/fx_timer_internal.h
  *  @brief  Interface of application timers.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2014 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
enum
{
    FX_TIMER_OK = FX_STATUS_OK,
    FX_TIMER_ALREADY_CANCELLED,
    FX_TIMER_CONCURRENT_USE,
    FX_TIMER_INTERNAL_ERR_MAX
};
#define FX_TIMER_MAX_RELATIVE_TIMEOUT UINT32_C(0x7FFFFFFF)
//----------------------------------------------------------------------------------------------------
struct _fx_timer_internal_context_t;
//!
//! Timer representation. 
//!
typedef struct _fx_timer_internal_t
{
    uint32_t timeout;                   //!< Timer deadline.
    uint32_t period;                    //!< Period. It is 0 for one-shot timers.
    int(*callback)(void*);              //!< Callback function to be called on timer expiration.
    void* callback_arg;                 //!< Argument of the callback.   
    rtl_list_linkage_t link;            //!< Timers queue linkage.
    struct _fx_timer_internal_context_t* volatile context; //!< Each armed timer is linked to some CPU.
    hal_barrier_t completion;           //!< Barrier is set when timer callback done.
}
fx_timer_internal_t;
//----------------------------------------------------------------------------------------------------
int fx_timer_internal_init(fx_timer_internal_t* timer, int(*func)(void*), void* arg);
uint32_t fx_timer_get_tick_count(void);
void fx_timer_set_tick_count(uint32_t ticks);
#define fx_timer_time_after(a, b) (((int32_t)(b) - (int32_t)(a)) < 0)
#define fx_timer_time_after_or_eq(a, b) (((int32_t)(b) - (int32_t)(a)) <= 0)
int fx_timer_internal_set_rel(fx_timer_internal_t* timer, uint32_t delay, uint32_t period);
int fx_timer_internal_set_abs(fx_timer_internal_t* timer, uint32_t delay, uint32_t period);
int fx_timer_internal_cancel(fx_timer_internal_t* timer);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_TIMER_INTERNAL, THREADED] }))
#endif
#ifndef _HAL_CPU_CONTEXT_KER_FRAME_BASED_HEADER_
#define _HAL_CPU_CONTEXT_KER_FRAME_BASED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   common/context/hal_cpu_context.h
  *  @brief  CPU context management functions.
  *
  *  When interrupt module uses the interrupt frame structure on interrupt entry/exit it may
  *  be used for context switching. If scheduler is implemened as software interrupt handler, then
  *  current frame pointer may be changed as a result of scheduling, in this case, interrupt exit 
  *  causes switching to new thread.
  *  This module may be used for any HAL wothout usermode threads support.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2013 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//-----------------------------------------------------------------------------------------------------
struct _hal_intr_frame_t;
//
// Hardware context contains only pointer to the current stack frame containing full register state.
//
typedef struct _hal_cpu_context_t
{
    struct _hal_intr_frame_t* frame;  
}
hal_cpu_context_t;
//-----------------------------------------------------------------------------------------------------
//!
//! Context initialization.
//! @param [in] context Pointer to context structure to be initialized.
//! @param [in] stack Kernel stack pointer to be used in new thread.
//! @param [in] entry Entry point for the new thread.
//! @param [in] arg Argument to be passed in entry-point function.
//!
void hal_context_ker_create(hal_cpu_context_t* context, uintptr_t stk, uintptr_t entry, uintptr_t arg);
//!
//! Context deinitialization.
//! @param [in] context Pointer to context structure to be deinitialized.
//!
#define hal_context_ker_delete(context)
//!
//! Context switching.
//! Context switch may be called ONLY in context of dispatch interrupt handler. 
//! @param [in] new_ctx New context to be set.
//! @param [out] old_ctx Previous context (context of current interrupted thread).
//!
void hal_context_switch(hal_cpu_context_t* new_ctx, hal_cpu_context_t* old_ctx);
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [HAL_CPU_CONTEXT, KER_FRAME_BASED] }))
//-----------------------------------------------------------------------------------------------------
#endif
#ifndef _FX_PROCESS_DISABLED_HEADER_
#define _FX_PROCESS_DISABLED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_process.h
  *  @brief  Stub for disabling process subsystem. This module is tightly coupled with threads impl.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2014 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//
// When process support is disabled the kernel does not handle hardware exceptions, only TERM
// exception is supported for internal kernel use.
//
#define FX_EXCEPTION_TERM 0
//
// Process object is empty when process support is disabled.
//
typedef struct _fx_process_t { int dummy; } fx_process_t;
//
// If only one function may be set as exception handler for single exception id then "set" functions
// may be safely disabled and "get" function should always return default handler.
//
typedef void (*fx_process_exception_handler_t)(void* target, unsigned id, void* arg);
//
// Exceptions are not supported. 
//
#define fx_process_set_exception(id, h, old)
extern void fx_thread_term_handler(void* target, unsigned id, void* arg);
#define fx_process_get_exception(exc_id) (&fx_thread_term_handler)
//
// Current process is always NULL. 
//
#define fx_process_self() (NULL)
//
// Process switching is not performed by the kernel.
//
#define fx_process_switch(newp, oldp)
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_PROCESS, DISABLED] }))
#endif
#ifndef _RTL_QUEUE_V1_HEADER_
#define _RTL_QUEUE_V1_HEADER_
/*
  * Copyright (c) 2005-2007, Kohsuke Ohtani
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions
  * are met:
  * 1. Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright
  *    notice, this list of conditions and the following disclaimer in the
  *    documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the author nor the names of any co-contributors
  *    may be used to endorse or promote products derived from this software
  *    without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
  * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
  * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
  * SUCH DAMAGE.
  */
#include <stddef.h>
//----------------------------------------------------------------------------------------------------
typedef struct _rtl_queue 
{
    struct _rtl_queue* next;
    struct _rtl_queue* prev;
}
rtl_queue_t, rtl_queue_linkage_t;
//----------------------------------------------------------------------------------------------------
#define rtl_queue_init(head) ((head)->next = (head)->prev = (head))
#define rtl_queue_empty(head) ((head)->next == (head))
#define rtl_queue_next(q) ((q)->next)
#define rtl_queue_prev(q) ((q)->prev)
#define rtl_queue_first(head) ((head)->next)
#define rtl_queue_last(head) ((head)->prev)
#define rtl_queue_end(head,q) ((q) == (head))
#define rtl_queue_is_item_linked(q) (((q)->next) && ((q)->prev))
#define rtl_queue_entry(q, type, member) ((type*)((char*)(q) - (unsigned)(&((type*)0)->member)))
#define rtl_queue_item_init(item) ((item)->next = (item)->prev = NULL)
#define RTL_QUEUE_INITIALIZER {NULL, NULL}
void rtl_enqueue(rtl_queue_t*, rtl_queue_t*);
rtl_queue_t* rtl_dequeue(rtl_queue_t*);
void rtl_queue_insert(rtl_queue_t*, rtl_queue_t*);
void rtl_queue_remove(rtl_queue_t*);
void rtl_queue_copy(rtl_queue_t* dst, rtl_queue_t* src);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [RTL_QUEUE, V1] }))
#endif
#ifndef _FX_SCHED_ALG_MPQ_FIFO_HEADER_
#define _FX_SCHED_ALG_MPQ_FIFO_HEADER_
/** 
  ****************************************************************************************************
  *  @file   mpq/fx_sched_alg.h
  *  @brief  Interface of scheduler container.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
FX_METADATA(({ interface: [FX_SCHED_ALG, MPQ_FIFO] }))
FX_METADATA(({ options: [                                               
    FX_SCHED_ALG_PRIO_NUM: {                                                      
        type: int, range: [8, 1024], default: 64,                                   
        description: "Number of scheduling priorities."}]}))
//----------------------------------------------------------------------------------------------------
#ifndef FX_SCHED_ALG_PRIO_NUM
#define FX_SCHED_ALG_PRIO_NUM 64
#endif
#if FX_SCHED_ALG_PRIO_NUM > 1024
#error Too many priority levels, it must be less than 1024!
#endif
//----------------------------------------------------------------------------------------------------
//! 
//! Scheduler container representation. 
//! For FIFO scheduler it contains N queues for each priority and bitmaps for fast search.
//!
typedef struct _fx_sched_container_t
{
    rtl_queue_t priority_queues[FX_SCHED_ALG_PRIO_NUM];
    unsigned map1;
    unsigned map2[lang_bits_to_words(FX_SCHED_ALG_PRIO_NUM)];
}
fx_sched_container_t;
//!
//! Scheduler container's item description. 
//!
typedef struct _fx_sched_params_t
{
    unsigned prio;              //!< Priority value.
    rtl_queue_linkage_t link;   //!< Linkage to queue of items with same priority.
} 
fx_sched_params_t;
//!
//! Lowest priority has maximum numeric value starting from 0.
//!
#define FX_SCHED_ALG_PRIO_IDLE (FX_SCHED_ALG_PRIO_NUM - 1)
//!
//! Cast scheduling params as integer. It is also used for "priority visualization" by external tools.
//!
#define fx_sched_params_as_number(s) ((s)->prio)
//!
//! Checks whether sched params A preempts parameters B. Used as items comparator.
//!
#define fx_sched_params_is_preempt(a, b) ((a)->prio < (b)->prio)
//!
//! Check for params equality.
//!
#define fx_sched_params_is_equal(a, b) ((a)->prio == (b)->prio)
//!
//! Checks whether the item unique at given priority level. It means that no pending work exist.
//!
#define fx_sched_params_is_unique(a) (rtl_queue_first(&((a)->link)) == rtl_queue_last(&((a)->link)))
//!
//! Copy constructor.
//! @param src Source item from which params will be copied to destination item.
//! @param dst Destination item.
//!
#define fx_sched_params_copy(src, dst) (dst)->prio = (src)->prio
//!
//! Schedulable item initializer. (priority-specific method).
//! Initialize schedulable item by specified priority. This method is intended only for libraries
//! and user programs which have knowledge about type of scheduler (about priorities). 
//! System itself never calls this method.
//! @param [in] item Schedulable item to be initialized.
//! @param [in] priority Priority which to be set in schedulable item.
//!
#define fx_sched_params_init_prio(item, priority) (item)->prio = (priority)
//! 
//! Default initializers for scheduler container's items. 
//!
typedef enum _fx_sched_params_init_t
{
    FX_SCHED_PARAMS_INIT_IDLE = 0,  //!< This value is being used by system to create IDLE-entity.
    FX_SCHED_PARAMS_INIT_DEFAULT,   //!< Default priority.
    FX_SCHED_PARAMS_INIT_SPECIFIED  //!< Copy scheduling params from another item.
}
fx_sched_params_init_t;
void fx_sched_params_init(fx_sched_params_t*, fx_sched_params_init_t, const fx_sched_params_t*);
void fx_sched_container_init(fx_sched_container_t* container);
void fx_sched_container_add(fx_sched_container_t* container, fx_sched_params_t* item);
void fx_sched_container_remove(fx_sched_container_t* container, fx_sched_params_t* item);
fx_sched_params_t* fx_sched_container_get(fx_sched_container_t* container);
#endif
#ifndef _HAL_ASYNC_ARMv7M_UNIFIED_HEADER_
#define _HAL_ASYNC_ARMv7M_UNIFIED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   CortexM/sync/unified/hal_async.h
  *  @brief  SPL management functions for unified interrupt architecture.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
FX_METADATA(({ interface: [HAL_ASYNC, ARMv7M_UNIFIED] }))
//!
//! Constants for SPL levels. These constants are tightly coupled with PRIMASK register format, so,
//! they must not be changed!
//!
typedef enum
{
    SPL_SYNC = 0x01,
    SPL_DISPATCH = SPL_SYNC,
    SPL_LOW = 0x00
}
spl_t;
spl_t hal_async_raise_spl(const spl_t new_spl); 
void hal_async_lower_spl(const spl_t new_spl);
spl_t hal_async_get_current_spl(void);
#define ICSR_ADDR 0xE000ED04
#define hal_async_request_swi(ignored) ((*(volatile unsigned int*)ICSR_ADDR) = 0x10000000)
                
#endif
#ifndef _FX_DBG_V1_HEADER_
#define _FX_DBG_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_dbg.h
  *  @brief  Debug interface for assertions.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2013 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//
// This function stops execution and optionally calls user-defined callback, if debug is enabled.
// If debug is disabled it stops the system silently.
//
void fx_panic_internal(const char* msg, const char* funcname);
//
// Panic macro uses C99 feature __func__, which is defined in each function as 
// const char* __func__ = "<funcion name>";
//
#define fx_panic(msg) fx_panic_internal(msg, __func__)
//!
//! Debug assertion macro. It causes system panic if condition is not satisfied.
//! It should be an expression.
//!
#ifdef FX_DBG_ENABLED
#define fx_dbg_assert(cond) ((!(cond)) ? fx_panic("Debug assertion"), 1 : 0)
#else
#define fx_dbg_assert(cond) ((void)0)
#endif
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_DBG, V1] }))
#endif 
  
#ifndef _TRACE_LOCKS_STUB_HEADER_
#define _TRACE_LOCKS_STUB_HEADER_
/** 
  ****************************************************************************************************
  *  @file   trace_locks.h
  *  @brief  Stub for trace subsystem. Disables spl tracing.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2014 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
#define trace_intr_lock()         ((void)0)
#define trace_intr_unlock()       ((void)0)
#define trace_dispatch_lock()     ((void)0)
#define trace_dispatch_unlock()   ((void)0)
#define trace_lock_enter(lock)    ((void)0)
#define trace_lock_leave(lock)    ((void)0)
FX_METADATA(({ interface: [TRACE_LOCKS, STUB] }))
#endif 
#ifndef _HAL_MP_STUB_V1_HEADER_
#define _HAL_MP_STUB_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   common/mp/hal_mp.h
  *  @brief  Multiprocessor interface stub for uniprocessor systems.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
#define HAL_MP_CPU_MAX 1
#define hal_mp_get_current_cpu() 0
#define hal_mp_get_cpu_count() 1
//
// IPI may be sent only to itself, just map it to software interrupt request ignoring CPU number.
//
#define hal_mp_request_ipi(cpu, level) ((void) (cpu), hal_async_request_swi(level))
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [HAL_MP, STUB_V1] }))
//-----------------------------------------------------------------------------------------------------
#endif
#ifndef _FX_SPL_UNIFIED_UP_HEADER_
#define _FX_SPL_UNIFIED_UP_HEADER_
/** 
  ****************************************************************************************************
  *  @file   unified_up/fx_spl.h
  *  @brief  Definitions for interruptible unified synchronization model for uniprocessor systems.
  *          In this sync scheme OS kernel operates at sync level with interrupts disabled.
  *          Spinlocks raise level to sync (disabling interrupts) from any level.
  *          Interrupts handlers run at level higher than DISPATCH and lower than SYNC, therefore,
  *          OS services may be used from interrupts directly.
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//-----------------------------------------------------------------------------------------------------
//
// In unified architecture scheduler operates at uninterruptible SPL level.
// 
#define FX_SPL_SCHED_LEVEL SPL_SYNC
lang_static_assert(SPL_DISPATCH == SPL_SYNC);
lang_static_assert(HAL_MP_CPU_MAX == 1);
//-----------------------------------------------------------------------------------------------------
//
// Abstract lock on uniprocessor systems is implemented just as uninterruptible SPL level.
//
typedef struct _lock_t { spl_t old_spl; } lock_t;
typedef spl_t fx_lock_intr_state_t;
//-----------------------------------------------------------------------------------------------------
//
// Initialize lock with dummy value (should be overwritten when lock is acquired).
//
#define fx_spl_spinlock_init(lock) (lock)->old_spl = SPL_LOW
//
// Scheduler works at SYNC level, so, in uniprocessor machine at scheduler level no need to do something, 
// interrupts are already disabled, just verify assert.
//
#define fx_spl_spinlock_get_from_sched(lock) ((void) (lock), trace_lock_enter(dummy))
#define fx_spl_spinlock_put_from_sched(lock) ((void) (lock), trace_lock_leave(dummy))
//
// In unified architecture scheduler works at level above all software and hardware interrupts, so,
// "higher" level is not guarantee SYNC. Locks from higher level on uniprocessor system should
// raise SPL.
//
#define fx_spl_spinlock_get_from_any(lock) \
  (fx_dbg_assert(hal_async_get_current_spl() != SPL_LOW),\
  (lock)->old_spl = hal_async_raise_spl(SPL_SYNC),\
  trace_lock_enter(dummy))
#define fx_spl_spinlock_put_from_any(lock) (trace_lock_leave(dummy), hal_async_lower_spl((lock)->old_spl))
//
// Locking scheduler from any allowed level (in unified architecture all levels are allowed).
//
#define fx_spl_raise_to_sched_from_low(prev_state) \
  ((*(prev_state)) = hal_async_raise_spl(SPL_SYNC), trace_dispatch_lock())
#define fx_spl_lower_to_low_from_sched(prev_state) (trace_dispatch_unlock(), hal_async_lower_spl(prev_state))
//
// Because in unified architecture all levels are allowed to lock the scheduler, SPL should
// be raised to SYNC.
//
#define fx_spl_raise_to_sched_from_disp(prev_state) \
  ((*(prev_state)) = hal_async_raise_spl(SPL_SYNC), trace_dispatch_lock())
#define fx_spl_lower_to_disp_from_sched(prev_state) (trace_dispatch_unlock(), hal_async_lower_spl(prev_state))
//
// Local interrupt locking.
//
#define fx_spl_raise_to_sync_from_any(old_state) ((*(old_state)) = hal_async_raise_spl(SPL_SYNC), trace_intr_lock())
#define fx_spl_lower_to_any_from_sync(old_state) (trace_intr_unlock(), hal_async_lower_spl(old_state))
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_SPL, UNIFIED_UP] }))
#endif
#ifndef _FX_SCHED_UP_FIFO_HEADER_
#define _FX_SCHED_UP_FIFO_HEADER_
/**
  ****************************************************************************************************
  *  @file   fx_sched.h
  *  @brief  Interface header for global uniprocessor scheduler.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
//!
//! Schedulable entity.
//!
typedef struct _fx_sched_item_t
{
    unsigned int suspend_count;     //!< Suspend counter. Item suspended if this value is nonzero.
    fx_sched_params_t sched_params; //!< Associated parameters, priority, deadline and so on.
}
fx_sched_item_t;
//
// Type conversions.
//
#define fx_sched_item_as_sched_params(item) (&((item)->sched_params))
//----------------------------------------------------------------------------------------------------
//!
//! Constructor of global scheduler module.
//! It should be called on each processor in the system.
//!
void fx_sched_ctor(void);
//
// Schedulabe item initialization.
//
void fx_sched_item_init(fx_sched_item_t* item, const fx_sched_params_init_t t, const fx_sched_params_t* arg);
//
// Add schedulable item to the scheduler. 
// In current implementation actual work is done in @ref fx_sched_item_resume.
//
#define fx_sched_item_add(item)
//
// Remove schedulable from the scheduler. 
//
void fx_sched_item_remove(fx_sched_item_t* item);
//!
//! Get scheduling parameters of schedulable entity.
//! @param [in] src Source of scheduling params.
//! @param [in,out] dst Placeholder of scheduling parameters extracted from source item.
//!
#define fx_sched_item_get_params(src, dst) fx_sched_params_copy(fx_sched_item_as_sched_params(src), dst)
//
// Set scheduling params for specified item. 
//
void fx_sched_item_set_params(fx_sched_item_t* dst, const fx_sched_params_t* src);
//
// Suspend schedulable entity. This may be called only by entity itself. 
//
unsigned int fx_sched_item_suspend(fx_sched_item_t* item);
//
// Activate schedulable entity. 
//
unsigned int fx_sched_item_resume(fx_sched_item_t* item);
//!
//! Place current thread at the end of scheduler queue.
//!
bool fx_sched_yield(fx_sched_item_t* item);
//
// Select next item to run.
//
fx_sched_item_t* fx_sched_get_next(void);
//
// Force rescheduling. 
//
void fx_sched_mark_resched_needed(void);
//
// Affinity API. On uniprocessor systems it is implemented as a stubs.
//
typedef int fx_sched_affinity_t;
#define fx_sched_set_affinity(item, affinity_ptr, self) ((void)(*(affinity_ptr)))
#define fx_sched_get_affinity(item, affinity_ptr) ((void)(*(affinity_ptr)))
//
// On uniprocessor system CPU number should be always 0. 
//
#define fx_sched_get_cpu(item) 0
//----------------------------------------------------------------------------------------------------
typedef spl_t fx_sched_state_t;
#define fx_sched_lock(prev_ptr) fx_spl_raise_to_sched_from_low(prev_ptr)
#define fx_sched_unlock(prev_sched_state) fx_spl_lower_to_low_from_sched(prev_sched_state)
#define fx_sched_lock_from_disp_spl(prev_state) fx_spl_raise_to_sched_from_disp(prev_state)
#define fx_sched_unlock_from_disp_spl(prev_state) fx_spl_lower_to_disp_from_sched(prev_state)
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_SCHED, UP_FIFO], ctor: [fx_sched_ctor, on_boot_cpu] }))
//----------------------------------------------------------------------------------------------------
#endif
#ifndef _FX_SYNC_UP_QUEUE_HEADER_
#define _FX_SYNC_UP_QUEUE_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_sync.h
  *  @brief  Basic synchronization layer. 
  *          It provides simple wait/notify interface, which can be used to implement sync primitives.
  *          This implementation supports uniprocessor systems only.
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
struct _fx_sync_waiter_t;           // Abstract waiter class, such as thread.
struct _fx_sync_waitable_t;         // Waitable object representing queue of waiters.
struct _fx_sync_wait_block_t;       // Wait block representing link between waiter and waitable object.
//----------------------------------------------------------------------------------------------------
//!
//! Scheduling policy for waitable object queue.
//!
typedef enum _fx_sync_policy_t
{
    FX_SYNC_POLICY_FIFO = 0,        //!< Release waiters in FIFO order.
    FX_SYNC_POLICY_PRIO = 1,        //!< Release waiter in priority order.
    FX_SYNC_POLICY_MAX,
    FX_SYNC_POLICY_DEFAULT = FX_SYNC_POLICY_FIFO
}
fx_sync_policy_t;
//!
//! Status of wait operation.
//!
typedef enum _fx_wait_status_t
{
    FX_WAIT_IN_PROGRESS = 0,        //!< Wait still in progress, wait blocks reside in the queue.
    FX_WAIT_SATISFIED   = 1,        //!< Wait is satisfied normally, wait block is removed from queue.
    FX_WAIT_CANCELLED   = 2,        //!< Wait is cancelled (reason determined by caller).
    FX_WAIT_DELETED     = 3,        //!< Waitable object become invalid during waiting.
    FX_WAIT_STATUS_MAX
}
fx_wait_status_t;
//----------------------------------------------------------------------------------------------------
//!
//! Waitable object representation. 
//! It is base class for all synchronization primitives.
//!
typedef struct _fx_sync_waitable_t
{
    //!
    //! Queue of wait blocks, each represents waiter of this object.
    //!
    rtl_queue_t wait_list;          
    //!
    //! Waitable test callback. It return false in case when wait block is actually inserted into the 
    //! queue.
    //!
    bool (*test_wait)(
        struct _fx_sync_waitable_t*, 
        struct _fx_sync_wait_block_t*, 
            const bool);
} 
fx_sync_waitable_t;
//!
//! Type cast to abstract queue. May be used to traverse through all waiters.
//!
#define fx_sync_waitable_as_queue(w) (&((w)->wait_list))
//!
//! Lock object.
//! It is assumed that this function is ALWAYS called from SPL = SCHED_LEVEL.
//! Since on uniprocessor systems SPL is already guarantees proper synchronization, function is unused.
//!
#define fx_sync_waitable_lock(w)
//!
//! Unlock object.
//! It is assumed that this function is ALWAYS called from SPL = SCHED_LEVEL.
//! Since on uniprocessor systems SPL is already guarantees proper synchronization, function is unused.
//!
#define fx_sync_waitable_unlock(w)
//!
//! Checks whether the wait queue is nonempty.
//!
#define _fx_sync_waitable_has_waiters(w) (!rtl_queue_empty(fx_sync_waitable_as_queue(w)))
void fx_sync_waitable_init(
    fx_sync_waitable_t*, 
    void*, 
    bool (*)(struct _fx_sync_waitable_t*, struct _fx_sync_wait_block_t*, const bool));
void _fx_sync_wait_start(fx_sync_waitable_t*, struct _fx_sync_wait_block_t*);
struct _fx_sync_wait_block_t* _fx_sync_wait_block_get(fx_sync_waitable_t*, fx_sync_policy_t);
void _fx_sync_wait_notify(fx_sync_waitable_t*, fx_wait_status_t, struct _fx_sync_wait_block_t*);
//----------------------------------------------------------------------------------------------------
//!
//! Base class of waiter.
//! N.B. Waiter methods is NOT thread safe. It is expected that waiter is a thread and therefore
//! all waiter methods are called in context of one thread (sequentially).
//!
typedef struct _fx_sync_waiter_t
{
    fx_sched_params_t* sched_params;  //!< Scheduling params associated with the waiter.
    struct _fx_sync_wait_block_t* wb; //!< Wait blocks allocated by waiter.
    unsigned int wb_num;              //!< Wait blocks array size.  
}
fx_sync_waiter_t;
//!
//! Type cast to scheduling parameters.
//!
#define fx_sync_waiter_as_sched_params(w) ((w)->sched_params)
//!
//! Waiter initializer. Note that parameters are saved by reference not by value!
//!
#define fx_sync_waiter_init(w, params) (w)->sched_params = params
//!
//! Preparing waiter for new wait operation. Should be perfermed before every wait operation.
//! This implementation supports only waits by OR, so, expected notification number is always
//! 1 and therefore unused.
//!
#define fx_sync_waiter_prepare(w, wb_array, wb_n, call_num) (w)->wb = (wb_array), (w)->wb_num = (wb_n)
//!
//! This function is used for optimizations between object test and thread suspend. Don't use it
//! in application code.
//!
#define fx_sync_is_waiter_satisfied(w) false
//!
//! Cancel any pending wait operations for given waiter.
//!
unsigned int fx_sync_wait_rollback(fx_sync_waiter_t* waiter);
//!
//! External function for waiter notification. Should be implemented in derived classes.
//!
void fx_sync_waiter_notify(fx_sync_waiter_t* waiter);
//----------------------------------------------------------------------------------------------------
//!
//! Representation of wait block. Wait block is a link between waiter and waitable.
//! Attribute is used to pass some info from waiter to primitive's logic. This is also used
//! in order to return primitive-specific info from wait functions.
//! N.B. Attribute is used before wait starts, so, it may never be used simultaneously with status,
//! which indicates status of notification, therefore they may reside in same memory location.
//! Warning! Do not use values reserved for status as attributes! In case when it is needed to
//! implement primitive's logic use indirect parameter passing (when attribute is a pointer to
//! memory location holding actual value).
//!
typedef struct _fx_sync_wait_block_t
{
    fx_sync_waiter_t* waiter;       //!< Waiter object which waits using this wait block.
    fx_sync_waitable_t* waitable;   //!< Synchronization primitive that owns this wait block.
    union
    {
        void* attribute;            //!< Wait operation may be associated with some attributes.
        fx_wait_status_t status;    //!< Status of this wait block.
    } u;                                        
    rtl_queue_linkage_t link;       //!< Link to waitable's queue.
}
fx_sync_wait_block_t;
#define fx_sync_wb_as_queue_item(wb) (&((wb)->link))
#define fx_sync_queue_item_as_wb(item) (rtl_queue_entry(item, fx_sync_wait_block_t, link))
#define FX_SYNC_WAIT_BLOCK_INITIALIZER(wtr, wtbl, attr) {(wtr), NULL, {attr}, RTL_QUEUE_INITIALIZER}
#define fx_sync_wait_block_get_status(wb) ((wb)->u.status)
#define fx_sync_wait_block_get_attr(wb) ((wb)->u.attribute)
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_SYNC, UP_QUEUE] }))
//----------------------------------------------------------------------------------------------------
#endif
#ifndef _FX_RTP_DISABLED_HEADER_
#define _FX_RTP_DISABLED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_rtp_disabled.h
  *  @brief  Stub of run-time protection module. Disables run-time checks of object consistency.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
typedef struct { int dummy; } _fx_rtp_t, fx_rtp_part_t, fx_rtp_key_t, fx_rtp_t;
//----------------------------------------------------------------------------------------------------
#define fx_rtp_init(target, key)
#define fx_rtp_deinit(target)
#define fx_rtp_check(target, key) (true)
//
// Partial object protection does not need a destructor.
//
#define fx_rtp_part_init(target, key) ((void)0)
#define fx_rtp_part_check(target, key) (true)
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_RTP, DISABLED] }))
#endif
#ifndef _FX_EVENT_V1_HEADER_
#define _FX_EVENT_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_event.h
  *  @brief  Interface of simple events.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
enum
{
    FX_EVENT_MAGIC = 0x45564E54, // EVNT
    FX_EVENT_OK = FX_STATUS_OK,
    FX_EVENT_INVALID_PTR = 1,
    FX_EVENT_INVALID_OBJ = 2,
    FX_EVENT_ERR_MAX
};
//----------------------------------------------------------------------------------------------------
//!
//! Internal event is embeddable object which does not perform any validation & error checking.
//!
typedef struct _fx_event_internal_t
{
    fx_sync_waitable_t waitable;  //!< Waitable object as a base class.
    bool state;                   //!< Event state.
    lock_t lock;                  //!< Low-level primitive to protect the event itself.
} 
fx_event_internal_t;
#define fx_internal_event_as_waitable(event) (&((event)->waitable))
//----------------------------------------------------------------------------------------------------
//!
//! Event representation. 
//!
typedef struct _fx_event_t
{
    fx_event_internal_t object;   //!< Embedded internal event.  
    fx_rtp_t rtp;                 //!< Run-time protection member.
} 
fx_event_t;
//----------------------------------------------------------------------------------------------------
#define fx_event_as_internal_event(event) (&((event)->object))
#define fx_event_as_waitable(event) fx_internal_event_as_waitable(fx_event_as_internal_event(event))
#define fx_event_as_rtp(event) (&((event)->rtp))
#define fx_event_is_valid(event) (fx_rtp_check(fx_event_as_rtp(event), FX_EVENT_MAGIC))
//----------------------------------------------------------------------------------------------------
void fx_event_internal_init(fx_event_internal_t* event, const bool state);
void fx_event_internal_set(fx_event_internal_t* event);
void fx_event_internal_reset(fx_event_internal_t* event);
bool fx_event_test_and_wait(fx_sync_waitable_t* object, fx_sync_wait_block_t* wb, const bool wait);
//----------------------------------------------------------------------------------------------------
int fx_event_init(fx_event_t* event, const bool state);
int fx_event_deinit(fx_event_t* event);
int fx_event_set(fx_event_t* event);
int fx_event_reset(fx_event_t* event);
int fx_event_pulse(fx_event_t* event); 
int fx_event_get_state(fx_event_t* event, bool* state);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_EVENT, V1] }))
#endif
#ifndef _FX_THREAD_APC_LIMITED_HEADER_
#define _FX_THREAD_APC_LIMITED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_thread_apc.h
  *  @brief  Interface of limited version of thread APC subsystem. 
  *          Only uniprocessor systems are supported (internal APC only).
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2013 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
typedef struct _fx_thread_apc_msg_t { int dummy; } fx_thread_apc_msg_t;
typedef struct _fx_thread_apc_target_t { int dummy; } fx_thread_apc_target_t;
typedef void (*fx_thread_apc_handler_t)(fx_thread_apc_target_t* t, fx_thread_apc_msg_t* apc, void* arg);
//----------------------------------------------------------------------------------------------------
extern void (*fx_thread_apc_on_receive)(fx_thread_apc_target_t*);
#define fx_thread_apc_ctor(idle_thread, on_receive_callback) fx_thread_apc_on_receive = on_receive_callback
#define fx_thread_apc_target_init(target)
#define fx_thread_apc_msg_init(msg, func, arg)
#define fx_thread_apc_insert(target, msg, accepted) (fx_dbg_assert(false), false)
#define fx_thread_apc_cancel(target, msg) false
#define fx_thread_apc_set_mask(new_mask) false
#define fx_thread_apc_pending(target) false
#define fx_thread_apc_deliver(target)
bool fx_thread_apc_insert_internal(fx_thread_apc_target_t* target, unsigned reason, void* arg);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_THREAD_APC, LIMITED] }))
#endif
#ifndef _FX_THREAD_CLEANUP_DISABLED_HEADER_
#define _FX_THREAD_CLEANUP_DISABLED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   disabled/fx_thread_cleanup.h
  *  @brief  Disables functionality of in-thread cleanup handlers.
  ****************************************************************************************************
  *  Copyright (C) 2008-2015 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
typedef struct _fx_thread_cleanup_context_t { int dummy; } fx_thread_cleanup_context_t;
typedef struct _fx_thread_cleanup_handler_t { int dummy; } fx_thread_cleanup_handler_t;
#define fx_thread_cleanup_switch_hook(old_target)
#define fx_thread_cleanup_init_target(target)
#define fx_thread_cleanup_handle(target)
#define fx_thread_cleanup_init(handler, f, a) ((void) (handler))
#define fx_thread_cleanup_add(target, k, handler)
#define fx_thread_cleanup_cancel(handler)
#define fx_thread_cleanup_set_hook(target, func)
//-----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_THREAD_CLEANUP, DISABLED] }))
//-----------------------------------------------------------------------------------------------------
#endif 
#ifndef _FX_STACKOVF_DISABLED_HEADER_
#define _FX_STACKOVF_DISABLED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_stackovf.h
  *  @brief  Disabled interface of the thread stack overflow protection.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2014 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
//
// If the subsystem is disabled, aspect structure contains only dummy variable in order to avoid
// compiler error.
//
typedef struct _fx_stackovf_info_t { int dummy; } fx_stackovf_info_t;
//----------------------------------------------------------------------------------------------------
#define fx_stackovf_init(object, stack, size)
#define fx_stackovf_check(object, current_stack)
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_STACKOVF, DISABLED] }))
//----------------------------------------------------------------------------------------------------
#endif
#ifndef _TRACE_THREAD_STUB_HEADER_
#define _TRACE_THREAD_STUB_HEADER_
/** 
  ****************************************************************************************************
  *  @file   trace_stub.h
  *  @brief  Stub for trace subsystem. Disables thread tracing.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
typedef struct {int dummy;} trace_thread_handle_t;
#define trace_thread_init_idle(thread_handle, prio)
#define trace_thread_init( thread_handle, prio )
#define trace_thread_init_failed()
#define trace_thread_deinit( thread_handle, prio )
#define trace_thread_suspend( thread_handle )
#define trace_thread_resume( thread_handle )
#define trace_thread_wakeup( thread_handle )
#define trace_thread_context_switch(from, to)
#define trace_thread_sleep(thread_handle, ticks)
#define trace_thread_delay_until(thread_handle, ticks) 
#define trace_thread_sched_param_set( thread_handle, param )
#define trace_thread_ceiling( thread_handle, old_prio, new_prio )
#define trace_thread_deceiling( thread_handle, old_prio, new_prio )
#define trace_thread_timeout( thread_handle, timeout )
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [TRACE_THREAD, STUB] }))
#endif
#ifndef _FX_THREAD_V1_HEADER_
#define _FX_THREAD_V1_HEADER_
/** 
  *****************************************************************************************************
  *  @file   fx_thread.h
  *  @brief  Threads implementation.
  *  The module implements basic threads interface on top of scheduler's container
  *****************************************************************************************************
  *  Copyright (C) 2008-2017 Eremex
  *  All rights reserved.
  ****************************************************************************************************/
  
//----------------------------------------------------------------------------------------------------
//!
//! Thread states.
//!
typedef enum _fx_thread_state_t
{
    FX_THREAD_STATE_READY = 0,              //!< Thread is either ready to execute or running.
    FX_THREAD_STATE_SUSPENDED = 1,          //!< Thread is explicitly suspended by fx_thread_suspend.
    FX_THREAD_STATE_WAITING = 2,            //!< Thread is blocked on sychronization object.
    FX_THREAD_STATE_COMPLETED = 3,          //!< Thread is either completed or terminated.    
}
fx_thread_state_t;
//----------------------------------------------------------------------------------------------------
//!
//! Thread representation.
//!
typedef struct _fx_thread_t
{
    fx_rtp_t rtp;                           //!< Run-time protection member.  
    fx_process_t* parent;                   //!< Parent process.
    fx_sync_waiter_t waiter;                //!< Abstract waiter. Used in wait functions.
    fx_sched_item_t sched_item;             //!< Schedulable item.
    uint32_t timeslice;                     //!< Quanta info for round-robin scheduling policy.  
    fx_thread_apc_target_t apcs;            //!< Asynchronous notifications container.
    fx_thread_cleanup_context_t cleanup;    //!< Cleanup handlers container.
    fx_timer_internal_t timer;              //!< Internal thread timer for timeouts implementation.
    fx_event_internal_t timer_event;        //!< Timeout-related event used by the timer.
    fx_event_internal_t completion;         //!< Thread completion event.
    
    hal_cpu_context_t hw_context;           //!< Hardware context.
    fx_stackovf_info_t stk_info;            //!< Kernel stack info for runtime stack checking.
    
    lock_t state_lock;                      //!< Lock protecting state field.
    fx_thread_state_t state;                //!< Specifies whether the thread is completed or suspended. 
    bool is_terminating;                    //!< Flag is set when the thread is in termination process.
    trace_thread_handle_t trace_handle;     //!< Handle of the thread in trace subsystem.
}
fx_thread_t;
//----------------------------------------------------------------------------------------------------
#define fx_thread_as_waiter(thread) (&((thread)->waiter))
#define fx_thread_as_sched_item(thread) (&((thread)->sched_item))
#define fx_thread_as_sched_params(thread) (fx_sched_item_as_sched_params(fx_thread_as_sched_item(thread)))
#define fx_thread_as_timer(thread) (&((thread)->timer))
#define fx_thread_as_event(thread) (&((thread)->completion))
#define fx_thread_as_timer_event(thread) (&((thread)->timer_event))
#define fx_thread_as_rtp(thread) (&((thread)->rtp))
#define fx_thread_as_hw_context(thread) (&((thread)->hw_context))
#define fx_thread_as_trace_handle(thread) (&((thread)->trace_handle))
#define fx_thread_as_apcs(thread) (&((thread)->apcs))
#define fx_thread_as_cleanup_context(thread) (&((thread)->cleanup))
#define fx_thread_as_stack_info(thread) (&((thread)->stk_info))
#define FX_THREAD_MAGIC 0x54485244 // 'THRD'
#define fx_thread_is_valid(thread) (fx_rtp_check(fx_thread_as_rtp(thread), FX_THREAD_MAGIC))
#define FX_THREAD_INFINITE_TIMEOUT UINT32_C(0xFFFFFFFF)
//----------------------------------------------------------------------------------------------------
//!
//! Parameter select for fx_thread_set/get_params
//! 
enum
{ 
    FX_THREAD_PARAM_PRIO = 0,       // Priority selection for parameter type in fx_thread_set_params.
    FX_THREAD_PARAM_TIMESLICE = 1,  // Timeslice selection for parameter type in fx_thread_set_params.
    FX_THREAD_PARAM_CPU = 2,        // Associated CPU selection for parameter type in fx_thread_set_params.
    FX_THREAD_PARAM_MAX,        
};
//----------------------------------------------------------------------------------------------------
//!
//! Threading API return codes.
//!
enum
{
    FX_THREAD_OK  = FX_STATUS_OK,   // Predefined success status.
    FX_THREAD_WAIT_CANCELLED,       // Wait cancelled (due to cancel condition).
    FX_THREAD_WAIT_DELETED,       // Wait aborted due to waitable object deletion.
    FX_THREAD_WAIT_INTERRUPTED,     // Wait cancelled due to incoming APC.  
    FX_THREAD_WAIT_TIMEOUT,         // Wait is timed out. Used in functions with the timeouts.
    FX_THREAD_WAIT_IN_PROGRESS,     // Reserved for internal use. Wait functions never return this value.
    FX_THREAD_INVALID_PTR,          // Invalid object pointer.
    FX_THREAD_INVALID_ENTRY,        // Invalid entry function passed to fx_thread_init.
    FX_THREAD_INVALID_PRIO,         // Invalid priority value.
    FX_THREAD_INVALID_CPU,          // Invalid CPU selection.
    FX_THREAD_NO_STACK,             // Incorrect stack parameters.
    FX_THREAD_INVALID_OBJ,          // Incorrect stack parameters.
    FX_THREAD_INVALID_TIMEOUT,      // Incorrect timeout value.
    FX_THREAD_INVALID_PARAM,        // Incorrect parameter.
    FX_THREAD_JOIN_SELF,            // Incorrect parameter.
    FX_THREAD_INVALID_TIMESLICE,    // Incorrect parameter.
    FX_THREAD_ERR_MAX
};
//
// Internal API functions (not intended to be used by applications).
//
//----------------------------------------------------------------------------------------------------
void fx_thread_ctor(void);
int fx_thread_wait_object(fx_sync_waitable_t* object, void* attr, fx_event_t* cancel_event);
int fx_thread_timedwait_object(fx_sync_waitable_t* object, void* attr, uint32_t timeout);
bool fx_thread_send_apc(fx_thread_t* thread, fx_thread_apc_msg_t* msg);
#define fx_thread_cancel_apc(thread, msg) fx_thread_apc_cancel(fx_thread_as_apcs(thread), msg)
#define fx_thread_enter_critical_region() ((void) fx_thread_apc_set_mask(true))
#define fx_thread_leave_critical_region() ((void) fx_thread_apc_set_mask(false))
//
// Public API.
//
//----------------------------------------------------------------------------------------------------
int fx_thread_init_ex(
        fx_process_t* parent,
        fx_thread_t* thread,
        void (*func)(void*), 
        void* arg, 
        unsigned int priority, 
        void* stack,
        size_t stack_sz, 
        bool create_suspended);
#define fx_thread_init(a, b, c, d, e, f, g) fx_thread_init_ex(fx_process_self(), a, b, c, d, e, f, g)
int fx_thread_deinit(fx_thread_t* thread);
int fx_thread_terminate(fx_thread_t* thread);
void fx_thread_exit(void);
int fx_thread_join(fx_thread_t* thread);
int fx_thread_suspend(void);
int fx_thread_resume(fx_thread_t* thread);
int fx_thread_sleep(uint32_t ticks);
int fx_thread_delay_until(uint32_t* prev_wake, uint32_t increment);
fx_thread_t* fx_thread_self(void);
void fx_thread_yield(void);
int fx_thread_get_params(fx_thread_t* thread, unsigned int type, unsigned int* value);
int fx_thread_set_params(fx_thread_t* thread, unsigned int type, unsigned int value);
int fx_thread_wait_event(fx_event_t* event, fx_event_t* cancel_event);
int fx_thread_timedwait_event(fx_event_t* event, uint32_t timeout);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_THREAD, V1], ctor: [fx_thread_ctor, on_each_cpu] }))
#endif
#ifndef _FX_APP_TIMER_THREADED_HEADER_
#define _FX_APP_TIMER_THREADED_HEADER_
/** 
  ****************************************************************************************************
  *  @file   threaded/fx_app_timer.h
  *  @brief  Interface of application timers.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2013 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
void fx_app_timer_ctor(void);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_APP_TIMER, THREADED], ctor: [fx_app_timer_ctor, on_each_cpu] }))
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ options: [                                               
    FX_TIMER_THREAD_PRIO: {                                                       
        type: int, range: [0, 32], default: 0,                                      
        description: "Priority of the timer handling thread."},                     
    FX_TIMER_THREAD_STACK_SIZE: {                                                 
        type: int, range: [0x200, 0xffffffff], default: 0x1000,                     
        description: "Size of the stack of timer handling thread."}]}))
#endif
#ifndef _FX_TIMER_DIRECT_HEADER_
#define _FX_TIMER_DIRECT_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_timer.h
  *  @brief  Interface of application timers with disabled error checking.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
//!
//! Without error checking tiner is directly mapped into internal timer. 
//!
typedef fx_timer_internal_t fx_timer_t;
//----------------------------------------------------------------------------------------------------
#define fx_timer_init(timer, func, arg) fx_timer_internal_init(timer, func, arg)
#define fx_timer_deinit(timer)
#define fx_timer_set_rel(timer, delay, period) fx_timer_internal_set_rel(timer, delay, period)
#define fx_timer_set_abs(timer, delay, period) fx_timer_internal_set_abs(timer, delay, period)
#define fx_timer_cancel(timer) fx_timer_internal_cancel(timer)
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_TIMER, DIRECT] }))
#endif
#ifndef _FX_DPC_STUB_HEADER_
#define _FX_DPC_STUB_HEADER_
/** 
  ***************************************************************************************************
  *  @file   unified_up/fx_dpc.h
  *  @brief  DPC stub.
  *
  *  Deferred procedures work in context of ISR.
  *  This DPC implementation is intended to be used with unified interrupt architecture.
  *
  ***************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  **************************************************************************************************/
  
//----------------------------------------------------------------------------------------------------
typedef struct { int dummy; } _fx_dpc_t, fx_dpc_t;
#define fx_dpc_init(dpc)
//
// Direct call of deferred function instead of putting it into queue. Because inserting of DPC
// is usually performed in ISR, deferred function may be called at SPL greater than DISPATCH.
//
#define fx_dpc_request(dpc, func, arg) (func(dpc, arg), true)
#define fx_dpc_cancel(dpc) (false)
#define fx_dpc_set_target_cpu(dpc, cpu) fx_dbg_assert(cpu == 0)
#define fx_dpc_environment() (false)
//
// No queue exist, so, nothing to handle.
//
#define fx_dpc_handle_queue()
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_DPC, STUB] }))
#endif
#ifndef _TRACE_SEM_STUB_HEADER_
#define _TRACE_SEM_STUB_HEADER_
/** 
  ****************************************************************************************************
  *  @file   trace_sem.h
  *  @brief  Stub for trace subsystem. Disables sem tracing.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
typedef struct {int dummy;} trace_sem_handle_t;
#define trace_sem_init( sem_handle, sem_val, sem_max )
#define trace_sem_init_failed()
#define trace_sem_deinit( sem_handle, sem_val, sem_max )
#define trace_sem_wait_ok( sem_handle, sem_val )
#define trace_sem_wait_block( sem_handle, blocked_thread_handle )
#define trace_sem_post( sem_handle, sem_val )
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [TRACE_SEM, STUB] }))
#endif
#ifndef _FX_SEM_V1_HEADER_
#define _FX_SEM_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_sem.h
  *  @brief  Interface of semaphores.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
enum
{
    FX_SEM_MAGIC = 0x53454D41, // SEMA
    FX_SEM_OK = FX_STATUS_OK,
    FX_SEM_INVALID_PTR = FX_THREAD_ERR_MAX,
    FX_SEM_INVALID_OBJ,
    FX_SEM_UNSUPPORTED_POLICY,
    FX_SEM_INVALID_VALUE,
    FX_SEM_INVALID_TIMEOUT,
    FX_SEM_ERR_MAX
};
//----------------------------------------------------------------------------------------------------
//!
//! Semaphore representation.
//!
typedef struct _fx_sem_t
{
    fx_sync_waitable_t waitable;      //!< Base class of semaphore is a waitable object.
    lock_t lock;                      //!< Spinlock associated with the semaphore.
    unsigned int semaphore;           //!< Semaphore counter.
    unsigned int max_count;           //!< Maximal value of semaphore counter.
    fx_sync_policy_t policy;          //!< Waiter releasing policy.
    fx_rtp_t rtp;                     //!< Run-time protection member.
    trace_sem_handle_t trace_handle;  //!<Handle of object used by trace subsystem.
} 
fx_sem_t;
#define fx_sem_as_waitable(sem)     (&((sem)->waitable))
#define fx_sem_as_rtp(sem)          (&((sem)->rtp))
#define fx_sem_as_trace_handle(sem) (&((sem)->trace_handle))
#define fx_sem_is_valid(sem) (fx_rtp_check(fx_sem_as_rtp(sem), FX_SEM_MAGIC))
//----------------------------------------------------------------------------------------------------
int fx_sem_init(fx_sem_t* sem, const unsigned init, const unsigned max_val, const fx_sync_policy_t p);
int fx_sem_deinit(fx_sem_t* sem);
int fx_sem_reset(fx_sem_t* sem);
int fx_sem_get_value(fx_sem_t* sem, unsigned int* value);
int fx_sem_post(fx_sem_t* sem);
int fx_sem_post_with_policy(fx_sem_t* sem, const fx_sync_policy_t policy);
int fx_sem_wait(fx_sem_t* sem, fx_event_t* event);
int fx_sem_timedwait(fx_sem_t* sem, uint32_t timeout);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_SEM, V1] }))
#endif
#ifndef _TRACE_MUTEX_STUB_HEADER_
#define _TRACE_MUTEX_STUB_HEADER_
/** 
  ****************************************************************************************************
  *  @file   trace_mutex.h
  *  @brief  Stub for trace subsystem. Disables mutex tracing.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
typedef struct {int dummy;} trace_mutex_handle_t;
#define trace_mutex_init( mutex_handle )  
#define trace_mutex_init_failed() 
#define trace_mutex_deinit( mutex_handle )  
#define trace_mutex_acquired( mutex_handle , owner_thread_handle)
#define trace_mutex_acquire_block( mutex_handle )
#define trace_mutex_released( mutex_handle, new_owner_handle )
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [TRACE_MUTEX, STUB] }))
#endif
#ifndef _FX_MUTEX_V1_HEADER_
#define _FX_MUTEX_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_mutex.h
  *  @brief  Interface of mutexes.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
enum
{
    FX_MUTEX_MAGIC = 0x4D555458, // MUTX
    FX_MUTEX_OK = FX_STATUS_OK,
    FX_MUTEX_INVALID_PTR = FX_THREAD_ERR_MAX,
    FX_MUTEX_INVALID_OBJ,
    FX_MUTEX_UNSUPPORTED_POLICY,
    FX_MUTEX_INVALID_PRIORITY,
    FX_MUTEX_INVALID_TIMEOUT,
    FX_MUTEX_WRONG_OWNER,
    FX_MUTEX_RECURSIVE_LIMIT,
    FX_MUTEX_ABANDONED,
    FX_MUTEX_ERR_MAX
};
//----------------------------------------------------------------------------------------------------
//!
//! Mutex representation.
//!
typedef struct _fx_mutex_t
{
    fx_sync_waitable_t waitable;              //!< Base class. Waitable object.
    lock_t lock;                              //!< Low-level primitive to protect the mutex itself.
    fx_thread_t* volatile owner;              //!< Current mutex owner, it is NULL if mutex free.
    volatile uint_fast16_t recursive_locks;   //!< Recursive locks by same owne.r
    bool ceiling_enabled;                     //!< Enable static ceiling for this object if true.
    fx_sched_params_t dyn_ceiling_params;     //!< Dynamic ceiling priority.
    fx_sched_params_t ceiling_params;         //!< Scheduling params which will be applied to owner.
    fx_sched_params_t owner_original_params;  //!< Owner's original scheduling parameters.
    fx_sync_policy_t policy;                  //!< Waiter releasing policy.
    fx_rtp_t rtp;                             //!< Run-time structure protection member.
    trace_mutex_handle_t  trace_handle;       //!< Handle of the mutex in trace subsystem.
} 
fx_mutex_t;
#define fx_mutex_as_waitable(mutex)       (&((mutex)->waitable))
#define fx_mutex_as_rtp(mutex)            (&((mutex)->rtp))
#define fx_mutex_as_trace_handle(mutex)   (&((mutex)->trace_handle))
#define fx_mutex_is_valid(mutex) (fx_rtp_check(fx_mutex_as_rtp(mutex), FX_MUTEX_MAGIC))
#define fx_mutex_limit_exceeded(mutex) ((mutex)->recursive_locks == UINT_FAST16_MAX)
#define fx_mutex_lock_counter_get(mutex) ((mutex)->recursive_locks)
#define fx_mutex_lock_counter_set(mutex, c) ((mutex)->recursive_locks = (c))
#define FX_MUTEX_CEILING_DISABLED (~((unsigned int) 0))
//----------------------------------------------------------------------------------------------------
int fx_mutex_init(fx_mutex_t* mutex, const unsigned int priority, const fx_sync_policy_t policy);
int fx_mutex_deinit(fx_mutex_t* mutex);
int fx_mutex_acquire(fx_mutex_t* mutex, fx_event_t* event);
int fx_mutex_timedacquire(fx_mutex_t* mutex, uint32_t tout);
int fx_mutex_release(fx_mutex_t* mutex);
int fx_mutex_release_with_policy(fx_mutex_t* mutex, const fx_sync_policy_t policy);
fx_thread_t* fx_mutex_get_owner(fx_mutex_t* mutex);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_MUTEX, V1] }))
#endif
#ifndef _TRACE_QUEUE_STUB_HEADER_
#define _TRACE_QUEUE_STUB_HEADER_
/** 
  ****************************************************************************************************
  *  @file   trace_queue.h
  *  @brief  Stub for trace subsystem. Disables queue tracing.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
typedef struct {int dummy;} trace_queue_handle_t;
#define trace_queue_init( queue_handle, items_max )
#define trace_queue_init_failed( queue_handle )
#define trace_queue_deinit( queue_handle, msg_count )
#define trace_queue_send( queue_handle, msg_count )
#define trace_queue_send_failed( queue_handle )
#define trace_queue_send_block( queue_handle )
#define trace_queue_send_forward( queue_handle)
#define trace_queue_receive( queue_handle, msg_count )
#define trace_queue_receive_failed( queue_handle )
#define trace_queue_receive_block( queue_handle )
#define trace_queue_receive_forward( queue_handle )
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [TRACE_QUEUE, STUB] }))
#endif
#ifndef _FX_MSGQ_CORE_V1_HEADER_
#define _FX_MSGQ_CORE_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_msgq_core.h
  *  @brief  Interface of core message queue services.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
typedef enum
{
    FX_MSGQ_SEND_FRONT,
    FX_MSGQ_SEND_BACK,
    FX_MSGQ_WAIT_TYPE_MAX
}
fx_msgq_wait_type_t;
//----------------------------------------------------------------------------------------------------
typedef struct _fx_msgq_wait_attr_t
{
    fx_msgq_wait_type_t operation;
    uintptr_t* buf;
}
fx_msgq_wait_attr_t;
//----------------------------------------------------------------------------------------------------
//!
//! Message queue representation. 
//!
typedef struct _fx_msgq_t
{
    fx_sync_waitable_t send_wtbl;       //!< Internal waitable object.
    fx_sync_waitable_t recv_wtbl;       //!< Internal waitable object.
    lock_t lock;                        //!< Lock associated with both queues. 
    uintptr_t* buf;                     //!< Message buffer.
    unsigned int items_max;             //!< Biffer capacity in words.
    unsigned int items;                 //!< Current number of items in the queue.
    unsigned int head;                  //!< Head pointer.
    unsigned int tail;                  //!< Tail pointer.
    fx_rtp_t rtp;                       //!< Run-time protection member.
    fx_sync_policy_t policy;            //!< Waiter releasing policy.
    trace_queue_handle_t trace_handle;  //!< Handle of the queue in trace subsystem.
} 
fx_msgq_t;
#define fx_msgq_as_send_waitable(msgq) (&((msgq)->send_wtbl))
#define fx_msgq_as_recv_waitable(msgq) (&((msgq)->recv_wtbl))
#define fx_msgq_as_rtp(msgq) (&((msgq)->rtp))
#define fx_msgq_as_trace_handle(msgq) (&((msgq)->trace_handle))
#define fx_msgq_as_spinlock(msgq) (&((msgq)->lock))
#define FX_MSGQ_MAGIC 0x4D534751 // 'MSGQ'
#define fx_msgq_is_valid(msgq) (fx_rtp_check(fx_msgq_as_rtp(msgq), FX_MSGQ_MAGIC))
//----------------------------------------------------------------------------------------------------
int fx_msgq_core_init(fx_msgq_t* msgq, uintptr_t* buf, const unsigned sz, const fx_sync_policy_t p);
int fx_msgq_core_deinit(fx_msgq_t* msgq);
int fx_msgq_core_flush(fx_msgq_t* msgq);
bool fx_msgq_test_and_wait_send(fx_sync_waitable_t* object, fx_sync_wait_block_t* wb, const bool wait);
bool fx_msgq_test_and_wait_recv(fx_sync_waitable_t* object, fx_sync_wait_block_t* wb, const bool wait);
FX_METADATA(({ interface: [FX_MSGQ_CORE, V1] }))
#endif
#ifndef _FX_MSGQ_V1_HEADER_
#define _FX_MSGQ_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_msgq.h
  *  @brief  Interface of message queue services for threads.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
enum
{
    FX_MSGQ_OK = FX_STATUS_OK,
    FX_MSGQ_INVALID_PTR = FX_THREAD_ERR_MAX,
    FX_MSGQ_INVALID_OBJ,
    FX_MSGQ_INVALID_BUF,
    FX_MSGQ_UNSUPPORTED_POLICY,
    FX_MSGQ_ERR_MAX
};
int fx_msgq_init(fx_msgq_t* msgq, uintptr_t* buf, const unsigned sz, const fx_sync_policy_t p);
int fx_msgq_deinit(fx_msgq_t* msgq);
int fx_msgq_flush(fx_msgq_t* msgq);
int fx_msgq_front_send(fx_msgq_t* msgq, uintptr_t msg, fx_event_t* cancel_event);
int fx_msgq_back_send(fx_msgq_t* msgq, uintptr_t msg, fx_event_t* cancel_event);
int fx_msgq_front_timedsend(fx_msgq_t* msgq, uintptr_t msg, uint32_t tout);
int fx_msgq_back_timedsend(fx_msgq_t* msgq, uintptr_t msg, uint32_t tout);
int fx_msgq_timedreceive(fx_msgq_t* msgq, uintptr_t* msg, uint32_t tout);
int fx_msgq_receive(fx_msgq_t* msgq, uintptr_t* msg, fx_event_t* cancel_event);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_MSGQ, V1] }))
#endif
#ifndef _FX_BLOCK_POOL_V1_HEADER_
#define _FX_BLOCK_POOL_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_block_pool.h
  *  @brief  Interface of memory block pool primitive.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2014 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
enum
{
    FX_BLOCK_POOL_MAGIC = 0x424C4B50, // 'BLKP'
    FX_BLOCK_POOL_OK = FX_STATUS_OK,
    FX_BLOCK_POOL_INVALID_PTR = FX_THREAD_ERR_MAX,
    FX_BLOCK_POOL_INVALID_OBJ,
    FX_BLOCK_POOL_NO_MEM,
    FX_BLOCK_POOL_IMPROPER_ALIGN,
    FX_BLOCK_POOL_UNSUPPORTED_POLICY,
    FX_BLOCK_POOL_ERR_MAX
};
//----------------------------------------------------------------------------------------------------
//!
//! Block pool representation. 
//!
typedef struct _fx_block_pool_t
{
    fx_sync_waitable_t  waitable; //!< Internal waitable object.
    fx_rtp_t rtp;                 //!< Runtime protection member (object type identifier).
    lock_t  lock;                 //!< Lock associated with the primitive.
    uintptr_t base;               //!< Current address of available memory blocks pool.
    size_t sz;                    //!< Size of memory block.
    size_t remaining_sz;          //!< Remaining memory size in pool.
    rtl_list_t free_blocks;       //!< List of bree blocks.
    unsigned int free_blocks_num; //!< Available blocks count.
    fx_sync_policy_t policy;      //!< Default releasing policy.
} 
fx_block_pool_t;
#define fx_block_pool_as_waitable(bp) (&((bp)->waitable))
#define fx_block_pool_as_rtp(bp) (&((bp)->rtp))
#define fx_block_pool_as_spinlock(bp) (&((bp)->lock))
#define fx_block_pool_is_valid(bp) (fx_rtp_check(fx_block_pool_as_rtp(bp), FX_BLOCK_POOL_MAGIC))
//----------------------------------------------------------------------------------------------------
typedef struct _fx_mem_block_t
{
    union
    {
        fx_block_pool_t* parent_pool;
        rtl_list_linkage_t link;
    }
    hdr;
} 
fx_mem_block_t;
//----------------------------------------------------------------------------------------------------
int fx_block_pool_init(fx_block_pool_t* bp, void* base, size_t sz, size_t blk_sz, fx_sync_policy_t p);
int fx_block_pool_deinit(fx_block_pool_t* bp);
int fx_block_pool_alloc(fx_block_pool_t* bp, void** alloc_blk, fx_event_t* cancel_event);
int fx_block_pool_timedalloc(fx_block_pool_t* bp, void** alloc_blk, uint32_t tout);
int fx_block_pool_release(void* blk_ptr);
int fx_block_pool_release_internal(void* blk_ptr, fx_sync_policy_t p);
int fx_block_pool_avail_blocks(fx_block_pool_t* bp, unsigned int* count);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_BLOCK_POOL, V1] })) 
#endif
#ifndef _FX_EV_FLAGS_V1_HEADER_
#define _FX_EV_FLAGS_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_ev_flags.h
  *  @brief  Interface of event flags.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2014 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
//!
//! Error codes.
//! 
enum
{
    FX_EV_FLAGS_MAGIC = 0x45564600, // EVF
    FX_EV_FLAGS_OK = FX_STATUS_OK,
    FX_EV_FLAGS_INVALID_PTR = FX_THREAD_ERR_MAX,
    FX_EV_FLAGS_INVALID_OBJ,
    FX_EV_FLAGS_INVALID_FLAGS,
    FX_EV_FLAGS_INVALID_OPTIONS,
    FX_EV_FLAGS_ERR_MAX
};
//----------------------------------------------------------------------------------------------------
//!
//! Wait options.
//! 
enum
{
    FX_EV_FLAGS_OR = 0,     //!< Waiting any flag from specified set to be 1.
    FX_EV_FLAGS_AND = 1,    //!< Waitinf all flags from specified set to be 1.
    FX_EV_FLAGS_CLEAR = 2,  //!< Consume (reset to 0) flags which make wait satisfied.
};
//----------------------------------------------------------------------------------------------------
//!
//! Event flags representation. 
//!
typedef struct _fx_ev_flags_t
{
    fx_sync_waitable_t waitable;  //!< Waitable object as a base class.
    rtl_queue_t temp;             //!< Temporary queue of items to be notified.
    uint_fast32_t flags;          //!< Event flags state.
    fx_rtp_t rtp;                 //!< Run-time protection member.
    lock_t lock;                  //!< Low-level primitive to protect the object itself.
} 
fx_ev_flags_t;
#define fx_ev_flags_as_waitable(evf) (&((evf)->waitable))
#define fx_ev_flags_as_rtp(evf) (&((evf)->rtp))
#define fx_ev_flags_is_valid(evf) (fx_rtp_check(fx_ev_flags_as_rtp(evf), FX_EV_FLAGS_MAGIC))
//----------------------------------------------------------------------------------------------------
int fx_ev_flags_init(fx_ev_flags_t* evf);
int fx_ev_flags_deinit(fx_ev_flags_t* evf);
int fx_ev_flags_wait(
    fx_ev_flags_t* evf, 
    const uint_fast32_t req_flags, 
    const unsigned int option, 
    uint_fast32_t* state, 
    fx_event_t* cancel_ev);
int fx_ev_flags_timedwait(
    fx_ev_flags_t* evf, 
    const uint_fast32_t req_flags, 
    const unsigned int option, 
    uint_fast32_t* state, 
    uint32_t tout);
int fx_ev_flags_set(fx_ev_flags_t* evf, const uint_fast32_t flags, const bool type);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_EV_FLAGS, V1] }))
#endif
#ifndef _FX_RWLOCK_V1_HEADER_
#define _FX_RWLOCK_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_rwlock.h
  *  @brief  Interface of read/write lock.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
enum
{
    FX_RWLOCK_MAGIC = 0x52574C4B, //RWLK
    FX_RWLOCK_OK = FX_STATUS_OK,
    FX_RWLOCK_INVALID_PTR = FX_THREAD_ERR_MAX,
    FX_RWLOCK_INVALID_OBJ,
    FX_RWLOCK_UNSUPPORTED_POLICY,
    FX_RWLOCK_INVALID_TIMEOUT,
    FX_RWLOCK_ERR_MAX
};
//----------------------------------------------------------------------------------------------------
//!
//! RW-lock representation. 
//!
typedef struct _fx_rwlock_t
{
    fx_sync_waitable_t rd_wtbl;     //!< Internal waitable object for readers.
    fx_sync_waitable_t wr_wtbl;     //!< Internal waitable object for writers.
    lock_t lock;                    //!< Lock associated with both queues. 
    fx_rtp_t rtp;                   //!< Run-time protection member.
    unsigned int readers;           //!< Active readers count.
    fx_thread_t* owner;             //!< Active writer (if any).
    fx_sync_policy_t policy;        //!< Waiter releasing policy.
} 
fx_rwlock_t;
#define fx_rwlock_as_rd_waitable(rwl)  (&((rwl)->rd_wtbl))
#define fx_rwlock_as_wr_waitable(rwl)  (&((rwl)->wr_wtbl))
#define fx_rwlock_as_rtp(rwl)          (&((rwl)->rtp))
#define fx_rwlock_as_spinlock(rwl)     (&((rwl)->lock))
#define fx_rwlock_is_valid(rwl) (fx_rtp_check(fx_rwlock_as_rtp(rwl), FX_RWLOCK_MAGIC))
//----------------------------------------------------------------------------------------------------
int fx_rwlock_init(fx_rwlock_t* rw, const fx_sync_policy_t policy);
int fx_rwlock_deinit(fx_rwlock_t* rw);
int fx_rwlock_rd_timedlock(fx_rwlock_t* rw, uint32_t tout);
int fx_rwlock_wr_timedlock(fx_rwlock_t* rw, uint32_t tout);
int fx_rwlock_rd_lock(fx_rwlock_t* rw, fx_event_t* cancel_event);
int fx_rwlock_wr_lock(fx_rwlock_t* rw, fx_event_t* cancel_event);
int fx_rwlock_unlock(fx_rwlock_t* rw);
int fx_rwlock_unlock_with_policy(fx_rwlock_t* rw, const fx_sync_policy_t policy);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_RWLOCK, V1] }))
#endif
#ifndef _FX_COND_V1_HEADER_
#define _FX_COND_V1_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_cond.h
  *  @brief  Interface of condition variables.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//----------------------------------------------------------------------------------------------------
enum
{
    FX_COND_MAGIC = 0x434F4E44, // COND
    FX_COND_OK = FX_STATUS_OK,
    FX_COND_INVALID_PTR = FX_THREAD_ERR_MAX,
    FX_COND_INVALID_OBJ,
    FX_COND_UNSUPPORTED_POLICY,
    FX_COND_INVALID_MUTEX,
    FX_COND_MUTEX_ERROR,
    FX_COND_INVALID_TIMEOUT,
    FX_COND_INVALID_PARAMETER,
    FX_COND_NO_WAITERS,
    FX_COND_ERR_MAX
};
//----------------------------------------------------------------------------------------------------
//!
//! Condition variable representation. 
//!
typedef struct _fx_cond_t
{
    fx_sync_waitable_t waitable;  //!< Waitable object as a base class.
    fx_rtp_t rtp;                 //!< Run-time protection member.
    lock_t lock;                  //!< Lower-level lock for condvar synchronization.
    fx_sync_policy_t policy;      //!< Policy of waiter releasing.
} 
fx_cond_t;
#define fx_cond_as_waitable(cond) (&((cond)->waitable))
#define fx_cond_as_rtp(cond) (&((cond)->rtp))
#define fx_cond_as_lock(cond) (&((cond)->lock))
#define fx_cond_is_valid(cond) (fx_rtp_check(fx_cond_as_rtp(cond), FX_COND_MAGIC))
//----------------------------------------------------------------------------------------------------
int fx_cond_init(fx_cond_t* cond, const fx_sync_policy_t policy);
int fx_cond_deinit(fx_cond_t* cond);
int fx_cond_signal(fx_cond_t* cond);
int fx_cond_signal_with_policy(fx_cond_t* cond, const fx_sync_policy_t policy);
int fx_cond_broadcast(fx_cond_t* cond);
int fx_cond_wait(fx_cond_t* cond, fx_mutex_t* mutex, fx_event_t* cancel_event);
int fx_cond_timedwait(fx_cond_t* cond, fx_mutex_t* mutex, uint32_t tout);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_COND, V1] }))
#endif
#ifndef _RTL_MEM_POOL_TLSF_HEADER_
#define _RTL_MEM_POOL_TLSF_HEADER_
/*
 * Two Level Segregated Fit memory allocator.
 *
 * Copyright (c) 2018 Eremex Ltd.
 * Copyright (c) 2016 National Cheng Kung University, Taiwan.
 * Copyright (c) 2006-2008, 2011, 2014 Matthew Conte.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL MATTHEW CONTE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#ifndef RTL_MEM_POOL_SUBDIV_LOG2
#define RTL_MEM_POOL_SUBDIV_LOG2 4
#endif
#ifndef RTL_MEM_POOL_MAX_CHUNK
#error RTL_MEM_POOL_MAX_CHUNK is not defined!
#endif
//
// Configuration constants.
//
enum 
{
    FL_INDEX_MAX = RTL_MEM_POOL_MAX_CHUNK,
    // log2 of number of linear subdivisions of block sizes. Larger
    // values require more memory in the control structure. Values of
    // 4 or 5 are typical.
    //
    SL_INDEX_COUNT_LOG2 = RTL_MEM_POOL_SUBDIV_LOG2,
    //
    // All allocation sizes and addresses are aligned.
    //
    ALIGN_SIZE_LOG2 = sizeof(void*) == 8 ? 3 : 2,
    ALIGN_SIZE = (1 << ALIGN_SIZE_LOG2),
    //
    // We support allocations of sizes up to (1 << FL_INDEX_MAX) bits.
    // However, because we linearly subdivide the second-level lists, and
    // our minimum size granularity is 4 bytes, it doesn't make sense to
    // create first-level lists for sizes smaller than SL_INDEX_COUNT * 4,
    // or (1 << (SL_INDEX_COUNT_LOG2 + 2)) bytes, as there we will be
    // trying to split size ranges into more slots than we have available.
    // Instead, we calculate the minimum threshold size, and place all
    // blocks below that size into the 0th first-level list.
    //
    SL_INDEX_COUNT = (1 << SL_INDEX_COUNT_LOG2),
    FL_INDEX_SHIFT = (SL_INDEX_COUNT_LOG2 + ALIGN_SIZE_LOG2),
    FL_INDEX_COUNT = (FL_INDEX_MAX - FL_INDEX_SHIFT + 1),
    SMALL_BLOCK_SIZE = (1 << FL_INDEX_SHIFT),
};
//
// Block header structure.
//
// There are several implementation subtleties involved:
// - The prev_phys_block field is only valid if the previous block is free.
// - The prev_phys_block field is actually stored at the end of the
//   previous block. It appears at the beginning of this structure only to
//   simplify the implementation.
// - The next_free / prev_free fields are only valid if the block is free.
//
typedef struct _rtl_block_header_t 
{
    struct _rtl_block_header_t* prev_phys_block; //!< Points to the previous physical block.
    size_t size;                                //!< The size of this block, minus the block header.
    struct _rtl_block_header_t *next_free;
    struct _rtl_block_header_t *prev_free;
} 
rtl_block_header_t;
//
// The TLSF pool structure.
//
typedef struct _rtl_mem_pool_t 
{
    rtl_block_header_t block_null;
    unsigned int fl_bitmap;
    unsigned int sl_bitmap[FL_INDEX_COUNT];
    rtl_block_header_t *blocks[FL_INDEX_COUNT][SL_INDEX_COUNT];
} 
rtl_mem_pool_t;
void rtl_mem_pool_init(rtl_mem_pool_t* pool);
bool rtl_mem_pool_add_mem(rtl_mem_pool_t* pool, void* mem, size_t bytes);
void* rtl_mem_pool_alloc(rtl_mem_pool_t* pool, size_t bytes);
void rtl_mem_pool_free(rtl_mem_pool_t* pool, void* ptr);
size_t rtl_mem_pool_get_max_blk(rtl_mem_pool_t* pool);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [RTL_MEM_POOL, TLSF] }))
//----------------------------------------------------------------------------------------------------
#endif
#ifndef _FX_MEM_POOL_TLSF_HEADER_
#define _FX_MEM_POOL_TLSF_HEADER_
/** 
  ****************************************************************************************************
  *  @file   fx_mem_pool.h
  *  @brief  Wrapper for memory pool.
  *
  ****************************************************************************************************
  *  Copyright (C) 2008-2012 Eremex
  *  All rights reserved.
  ***************************************************************************************************/
//
// Error codes.
//
enum
{
    FX_MEM_POOL_OK = FX_STATUS_OK,
    FX_MEM_POOL_INVALID_PTR,
    FX_MEM_POOL_INVALID_OBJ,
    FX_MEM_POOL_INVALID_BUF,
    FX_MEM_POOL_ZERO_SZ,
    FX_MEM_POOL_NO_MEM,
    FX_MEM_POOL_ERR_MAX
};
//
// The TLSF pool structure.
//
typedef struct _fx_mem_pool_t 
{
    lock_t lock;
    rtl_mem_pool_t rtl_pool;
} 
fx_mem_pool_t;
int fx_mem_pool_init(fx_mem_pool_t* pool);
int fx_mem_pool_deinit(fx_mem_pool_t* pool);
int fx_mem_pool_add_mem(fx_mem_pool_t* pool, uintptr_t mem, size_t bytes);
int fx_mem_pool_alloc(fx_mem_pool_t* pool, size_t bytes, void** p);
int fx_mem_pool_free(fx_mem_pool_t* pool, void* ptr);
int fx_mem_pool_get_max_free_chunk(fx_mem_pool_t* pool, size_t* blk_sz);
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FX_MEM_POOL, TLSF] }))
//----------------------------------------------------------------------------------------------------
#endif
#ifndef _FXRTOS_STANDARD_CM3_GNU_HEADER_
#define _FXRTOS_STANDARD_CM3_GNU_HEADER_
//----------------------------------------------------------------------------------------------------
FX_METADATA(({ interface: [FXRTOS, STANDARD_CORTEX_M3_GNU] }))
#endif
