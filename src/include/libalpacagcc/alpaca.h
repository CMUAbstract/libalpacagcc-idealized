#ifndef ALPACA_H
#define ALPACA_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <libmsp/mem.h>

typedef void (task_func_t)(void);
typedef unsigned task_idx_t;

/** @brief Task */
typedef struct {
	/** @brief function address */
	task_func_t *func;
	/** @brief index (only used for showing progress) */
	task_idx_t idx;
} task_t;

/** @brief Execution context */
typedef struct _context_t {
	/** @brief current running task */
	task_t *task;
	/** @brief indicate whether to jump to commit stage on power failure*/
	uint8_t needCommit;
} context_t;

extern volatile unsigned _numBoots;
extern volatile unsigned num_dirty_gv;
extern context_t * volatile curctx;
/** @brief LLVM generated function that clears all isDirty_ array */
extern void clear_isDirty();
/** @brief Function called on every reboot
 *  @details This function usually initializes hardware, such as GPIO
 *           direction. The application must define this function because
 *           different app uses different GPIO.
 */
extern void init();

void task_prologue();
void transition_to(task_t *task);
//void write_to_gbuf(uint8_t *data_src, uint8_t *data_dest, size_t var_size); 

/** @brief Internal macro for constructing name of task symbol */
#define TASK_SYM_NAME(func) _task_ ## func

/** @brief Declare a task
 *
 *  @param idx      Global task index, zero-based
 *  @param func     Pointer to task function
 *
 */
#define TASK(idx, func) \
	void func(); \
__nv task_t TASK_SYM_NAME(func) = { func, idx }; \

/** @brief Macro for getting address of task */
#define TASK_REF(func) &TASK_SYM_NAME(func)

/** @brief First task to run when the application starts
 *  @details Symbol is defined by the ENTRY_TASK macro.
 *           This is not wrapped into a delaration macro, because applications
 *           are not meant to declare tasks -- internal only.
 *
 *  TODO: An alternative would be to have a macro that defines
 *        the curtask symbol and initializes it to the entry task. The
 *        application would be required to have a definition using that macro.
 *        An advantage is that the names of the tasks in the application are
 *        not constrained, and the whole thing is less magical when reading app
 *        code, but slightly more verbose.
 */
extern task_t TASK_SYM_NAME(_entry_task);

/** @brief Declare the first task of the application
 *  @details This macro defines a function with a special name that is
 *           used to initialize the current task pointer.
 *
 *           This does incur the penalty of an extra task transition, but it
 *           happens only once in application lifetime.
 *
 *           The alternatives are to force the user to define functions
 *           with a special name or to define a task pointer symbol outside
 *           of the library.
 */
#define ENTRY_TASK(task) \
	TASK(0, _entry_task) \
void _entry_task() { TRANSITION_TO(task); }

/** @brief Init function prototype
 *  @details We rely on the special name of this symbol to initialize the
 *           current task pointer. The entry function is defined in the user
 *           application through a macro provided by our header.
 */
void _init();

/** @brief Declare the function to be called on each boot
 *  @details The same notes apply as for entry task.
 */
#define INIT_FUNC(func) void _init() { func(); }

/**
 *  @brief way to simply rename vars. I don't need it actually.
 *  I should remove it or rename it..
 *  Actually I should just remove this thing!
 */
#define GLOBAL_SB(type, name, ...) GLOBAL_SB_(type, name, ##__VA_ARGS__, 3, 2)
#define GLOBAL_SB_(type, name, size, n, ...) GLOBAL_SB##n(type, name, size)
#define GLOBAL_SB2(type, name, ...) __nv type _global_ ## name
#define GLOBAL_SB3(type, name, size) __nv type _global_ ## name[size]

/**
 *  @brief way to simply reference renamed vars. I don't need it actually.
 *  I should remove it or rename it..
 *  Actually I should just remove this thing!
 */
#define GV(type, ...) GV_(type, ##__VA_ARGS__, 2, 1)
#define GV_(type, i, n, ...) GV##n(type, i)
#define GV1(type, ...) _global_ ## type
#define GV2(type, i) _global_ ## type[i]

/** @brief Transfer control to the given task
 *  @param task     Name of the task function
 *  */
#define TRANSITION_TO(task) transition_to(TASK_REF(task))


 /**
  * @brief A set of extra definitions to get the event timing
  */
#if defined(LIBALPACA_TEST_EV_TIME) || defined(LIBALPACA_TEST_WAIT_TIME) \
  || defined(LIBALPACA_TEST_TIMING)

#define TIMER_INIT \
  TA0CTL = TASSEL__SMCLK | MC__STOP | ID_3 | TACLR | TAIE;

void add_ticks(unsigned *overflow, unsigned *ticks, unsigned new_ticks);

#else

#define APP_FINISHED \
  ;

#define TIMER_INIT \
  ;

#endif

#ifdef LIBALPACA_TEST_EV_TIME

extern unsigned overflows_ev;
extern unsigned ev_ticks;
extern unsigned ev_count;

#pragma message("Setting ev timer")

#define EV_TIMER_START \
  __delay_cycles(4000); \
  /*printf("T %u\r\n",TA0R);*/ \
  ev_count++; \
  TA0CTL |= MC__CONTINUOUS;

// Pause the timer by setting the MC bits to 0
#define MODE_SHIFT 4
#define EV_TIMER_STOP \
  /*printf("P T0: %u + %u / 65536\r\n",overflows, TA0R); */\
  TA0CTL &= ~(0x3 << MODE_SHIFT); \
  add_ticks(&overflows_ev, &ev_ticks, TA0R);\
  /*printf("EV:%u %u\r\n",overflows_ev,ev_ticks);*/\
  TA0CTL |= TACLR; \
  TA0R = 0;

#define APP_FINISHED \
  printf("Events time: %u + %u\r\n",overflows_ev, ev_ticks);\
  printf("Ev starts: %u\r\n",ev_count);

#else
  #pragma message ("No I didn't")

#define EV_TIMER_START \
  ;

#define EV_TIMER_STOP \
  ;

#endif // EV_TIME

#ifdef LIBALPACA_TEST_WAIT_TIME
  #pragma message "test wait time"
extern unsigned overflows_wait;
extern unsigned wait_ticks;
extern unsigned wait_count;

#define WAIT_TIMER_START \
  /*printf("T %u\r\n",TA0R);*/ \
  if(!wait_count) {\
    wait_count=1; \
    TA0CTL |= MC__CONTINUOUS;\
  }

// Pause the timer by setting the MC bits to 0
#define MODE_SHIFT 4
#define WAIT_TIMER_STOP \
  /*printf("P T0: %u + %u / 65536\r\n",overflows, TA0R); */\
  TA0CTL &= ~(0x3 << MODE_SHIFT); \
  add_ticks(&overflows_wait, &wait_ticks, TA0R);\
  /*printf("EV:%u %u\r\n",overflows_ev,ev_ticks);*/\
  TA0CTL |= TACLR; \
  TA0R = 0; \
  wait_count = 0;

#define APP_FINISHED \
  printf("Wait time: %u + %u\r\n",overflows_wait, wait_ticks);\

#else

#define WAIT_TIMER_START \
  ;

#define WAIT_TIMER_STOP \
  ;

#endif // WAIT_TIME

#ifdef LIBALPACA_TEST_TIMING
extern unsigned overflows_tsk_tran;
extern unsigned tsk_tran_ticks;
extern unsigned trans_starts;
extern unsigned trans_stops;
extern unsigned instrument;

#define NI_TRANSITION_TO(task) \
  instrument = 0; \
  transition_to(TASK_REF(task))

//--------------TIMER I0 initialization stuff----------------
// Restart the timer by setting conintuous mode, we can only get away with doing
// it this way because pausing sets the MC bits to 0, otherwise we'd need to
// clear and then overwrite :) 
#define TRANS_TIMER_START \
  if(instrument) { \
  /*P1OUT |= BIT0;*/ \
  /*P1DIR |= BIT0;*/ \
  /*__delay_cycles(4000);*/\
  trans_starts++; \
  /*printf("Start ");*/\
  /*printf("A %u %u\r\n",trans_starts,trans_stops);*/ \
  TA0CTL |= MC__CONTINUOUS;\
  } \

// Pause the timer by setting the MC bits to 0
#define MODE_SHIFT 4
#define TRANS_TIMER_STOP \
  if(instrument) { \
  TA0CTL &= ~(0x3 << MODE_SHIFT); \
  add_ticks(&overflows_tsk_tran, &tsk_tran_ticks, TA0R);\
  /*printf("F T:%u %u\r\n",overflows,transition_ticks);*/\
  TA0CTL |= TACLR; \
  TA0R = 0; \
  trans_stops++;\
  } \
  else { instrument = 1;}
  //printf("X %u %u\r\n",trans_starts,trans_stops);

#define APP_FINISHED \
    printf("Time in tsk-only transition = %u + %u\r\n", \
                overflows_tsk_tran,tsk_tran_ticks);\
    printf("Total start stops: %u %u\r\n",trans_starts, trans_stops);

#define NI_TRANS \
  instrument = 0;

#else // timer

#define NI_TRANS \
  ;

#define NI_TRANSITION_TO(task) transition_to(TASK_REF(task))

#define TRANS_TIMER_START \
  ;
#define TRANS_TIMER_STOP \
  ;
#endif

#endif // ALPACA_H
