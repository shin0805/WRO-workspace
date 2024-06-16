#include "target_test.h"

#ifndef STACK_SIZE
#define	STACK_SIZE		4096
#endif

#ifndef TOPPERS_MACRO_ONLY
extern void main_task(intptr_t exinf);
extern void trace_task(intptr_t exinf);
extern void trace2_task(intptr_t exinf);
extern void trace3_task(intptr_t exinf);
extern void trace22_task(intptr_t exinf);
extern void trace33_task(intptr_t exinf);
extern void right_task(intptr_t exinf);
extern void left_task(intptr_t exinf);
extern void rotation_task(intptr_t exinf);
extern void change_yb_task(intptr_t exinf);
extern void change_rg_task(intptr_t exinf);
extern void scan_task(intptr_t exinf);
extern void ship_left_task(intptr_t exinf);
extern void ship_right_task(intptr_t exinf);
extern void catch_task(intptr_t exinf);
extern void ht1_task(intptr_t exinf);
extern void ht4_task(intptr_t exinf);
extern void trapezoidal_task(intptr_t exinf);
extern void curve_task(intptr_t exinf);
#endif
