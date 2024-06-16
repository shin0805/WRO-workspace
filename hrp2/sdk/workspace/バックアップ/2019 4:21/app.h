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
extern void arm_task(intptr_t exinf);
extern void right_task(intptr_t exinf);
extern void left_task(intptr_t exinf);
extern void rotation_task(intptr_t exinf);
extern void ht1_task(intptr_t exinf);
extern void ht1rgb_task(intptr_t exinf);
extern void twist_task(intptr_t exinf);
extern void expansion_task(intptr_t exinf);
extern void drop_task(intptr_t exinf);
extern void foward_task(intptr_t exinf);
extern void back_task(intptr_t exinf);
#endif
