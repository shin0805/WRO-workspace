#include "target_test.h"

#ifndef STACK_SIZE
#define	STACK_SIZE		4096		
#endif

#ifndef TOPPERS_MACRO_ONLY
extern void	main_task(intptr_t exinf);
extern void run_task(intptr_t exinf);
extern void cyclic_task(intptr_t exinf);
#endif 