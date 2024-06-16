#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define R EV3_PORT_B
#define L EV3_PORT_D

void run_task(intptr_t unused) {
	ev3_motor_reset_counts(R);
	ev3_motor_reset_counts(L);
	
	char ran, lan;
	
	ran = ev3_motor_get_counts(R);
	lan = ev3_motor_get_counts(L);
	
	ev3_lcd_draw_string(ran,0,0);
}

void main_task(intptr_t unused) {
	ev3_motor_config(R, LARGE_MOTOR);
	ev3_motor_config(L, LARGE_MOTOR);
	
	act_tsk(RUN_TASK);
}
	
