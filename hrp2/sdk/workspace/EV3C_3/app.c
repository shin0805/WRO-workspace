#include "ev3api.h"
#include"app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define R EV3_PORT_B
#define L EV3_PORT_D

void run_task(intptr_t unused) {
	int angle;
	ev3_motor_reset_counts(R);
	ev3_motor_set_power(R, -1000);
	ev3_motor_set_power(L, -1000);
	do{
		angle = ev3_motor_get_counts(R);
	}while(angle >= -100);
	ev3_motor_reset_counts(R);
	ev3_motor_set_power(R, -20);
	ev3_motor_set_power(L, -20);
	do{
		angle = ev3_motor_get_counts(R);
	}while(angle >= -200);
	ev3_motor_reset_counts(R);
	ev3_motor_set_power(R, -10);
	ev3_motor_set_power(L, -10);
	do{
		angle = ev3_motor_get_counts(R);
	}while(angle >= -100);
	ev3_motor_stop(R, true);
	ev3_motor_stop(L,true);
	tslp_tsk(200);
	
	wup_tsk(MAIN_TASK);
}

void run2_task(intptr_t unused) {
	ev3_motor_rotate(R, 626, 20, false);
	ev3_motor_rotate(L, -626, 20, true);
	
	ev3_motor_stop(R, true);
	ev3_motor_stop(L,true);
	tslp_tsk(200);
	wup_tsk(MAIN_TASK);
}
	

void main_task(intptr_t unused)  {
	ev3_motor_config(R, LARGE_MOTOR);
	ev3_motor_config(L, LARGE_MOTOR);
	

    act_tsk(RUN_TASK);
    
    slp_tsk();
    
    act_tsk(RUN2_TASK);
    

	
}
	