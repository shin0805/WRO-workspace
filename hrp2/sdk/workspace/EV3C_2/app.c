#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define R	EV3_PORT_B
#define L	EV3_PORT_D

void run_task(intptr_t unused) {
	ev3_motor_steer(L, R, 20, 0 );
	tslp_tsk(3000);

	ev3_motor_stop(L, true);
	ev3_motor_stop(R, true);
	tslp_tsk(200);
}

void main_task(intptr_t unused) {
	ev3_motor_config( L , LARGE_MOTOR );
	ev3_motor_config( R , LARGE_MOTOR );
	
	act_tsk(RUN_TASK);
}