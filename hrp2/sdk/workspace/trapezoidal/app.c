#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

double bt, ct, powert, anglet;
double maxt = 80.0;
double mint = 20.0;
double at = 0.3;
int *pTt;
int Tt;

void trapezoidal_task(intptr_t unused) {
	bt = (maxt - mint) / at;
	ct = Tt - 2 * bt;

	ev3_motor_reset_counts(EV3_PORT_A);

	do{
		anglet = ev3_motor_get_counts(EV3_PORT_A);
		powert = at * -1.0 * anglet + mint;
		ev3_motor_set_power(EV3_PORT_D, -1.0 * powert);
		ev3_motor_set_power(EV3_PORT_A, -1.0 * powert);
	}while(anglet > -1.0 * bt);

	ev3_motor_set_power(EV3_PORT_D, -1.0 * maxt);
	ev3_motor_set_power(EV3_PORT_A, -1.0 * maxt);
	do{
		anglet = ev3_motor_get_counts(EV3_PORT_A);
	}while(anglet > -1.0 * ct);

	do{
		anglet = ev3_motor_get_counts(EV3_PORT_A);
		powert = -1.0 * at * -1.0 * anglet + at * Tt + mint;
		ev3_motor_set_power(EV3_PORT_D, -1.0 * powert);
		ev3_motor_set_power(EV3_PORT_A, -1.0 * powert);
	}while(anglet > -1.0 * Tt);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
}

void main_task(intptr_t unused) {
	ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);
	ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);
	pTt = &Tt;
	Tt = 1250.0;

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);


	act_tsk(RUN_TASK);
}
