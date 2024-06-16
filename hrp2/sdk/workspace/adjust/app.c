#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define BLACK1 60
#define BLACK2 60

int port1, port4;
int backc = 0;
int backc2 = 0;

void adjust_task(intptr_t unused) {
	backc = 0;
	backc2 = 0;
	do{
	if (ev3_color_sensor_get_reflect(EV3_PORT_3) > 60 && ev3_color_sensor_get_reflect(EV3_PORT_1) > 60){
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		if (backc == 1){
		backc2 += 1;
		backc = 0;
		}
	}else if (ev3_color_sensor_get_reflect(EV3_PORT_3) > 60 && ev3_color_sensor_get_reflect(EV3_PORT_1) <= 60){
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		if (backc == 1){
		backc2 += 1;
		backc = 0;
		}
	}else if (ev3_color_sensor_get_reflect(EV3_PORT_3) <= 60 && ev3_color_sensor_get_reflect(EV3_PORT_1) > 60){
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		if (backc == 1){
		backc2 += 1;
		backc = 0;
		}
	}else if (backc2 !=6){
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		backc = 1;
	}else{
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
		break;
	}
	}while(1);

	wup_tsk(MAIN_TASK);
}

void main_task(intptr_t unused) {
	ev3_sensor_config(EV3_PORT_1, COLOR_SENSOR);
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);
	ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);
	ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);

	act_tsk(ADJUST_TASK);

	slp_tsk();


}
