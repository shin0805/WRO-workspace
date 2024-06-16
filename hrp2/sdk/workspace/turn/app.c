#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define KP 0.4104
#define KI 1.368
#define KD 0.03078
#define DELTA_T 0.004

int diff[2];
int integral;

int p, i, d, sensor2, sensor3, motor, reflect2, reflect3, angle;
int count = 0;

void cyclic_task(intptr_t unused) {
	diff[0] = diff[1];
	sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	sensor3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
	if(sensor2 - sensor3 > 0){
		diff[1] = sensor2 - sensor3;
		count = 1;
	}else{
		diff[1] = sensor3 - sensor2;
		count = 0;
	}
	integral += (diff[0] + diff[1]) / 2.0 * DELTA_T;
	
	p = KP * diff[1];
	i = KI * integral;
	d = KD * (diff[1] - diff[0]) / DELTA_T;
	
	motor = p + i + d;
	
	
	if(count == 1){
		ev3_motor_set_power(EV3_PORT_B, -motor);
	    ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		ev3_motor_set_power(EV3_PORT_B, motor);
	    ev3_motor_set_power(EV3_PORT_D, -motor);
	}	
}
	
void run_task(intptr_t unused) {
	do{
		angle = ev3_motor_get_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, 40);
		ev3_motor_set_power(EV3_PORT_D, -40);
	}while(angle <= 100);
	
	do{
	reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	ev3_motor_set_power(EV3_PORT_B, 40);
	ev3_motor_set_power(EV3_PORT_D, -40);
	}while(reflect2 >= 50);
	
	ev3_sta_cyc(CYCHDR1);
	
	do{
		reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	    reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
	}while(reflect3 - reflect2 > 10 );
	
	ev3_stp_cyc(CYCHDR1);
	
	ev3_motor_stop(EV3_PORT_B, true);
	ev3_motor_stop(EV3_PORT_D, true);
}

void main_task(intptr_t unused) {
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);
	ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
	ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);
	
	reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
	
	act_tsk(RUN_TASK);
}
