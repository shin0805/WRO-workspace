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
int p, i, d, sensor1, sensor2, motor, count, power;
int angle;

void cyclic_task1(intptr_t unused) {
		diff[0] = diff[1];
	sensor1 = ev3_color_sensor_get_reflect(EV3_PORT_1);
	sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	if(sensor1 - sensor2 > 0){
		diff[1] = sensor1 - sensor2;
		count = 1;
	}else{
		diff[1] = sensor2 - sensor1;
		count = 0;
	}
	integral += (diff[0] + diff[1]) / 2.0 * DELTA_T;
	
	p = KP * diff[1];
	i = KI * integral;
	d = KD * (diff[1] - diff[0]) / DELTA_T;
	
	motor = power - (p + i + d);
	
	if(count == 1){
		ev3_motor_set_power(EV3_PORT_A, power);
	    ev3_motor_set_power(EV3_PORT_B, motor);
	}else{
		ev3_motor_set_power(EV3_PORT_A, motor);
	    ev3_motor_set_power(EV3_PORT_B, power);
	}    
}

void main_task(intptr_t unused) {
	ev3_sensor_config(EV3_PORT_1, COLOR_SENSOR);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);
	ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);
	ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
	
	power = 10;
	ev3_sta_cyc(CYCHDR1);
	
	
	
}
