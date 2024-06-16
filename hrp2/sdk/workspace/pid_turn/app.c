#include "ev3api.h"
#include"app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define KP 0.4104
#define KI 1.368
#define KD 0.03078
#define DELTA_T 0.004
#define POWER -40
int diff[2];
int integral;
int p, i, d, sensor2, sensor3, motor;
int count = 0;

int diff2[2];
int integral2;
int p2, i2, d2, sensor22, sensor32, motor2, angle, reflect2, reflect3;
int count2 = 0;



void trace_task(intptr_t unused) {
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
	
	motor = POWER + p + i + d;
	
	if(count == 1){
		ev3_motor_set_power(EV3_PORT_B, POWER);
	    ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		ev3_motor_set_power(EV3_PORT_B, motor);
	    ev3_motor_set_power(EV3_PORT_D, POWER);
	}	
}

	
	
void turn_task(intptr_t unused) {
	diff2[0] = diff2[1];
	sensor22 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	sensor32 = ev3_color_sensor_get_reflect(EV3_PORT_3);
	if(sensor22 - sensor32 > 0){
		diff2[1] = sensor22 - sensor32;
		count2 = 1;
	}else{
		diff2[1] = sensor32 - sensor22;
		count2 = 0;
	}
	integral2 += (diff2[0] + diff2[1]) / 2.0 * DELTA_T;
	
	p2 = KP * diff2[1];
	i2 = KI * integral2;
	d2 = KD * (diff2[1] - diff2[0]) / DELTA_T;
	
	motor2 = p2 + i2 + d2;
	
	
	if(count2 == 1){
		ev3_motor_set_power(EV3_PORT_B, -motor2);
	    ev3_motor_set_power(EV3_PORT_D, motor2);
	}else{
		ev3_motor_set_power(EV3_PORT_B, motor2);
	    ev3_motor_set_power(EV3_PORT_D, -motor2);
	}	
}



void turn2_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_B);
	
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
	
	ev3_sta_cyc(CYCHDR2);
	
	do{
		reflect2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	    reflect3 = ev3_color_sensor_get_reflect(EV3_PORT_3);
	}while(reflect3 - reflect2 > 10 );
	
	ev3_stp_cyc(CYCHDR2);
	
	ev3_motor_stop(EV3_PORT_B, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	
	wup_tsk(MAIN_TASK);
}



void main_task(intptr_t unused)  {
	int touch1, touch2;
	ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
	ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);
	ev3_sensor_config(EV3_PORT_1, TOUCH_SENSOR);
	ev3_sensor_config(EV3_PORT_4, TOUCH_SENSOR);
	
	while(1){
	ev3_sta_cyc(CYCHDR1);
	
	do{
		touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
		touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
	}while(touch1 == false || touch2 == false);
	
	ev3_stp_cyc(CYCHDR1);
	
	ev3_motor_stop(EV3_PORT_B, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	
	ev3_motor_rotate(EV3_PORT_B, 125, 40, false);
	ev3_motor_rotate(EV3_PORT_D, 125, 40, true);
	
	ev3_motor_stop(EV3_PORT_B, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	
	act_tsk(TURN2_TASK);
	
	slp_tsk();
	
	}
}
	