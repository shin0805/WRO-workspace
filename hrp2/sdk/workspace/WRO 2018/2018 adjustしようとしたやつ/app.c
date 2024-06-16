#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

//PID制御の変数
double kp, ki, kd, gain_rate;
double delta_t = 0.004;
int diff[2];
int integral, p, i, d, sensor2, sensor3, motor, count, power, sensor1;

//角度の変数(使い捨て)
int angle;

//コンテナの色の変数
int container[4];

//船の色の変数
int ship[4];

//コンテナとコントローラのカウント、コンテナ色読みのカウント、シップのカウント、コンテナのカウント
int counter = 0;

//コンテナ色読みのカウント
int counter2 = 0;

//マルチタスクからマルチタスクをおこすカウント
int counter3 = 0;

//HTのよみこみの変数
uint8_t *pHT = 0;
uint8_t ht = 0;

//速度の変数
int fast = -30;
int slow = -30;

//gainの変数
int two_fast = 1.14;
int two_slow = 1.14;
int one_fast = 2.0;
int one_slow = 2.0;

//adjustの変数
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

	if (counter3 == 0) {
		wup_tsk(MAIN_TASK);
	}else if (counter3 == 1) {
		wup_tsk(CHANGE_TASK);
	}else if (counter3 == 2) {
		wup_tsk(SHIP_LEFT_TASK);
	}else if (counter3 == 3){
		wup_tsk(SHIP_RIGHT_TASK);
	}else{

	}
}


void trace_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	sensor3 = ev3_color_sensor_get_reflect(EV3_PORT_3);

	if(sensor3 > sensor2) {
		diff[1] = sensor3 - sensor2;
		count = 1;
	}else{
		diff[1] = sensor2 - sensor3;
		count = 0;
	}

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;
	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;
	motor = power + p + i + d;

	if(count == 1) {
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void trace1_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor1 = ev3_color_sensor_get_reflect(EV3_PORT_1);

	if(50 < sensor1) {
		diff[1] = sensor1 - 50;
		count = 1;
	}else{
		diff[1] = 50 - sensor1;
		count = 0;
	}

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;
	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;
	motor = power - p - i - d;

	if(count == 1) {
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void trace2_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);

	if(50 > sensor2) {
		diff[1] = 50 - sensor2;
		count = 1;
	}else{
		diff[1] = sensor2 - 50;
		count = 0;
	}

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;
	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;
	motor = power + p + i + d;

	if(count == 1) {
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void trace3_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor3 = ev3_color_sensor_get_reflect(EV3_PORT_3);

	if(sensor3 > 50) {
		diff[1] = sensor3 - 50;
		count = 1;
	}else{
		diff[1] = 50 - sensor3;
		count = 0;
	}

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;
	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;
	motor = power + p + i + d;

	if(count == 1) {
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void trace22_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);

	if(sensor2 > 50) {
		diff[1] = sensor2 - 50;
		count = 1;
	}else{
		diff[1] = 50 - sensor2;
		count = 0;
	}

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;
	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;
	motor = power + p + i + d;

	if(count == 1) {
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void trace33_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor3 = ev3_color_sensor_get_reflect(EV3_PORT_3);

	if(50 > sensor3) {
		diff[1] = 50 - sensor3;
		count = 1;
	}else{
		diff[1] = sensor3 - 50;
		count = 0;
	}

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;
	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;
	motor = power + p + i + d;

	if(count == 1) {
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void right_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, -30);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 50);

	do{
		ev3_motor_set_power(EV3_PORT_A, 40);
		ev3_motor_set_power(EV3_PORT_D, -40);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 50);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, -30);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 30);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void left_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 30);
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 50);

	do{
		ev3_motor_set_power(EV3_PORT_A, -40);
		ev3_motor_set_power(EV3_PORT_D, 40);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= 50);

	ev3_motor_reset_counts(EV3_PORT_D);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 30);
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 30);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void rotation_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, -30);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 200);

	do{
		ev3_motor_set_power(EV3_PORT_A, 40);
		ev3_motor_set_power(EV3_PORT_D, -40);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 50);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, -30);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 30);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void scan_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	if(counter == 2){
		ev3_motor_set_power(EV3_PORT_A, 10);
		ev3_motor_set_power(EV3_PORT_D, 10);
		ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
		*pHT = 0;
		do{
			ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
			if((*pHT == 1 || *pHT == 7 || *pHT == 8 || *pHT == 9 || *pHT == 10) && container[counter] == 0) {
				container[counter] = 1;
				ev3_speaker_play_tone(1975.53, 200);
			}else if((*pHT == 4 || *pHT == 13) && container[counter] == 0) {
				container[counter] = 2;
				ev3_speaker_play_tone(1396.91, 200);
			}else if(*pHT == 5 && container[counter] == 0) {
				container[counter] = 3;
				ev3_speaker_play_tone(880.00, 200);
			}else if((*pHT == 2 || *pHT == 3) && container[counter] == 0) {
				container[counter] = 4;
				ev3_speaker_play_tone(261.63, 200);
			}else{

			}
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 93);
	}else{
		ev3_motor_set_power(EV3_PORT_A, -10);
		ev3_motor_set_power(EV3_PORT_D, -10);
		ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
		*pHT = 0;
		do{
			ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
			if((*pHT == 1 || *pHT == 7 || *pHT == 8 || *pHT == 9 || *pHT == 10) && container[counter] == 0) {
				container[counter] = 1;
				ev3_speaker_play_tone(1975.53, 200);
			}else if((*pHT == 4 || *pHT == 13) && container[counter] == 0) {
				container[counter] = 2;
				ev3_speaker_play_tone(1396.91, 200);
			}else if(*pHT == 5 && container[counter] == 0) {
				container[counter] = 3;
				ev3_speaker_play_tone(880.00, 200);
			}else if((*pHT == 2 || *pHT == 3) && container[counter] == 0) {
				container[counter] = 4;
				ev3_speaker_play_tone(261.63, 200);
			}else{

			}
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -93);
	}

	wup_tsk(MAIN_TASK);
}


void ship_left_task(intptr_t unused) {
	//方向転換
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	ev3_motor_set_power(EV3_PORT_A, 20);
	ev3_motor_set_power(EV3_PORT_D, -20);
	}while(ev3_motor_get_counts(EV3_PORT_D) >= -60);

	ev3_motor_rotate(EV3_PORT_A, 105, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -105, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_A, 20);
	ev3_motor_set_power(EV3_PORT_D, 20);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 && ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);

	ev3_motor_rotate(EV3_PORT_A, 30, -1 * slow, false);
	ev3_motor_rotate(EV3_PORT_D, 30, -1 * slow, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	counter3 = 2;
	act_tsk(ADJUST_TASK);

	slp_tsk();

	counter3 = 0;


	angle = 47 + (3 - ship[container[counter] - 1]) * 270;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= angle);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//フードとコントローラーをシップに乗せる
	ev3_motor_set_power(EV3_PORT_C, -10);

	//シップを出航させる
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(2000);

	ev3_motor_stop(EV3_PORT_C, true);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//進む
	ev3_motor_rotate(EV3_PORT_A, -350, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -350, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, -160, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 160, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//アームを元に戻す
	ev3_motor_set_power(EV3_PORT_C, 50);
	tslp_tsk(800);

	//シップを出航させた、完了の0
	ship[container[counter] - 1] = 0;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, fast);
	ev3_motor_set_power(EV3_PORT_D, fast);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -1 * angle - 95);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);



	wup_tsk(MAIN_TASK);
}


void ship_right_task(intptr_t unused) {
	//方向転換
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	ev3_motor_set_power(EV3_PORT_A, -20);
	ev3_motor_set_power(EV3_PORT_D, 20);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -60);

	ev3_motor_rotate(EV3_PORT_A, -105, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 105, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_A, 20);
	ev3_motor_set_power(EV3_PORT_D, 20);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 && ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_rotate(EV3_PORT_A, 30, -1 * slow, false);
	ev3_motor_rotate(EV3_PORT_D, 30, -1 * slow, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	counter3 = 3;
	act_tsk(ADJUST_TASK);

	slp_tsk();

	counter3 = 0;

	angle = 47 + (ship[container[counter] - 1] - 4) * 270;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= angle);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, -160, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 160, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//フードとコントローラーをシップに乗せる
	ev3_motor_set_power(EV3_PORT_C, -10);

	//シップを出航させる
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(2000);

	ev3_motor_stop(EV3_PORT_C, true);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//進む
	ev3_motor_rotate(EV3_PORT_A, -350, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -350, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//アームを元に戻す
	ev3_motor_set_power(EV3_PORT_C, 50);
	tslp_tsk(800);

	//シップを出航させた、完了の0
	ship[container[counter] - 1] = 0;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, fast);
	ev3_motor_set_power(EV3_PORT_D, fast);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -1 * angle - 95);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void catch_task(intptr_t unused) {
	if(counter == 1) {
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 60);

		ev3_motor_rotate(EV3_PORT_A, 90, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -90, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -60);

		ev3_motor_rotate(EV3_PORT_A, -90, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 90, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	if (counter != 2){
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		tslp_tsk(800);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	if(counter == 0) {
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, -30);
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -160);

		ev3_motor_rotate(EV3_PORT_A, -50, 10, false);
		ev3_motor_rotate(EV3_PORT_D, -50, 10, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else if(counter == 2) {
		ev3_motor_rotate(EV3_PORT_A, -69, 10, false);
		ev3_motor_rotate(EV3_PORT_D, -69, 10, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, -30);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -270);

		ev3_motor_rotate(EV3_PORT_A, -50, 10, false);
		ev3_motor_rotate(EV3_PORT_D, -50, 10, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	ev3_motor_set_power(EV3_PORT_B, 80);
	tslp_tsk(800);

	wup_tsk(MAIN_TASK);
}


void change_task(intptr_t unused) {
	counter3 = 1;
	//コントローラーの横まで進む
	if(container[counter] == 1 || container[counter] == 3){
		ev3_motor_rotate(EV3_PORT_A, 145, -1 * fast, false);
		ev3_motor_rotate(EV3_PORT_D, 145, -1 * fast, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_motor_rotate(EV3_PORT_A, -318, -1 * fast, false);
		ev3_motor_rotate(EV3_PORT_D, -318, -1 * fast, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	if (container[counter] == 1 || container[counter] == 2) {
		//方向転換
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) >= -60);

		ev3_motor_rotate(EV3_PORT_A, 105, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -105, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}else{
		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -60);

		ev3_motor_rotate(EV3_PORT_A, -105, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 105, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}


	//色まで進む
	ev3_motor_set_power(EV3_PORT_A, -20);
	ev3_motor_set_power(EV3_PORT_D, -20);
	do{
	}while(ev3_color_sensor_get_color(EV3_PORT_2) == 6);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//コントローラーとコンテナのいれかえ
	ev3_motor_rotate(EV3_PORT_B, -290, 100, true);
	ev3_motor_stop(EV3_PORT_B, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_A, 30, 20, false);
	ev3_motor_rotate(EV3_PORT_D, 30, 20, true);

	ev3_motor_set_power(EV3_PORT_B, -100);
	tslp_tsk(1200);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_motor_stop(EV3_PORT_B, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_A, 210, -1 * slow, false);
	ev3_motor_rotate(EV3_PORT_D, 210, -1 * slow, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_B, 150, 30, true);

	if(container[counter] == 1 || container[counter] == 2){
		//方向転換
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) >= -60);

		ev3_motor_rotate(EV3_PORT_A, 90, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -90, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -60);

		ev3_motor_rotate(EV3_PORT_A, -90, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 90, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	//adjust
	if(container[counter] == 1 || container[counter] == 3){
		ev3_motor_rotate(EV3_PORT_A, 464, -1 * fast, false);
		ev3_motor_rotate(EV3_PORT_D, 464, -1 * fast, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}
	act_tsk(ADJUST_TASK);
	slp_tsk();
	counter3 = 0;

	wup_tsk(MAIN_TASK);
}


void main_task(intptr_t unused) {
	ev3_lcd_set_font (EV3_FONT_MEDIUM);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);	//右前カラーセンサー
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);	//左前カラーセンサー
	ev3_sensor_config(EV3_PORT_1, COLOR_SENSOR);	//後カラセンサー
	ev3_sensor_config(EV3_PORT_4, HT_NXT_COLOR_SENSOR);	//後HTセンサー
	ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);	//右タイヤ
	ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);	//左タイヤ
	ev3_motor_config(EV3_PORT_B, MEDIUM_MOTOR);	//アーム
	ev3_motor_config(EV3_PORT_C, MEDIUM_MOTOR);	//下ろすやつ

	ev3_color_sensor_get_reflect(EV3_PORT_1);
	ev3_color_sensor_get_reflect(EV3_PORT_2);
	ev3_color_sensor_get_reflect(EV3_PORT_3);

	//HT色読み さす
	pHT = &ht;

	//ループ処理の変数
	int i;

	//角度調節の変数
	int angle2 = 0;

	//丁字路まで進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -110);

	power = fast;
	gain_rate = one_fast;
	ev3_sta_cyc(TRACE2);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
	ev3_speaker_play_tone(1174.66, 200);

	//軸に乗せる
	power = slow;
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -65);
	ev3_stp_cyc(TRACE2);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 30);
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 80);

	do{
		ev3_motor_set_power(EV3_PORT_A, -40);
		ev3_motor_set_power(EV3_PORT_D, 40);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 50);

	ev3_motor_reset_counts(EV3_PORT_D);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 30);
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 20);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//container[0]まで進む
	power = fast;
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_sta_cyc(TRACE22);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -570);

	ev3_stp_cyc(TRACE22);

	ev3_motor_set_power(EV3_PORT_A, fast);
	ev3_motor_set_power(EV3_PORT_D, fast);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -48);

	ev3_sta_cyc(TRACE22);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -684);

	ev3_stp_cyc(TRACE22);

	//container[0]をよみこむ
	counter = 0;
	act_tsk(SCAN_TASK);
	slp_tsk();

	//container[0]があった場合
	if(container[0] != 0) {
		//十字路まで進む
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[0]を回収

		ev3_motor_rotate(EV3_PORT_A, 265, -1 * slow, false);
		ev3_motor_rotate(EV3_PORT_D, 265, -1 * slow, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		counter = 0;
		act_tsk(CATCH_TASK);
		slp_tsk();

		ev3_motor_rotate(EV3_PORT_A, -480, -1 * slow, false);
		ev3_motor_rotate(EV3_PORT_D, -480, -1 * slow, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//コントローラーとコンテナの入れ替えに向かう
		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -60);

		ev3_motor_rotate(EV3_PORT_A, -105, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 105, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//線へ
		ev3_motor_rotate(EV3_PORT_A, -360, -1 * fast, false);
		ev3_motor_rotate(EV3_PORT_D, -360, -1 * fast, true);

		//adjust
		act_tsk(ADJUST_TASK);

		slp_tsk();

		//コンテナとコントローラーの入れ替え
		counter = 0;
		act_tsk(CHANGE_TASK);

		slp_tsk();




		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, fast);
		ev3_motor_set_power(EV3_PORT_D, fast);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -815);

		//container[3]をよみこむ
		counter = 3;
		act_tsk(SCAN_TASK);
		slp_tsk();

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, fast);
		ev3_motor_set_power(EV3_PORT_D, fast);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -430);


		//adjust
		act_tsk(ADJUST_TASK);
		slp_tsk();

		//軸に乗せる
		ev3_motor_rotate(EV3_PORT_A, -90, -1 * slow, false);
		ev3_motor_rotate(EV3_PORT_D, -90, -1 * slow, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		act_tsk(LEFT_TASK);
		slp_tsk();

		//L字路まで進む
		gain_rate = one_fast;
		power = fast;
		ev3_sta_cyc(TRACE2);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
		ev3_speaker_play_tone(1174.66, 200);
		ev3_stp_cyc(TRACE2);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[2]をよみこむ
		counter = 2;
		act_tsk(SCAN_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//壁合わせ
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		tslp_tsk(2500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//少し下がる
		ev3_motor_rotate(EV3_PORT_A, 90, slow * -1, false);
		ev3_motor_rotate(EV3_PORT_D, 90, slow * -1, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);


		//ラインまで進む
		ev3_motor_set_power(EV3_PORT_A, fast);
		ev3_motor_set_power(EV3_PORT_D, fast);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		//軸に乗せる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//壁合わせ
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		tslp_tsk(2000);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//シップまで進む
		gain_rate = one_slow;
		power =  slow;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_sta_cyc(TRACE33);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -83);
		ev3_stp_cyc(TRACE33);

		for (i = 1; i <= 5; i++) {
			//シップをよみこむ
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -10);
			ev3_motor_set_power(EV3_PORT_D, -10);
			ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
			*pHT = 0;
			do{
				ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
				if((*pHT == 7 || *pHT == 8 || *pHT == 9) && ship[0] == 0) {
					ev3_speaker_play_tone(1975.53, 200);
					ship[0] = i;
				}else if((*pHT == 4 || *pHT == 13) && ship[1] == 0) {
					ev3_speaker_play_tone(1396.91, 200);
					ship[1] = i;
				}else if(*pHT == 5 && ship[2] == 0) {
					ev3_speaker_play_tone(880.00, 200);
					ship[2] = i;
				}else if((*pHT == 2 || *pHT == 3) && ship[3] == 0) {
					ev3_speaker_play_tone(261.63, 200);
					ship[3] = i;
				}else{

				}
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -83);

			if (ship[container[0] - 1] == i) {
				//よみこんだシップとcontainer[0]が等しい場合、コントローラーとフードをシップに乗せ、出航させる
				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//下がる
				ev3_motor_rotate(EV3_PORT_A, 130, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 130, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//フードとコントローラーをシップに乗せる
				ev3_motor_set_power(EV3_PORT_C, -10);

				//シップを出航させる
				ev3_motor_set_power(EV3_PORT_A, 30);
				ev3_motor_set_power(EV3_PORT_D, 30);
				tslp_tsk(2000);

				ev3_motor_stop(EV3_PORT_C, true);
				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//進む
				ev3_motor_rotate(EV3_PORT_A, -355, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -355, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

			  //方向転換
				ev3_motor_rotate(EV3_PORT_A, -160, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 160, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//アームを元に戻す
				ev3_motor_set_power(EV3_PORT_C, 50);
				tslp_tsk(800);

				//進む
				if (i != 2){
					ev3_motor_rotate(EV3_PORT_A, -85, 30, false);
					ev3_motor_rotate(EV3_PORT_D, -85, 30, true);
				}else{
					ev3_motor_reset_counts(EV3_PORT_A);
					power = slow;
					gain_rate = one_slow;
					ev3_sta_cyc(TRACE33);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -85);
					ev3_stp_cyc(TRACE33);
				}

				//シップを出航させた、完了の0
				ship[container[0] - 1] = 0;

			}

			//次のシップまで進む
			if (i == 2) {
				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 20);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -20);
			}else{
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_sta_cyc(TRACE33);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -175);
				ev3_stp_cyc(TRACE33);
			}

		}

		//6つ目のシップをよみこむ
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -10);
		ev3_motor_set_power(EV3_PORT_D, -10);
		ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
		*pHT = 0;
		do{
			ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
			if((*pHT == 7 || *pHT == 8 || *pHT == 9) && ship[0] == 0) {
				ev3_speaker_play_tone(1975.53, 200);
				ship[0] = 6;
			}else if((*pHT == 4 || *pHT == 13) && ship[1] == 0) {
				ev3_speaker_play_tone(1396.91, 200);
				ship[1] = 6;
			}else if(*pHT == 5 && ship[2] == 0) {
				ev3_speaker_play_tone(880.00, 200);
				ship[2] = 6;
			}else if((*pHT == 2 || *pHT == 3) && ship[3] == 0) {
				ev3_speaker_play_tone(261.63, 200);
				ship[3] = 6;
			}else{

			}
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -45);

		if (ship[container[0] - 1] == 6) {
			//6つ目のシップとcontainer[0]が等しい場合、コントローラーとフードをシップに乗せ、出航させる
			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);


			//下がる
			ev3_motor_rotate(EV3_PORT_A, 57, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 57, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//フードとコントローラーをシップに乗せる
			ev3_motor_set_power(EV3_PORT_C, -10);

			//シップを出航させる
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			tslp_tsk(2000);

			ev3_motor_stop(EV3_PORT_C, true);
			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//進む
			ev3_motor_rotate(EV3_PORT_A, -350, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -350, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, -160, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 160, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//アームを元に戻す
			ev3_motor_set_power(EV3_PORT_C, 50);
			tslp_tsk(800);

			//進む
			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			tslp_tsk(1000);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//シップを出航させた、完了の0
			ship[container[0] - 1] = 0;
		}

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//真ん中まで下がる
		ev3_motor_rotate(EV3_PORT_A, 710, fast * -1, false);
		ev3_motor_rotate(EV3_PORT_D, 710, fast * -1, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		act_tsk(RIGHT_TASK);
		slp_tsk();

		//ここからcontainer[2]
		//丁字路まで進む
		gain_rate = two_fast;
		power = fast;
		ev3_sta_cyc(TRACE);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//軸に乗せる
		ev3_motor_rotate(EV3_PORT_A, -95, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -95, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		act_tsk(RIGHT_TASK);
		slp_tsk();

		//L字路まで進む
		gain_rate = one_fast;
		ev3_sta_cyc(TRACE2);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE2);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[2]の回収
		ev3_motor_rotate(EV3_PORT_A, 155, -1 * slow, false);
		ev3_motor_rotate(EV3_PORT_D, 155, -1 * slow, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		counter = 2;
		act_tsk(CATCH_TASK);
		slp_tsk();

		//丁字路まで進む
		gain_rate = two_fast;
		power = fast;
		ev3_motor_set_power(EV3_PORT_A, fast);
		ev3_motor_set_power(EV3_PORT_D, fast);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_rotate(EV3_PORT_A, -85, -1 * slow, false);
		ev3_motor_rotate(EV3_PORT_D, -85, -1 * slow, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		act_tsk(LEFT_TASK);
		slp_tsk();

		//クロスまですすむ
		gain_rate = two_fast;
		power = fast;
		ev3_sta_cyc(TRACE);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//中心まですすむ
		ev3_motor_rotate(EV3_PORT_A, -96, -1 * slow, false);
		ev3_motor_rotate(EV3_PORT_D, -96, -1 * slow, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//コントローラーとコンテナの入れ替えに向かう
		//方向転換
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		}while(ev3_motor_get_counts(EV3_PORT_D) >= -60);

		ev3_motor_rotate(EV3_PORT_A, 105, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -105, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//線へ
		ev3_motor_rotate(EV3_PORT_A, -759, -1 * fast, false);
		ev3_motor_rotate(EV3_PORT_D, -759, -1 * fast, true);

		//adjust
		act_tsk(ADJUST_TASK);

		slp_tsk();


		//コンテナとコントローラーの入れ替え
		counter = 2;
		act_tsk(CHANGE_TASK);

		slp_tsk();

		//線へ
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		ev3_motor_set_power(EV3_PORT_A, fast);
		ev3_motor_set_power(EV3_PORT_D, fast);
		}while(ev3_motor_get_counts(EV3_PORT_D) >= -1243);

		//adjust
		act_tsk(ADJUST_TASK);

		slp_tsk();

		ev3_motor_rotate(EV3_PORT_A, -300, -1 * slow, false);
		ev3_motor_rotate(EV3_PORT_D, -300, -1 * slow, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		counter = 2;

		if ((ship[container[counter] - 1]) <= 3) {
			act_tsk(SHIP_LEFT_TASK);
			slp_tsk();

			act_tsk(RIGHT_TASK);
			slp_tsk();

		}else{
			act_tsk(SHIP_RIGHT_TASK);
			slp_tsk();

			act_tsk(LEFT_TASK);
			slp_tsk();
		}

		//container[3]の作業





		//container[1]の特定
		for (i = 0; i <=3; i++) {
			if (ship[i] != 0) {
				break;
			}
		}

		container[3] = i + 1;

















	}


}
