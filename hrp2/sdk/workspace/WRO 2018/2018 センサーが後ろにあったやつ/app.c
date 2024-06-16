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


void change_yb_task(intptr_t unused) {
	//コントローラーに対する位置の調節
	if(container[counter] == 4){
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -435);

		ev3_stp_cyc(TRACE);
	}else{
		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		ev3_motor_rotate(EV3_PORT_A, 30, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 30, 30, true);
	}

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//コンテナとコントローラーの入れ替え
	ev3_motor_set_power(EV3_PORT_A, 30);
	tslp_tsk(1500);

	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(500);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_A, -180, 10, false);
	ev3_motor_rotate(EV3_PORT_D, -180, 10, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_B, -100);
	tslp_tsk(1500);

	ev3_motor_rotate(EV3_PORT_A, 79, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 79, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_B, 150, 30, true);

	wup_tsk(MAIN_TASK);
}


void change_rg_task(intptr_t unused) {
	//コントローラーに対する位置の調節
	if(container[counter] == 2){
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -435);

		ev3_stp_cyc(TRACE);
	}else{
		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		ev3_motor_rotate(EV3_PORT_A, 30, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 30, 30, true);
	}

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//コンテナとコントローラーの入れ替え
	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(1500);

	ev3_motor_set_power(EV3_PORT_A, 30);
	tslp_tsk(500);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_A, -180, 10, false);
	ev3_motor_rotate(EV3_PORT_D, -180, 10, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_B, -100);
	tslp_tsk(1500);

	ev3_motor_rotate(EV3_PORT_A, 79, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 79, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_B, 150, 30, true);

	wup_tsk(MAIN_TASK);
}


void scan_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	if(counter == 1){
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
	//ラインまで進む
	ev3_motor_set_power(EV3_PORT_A, fast);
	ev3_motor_set_power(EV3_PORT_D, fast);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	//軸に乗せる
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -55);

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

	angle = 40 + (ship[container[counter] - 1] - 1) * 270;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, -30);
	ev3_motor_set_power(EV3_PORT_D, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -angle);

	ev3_stp_cyc(TRACE33);
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
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -690 + angle);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void ship_right_task(intptr_t unused) {
	//ラインまで進む
	ev3_motor_set_power(EV3_PORT_A, fast);
	ev3_motor_set_power(EV3_PORT_D, fast);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	//軸に乗せる
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -55);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, -160, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 160, 30, true);

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

	angle = 40 + (6 - ship[container[counter] - 1]) * 270;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, -30);
	ev3_motor_set_power(EV3_PORT_D, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -angle);

	ev3_stp_cyc(TRACE22);
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
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -690 + angle);

	ev3_motor_reset_counts(EV3_PORT_A);
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

		ev3_motor_rotate(EV3_PORT_A, 100, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -100, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -60);

		ev3_motor_rotate(EV3_PORT_A, -100, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 100, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	ev3_motor_set_power(EV3_PORT_A, 20);
	ev3_motor_set_power(EV3_PORT_D, 20);
	tslp_tsk(800);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	if(counter == 0 || counter == 2) {
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
		ev3_motor_rotate(EV3_PORT_A, 255, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 255, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		counter = 0;
		act_tsk(CATCH_TASK);
		slp_tsk();

		ev3_motor_rotate(EV3_PORT_A, 110, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 110, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	if(container[0] == 3 || container[0] == 4)
	{
		//container[0]が黄、青だった場合、コンテナとコントローラーを入れ替える
		//方向転換
		act_tsk(LEFT_TASK);
		slp_tsk();

		//十字路まで進む
		gain_rate = two_fast;
		power = fast;
		ev3_sta_cyc(TRACE);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		//container[0]とコントローラーの入れ替え
		counter = 0;
		act_tsk(CHANGE_YB_TASK);
		slp_tsk();

		//方向転換
		act_tsk(RIGHT_TASK);
		slp_tsk();

	}else if (container[0] == 1 || container[0] == 2) {
		//container[0]が赤、黄だった場合、コンテナとコントローラーを入れ替えない
		act_tsk(RIGHT_TASK);
		slp_tsk();
	}else{
		//container[0]がなかった場合
	}

	power = fast;
	ev3_sta_cyc(TRACE);

	if(container[0] == 4) {
		//container[0]が青の場合、十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
	}

	//十字路まで進む
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	//十字路をこえる
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

	ev3_stp_cyc(TRACE);

	//L字路まで進む
	gain_rate = one_fast;
	ev3_sta_cyc(TRACE2);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
	ev3_speaker_play_tone(1174.66, 200);

	//軸に乗せる
	ev3_stp_cyc(TRACE2);
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	act_tsk(LEFT_TASK);
	slp_tsk();

	//丁字路まで進む
	ev3_sta_cyc(TRACE3);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
	ev3_stp_cyc(TRACE3);

	//container[2]まで進む
	gain_rate = two_fast;
	ev3_sta_cyc(TRACE);
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -369);

	ev3_stp_cyc(TRACE);

	//container[2]をよみこむ
	counter = 2;
	act_tsk(SCAN_TASK);
	slp_tsk();

  //L字路まで進む
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);

	if(container[0] == 1 || container[0] == 2){
		//container[0]が赤、緑だった場合、コンテナとコントローラーを入れ替える
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//少し下がる
		ev3_motor_rotate(EV3_PORT_A, 20, slow * -1, false);
		ev3_motor_rotate(EV3_PORT_D, 20, slow * -1, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_rotate(EV3_PORT_A, -330, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		tslp_tsk(200);

		gain_rate = two_fast;
		ev3_sta_cyc(TRACE);

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

		//十字路まで進む
		gain_rate = two_fast;
		power = fast;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		//container[0]とコントローラーのいれかえ
		counter = 0;
		act_tsk(CHANGE_RG_TASK);
		slp_tsk();

		//方向転換
		act_tsk(LEFT_TASK);
		slp_tsk();

		ev3_sta_cyc(TRACE);
		if(container[0] == 2) {
			//container[0]が緑の場合、十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
		}

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

		ev3_stp_cyc(TRACE);

		//L字路まで進む
		gain_rate = one_fast;
		ev3_sta_cyc(TRACE3);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
		ev3_speaker_play_tone(1174.66, 200);

		//軸に乗せる
		ev3_stp_cyc(TRACE3);
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);
	}else{
		//container[0]が青、黄、なかった場合、コンテナとコントローラーは入れ替えない
		//壁合わせ
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		tslp_tsk(2000);

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
	}

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
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -65);

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

	//container[2]へ

	//丁字路まで進む
	gain_rate = two_fast;
	power = fast;
	ev3_sta_cyc(TRACE);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE);

	//container[2]がない場合
	if(container[2] == 0) {
		//軸に乗せる
		ev3_motor_rotate(EV3_PORT_A, -166, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -166, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, -30);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_1) <= 70);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_1) >= 50);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_1) <= 70);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_1) >= 50);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[1]まで進む調節
		angle2 = 520;

	}else{
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

		//軸に乗せる
		ev3_motor_rotate(EV3_PORT_A, -95, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -95, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		act_tsk(LEFT_TASK);
		slp_tsk();

		//十字路まで進む
		gain_rate = two_fast;
		ev3_sta_cyc(TRACE);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[2]を回収
		ev3_motor_rotate(EV3_PORT_A, 255, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 255, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		counter = 2;
		act_tsk(CATCH_TASK);
		slp_tsk();

		if(container[2] == 3 || container[2] == 4){
			//container[2]が黄、青の場合、先にcontainer[1]をよみこむ
			//方向転換
			ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//軸に乗せる
			ev3_motor_rotate(EV3_PORT_A, 110, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 110, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, -30);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_1) <=60);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_1) >=30);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//container[1]に進む調節
			angle2 = 50;

		}else{
			//container[2]が赤、緑の場合、コンテナとコントローラーを入れ替える
			ev3_motor_rotate(EV3_PORT_A, 110, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 110, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			//十字路まで進む
			ev3_sta_cyc(TRACE);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//container[2]とコントローラーの入れ替え
			counter = 2;
			act_tsk(CHANGE_RG_TASK);
			slp_tsk();

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			ev3_sta_cyc(TRACE);
			if(container[2] == 2) {
				//container[2]が緑の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			ev3_stp_cyc(TRACE);

			//L字路まで進む
			gain_rate = one_fast;
			ev3_sta_cyc(TRACE3);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE3);

			//軸に乗せる
			ev3_motor_rotate(EV3_PORT_A, -35, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -35, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			do{
				ev3_motor_set_power(EV3_PORT_A, -30);
				ev3_motor_set_power(EV3_PORT_D, 30);
			}while(ev3_color_sensor_get_reflect(EV3_PORT_1) >= 50);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);
		}

	}



	//container[2]がない場合、まずここまで飛ぶ
	//container[1]まで進む
	power = fast * -1;
	gain_rate = one_fast;
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_sta_cyc(TRACE1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 690 - angle2);

	ev3_stp_cyc(TRACE1);

	//container[1]をよみこむ
	counter = 1;
	act_tsk(SCAN_TASK);
	slp_tsk();

	if(container[2] == 3 || container[2] == 4) {
		//container[2] が黄、青の場合、コンテナとコントローラをいれかえる
		//壁合わせ
		ev3_motor_rotate(EV3_PORT_A, 442, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 442, 30, true);
		ev3_motor_rotate(EV3_PORT_A, 0, 0, true);

		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		tslp_tsk(1500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		ev3_motor_rotate(EV3_PORT_A, -90, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -90, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		act_tsk(LEFT_TASK);
		slp_tsk();

		//十字路まで進む
		power = fast;
		gain_rate = two_fast;
		ev3_sta_cyc(TRACE);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		//container[2]とコントローラーの入れ替え
		counter = 2;
		act_tsk(CHANGE_YB_TASK);
		slp_tsk();

		//方向転換
		act_tsk(RIGHT_TASK);
		slp_tsk();

		ev3_sta_cyc(TRACE);
		if(container[2] == 4) {
			//container[2]が青の場合、十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
		}

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

		ev3_stp_cyc(TRACE);

		//L字路まで進む
		gain_rate = one_fast;
		ev3_sta_cyc(TRACE2);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
		ev3_speaker_play_tone(1174.66, 200);

		//軸に乗せる L字路をこえる
		ev3_stp_cyc(TRACE2);
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);


		if(ship[container[2] - 1] <= 3 ){
			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			//丁字路まで進む
			power = fast;
			gain_rate = one_fast;
			ev3_sta_cyc(TRACE3);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
			ev3_speaker_play_tone(1174.66, 200);

			//丁字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
			ev3_stp_cyc(TRACE3);

			gain_rate = two_fast;
			ev3_sta_cyc(TRACE);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -462);

			ev3_stp_cyc(TRACE);

		  //L字路まで進む
			ev3_motor_set_power(EV3_PORT_A, fast);
			ev3_motor_set_power(EV3_PORT_D, fast);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//壁合わせ
			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			tslp_tsk(2000);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);
		}else{

		}
	}else if(container[2] == 1 || container[2] == 2) {
		//container[2]が赤、緑の場合
		if(ship[container[2] - 1] <= 3 ){
			//壁合わせ
			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			ev3_motor_rotate(EV3_PORT_A, -759, fast * -1, false);
			ev3_motor_rotate(EV3_PORT_D, -759, fast * -1, true);
			ev3_motor_rotate(EV3_PORT_A, 0, 0, true);

			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			tslp_tsk(1500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);
		}else{
			//壁合わせ
			ev3_motor_rotate(EV3_PORT_A, 442, fast * -1, false);
			ev3_motor_rotate(EV3_PORT_D, 442, fast * -1, true);
			ev3_motor_rotate(EV3_PORT_A, 0, 0, true);

			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			tslp_tsk(1500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し進む
			ev3_motor_rotate(EV3_PORT_A, -90, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, -90, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);
		}
	}

	//2回目のシップ作業
	//container[2]がなければ行わない
	if(container[2] != 0) {
		if(ship[container[2] - 1] <= 3 ){
			//シップが左側にある場合
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

			//container[2]のシップの作業
			counter = 2;
			act_tsk(SHIP_LEFT_TASK);
			slp_tsk();

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

		}else{
			//シップが右側にある場合
			//container[2]のシップの作業
			counter = 2;
			act_tsk(SHIP_RIGHT_TASK);
			slp_tsk();

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();
		}

		if (container[1] != 0) {
			//container[1]がなければ行わない
			//container[1]の回収
			//丁字路まで進む
			power = fast;
			gain_rate = two_fast;
			ev3_sta_cyc(TRACE);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);

			//軸に乗せる
			ev3_motor_rotate(EV3_PORT_A, -95, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, -95, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			//L字路まで進む
			gain_rate = one_fast;
			ev3_sta_cyc(TRACE3);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE3);

			//軸に乗せる
			ev3_motor_rotate(EV3_PORT_A, -95, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, -95, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();
		}

		}else{
			//container[2]がなければ
			//壁合わせ
			ev3_motor_rotate(EV3_PORT_A, 442, fast * -1, false);
			ev3_motor_rotate(EV3_PORT_D, 442, fast * -1, true);
			ev3_motor_rotate(EV3_PORT_A, 0, 0, true);

			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			tslp_tsk(1500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			ev3_motor_rotate(EV3_PORT_A, -90, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, -90, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();
		}

	if (container[1] != 0) {
		//container[1]がなければ行わない
		//十字路まで進む
		power = slow;
		gain_rate = two_slow;
		ev3_sta_cyc(TRACE);

		power = fast;
		gain_rate = two_fast;

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[1]を回収
		ev3_motor_rotate(EV3_PORT_A, 166, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 166, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		counter = 1;
		act_tsk(CATCH_TASK);
		slp_tsk();

		if(container[1] == 1 || container[1] == 2) {
			//container[1]が赤、緑の場合
			ev3_motor_set_power(EV3_PORT_A, fast);
			ev3_motor_set_power(EV3_PORT_D, fast);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_rotate(EV3_PORT_A, 20, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 20, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, -330, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			tslp_tsk(200);

			//十字路まで進む
			gain_rate = 1.3;
			ev3_sta_cyc(TRACE);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);


			//container[1]とコントローラーの入れ替え
			counter = 1;
			act_tsk(CHANGE_RG_TASK);
			slp_tsk();

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			ev3_sta_cyc(TRACE);
			if(container[1] == 2) {
				//container[1]が緑の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			ev3_stp_cyc(TRACE);

			//L字路まで進む
			gain_rate = one_fast;
			ev3_sta_cyc(TRACE3);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE3);

			//軸に乗せる L字路をこえる
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, -30);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

			if(ship[container[1] - 1] <= 3 ) {
				//シップが左側にある場合
				//container[1]のシップの作業
				counter = 1;
				act_tsk(SHIP_LEFT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(RIGHT_TASK);
				slp_tsk();

			}else{
				//シップが右側にある場合
				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				act_tsk(RIGHT_TASK);
				slp_tsk();

				//丁字路まで進む
				gain_rate = one_fast;
				ev3_sta_cyc(TRACE2);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
				ev3_speaker_play_tone(1174.66, 200);

				//丁字路をこえる
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
				ev3_stp_cyc(TRACE2);

				//L字路まで進む
				ev3_sta_cyc(TRACE3);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
				ev3_speaker_play_tone(1174.66, 200);
				ev3_stp_cyc(TRACE3);

				//L字路をこえる
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

				//壁合わせ
				ev3_motor_set_power(EV3_PORT_A, -20);
				ev3_motor_set_power(EV3_PORT_D, -20);
				tslp_tsk(1000);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//少し下がる
				ev3_motor_rotate(EV3_PORT_A, 90, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 90, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, -160, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 160, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//container[1]のシップの作業
				counter = 1;
				act_tsk(SHIP_RIGHT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(LEFT_TASK);
				slp_tsk();
			}

		}else{
			//container[1]が黄、青の場合
			ev3_motor_rotate(EV3_PORT_A, 200, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 200, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			//十字路まで進む
			gain_rate = two_fast;
			ev3_sta_cyc(TRACE);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);


			//container[1]とコントローラーの入れ替え
			counter = 1;
			act_tsk(CHANGE_YB_TASK);
			slp_tsk();

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			ev3_sta_cyc(TRACE);
			if(container[1] == 4) {
				//container[1]が青の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			ev3_stp_cyc(TRACE);

			//L字路まで進む
			gain_rate = one_fast;
			ev3_sta_cyc(TRACE2);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE2);

			//軸に乗せる L字路をこえる
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, -30);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

			if(ship[container[1] - 1] <= 3 ) {
				//シップが左側にある場合
				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				act_tsk(LEFT_TASK);
				slp_tsk();

				//丁字路まで進む
				gain_rate = one_fast;
				ev3_sta_cyc(TRACE3);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
				ev3_speaker_play_tone(1174.66, 200);

				//丁字路をこえる
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
				ev3_stp_cyc(TRACE3);

				//L字路まで進む
				ev3_sta_cyc(TRACE2);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
				ev3_speaker_play_tone(1174.66, 200);
				ev3_stp_cyc(TRACE2);

				//L字路をこえる
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

				//壁合わせ
				ev3_motor_set_power(EV3_PORT_A, -20);
				ev3_motor_set_power(EV3_PORT_D, -20);
				tslp_tsk(1000);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//少し下がる
				ev3_motor_rotate(EV3_PORT_A, 90, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 90, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//container[1]のシップの作業
				counter = 1;
				act_tsk(SHIP_LEFT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(RIGHT_TASK);
				slp_tsk();

			}else{
				//シップが右側にある場合
				//container[1]のシップの作業
				counter = 1;
				act_tsk(SHIP_RIGHT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(LEFT_TASK);
				slp_tsk();
			}
		}
	}


	if (container[0] == 0 || container[1] == 0 || container[2] ==  0){
		//container[3]を取りに行く
		//container[3]の色の特定
		for (i = 0; i <=3; i++) {
			if (ship[i] != 0) {
				break;
			}
		}

		container[3] = i + 1;

		//丁字路まで進む
		power = fast;
		gain_rate = two_fast;
		ev3_sta_cyc(TRACE);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//軸に乗せる
		ev3_motor_rotate(EV3_PORT_A, -95, slow * -1, false);
		ev3_motor_rotate(EV3_PORT_D, -95, slow * -1, true);

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

		//軸に乗せる
		ev3_motor_rotate(EV3_PORT_A, -95, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -95, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		act_tsk(LEFT_TASK);
		slp_tsk();

		//十字路まで進む
		gain_rate = two_fast;
		ev3_sta_cyc(TRACE);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
		ev3_speaker_play_tone(1174.66, 200);
		ev3_stp_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -352);


		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[3]を回収
		counter = 3;
		act_tsk(CATCH_TASK);
		slp_tsk();

		if(container[3] == 3 || container[3] == 4) {
			//container[3]が黄、青の場合
			ev3_motor_set_power(EV3_PORT_A, fast);
			ev3_motor_set_power(EV3_PORT_D, fast);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_rotate(EV3_PORT_A, 20, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 20, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_D, -330, 30, true);

			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//十字路まで進む
			gain_rate = two_fast;
			power = fast;
			ev3_sta_cyc(TRACE);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);


			//container[3]とコントローラーの入れ替え
			counter = 3;
			act_tsk(CHANGE_YB_TASK);
			slp_tsk();

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			ev3_sta_cyc(TRACE);
			if(container[3] == 4) {
				//container[3]が青の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			ev3_stp_cyc(TRACE);

			//L字路まで進む
			gain_rate = one_fast;
			ev3_sta_cyc(TRACE2);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE2);

			//軸に乗せる L字路をこえる
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, -30);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

			if(ship[container[3] - 1] <= 3 ) {
				//シップが左側にある場合
				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				act_tsk(LEFT_TASK);
				slp_tsk();

				//丁字路まで進む
				gain_rate = one_fast;
				ev3_sta_cyc(TRACE3);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
				ev3_speaker_play_tone(1174.66, 200);

				//丁字路をこえる
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
				ev3_stp_cyc(TRACE3);

				//L字路まで進む
				ev3_sta_cyc(TRACE2);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
				ev3_speaker_play_tone(1174.66, 200);
				ev3_stp_cyc(TRACE2);

				//L字路をこえる
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

				//壁合わせ
				ev3_motor_set_power(EV3_PORT_A, -20);
				ev3_motor_set_power(EV3_PORT_D, -20);
				tslp_tsk(1000);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//少し下がる
				ev3_motor_rotate(EV3_PORT_A, 90, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 90, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, 160, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -160, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//container[3]のシップの作業
				counter = 3;
				act_tsk(SHIP_LEFT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(RIGHT_TASK);
				slp_tsk();

			}else{
				//シップが右側にある場合
				//container[3]のシップの作業
				counter = 3;
				act_tsk(SHIP_RIGHT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(LEFT_TASK);
				slp_tsk();
			}

		}else{
			//container[3]が 赤、黄の場合
			ev3_motor_rotate(EV3_PORT_A, 200, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 200, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			//十字路まで進む
			gain_rate = two_fast;
			power = fast;
			ev3_sta_cyc(TRACE);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//container[3]とコントローラーの入れ替え
			counter = 3;
			act_tsk(CHANGE_RG_TASK);
			slp_tsk();

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			ev3_sta_cyc(TRACE);
			if(container[3] == 2) {
				//container[3]が緑の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
			ev3_speaker_play_tone(1174.66, 200);

			//十字路をこえる
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

			ev3_stp_cyc(TRACE);

			//L字路まで進む
			gain_rate = one_fast;
			ev3_sta_cyc(TRACE3);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE3);

			//軸に乗せる L字路をこえる
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, -30);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

			if(ship[container[3] - 1] <= 3 ) {
				//シップが左側にある場合
				//container[3]のシップの作業
				counter = 3;
				act_tsk(SHIP_LEFT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(RIGHT_TASK);
				slp_tsk();

			}else{
				//シップが右側にある場合
				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				act_tsk(RIGHT_TASK);
				slp_tsk();

				//丁字路まで進む
				gain_rate = two_fast;
				ev3_sta_cyc(TRACE2);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
				ev3_speaker_play_tone(1174.66, 200);

				//丁字路をこえる
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);
				ev3_stp_cyc(TRACE2);

				//L字路まで進む
				ev3_sta_cyc(TRACE3);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=70);
				ev3_speaker_play_tone(1174.66, 200);
				ev3_stp_cyc(TRACE3);

				//L字路をこえる
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -95);

				//壁合わせ
				ev3_motor_set_power(EV3_PORT_A, -20);
				ev3_motor_set_power(EV3_PORT_D, -20);
				tslp_tsk(1000);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//少し下がる
				ev3_motor_rotate(EV3_PORT_A, 90, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 90, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, -160, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 160, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//container[3]のシップの作業
				counter = 3;
				act_tsk(SHIP_RIGHT_TASK);
				slp_tsk();

				//方向転換
				act_tsk(LEFT_TASK);
				slp_tsk();
			}
		}
	}else{
		//container[3]はない
	}

	//かえる
	//丁字路まで進む
	power = fast;
	gain_rate = two_fast;
	ev3_sta_cyc(TRACE);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE);

	//軸に乗せる
	ev3_motor_rotate(EV3_PORT_A, -95, slow * -1, false);
	ev3_motor_rotate(EV3_PORT_D, -95, slow * -1, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	act_tsk(RIGHT_TASK);
	slp_tsk();

	//L字路まで進む
	gain_rate = 2.0;
	ev3_sta_cyc(TRACE2);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_2) >=70);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE2);

	//軸に乗せる
	ev3_motor_rotate(EV3_PORT_A, -95, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -95, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	act_tsk(LEFT_TASK);
	slp_tsk();

	//十字路まで進む
	gain_rate = two_fast;
	ev3_sta_cyc(TRACE);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	//十字路をこえる
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

	//十字路まで進む
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >=20 || ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
	ev3_speaker_play_tone(1174.66, 200);

	//十字路をこえる
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -100);

	//十字路まで進む
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
	act_tsk(LEFT_TASK);
	slp_tsk();

	//ゴールエリアまで進む
	gain_rate = two_fast;
	ev3_sta_cyc(TRACE);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -276);

	ev3_stp_cyc(TRACE);

	//ゴールエリアにinする
	ev3_motor_rotate(EV3_PORT_A, -330, slow * -1, false);
	ev3_motor_rotate(EV3_PORT_D, -330, slow * -1, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
}
