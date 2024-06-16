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
int integral, p, i, d, sensor2, sensor3, motor, count, power, sensor1, pid;

//角度の変数(使い捨て)
int angle;

//コンテナの色の変数
int container[4];

//船の位置の変数
int ship[4];

//船の色の変数
int ship2[3];

//コンテナとコントローラのカウント、コンテナ色読みのカウント、シップのカウント、コンテナのカウント
int counter = 0;

//コンテナ色読みのカウント
int counter2 = 0;

//shipのカウント
int counter3 = 0;

//台形駆動のカウント
int counter4 = 0;

//HTのよみこみの変数
uint8_t *pHT = 0;
uint8_t ht = 0;

//速度の変数
int fast = -80;
int slow = -30;

//gainの変数
int two_fast = 2.14;
int two_slow = 2.14;
int one_fast = 3.9;
int one_slow = 2.3;

//カウンター
int counter_a = 0;
int counter_b = 0;
int counter_c = 0;
int counter_d = 0;

//存在しないコンテナ
int no_container = -1;

//ship2のカウンター
int j = 0;

//scanのポートのカウンター
int port = 0;

//角度調節の比例定数
double a = 1.00;

//方向転換の変数
int turn = 145;

//センサーの閾値
int black = 25;
int white = 70;
int threshold = 45; //trace22とtrace33は別規定

//containerの青と緑を読み間違わないようにする変数
int separate = 0;


void trace_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
	sensor3 = ev3_color_sensor_get_reflect(EV3_PORT_3);

	diff[1] = sensor2 - sensor3;

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;

	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;

	pid = p + i + d;

	if(pid >= 0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}
}


void trace2_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);

	diff[1] = sensor2 - threshold;

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;

	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;

	pid = p + i + d;

	if(pid >= 0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}
}


void trace3_task(intptr_t unused) {
	kp = 0.36 * gain_rate;
	ki = kp / 0.3;
	kd = kp * 0.075;

	diff[0] = diff[1];
	sensor3 = ev3_color_sensor_get_reflect(EV3_PORT_3);

	diff[1] = sensor3 - threshold;

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;

	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;

	pid = p + i + d;

	if(pid >=0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		motor = power - pid;
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

	diff[1] = sensor2 - 25;

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;

	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;

	pid = p + i + d;

	if(pid >= 0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		motor = power - pid;
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

	diff[1] = sensor3 - 35;

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;

	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;

	pid = p + i + d;

	if(pid >= 0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power);
	}
}


void right_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30 * a);
		ev3_motor_set_power(EV3_PORT_D, -30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 50 * a);

	do{
		ev3_motor_set_power(EV3_PORT_A, 40 * a);
		ev3_motor_set_power(EV3_PORT_D, -40 * a);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 50);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30 * a);
		ev3_motor_set_power(EV3_PORT_D, -30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 30 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void left_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30 * a);
		ev3_motor_set_power(EV3_PORT_D, 30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 50 * a);

	do{
		ev3_motor_set_power(EV3_PORT_A, -40 * a);
		ev3_motor_set_power(EV3_PORT_D, 40 * a);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= 50);

	ev3_motor_reset_counts(EV3_PORT_D);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30 * a);
		ev3_motor_set_power(EV3_PORT_D, 30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 30 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void rotation_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30 * a);
		ev3_motor_set_power(EV3_PORT_D, -30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);

	do{
		ev3_motor_set_power(EV3_PORT_A, 40 * a);
		ev3_motor_set_power(EV3_PORT_D, -40 * a);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 50);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30 * a);
		ev3_motor_set_power(EV3_PORT_D, -30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 30 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void change_yb_task(intptr_t unused) {
	//コントローラーに対する位置の調節
	if(container[counter] == 4){
		gain_rate = two_slow;
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -385 * a);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 15 * a);
	}

	//コンテナとコントローラーの入れ替え
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, 40);
	ev3_motor_set_power(EV3_PORT_D, 0);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 40 * a);

	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(500);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_B, -10);
	ev3_motor_rotate(EV3_PORT_A, -170 * a, 20, false);
	ev3_motor_rotate(EV3_PORT_D, -170 * a, 20, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_B, -100);

	ev3_motor_rotate(EV3_PORT_A, 55 * a, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 55 * a, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);

	tslp_tsk(500);

	ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_rotate(EV3_PORT_B, 120, 30, true);

	gain_rate = one_fast;
	power = fast;

	wup_tsk(MAIN_TASK);
}


void change_rg_task(intptr_t unused) {
	//コントローラーに対する位置の調節
	if(container[counter] == 2){
		gain_rate = two_slow;
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -385 * a);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D)) / 2  <= 15 * a);
	}

	//コンテナとコントローラーの入れ替え
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, 0);
	ev3_motor_set_power(EV3_PORT_D, 40);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 180 * a);

	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, -30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) <= 40 * a);

	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(500);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_B, -10);
	ev3_motor_rotate(EV3_PORT_A, -170 * a, 20, false);
	ev3_motor_rotate(EV3_PORT_D, -170 * a, 20, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_set_power(EV3_PORT_B, -100);

	ev3_motor_rotate(EV3_PORT_A, 55 * a, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 55 * a, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);

	tslp_tsk(500);

	ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_rotate(EV3_PORT_B, 120, 30, true);

	gain_rate = one_fast;
	power = fast;

	wup_tsk(MAIN_TASK);
}


void scan_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	if(port == 1){
		ht_nxt_color_sensor_measure_color(EV3_PORT_1, pHT);
		*pHT = 0;
		ev3_sta_cyc(HT1);
		do{
			if((*pHT == 1 || *pHT == 7 || *pHT == 8 || *pHT == 9 || *pHT == 10) && container[counter] == 0) {
				container[counter] = 1;
				ev3_speaker_play_tone(1975.53, 200);
			}else if((*pHT == 4 || *pHT == 13) && container[counter] == 0) {
				container[counter] = 2;
				ev3_speaker_play_tone(1396.91, 200);
			}else if((*pHT == 5 || *pHT == 6) && container[counter] == 0) {
				container[counter] = 3;
				ev3_speaker_play_tone(880.00, 200);
			}else if((*pHT == 2 /*|| *pHT == 3*/) && container[counter] == 0) {
				container[counter] = 4;
				ev3_speaker_play_tone(261.63, 200);
			}else if(*pHT == 3 && container[counter] == 0) {
				separate = 1;
			}else{

			}
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -93 * a);
		ev3_stp_cyc(HT1);

		if(separate == 1 && container[counter] == 0){
			container[counter] = 4;
			ev3_speaker_play_tone(261.63, 200);
		}

		separate = 0;

	}else{
		ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
		*pHT = 0;
		ev3_sta_cyc(HT4);
		do{
			if((*pHT == 1 || *pHT == 7 || *pHT == 8 || *pHT == 9 || *pHT == 10) && container[counter] == 0) {
				container[counter] = 1;
				ev3_speaker_play_tone(1975.53, 200);
			}else if((*pHT == 4 || *pHT == 13) && container[counter] == 0) {
				container[counter] = 2;
				ev3_speaker_play_tone(1396.91, 200);
			}else if((*pHT == 5 || *pHT == 6) && container[counter] == 0) {
				container[counter] = 3;
				ev3_speaker_play_tone(880.00, 200);
			}else if((*pHT == 2 /*|| *pHT == 3*/) && container[counter] == 0) {
				container[counter] = 4;
				ev3_speaker_play_tone(261.63, 200);
			}else if(*pHT == 3 && container[counter] == 0) {
				separate = 1;
			}else{

			}
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -93 * a);
		ev3_stp_cyc(HT4);

		if(separate == 1 && container[counter] == 0){
			container[counter] = 4;
			ev3_speaker_play_tone(261.63, 200);
		}
	}

	separate = 0;

	wup_tsk(MAIN_TASK);
}


void ship_left_task(intptr_t unused) {
	//ラインまで進む
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	//軸に乗せる
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -35 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//壁合わせ
	ev3_motor_set_power(EV3_PORT_A, 20);
	ev3_motor_set_power(EV3_PORT_D, 20);
	tslp_tsk(1000);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	counter_d = ship[container[counter] - 1];

	angle = 45 + (ship[container[counter] - 1] - 1) * 249 * a;

	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) >= -angle);

	ev3_stp_cyc(TRACE33);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//フードとコントローラーをシップに乗せる
	ev3_motor_set_power(EV3_PORT_C, -10);

	//シップを出航させる
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(2000);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_stop(EV3_PORT_C, true);

	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -165 * a);

	//アームを元に戻す
	ev3_motor_set_power(EV3_PORT_C, 80);

	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -200 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_motor_stop(EV3_PORT_C, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
	ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//シップを出航させた、完了の0
	ship[container[counter] - 1] = 0;
	container[counter] = 0;

	counter_b = 1;

	wup_tsk(MAIN_TASK);
}


void ship_right_task(intptr_t unused) {
	//ラインまで進む
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	//軸に乗せる
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -35 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
	ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//壁合わせ
	ev3_motor_set_power(EV3_PORT_A, 20);
	ev3_motor_set_power(EV3_PORT_D, 20);
	tslp_tsk(1000);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	counter_d = ship[container[counter] - 1];

	angle = 45 + (6 - ship[container[counter] - 1]) * 244 * a;

	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) >= -angle);

	ev3_stp_cyc(TRACE22);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
	ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//フードとコントローラーをシップに乗せる
	ev3_motor_set_power(EV3_PORT_C, -10);

	//シップを出航させる
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, 30);
	tslp_tsk(2000);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_stop(EV3_PORT_C, true);


	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -165 * a);

	//アームを元に戻す
	ev3_motor_set_power(EV3_PORT_C, 80);

	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -200 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_motor_stop(EV3_PORT_C, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
	ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//シップを出航させた、完了の0
	ship[container[counter] - 1] = 0;
	container[counter] = 0;

	counter_b = 2;

	wup_tsk(MAIN_TASK);
}


void catch_task(intptr_t unused) {
	if(counter == 1) {
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 0);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 185 * a);


		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 55 * a);


		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		tslp_tsk(500);

	}else if(counter == 0) {
		ev3_motor_stop(EV3_PORT_A, true);

		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 0);
		ev3_motor_set_power(EV3_PORT_D, 40);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) <= 250 * a);

		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		tslp_tsk(500);

	}else if (counter == 2) {
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 0);
		ev3_motor_set_power(EV3_PORT_D, 30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) <= 185 * a);

		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) <= 45 * a);

		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		tslp_tsk(500);

	}else{
		ev3_motor_rotate(EV3_PORT_A, -turn * a, 30, false);
		ev3_motor_rotate(EV3_PORT_D, turn * a, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		tslp_tsk(800);
	}

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	if(counter == 0 || counter == 2) {
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, -30);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -150 * a);

		ev3_motor_rotate(EV3_PORT_A, -50 * a, 10, false);
		ev3_motor_rotate(EV3_PORT_D, -50 * a, 10, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, -30);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -250 * a);

		ev3_motor_rotate(EV3_PORT_A, -50 * a, 10, false);
		ev3_motor_rotate(EV3_PORT_D, -50 * a, 10, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}

	ev3_motor_set_power(EV3_PORT_B, 80);
	tslp_tsk(800);

	ev3_motor_rotate(EV3_PORT_B, -290, 80, true);

	wup_tsk(MAIN_TASK);
}


void ht1_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_color(EV3_PORT_1, pHT);
}


void ht4_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
}


void main_task(intptr_t unused) {
	ev3_lcd_set_font (EV3_FONT_MEDIUM);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);	//右前カラーセンサー
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);	//左前カラーセンサー
	ev3_sensor_config(EV3_PORT_1, HT_NXT_COLOR_SENSOR);	//右後HTセンサー
	ev3_sensor_config(EV3_PORT_4, HT_NXT_COLOR_SENSOR);	//左後HTセンサー
	ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);	//右タイヤ
	ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);	//左タイヤ
	ev3_motor_config(EV3_PORT_B, MEDIUM_MOTOR);	//アーム
	ev3_motor_config(EV3_PORT_C, MEDIUM_MOTOR);	//下ろすやつ

	ev3_speaker_play_tone(1174.66, 200);

	do{
	}while(ev3_button_is_pressed(ENTER_BUTTON) == false);

	tslp_tsk(500);

	ev3_color_sensor_get_reflect(EV3_PORT_2);
	ev3_color_sensor_get_reflect(EV3_PORT_3);

	//HT色読み さす
	pHT = &ht;

	//ループ処理の変数
	int i;
	int k;

	//丁字路まで進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -150 * a);

	power = fast;
	gain_rate = one_fast;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_sta_cyc(TRACE2);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -220 * a);

	power = slow;
	gain_rate = one_slow;
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black || ev3_color_sensor_get_reflect(EV3_PORT_2) >= white);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE2);

	//方向転換
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, -30);
	ev3_motor_set_power(EV3_PORT_D, 0);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -25 * a);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, -30);
	ev3_motor_set_power(EV3_PORT_D, 10);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -150 * a);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, -30);
	ev3_motor_set_power(EV3_PORT_D, 10);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -40 * a);


	//container[0]まで進む
	power = fast;
	gain_rate = one_fast;
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_sta_cyc(TRACE22);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -500 * a);

	ev3_stp_cyc(TRACE22);

	ev3_motor_set_power(EV3_PORT_A, fast);
	ev3_motor_set_power(EV3_PORT_D, fast);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -48 * a);

	ev3_sta_cyc(TRACE22);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -484 * a);

	power = slow;
	gain_rate = one_slow;
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -200 * a);

	//container[0]をよみこむ
	counter = 0;
	port = 4;
	act_tsk(SCAN_TASK);
	slp_tsk();

	ev3_stp_cyc(TRACE22);

	//十字路まで進む
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	if(container[0] != 0) {
		//container[0]があった場合
		counter_a = 0;

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[0]を回収
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 160 * a);

		counter = 0;
		act_tsk(CATCH_TASK);
		slp_tsk();

		if(container[0] == 3 || container[0] == 4) {
			ev3_motor_rotate(EV3_PORT_A, 85 * a, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 85 * a, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//container[0]が黄、青だった場合
			//コンテナとコントローラーの入れ替え

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			//Foodの回収
			ev3_motor_reset_counts(EV3_PORT_B);
			ev3_motor_set_power(EV3_PORT_B, -80);

			//十字路まで進む
			gain_rate = two_fast;
			power = fast;
			ev3_sta_cyc(TRACE);

			do{
			}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

			ev3_motor_stop(EV3_PORT_B, true);
			tslp_tsk(200);

			ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

			//container[0]が黄ならゆっくり
			if(container[0] == 3){
				gain_rate = two_slow;
				power = slow;
			}

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//container[0]とコントローラーの入れ替え
			counter = 0;
			act_tsk(CHANGE_YB_TASK);
			slp_tsk();

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			power = fast;
			gain_rate = two_fast;
			ev3_sta_cyc(TRACE);

			if(container[0] == 4) {
				//container[0]が青の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -500 * a);
			}
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -300 * a);

			power = slow;
			gain_rate = two_slow;

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -80 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

			//container[3]へ
			power = slow;
			gain_rate = one_slow;
			ev3_sta_cyc(TRACE22);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE22);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -138 * a);

			ev3_sta_cyc(TRACE22);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -280 * a);

			//container[3]のscan
			counter = 3;
			port = 4;
			act_tsk(SCAN_TASK);
			slp_tsk();

			ev3_stp_cyc(TRACE22);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -80 * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);


			power = slow;
			gain_rate = one_slow;
			ev3_sta_cyc(TRACE33);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -380 * a);

			ev3_stp_cyc(TRACE33);

		}else{
			//container[0]が赤、黄だった場合
			//container[3]のscan
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -15);
			ev3_motor_set_power(EV3_PORT_D, -65);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -200 * a);

			ev3_sta_cyc(TRACE3);
			power = slow;
			gain_rate = one_slow;

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= white || ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
			ev3_speaker_play_tone(1174.66, 200);
			ev3_stp_cyc(TRACE3);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -155 * a);

			//container[3]へ
			power = slow;
			gain_rate = one_slow;

			ev3_sta_cyc(TRACE22);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -305 * a);

			//container[3]のscan
			counter = 3;
			port = 4;
			act_tsk(SCAN_TASK);
			slp_tsk();

			ev3_stp_cyc(TRACE22);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 10);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 10);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

			//Foodの回収
			ev3_motor_reset_counts(EV3_PORT_B);
			ev3_motor_set_power(EV3_PORT_B, -80);

			gain_rate = two_fast;
			power = fast;

			ev3_sta_cyc(TRACE);

			do{
			}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

			ev3_motor_stop(EV3_PORT_B, true);
			tslp_tsk(200);

			ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

			//container[0]が赤ならゆっくり
			if (container[0] == 1){
				gain_rate = two_slow;
				power = slow;
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);


			//container[0]とコントローラーの入れ替え
			counter = 0;
			act_tsk(CHANGE_RG_TASK);
			slp_tsk();

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			power = fast;
			gain_rate = two_fast;
			ev3_sta_cyc(TRACE);

			if(container[0] == 2) {
				//container[0]が緑の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -90 * a);

			power = slow;
			gain_rate = one_slow;
			ev3_sta_cyc(TRACE33);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -400 * a);

			ev3_stp_cyc(TRACE33);

		}
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);

		//container[2]のscan
		counter = 2;
		port = 1;
		act_tsk(SCAN_TASK);
		slp_tsk();

		//no_container の特定
		for (i = 0; i <=3; i++) {
			if (container[i] < 0) {
				no_container = i;
			}else{

			}
		}

		if (no_container < 0) {
			no_container = 1;
		}

		//shipのscan container[0]のship作業
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -60 * a);

		//ラインまで進む
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//軸に乗せる
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -80 * a);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		do{
			ev3_motor_set_power(EV3_PORT_A, 30 * a);
			ev3_motor_set_power(EV3_PORT_D, -30 * a);
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) <= 30);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//壁合わせ
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		tslp_tsk(1000);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}else{
		//container[0]がなかった場合
		counter_a = 3;

		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 0);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -90 * a);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

		//container[3]へ
		power = slow;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE22);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= 20);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -138 * a);

		ev3_sta_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -280 * a);

		//container[3]のscan
		counter = 3;
		port = 4;
		act_tsk(SCAN_TASK);
		slp_tsk();

		ev3_stp_cyc(TRACE22);

		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 0);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -150 * a);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 10);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 10);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

		//container[3]のcatch
		power = slow;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE);
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -190 * a);
		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		act_tsk(CATCH_TASK);
		slp_tsk();

		//コンテナとコントローラの入れ替え
		if(container[3] == 3 || container[3] == 4) {
			//container[3]が黄、青だった場合
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, fast);
			ev3_motor_set_power(EV3_PORT_D, fast);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -690 * a);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -100 * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 10);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -100 * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 10);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);

			//Foodの回収
			ev3_motor_reset_counts(EV3_PORT_B);
			ev3_motor_set_power(EV3_PORT_B, -80);

			//十字路まで進む
			gain_rate = two_fast;
			power = fast;

			//黄ならゆっくり
			if(container[3] == 3){
				gain_rate = two_slow;
				power = slow;
			}
			ev3_sta_cyc(TRACE);

			do{
			}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

			ev3_motor_stop(EV3_PORT_B, true);
			tslp_tsk(200);

			ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			counter = 3;
			act_tsk(CHANGE_YB_TASK);
			slp_tsk();

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			power = fast;
			gain_rate = two_fast;
			ev3_sta_cyc(TRACE);
			if(container[3] == 4) {
				//container[3]が青の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) > black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -300 * a);
			}
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -500 * a);

			power = slow;
			gain_rate = two_slow;

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);
			//左に行きcontainer[2]をscan
			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 10);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 10);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);


			power = fast;
			gain_rate = two_fast;
			ev3_sta_cyc(TRACE);
			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -400 * a);

			power = slow;
			gain_rate = two_slow;
			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);
			ev3_stp_cyc(TRACE);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >=  black * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -80 * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);

			power = slow;
			gain_rate = one_slow;
			ev3_sta_cyc(TRACE33);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -350 * a);

			ev3_stp_cyc(TRACE33);

	  }else{
			//contaienr[3]が赤、緑の場合
			ev3_motor_rotate(EV3_PORT_A, 200 * a, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 200 * a, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			//Foodの回収
			ev3_motor_reset_counts(EV3_PORT_B);
			ev3_motor_set_power(EV3_PORT_B, -80);

			//十字路まで進む
			gain_rate = two_fast;
			power = fast;
			ev3_sta_cyc(TRACE);

			do{
			}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

			//container[3]が赤ならゆっくり
			if(container[3] == 1){
				gain_rate = two_slow;
				power = slow;
			}

			ev3_motor_stop(EV3_PORT_B, true);
			tslp_tsk(200);

			ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
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
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -10);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -90 * a);

			power = slow;
			gain_rate = one_slow;
			ev3_sta_cyc(TRACE33);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -400 * a);

			ev3_stp_cyc(TRACE33);


		}
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);

			//container[2]のscan
			counter = 2;
			port = 1;
			act_tsk(SCAN_TASK);
			slp_tsk();

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -60 * a);

			//shipのscan container[3]のship作業
			//ラインまで進む
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//軸に乗せる
			ev3_motor_rotate(EV3_PORT_A, -80 * a, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, -80 * a, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			do{
				ev3_motor_set_power(EV3_PORT_A, 30 * a);
				ev3_motor_set_power(EV3_PORT_D, -30 * a);
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) <= 30);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//壁合わせ
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			tslp_tsk(1000);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

	}
	//shipのscan

	//シップまで進む
	gain_rate = one_slow;
	power = slow;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_sta_cyc(TRACE2);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -85 * a);


	for (i = 1; i <= 5; i++) {
		//シップをよみこむ
		ev3_motor_reset_counts(EV3_PORT_A);
		ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
		*pHT = 0;
		ev3_sta_cyc(HT4);
		do{
			if((*pHT == 7 || *pHT == 8 || *pHT == 9) && ship[0] == 0) {
				ev3_speaker_play_tone(1975.53, 200);
				ship[0] = i;
				ship2[j] = 1;
				j += 1;
			}else if((*pHT == 4 || *pHT == 13) && ship[1] == 0) {
				ev3_speaker_play_tone(1396.91, 200);
				ship[1] = i;
				ship2[j] = 2;
				j += 1;
			}else if((*pHT == 5 || *pHT == 6) && ship[2] == 0) {
				ev3_speaker_play_tone(880.00, 200);
				ship[2] = i;
				ship2[j] = 3;
				j += 1;
			}else if((*pHT == 2 || *pHT == 3) && ship[3] == 0) {
				ev3_speaker_play_tone(261.63, 200);
				ship[3] = i;
				ship2[j] = 4;
				j += 1;
			}else{

			}
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);
		ev3_stp_cyc(HT4);

		if (ship[container[counter_a] - 1] == i) {
			//よみこんだシップとcontainerが等しい場合、コントローラーとフードをシップに乗せ、出航させる
			//位置調節
			power = -20;
			do{
			}while(ev3_color_sensor_get_color(EV3_PORT_3) != 1);
			ev3_stp_cyc(TRACE2);
			power = slow;

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			ev3_motor_rotate(EV3_PORT_A, 50 * a, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 50 * a, 30, true);

			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, 300 * a, 30, true);

			//フードとコントローラーをシップに乗せる
			ev3_motor_set_power(EV3_PORT_C, -10);

			//シップを出航させる
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			tslp_tsk(1500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(500);

			ev3_motor_stop(EV3_PORT_C, true);

			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -155 * a);

			//アームを元に戻す
			ev3_motor_set_power(EV3_PORT_C, 80);

			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -135 * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			ev3_motor_stop(EV3_PORT_C, true);
			tslp_tsk(200);

			//方向転換と進む
			if (i == 3){
				ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
				ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -70 * a);

			}else{
				do{
					ev3_motor_set_power(EV3_PORT_A, -30 * a);
					ev3_motor_set_power(EV3_PORT_D, 30 * a);
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= 50);

				ev3_motor_reset_counts(EV3_PORT_D);
				do{
					ev3_motor_set_power(EV3_PORT_A, -30 * a);
					ev3_motor_set_power(EV3_PORT_D, 30 * a);
				}while(ev3_motor_get_counts(EV3_PORT_D) <= 30 * a);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);
			}

			ev3_sta_cyc(TRACE2);

		}

		//次のシップまで進む
		if (i == 2) {
			diff[0] = 0;
			diff[1] = 0;
			integral = 0;
			sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
			do{
			}while(ev3_color_sensor_get_color(EV3_PORT_3) != 1);
			ev3_stp_cyc(TRACE2);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_color(EV3_PORT_2) != 1);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -30 * a);

		}else{
			diff[0] = 0;
			diff[1] = 0;
			integral = 0;
			sensor2 = ev3_color_sensor_get_reflect(EV3_PORT_2);
			do{
			}while(ev3_color_sensor_get_color(EV3_PORT_3) != 1);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -150 * a);
			ev3_stp_cyc(TRACE2);
		}

		ev3_sta_cyc(TRACE2);

	}

	//6つ目のシップをよみこむ
	ev3_motor_reset_counts(EV3_PORT_A);
	ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
	*pHT = 0;
	ev3_sta_cyc(HT4);
	do{
		if((*pHT == 7 || *pHT == 8 || *pHT == 9) && ship[0] == 0) {
			ev3_speaker_play_tone(1975.53, 200);
			ship[0] = 6;
			ship2[j] = 1;
		}else if((*pHT == 4 || *pHT == 13) && ship[1] == 0) {
			ev3_speaker_play_tone(1396.91, 200);
			ship[1] = 6;
			ship2[j] = 2;
		}else if((*pHT == 5 || *pHT == 6) && ship[2] == 0) {
			ev3_speaker_play_tone(880.00, 200);
			ship[2] = 6;
			ship2[j] = 3;
		}else if((*pHT == 2 || *pHT == 3) && ship[3] == 0) {
			ev3_speaker_play_tone(261.63, 200);
			ship[3] = 6;
			ship2[j] = 4;
		}else{

		}
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -80 * a);
	ev3_stp_cyc(HT4);
	ev3_stp_cyc(TRACE2);

	if (ship[container[counter_a] - 1] == 6) {
		//6つ目のシップとcontainerが等しい場合、コントローラーとフードをシップに乗せ、出航させる
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//下がる
		ev3_motor_rotate(EV3_PORT_A, 140 * a, 30, false);
		ev3_motor_rotate(EV3_PORT_D, 140 * a, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_rotate(EV3_PORT_A, 75 * a, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -75 * a, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//進む
		ev3_motor_rotate(EV3_PORT_A, -135 * a, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -135 * a, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_rotate(EV3_PORT_A, 65 * a, 30, false);
		ev3_motor_rotate(EV3_PORT_D, -65 * a, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//フードとコントローラーをシップに乗せる
		ev3_motor_set_power(EV3_PORT_C, -10);

		//シップを出航させる
		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		tslp_tsk(700);

		//フードとコントローラーをシップに乗せる
		ev3_motor_set_power(EV3_PORT_C, -10);

		ev3_motor_set_power(EV3_PORT_A, 30);
		ev3_motor_set_power(EV3_PORT_D, 30);
		tslp_tsk(800);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(500);

		ev3_motor_stop(EV3_PORT_C, true);

		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -165 * a);

		//アームを元に戻す
		ev3_motor_set_power(EV3_PORT_C, 80);

		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -170 * a);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
		ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//進む
		ev3_motor_set_power(EV3_PORT_A, -20);
		ev3_motor_set_power(EV3_PORT_D, -20);
		tslp_tsk(1000);
	}

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//contianer[1]の特定
	container[1] = (ship2[0] + ship2[1] + ship2[2]) - (container[0] + container[2] + container[3]);


	//シップを出航させた、完了の0
	ship[container[counter_a] - 1] = 0;
	container[counter_a] = 0;


	for (k = 0; k < 2 ; k++) {
		//どのcontaienrをとるかの特定 and 壁合わせ and 方向転換
		if (k == 0) {
			//1回目
			if (container[1] == 0) {
				counter_c = 2;
				//壁合わせ and 方向転換　(左へ)
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow * -1);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 50 * a);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, 0);
				ev3_motor_set_power(EV3_PORT_D, slow * -1);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, fast * -1);
				ev3_motor_set_power(EV3_PORT_D, fast * -1);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 1150 * a);

				ev3_motor_set_power(EV3_PORT_A, 20);
				ev3_motor_set_power(EV3_PORT_D, 20);
				tslp_tsk(1000);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//少し下がる
				ev3_motor_rotate(EV3_PORT_A, -83 * a, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -83 * a, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

	  	}else{
		  	counter_c = 1;
				//壁合わせ and 方向転換　そのままかな
				if (ship[0] == 6 || ship[1] == 6 || ship[2] == 6 || ship[3] == 6){
				//右端のshipをよける方向転換
				//下がる
				ev3_motor_rotate(EV3_PORT_A, 150 * a, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 150 * a, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, 65 * a, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -65 * a, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//進む
				ev3_motor_rotate(EV3_PORT_A, -70 * a, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -70 * a, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, 80 * a, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -80 * a, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);
			}else{
				//右端のshipをよけない方向転換
				//少し下がる
				ev3_motor_rotate(EV3_PORT_A, 95 * a, 30, false);
				ev3_motor_rotate(EV3_PORT_D, 95 * a, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
				ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

			}

			}

  	}else{
			//2回目
			for (i = 0; i < 4; i++) {
				if (container[i] > 0) {
					counter_c = i;
				}
			}

			if (counter_c == 1) {
				//壁合わせ and 方向転換 (右側の壁へ)
				if (counter_b == 1) {
					ev3_motor_reset_counts(EV3_PORT_A);
					if(1250 * a - angle >= 100 * a) {
						ev3_motor_set_power(EV3_PORT_A, fast);
						ev3_motor_set_power(EV3_PORT_D, fast);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle - 100 * a) * -1);
					}
					ev3_motor_set_power(EV3_PORT_A, slow);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle) * -1);

					ev3_motor_set_power(EV3_PORT_A, -20);
					ev3_motor_set_power(EV3_PORT_D, -20);
					tslp_tsk(1000);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//少し下がる
					ev3_motor_rotate(EV3_PORT_A, 95 * a, 30, false);
					ev3_motor_rotate(EV3_PORT_D, 95 * a, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
					ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

				}else{
					ev3_motor_reset_counts(EV3_PORT_A);
					if(angle >= 100 * a) {
						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, fast * -1);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= (angle - 100 * a));
					}
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= angle);

					ev3_motor_set_power(EV3_PORT_A, 20);
					ev3_motor_set_power(EV3_PORT_D, 20);
					tslp_tsk(1000);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//少し下がる
					ev3_motor_rotate(EV3_PORT_A, -83 * a, 30, false);
					ev3_motor_rotate(EV3_PORT_D, -83 * a, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
					ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);


				}

			}else{
				//壁合わせ and 方向転換 (左側の壁へ)
				if (counter_b == 1) {
					ev3_motor_reset_counts(EV3_PORT_A);
					if(angle >= 100 * a) {
						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, fast * -1);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= (angle - 100 * a));
					}
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= angle);

					ev3_motor_set_power(EV3_PORT_A, 20);
					ev3_motor_set_power(EV3_PORT_D, 20);
					tslp_tsk(1000);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//少し下がる
					ev3_motor_rotate(EV3_PORT_A, -83 * a, 30, false);
					ev3_motor_rotate(EV3_PORT_D, -83 * a, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
					ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

				}else{
					ev3_motor_reset_counts(EV3_PORT_A);
					if(1250 * a - angle >= 100 * a) {
						ev3_motor_set_power(EV3_PORT_A, fast);
						ev3_motor_set_power(EV3_PORT_D, fast);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle - 100 * a) * -1);
					}
					ev3_motor_set_power(EV3_PORT_A, slow);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle) * -1);

					ev3_motor_set_power(EV3_PORT_A, -20);
					ev3_motor_set_power(EV3_PORT_D, -20);
					tslp_tsk(1000);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//少し下がる
					ev3_motor_rotate(EV3_PORT_A, 95 * a, 30, false);
					ev3_motor_rotate(EV3_PORT_D, 95 * a, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
					ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);


				}

			}

  	}

		switch (counter_c) {
			case 1:
				//ラインまで進む
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -35 * a);

				//container[1]のcatch
				//十字路まで進む
				gain_rate = one_slow;
				power =  slow;
				ev3_sta_cyc(TRACE);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_stp_cyc(TRACE);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, 30);
				ev3_motor_set_power(EV3_PORT_D, 30);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 50 * a);

				counter = 1;
				act_tsk(CATCH_TASK);
				slp_tsk();

				if(container[1] == 1 || container[1] == 2) {
					//container[1]が赤、緑の場合
					if(container[2] == 0){
						ev3_motor_set_power(EV3_PORT_A, fast);
						ev3_motor_set_power(EV3_PORT_D, fast);
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -690 * a);

						ev3_motor_set_power(EV3_PORT_A, slow);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//方向転換
						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, -30);
						ev3_motor_set_power(EV3_PORT_D, 0);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, -30);
						ev3_motor_set_power(EV3_PORT_D, 10);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, -30);
						ev3_motor_set_power(EV3_PORT_D, 10);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

						//十字路まで進む
						gain_rate = two_fast;
						power = fast;
						ev3_sta_cyc(TRACE);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//十字路をこえる(次の十字路の手前まで速く行く補正)
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -550 * a);
						ev3_stp_cyc(TRACE);


					}else{

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, -65);
						ev3_motor_set_power(EV3_PORT_D, -15);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -200 * a);

						ev3_sta_cyc(TRACE2);
						power = slow;
						gain_rate = one_slow;

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black || ev3_color_sensor_get_reflect(EV3_PORT_2) >= white);
						ev3_speaker_play_tone(1174.66, 200);
						ev3_stp_cyc(TRACE2);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, fast);
						ev3_motor_set_power(EV3_PORT_D, fast);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -500 * a);

						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, 0);
						ev3_motor_set_power(EV3_PORT_D, fast);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -160 * a);

						ev3_motor_set_power(EV3_PORT_A, slow);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//方向転換
						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, -30);
						ev3_motor_set_power(EV3_PORT_D, 0);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, -30);
						ev3_motor_set_power(EV3_PORT_D, 10);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, -30);
						ev3_motor_set_power(EV3_PORT_D, 10);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

					}

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -80);

					//十字路まで進む
					gain_rate = two_fast;
					power = fast;

					//container[1]が赤ならゆっくり
					if (container[1] == 1){
						gain_rate = two_slow;
						power = slow;
					}

					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

					ev3_motor_stop(EV3_PORT_B, true);
					tslp_tsk(200);

					ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
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
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

					ev3_stp_cyc(TRACE);

					//L字路まで進む
					gain_rate = one_fast;
					ev3_sta_cyc(TRACE3);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >=white);
					ev3_speaker_play_tone(1174.66, 200);

					ev3_stp_cyc(TRACE3);

					//L字路をこえる
					ev3_motor_set_power(EV3_PORT_A, -30);
					ev3_motor_set_power(EV3_PORT_D, -30);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -90 * a);

					//container[1]のシップの作業
					counter = 1;
					act_tsk(SHIP_LEFT_TASK);
					slp_tsk();

				}else{
					//container[1]が黄、青の場合
					ev3_motor_rotate(EV3_PORT_A, 200 * a, 30, false);
					ev3_motor_rotate(EV3_PORT_D, 200 * a, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					act_tsk(LEFT_TASK);
					slp_tsk();

					//十字路まで進む
					gain_rate = two_fast;
					power = fast;
					ev3_sta_cyc(TRACE);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(十字路の手前まで速く行く補正)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -550 * a);

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -80);

					//十字路まで進む
					gain_rate = two_fast;
					power = fast;

					//container[1]が黄ならゆっくり
					if (container[1] == 3){
						gain_rate = two_slow;
						power = slow;
					}

					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

					ev3_motor_stop(EV3_PORT_B, true);
					tslp_tsk(200);

					ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
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
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

					ev3_stp_cyc(TRACE);

					//L字路まで進む
					gain_rate = one_fast;
					ev3_sta_cyc(TRACE2);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black || ev3_color_sensor_get_reflect(EV3_PORT_2) >= white);
					ev3_speaker_play_tone(1174.66, 200);

					ev3_stp_cyc(TRACE2);

					//L字路をこえる
					ev3_motor_set_power(EV3_PORT_A, -30);
					ev3_motor_set_power(EV3_PORT_D, -30);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -90 * a);

					//container[1]のシップの作業
					counter = 1;
					act_tsk(SHIP_RIGHT_TASK);
					slp_tsk();

				}

				break;

			case 2:
				//ラインまで進む
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=20);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

				//container[2]のcatch
				//十字路まで進む
				gain_rate = one_slow;
				power =  slow;
				ev3_sta_cyc(TRACE);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_stp_cyc(TRACE);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, 30);
				ev3_motor_set_power(EV3_PORT_D, 30);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 145 * a);

				counter = 2;
				act_tsk(CATCH_TASK);
				slp_tsk();

				//containerとcontrollerの入れ替え
				if(container[2] == 3 || container[2] == 4) {
					//container[2]が黄、青の場合
					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, -15);
					ev3_motor_set_power(EV3_PORT_D, -65);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -200 * a);

					ev3_sta_cyc(TRACE3);
					power = slow;
					gain_rate = one_slow;

					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= white || ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
					ev3_speaker_play_tone(1174.66, 200);
					ev3_stp_cyc(TRACE3);

					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, fast);
					ev3_motor_set_power(EV3_PORT_D, fast);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -500 * a);

					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, fast);
					ev3_motor_set_power(EV3_PORT_D, 0);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -155 * a);

					ev3_motor_set_power(EV3_PORT_A, slow);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//方向転換
					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, 0);
					ev3_motor_set_power(EV3_PORT_D, -30);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -100 * a);

					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, 10);
					ev3_motor_set_power(EV3_PORT_D, -30);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -100 * a);

					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, 10);
					ev3_motor_set_power(EV3_PORT_D, -30);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -80);

					//十字路まで進む
					gain_rate = two_fast;
					power = fast;

					//container[2]が黄ならゆっくり
					if (container[2] == 3){
						gain_rate = two_slow;
						power = slow;
					}

					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

					ev3_motor_stop(EV3_PORT_B, true);
					tslp_tsk(200);

					ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
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
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

					ev3_stp_cyc(TRACE);

					//L字路まで進む
					gain_rate = one_fast;
					ev3_sta_cyc(TRACE2);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black || ev3_color_sensor_get_reflect(EV3_PORT_2) >= white);
					ev3_speaker_play_tone(1174.66, 200);

					ev3_stp_cyc(TRACE2);

					//L字路をこえる
					ev3_motor_set_power(EV3_PORT_A, -30);
					ev3_motor_set_power(EV3_PORT_D, -30);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -90 * a);

					//container[2]のシップの作業
					counter = 2;
					act_tsk(SHIP_RIGHT_TASK);
					slp_tsk();


				}else{
					//container[2]が赤、緑の場合
					ev3_motor_rotate(EV3_PORT_A, 105 * a, 30, false);
					ev3_motor_rotate(EV3_PORT_D, 105 * a, 30, true);

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
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(十字路の手前まで速く行く補正)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -550 * a);

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -80);

					//十字路まで進む
					gain_rate = two_fast;
					power = fast;

					//container[2]が赤ならゆっくり
					if (container[2] == 1){
						gain_rate = two_slow;
						power = slow;
					}

					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

					ev3_motor_stop(EV3_PORT_B, true);
					tslp_tsk(200);

					ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
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
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

					ev3_stp_cyc(TRACE);

					//L字路まで進む
					gain_rate = one_fast;
					ev3_sta_cyc(TRACE3);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= white);
					ev3_speaker_play_tone(1174.66, 200);

					ev3_stp_cyc(TRACE3);

					//L字路をこえる
					ev3_motor_set_power(EV3_PORT_A, -30);
					ev3_motor_set_power(EV3_PORT_D, -30);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -90 * a);

					//container[2]のシップの作業
					counter = 2;
					act_tsk(SHIP_LEFT_TASK);
					slp_tsk();

				}


				break;

			case 3:
				//ラインまで進む
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

				//container[3]のcatch
				//十字路まで進む
				gain_rate = two_fast;
				power = fast;
				ev3_sta_cyc(TRACE);

				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				gain_rate = two_slow;
				power = slow;
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -320 * a);

				ev3_stp_cyc(TRACE);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				counter = 3;
				act_tsk(CATCH_TASK);
				slp_tsk();

				//containerとcontrollerの入れ替え
				if(container[3] == 3 || container[3] == 4) {
					//container[3]が黄、青の場合
					ev3_motor_set_power(EV3_PORT_A, fast);
					ev3_motor_set_power(EV3_PORT_D, fast);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -690 * a);

					ev3_motor_set_power(EV3_PORT_A, slow);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//方向転換
					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, 0);
					ev3_motor_set_power(EV3_PORT_D, -30);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -100 * a);

					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, 10);
					ev3_motor_set_power(EV3_PORT_D, -30);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -100 * a);

					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, 10);
					ev3_motor_set_power(EV3_PORT_D, -30);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -80);

					//十字路まで進む
					gain_rate = two_fast;
					power = fast;

					//container[3]が黄ならゆっくり
					if (container[3] == 3){
						gain_rate = two_slow;
						power = slow;
					}

					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

					ev3_motor_stop(EV3_PORT_B, true);
					tslp_tsk(200);

					ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
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
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

					ev3_stp_cyc(TRACE);

					//L字路まで進む
					gain_rate = one_fast;
					ev3_sta_cyc(TRACE2);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black || ev3_color_sensor_get_reflect(EV3_PORT_2) >= white);
					ev3_speaker_play_tone(1174.66, 200);

					ev3_stp_cyc(TRACE2);

					//L字路をこえる
					ev3_motor_set_power(EV3_PORT_A, -30);
					ev3_motor_set_power(EV3_PORT_D, -30);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -90 * a);

					//container[3]のシップの作業
					counter = 3;
					act_tsk(SHIP_RIGHT_TASK);
					slp_tsk();

				}else{
					//container[3]が赤、緑の場合
					ev3_motor_rotate(EV3_PORT_A, 200 * a, 30, false);
					ev3_motor_rotate(EV3_PORT_D, 200 * a, 30, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					act_tsk(RIGHT_TASK);
					slp_tsk();

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -80);

					//十字路まで進む
					gain_rate = two_fast;
					power = fast;

					//container[3]が赤ならゆっくり
					if (container[3] == 1){
						gain_rate = two_slow;
						power = slow;
					}

					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= -260);

					ev3_motor_stop(EV3_PORT_B, true);
					tslp_tsk(200);

					ev3_motor_rotate(EV3_PORT_B, 400, 80, false);

					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
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
						//container[2]が緑の場合、十字路まで進む
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

					ev3_stp_cyc(TRACE);

					//L字路まで進む
					gain_rate = one_fast;
					ev3_sta_cyc(TRACE3);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= white);
					ev3_speaker_play_tone(1174.66, 200);

					ev3_stp_cyc(TRACE3);

					//L字路をこえる
					ev3_motor_set_power(EV3_PORT_A, -30);
					ev3_motor_set_power(EV3_PORT_D, -30);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= -90 * a);

					//container[3]のシップの作業
					counter = 3;
					act_tsk(SHIP_LEFT_TASK);
					slp_tsk();

				}

				break;

			default:
				break;
		}

	}

	//壁合わせ and 方向転換
	if (counter_b == 1) {
		if (counter_d <= 3) {
			ev3_motor_reset_counts(EV3_PORT_A);
			if(angle >= 300 * a) {
				ev3_motor_set_power(EV3_PORT_A, fast * -1);
				ev3_motor_set_power(EV3_PORT_D, fast * -1);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= (angle - 300 * a));
			}

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= angle);

			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			tslp_tsk(500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_rotate(EV3_PORT_A, -83 * a, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -83 * a, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			counter_a = 1;

		}else{
			ev3_motor_reset_counts(EV3_PORT_A);
			if(1250 * a - angle >= 100 * a) {
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle - 100 * a) * -1);
			}
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle) * -1);

			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			tslp_tsk(500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_rotate(EV3_PORT_A, 95 * a, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 95 * a, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, turn, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -turn, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			counter_a = 2;

		}
	}else{
		if (counter_d <= 3) {
			ev3_motor_reset_counts(EV3_PORT_A);
			if(1250 * a - angle >= 100 * a) {
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle - 100 * a) * -1);
			}
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (1250 * a - angle) * -1);

			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			tslp_tsk(600);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_rotate(EV3_PORT_A, 95 * a, 30, false);
			ev3_motor_rotate(EV3_PORT_D, 95 * a, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
			ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			counter_a = 1;

		}else{
			ev3_motor_reset_counts(EV3_PORT_A);
			if(angle >= 300 * a) {
				ev3_motor_set_power(EV3_PORT_A, fast * -1);
				ev3_motor_set_power(EV3_PORT_D, fast * -1);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= (angle - 300 * a));
			}

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= angle);

			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			tslp_tsk(600);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_rotate(EV3_PORT_A, -83 * a, 30, false);
			ev3_motor_rotate(EV3_PORT_D, -83 * a, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_rotate(EV3_PORT_A, -turn, 30, false);
			ev3_motor_rotate(EV3_PORT_D, turn, 30, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			counter_a = 2;


		}

	}

	//スタートエリアに戻る
	if (counter_a == 1) {
		//ラインまで進む
		ev3_motor_set_power(EV3_PORT_A, fast);
		ev3_motor_set_power(EV3_PORT_D, fast);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >=black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

		//十字路まで進む
		gain_rate = two_fast;
		power = fast;
		ev3_sta_cyc(TRACE);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

		//十字路まで進む
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -450 * a);

		gain_rate = two_slow;
		power = slow;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 0);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -150 * a);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 10);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, -30);
		ev3_motor_set_power(EV3_PORT_D, 10);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

	}else{
		//ラインまで進む
		ev3_motor_set_power(EV3_PORT_A, fast);
		ev3_motor_set_power(EV3_PORT_D, fast);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

		//十字路まで進む
		gain_rate = two_fast;
		power = fast;
		ev3_sta_cyc(TRACE);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -100 * a);

		//十字路まで進む
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -450 * a);

		gain_rate = two_slow;
		power = slow;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//方向転換
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 0);
		ev3_motor_set_power(EV3_PORT_D, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) >= -150 * a);

		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 10);
		ev3_motor_set_power(EV3_PORT_D, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);

		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_A, 10);
		ev3_motor_set_power(EV3_PORT_D, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) >= -50 * a);

	}

	//ゴールエリアまで進む
	gain_rate = two_fast;
	power = fast;
	ev3_sta_cyc(TRACE);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -240 * a);

	ev3_stp_cyc(TRACE);

	//ゴールエリアにinする
	ev3_motor_rotate(EV3_PORT_A, -310 * a, slow * -1, false);
	ev3_motor_rotate(EV3_PORT_D, -310 * a, slow * -1, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

}
