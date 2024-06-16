#include "ev3api.h"
#include "app.h"
#include "stdlib.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/*
[s11 船が3つ増えて全部押し出す]
対応時間 1時間
試走回数 5回
竸技時間 1:37
変更内容
・船読みのところを、4色以外の色で0以外を読んだら押し込むプログラムをはさむ
・アームをたたむコマンドを場合によってよける
・counter_dの判断基準注意
*/

//PID制御の変数
double kp, ki, kd, gain_rate;
double delta_t = 0.004;
int diff[2];
int integral, p, i, d, sensor2, sensor3, motor, count, power, sensor1, pid;

//角度の変数(使い捨て)
int angle;

//角度の変数(使い捨て)
int angle2;

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

//container[3]が1回目か最後かの変数
int counter5 = 0;

//最初のshipの作業をしたかどうか
int counter6 = 0;

//HTのよみこみの変数
uint8_t *pHT = 0;
uint8_t ht = 0;

//速度の変数
int fast = -80;
int slow = -20;
int slows = -30;

//gainの変数
int two_fast = 2.14;
int two_slow = 2.14;
int one_fast = 4.0;
int one_slow = 1.9;

//カウンター
int counter_a = 0;
int counter_b = 0;
int counter_c = 0;
int counter_d = 0;
int counter_e = 0; //使い捨て

//存在しないコンテナ
int no_container = 0;

//存在しないコンテナ
int no_color = 0;

//存在するコンテナの値 あるなら その色　ないなら 0
int yes_container[4];

//ship2のカウンター
int j = 0;

//scanのポートのカウンター
int port = 0;

//角度調節の比例定数
double a = 1.00;

//方向転換の変数
int turn = 185;

//センサーの閾値
int black = 20;
int white = 55;
int threshold = 40;

//containerの青と緑を読み間違わないようにする変数 → 船の赤と黄色を読み間違わないようにする変数
int separate = 0;

//台形駆動
double bt, ct, powert, anglet;
double maxt = 80.0;
double mint = 20.0;
double at = 0.3;
int *pTt;
int Tt;
int countt = 0;

//curveの変数
int curve = 0;

//ship二度読み禁止の変数
int twice = 0;

//ship後の壁からの角度
int anfw = -90;
int anbw = 102;

//アームをあげる角度
int arm = -200;

int i = 0;

int counter_s;


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
		ev3_motor_set_power(EV3_PORT_A, power * -1);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor * -1);
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
		ev3_motor_set_power(EV3_PORT_A, power * -1);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor * -1);
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
		ev3_motor_set_power(EV3_PORT_A, motor * -1);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, power * -1);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void trace22_task(intptr_t unused) {
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
		ev3_motor_set_power(EV3_PORT_A, motor * -1);
		ev3_motor_set_power(EV3_PORT_D, power);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, power * -1);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}
}


void trace33_task(intptr_t unused) {
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

	if(pid >= 0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, power * -1);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor * -1);
		ev3_motor_set_power(EV3_PORT_D, power);
	}
}


void right_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30 * a);
		ev3_motor_set_power(EV3_PORT_D, -30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

	do{
		ev3_motor_set_power(EV3_PORT_A, -40 * a);
		ev3_motor_set_power(EV3_PORT_D, -40 * a);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 50);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, -20 * a);
		ev3_motor_set_power(EV3_PORT_D, -20 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -40 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void left_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 30 * a);
		ev3_motor_set_power(EV3_PORT_D, 30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 50 * a);

	do{
		ev3_motor_set_power(EV3_PORT_A, 40 * a);
		ev3_motor_set_power(EV3_PORT_D, 40 * a);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= 50);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, 20 * a);
		ev3_motor_set_power(EV3_PORT_D, 20 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 40 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void rotation_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30 * a);
		ev3_motor_set_power(EV3_PORT_D, -30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -200 * a);

	do{
		ev3_motor_set_power(EV3_PORT_A, -40 * a);
		ev3_motor_set_power(EV3_PORT_D, -40 * a);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 50);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30 * a);
		ev3_motor_set_power(EV3_PORT_D, -30 * a);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -40 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void change_yb_task(intptr_t unused) {
	//コントローラーに対する位置の調節
	if(container[counter] == 4){
		gain_rate = two_slow;
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

		power = slow;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -390 * a);

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
		ev3_motor_set_power(EV3_PORT_D, slow * -1);
		ev3_motor_set_power(EV3_PORT_A, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -20 * a);
	}

	//コンテナとコントローラーの入れ替え
	curve = 5;
	counter_e = 1;
	act_tsk(CURVE_TASK);
	slp_tsk();

	ev3_motor_set_power(EV3_PORT_B, -25);
	ev3_motor_rotate(EV3_PORT_A, 175 * a, slow * -1, false);
	ev3_motor_rotate(EV3_PORT_D, -175 * a, slow * -1, true);

	ev3_motor_set_power(EV3_PORT_B, -100);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_A, -55 * a, 10, false);
	ev3_motor_rotate(EV3_PORT_D, 55 * a, 10, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);

	tslp_tsk(200);

	ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_rotate(EV3_PORT_B, 150, 30, true);

	gain_rate = one_fast;
	power = fast;

	wup_tsk(MAIN_TASK);
}


void change_rg_task(intptr_t unused) {
	//コントローラーに対する位置の調節
	if(container[counter] == 2){
		gain_rate = two_slow;
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

		power = slow;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -390 * a);
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
		ev3_motor_set_power(EV3_PORT_D, slow * -1);
		ev3_motor_set_power(EV3_PORT_A, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= -20 * a);
	}

	//コンテナとコントローラーの入れ替え
	curve = 6;
	counter_e = 1;
	act_tsk(CURVE_TASK);
	slp_tsk();

	ev3_motor_set_power(EV3_PORT_B, -25);
	ev3_motor_rotate(EV3_PORT_A, 175 * a, slow * -1, false);
	ev3_motor_rotate(EV3_PORT_D, -175 * a, slow * -1, true);

	ev3_motor_set_power(EV3_PORT_B, -100);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_rotate(EV3_PORT_A, -55 * a, 10, false);
	ev3_motor_rotate(EV3_PORT_D, 55 * a, 10, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);

	tslp_tsk(200);

	ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_rotate(EV3_PORT_B, 150, 30, true);

	gain_rate = one_fast;
	power = fast;

	wup_tsk(MAIN_TASK);
}


void scan_task(intptr_t unused) {
	power = slow;
	if(port == 1){
		ht_nxt_color_sensor_measure_color(EV3_PORT_1, pHT);
		*pHT = 0;
		ev3_sta_cyc(HT1);
	}else{
		ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
		*pHT = 0;
		ev3_sta_cyc(HT4);
	}

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	if(counter == 3){
		do{
			if((*pHT == 1 || *pHT == 7 || *pHT == 8) && container[counter] == 0) {
				container[counter] = 1;
				yes_container[counter] = 1;
				ev3_speaker_play_tone(1975.53, 200);
			}else if((*pHT == 4 || *pHT == 13) && container[counter] == 0) {
				container[counter] = 2;
				yes_container[counter] = 2;
				ev3_speaker_play_tone(1396.91, 200);
			}else if((*pHT == 5 || *pHT == 6) && container[counter] == 0) {
				container[counter] = 3;
				yes_container[counter] = 3;
				ev3_speaker_play_tone(880.00, 200);
			}else if((*pHT == 2 || *pHT == 11) && container[counter] == 0) {
				container[counter] = 4;
				yes_container[counter] = 4;
				ev3_speaker_play_tone(261.63, 200);
			/*}else if(*pHT == 14){
				separate = 1;*/
			}else{

			}
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 150 * a);

		if(separate == 1 && container[counter] == 0) {
			container[counter] = 2;
			yes_container[counter] = 2;
			ev3_speaker_play_tone(1396.91, 200);
		}
		separate = 0;
	}else{
		do{
			if((*pHT == 1 || *pHT == 7 || *pHT == 10) && container[counter] == 0) {
				container[counter] = 1;
				yes_container[counter] = 1;
				ev3_speaker_play_tone(1975.53, 200);
			}else if((*pHT == 4 || *pHT == 13) && container[counter] == 0) {
				container[counter] = 2;
				yes_container[counter] = 2;
				ev3_speaker_play_tone(1396.91, 200);
			}else if(*pHT == 5 && container[counter] == 0) {
				container[counter] = 3;
				yes_container[counter] = 3;
				ev3_speaker_play_tone(880.00, 200);
			}else if((*pHT == 2 || *pHT == 11) && container[counter] == 0) {
				container[counter] = 4;
				yes_container[counter] = 4;
				ev3_speaker_play_tone(261.63, 200);
			}else if(*pHT == 12) {
				separate = 1;
			}else{

			}
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 150 * a);

		if(separate == 1 && container[counter] == 0) {
			container[counter] = 2;
			yes_container[counter] = 2;
			ev3_speaker_play_tone(1396.91, 200);
		}
		separate = 0;
	}

	if(port == 1){
		ev3_stp_cyc(HT1);
	}else{
		ev3_stp_cyc(HT4);
	}

	wup_tsk(MAIN_TASK);
}


void ship_left_task(intptr_t unused) {
	//ラインまで進む
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black && ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	//軸に乗せる
	ev3_motor_rotate(EV3_PORT_A, 45 * a, slow * -1, false);
	ev3_motor_rotate(EV3_PORT_D, -45 * a, slow * -1, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

	ev3_motor_set_power(EV3_PORT_A, slows);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//壁合わせ
	ev3_motor_set_power(EV3_PORT_A, -10);
	ev3_motor_set_power(EV3_PORT_D, 10);
	tslp_tsk(1000);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	counter_d = ship[container[counter] - 1];

	angle = 55 + (counter_d - 1) * 246 * a;//55

	if(counter_d == 1){
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);

		ev3_motor_set_power(EV3_PORT_A, 10);
		ev3_motor_set_power(EV3_PORT_D, -10);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle);

		turn = turn - 2;

		counter_e = 1;

	}else if(counter_d == 6){
		power = slows;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE33);

		angle = angle - 5;

		turn = turn + 2;

		counter_e = 1;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle - 200);

		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);

		ev3_stp_cyc(TRACE33);

	}else{
		power = slows;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE33);

		if(angle >= 300) {
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle - 200);

			power = slow;
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);


		}else{
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle);
		}

		ev3_stp_cyc(TRACE33);
	}

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(300);

	//方向転換
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

	ev3_motor_set_power(EV3_PORT_A, slows);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	if(counter_e == 1){
		turn = turn - 2;
	}else if(counter_e == 2){
		turn = turn + 2;
	}

	counter_e = 0;

	//シップを出航させる 1
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	tslp_tsk(100);

	//フードとコントローラーをシップに乗せる
	ev3_motor_set_power(EV3_PORT_C, -10);

	//シップを出航させる 2
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	tslp_tsk(1600);//1900

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//下ろしたアームが引っかからなくなるまで進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 55 * a);

	//シップを出航させた、完了の0
	ship[container[counter] - 1] = 0;
	container[counter] = 0;
	yes_container[counter] = 0;

	counter_b = 1;

	wup_tsk(MAIN_TASK);
}


void ship_right_task(intptr_t unused) {
	//ラインまで進む
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black && ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	//軸に乗せる
	ev3_motor_rotate(EV3_PORT_A, 45 * a, slow * -1, false);
	ev3_motor_rotate(EV3_PORT_D, -45 * a, slow * -1, true);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	ev3_motor_set_power(EV3_PORT_D, slows * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= turn * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//壁合わせ
	ev3_motor_set_power(EV3_PORT_A, -10);
	ev3_motor_set_power(EV3_PORT_D, 10);
	tslp_tsk(1000);

	counter_d = ship[container[counter] - 1];

	angle = 55 + (6 - counter_d) * 246 * a;

	if(counter_d == 6){
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);

		ev3_motor_set_power(EV3_PORT_A, 10);
		ev3_motor_set_power(EV3_PORT_D, -10);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle);

		turn = turn - 2;

		counter_e = 2;
	}else if(counter_d == 1){
		power = slows;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE22);

		angle = angle - 5;

		turn = turn + 2;

		counter_e = 1;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle - 200);

		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);

		ev3_stp_cyc(TRACE22);

	}else{
		power = slows;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE22);

		if(angle >= 300) {
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle - 200);

			power = slow;
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 100);
		}else{
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= angle);
		}

		ev3_stp_cyc(TRACE22);
	}

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(300);

	//方向転換
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	ev3_motor_set_power(EV3_PORT_D, slows * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= turn * a);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	if(counter_e == 1){
		turn = turn - 2;
	}else if(counter_e == 2){
		turn = turn + 2;
	}

	counter_e = 0;

	//シップを出航させる 1
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	tslp_tsk(100);

	//フードとコントローラーをシップに乗せる
	ev3_motor_set_power(EV3_PORT_C, -10);

	//シップを出航させる 2
	ev3_motor_set_power(EV3_PORT_A, slow);
	ev3_motor_set_power(EV3_PORT_D, slow * -1);
	tslp_tsk(1600);//1900

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//下ろしたアームが引っかからなくなるまで進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 55 * a);

	//シップを出航させた、完了の0
	ship[container[counter] - 1] = 0;
	container[counter] = 0;
	yes_container[counter] = 0;

	counter_b = 2;

	wup_tsk(MAIN_TASK);
}


void catch_task(intptr_t unused) {
	switch (counter) {
		case 0:
			if (counter5 == 0){
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, -30);
				ev3_motor_set_power(EV3_PORT_D, 30);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -135 * a);//125

				curve = 6;
				counter_e = 2;
				act_tsk(CURVE_TASK);
				slp_tsk();
			}else{
				power = slows;
				ev3_sta_cyc(TRACE);
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

				power = slow;
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -430 * a);
				ev3_stp_cyc(TRACE);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(300);

				//方向転換
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

				ev3_motor_set_power(EV3_PORT_A, slows);
				ev3_motor_set_power(EV3_PORT_D, slows);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				ev3_motor_set_power(EV3_PORT_A, -10);
				ev3_motor_set_power(EV3_PORT_D, 10);
				tslp_tsk(800);

			}
			counter5 = 0;

			break;

		case 1:
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -30 * a);

			curve = 5;
			counter_e = 2;
			act_tsk(CURVE_TASK);
			slp_tsk();


			break;

		case 2:
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -125 * a);

			curve = 6;
			counter_e = 2;
			act_tsk(CURVE_TASK);
			slp_tsk();

			break;

		case 3:
			power = slow;
			gain_rate = two_slow;
			ev3_sta_cyc(TRACE);

			if (counter5 == 0){
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);
				ev3_stp_cyc(TRACE);
			}else{
				power = slows;
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -110 * a);

				power = slow;
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -310 * a);
				ev3_stp_cyc(TRACE);
			}
			counter5 = 0;

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(300);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			ev3_motor_set_power(EV3_PORT_D, slows * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= turn * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			ev3_motor_set_power(EV3_PORT_A, -10);
			ev3_motor_set_power(EV3_PORT_D, 10);
			tslp_tsk(800);

			break;

		default:
			break;
	}

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	if(counter == 0 || counter == 2) {
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);

		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 70 * a);

		ev3_motor_set_power(EV3_PORT_B, 80);

		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 150 * a);

		ev3_motor_rotate(EV3_PORT_A, 50 * a, 10, false);
		ev3_motor_rotate(EV3_PORT_D, -50 * a, 10, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}else{
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);

		ev3_motor_reset_counts(EV3_PORT_A);

		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);

		ev3_motor_set_power(EV3_PORT_B, 80);

		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 250 * a);

		ev3_motor_rotate(EV3_PORT_A, 50 * a, 10, false);
		ev3_motor_rotate(EV3_PORT_D, -50 * a, 10, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);
	}
	/*ev3_motor_set_power(EV3_PORT_B, 80);*/
	tslp_tsk(100);

	ev3_motor_rotate(EV3_PORT_B, -285, 80, true);

	wup_tsk(MAIN_TASK);
}


void ht1_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_color(EV3_PORT_1, pHT);
}


void ht4_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
}


void trapezoidal_task(intptr_t unused) {
	bt = (maxt - mint) / at;
	ct = Tt - 2 * bt;

	ev3_motor_reset_counts(EV3_PORT_D);

	do{
		anglet = ev3_motor_get_counts(EV3_PORT_D);
		powert = at * -1.0 * anglet + mint;
		ev3_motor_set_power(EV3_PORT_A, 1.0 * powert);
		ev3_motor_set_power(EV3_PORT_D, -1.0 * powert);
	}while(anglet > -1.0 * bt);

	ev3_motor_set_power(EV3_PORT_A, 1.0 * maxt);
	ev3_motor_set_power(EV3_PORT_D, -1.0 * powert);
	do{
		anglet = ev3_motor_get_counts(EV3_PORT_D);
	}while(anglet > -1.0 * ct);

	do{
		anglet = ev3_motor_get_counts(EV3_PORT_D);
		powert = -1.0 * at * -1.0 * anglet + at * Tt + mint;
		ev3_motor_set_power(EV3_PORT_A, 1.0 * powert);
		ev3_motor_set_power(EV3_PORT_D, -1.0 * powert);
	}while(anglet > -1.0 * Tt);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	if (countt == 0) {
		wup_tsk(SHIP_LEFT_TASK);
	}else{
		wup_tsk(SHIP_RIGHT_TASK);
	}
}


void curve_task(intptr_t unused) {
	switch (curve) {
		case 1://真ん中への右
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -100 * a);//-120//-110

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -30 * a);
			ev3_motor_set_power(EV3_PORT_D, -30 * a);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);

			ev3_motor_set_power(EV3_PORT_A, -30 * a);
			ev3_motor_set_power(EV3_PORT_D, -30 * a);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);

			ev3_motor_set_power(EV3_PORT_A, -10 * a);
			ev3_motor_set_power(EV3_PORT_D, -10 * a);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -20 * a);

			wup_tsk(MAIN_TASK);

			break;

		case 2://真ん中への左
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);//120//110

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30 * a);
			ev3_motor_set_power(EV3_PORT_D, 30 * a);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 50 * a);

			ev3_motor_set_power(EV3_PORT_A, 30 * a);
			ev3_motor_set_power(EV3_PORT_D, 30 * a);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

			ev3_motor_set_power(EV3_PORT_A, 10 * a);
			ev3_motor_set_power(EV3_PORT_D, 10 * a);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);

			wup_tsk(MAIN_TASK);

			break;

		case 3://内側への右
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -60 * a);//80

			ev3_motor_set_power(EV3_PORT_A, -30 * a);
			ev3_motor_set_power(EV3_PORT_D, -30 * a);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) <= 50);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -10 * a);
			ev3_motor_set_power(EV3_PORT_D, -10 * a);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -10 * a);

			wup_tsk(MAIN_TASK);

			break;

		case 4://内側への左
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 60 * a);//80

			ev3_motor_set_power(EV3_PORT_A, 30 * a);
			ev3_motor_set_power(EV3_PORT_D, 30 * a);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) <= 50);

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 10 * a);
			ev3_motor_set_power(EV3_PORT_D, 10 * a);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 10 * a);

			wup_tsk(MAIN_TASK);

			break;

		case 5://壁による方向転換 右
			ev3_motor_stop(EV3_PORT_D, true);
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -250 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows);
			ev3_motor_set_power(EV3_PORT_D, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -60 * a);

			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			tslp_tsk(300);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			if(counter_e == 1){
				wup_tsk(CHANGE_YB_TASK);
			}else if(counter_e == 2){
				wup_tsk(CATCH_TASK);
			}else{

			}

			break;

		case 6://壁による方向転換 左
			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_D, slows * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) <= 250 * a);//240

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			ev3_motor_set_power(EV3_PORT_D, slows * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) <= 60 * a);//70

			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			tslp_tsk(300);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			if(counter_e == 1){
				wup_tsk(CHANGE_RG_TASK);
			}else if(counter_e == 2){
				wup_tsk(CATCH_TASK);
			}else{

			}

			break;

		default:
			break;
	}

}


void main_task(intptr_t unused) {
	ev3_lcd_set_font (EV3_FONT_MEDIUM);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);	//右前カラーセンサー
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);	//左前カラーセンサー
	ev3_sensor_config(EV3_PORT_1, HT_NXT_COLOR_SENSOR);	//右後HTセンサー
	ev3_sensor_config(EV3_PORT_4, HT_NXT_COLOR_SENSOR);	//左後HTセンサー
	ev3_motor_config(EV3_PORT_A, MEDIUM_MOTOR);	//右タイヤ
	ev3_motor_config(EV3_PORT_D, MEDIUM_MOTOR);	//左タイヤ
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

	//台形駆動 さす
	pTt = &Tt;

	//ループ処理の変数
	int k;

	/*実験用のループ
	while(1){
		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, 20);
		ev3_motor_set_power(EV3_PORT_D, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= turn * 14 * a);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);


		do{
		}while(ev3_button_is_pressed(ENTER_BUTTON) == false);
		tslp_tsk(200);
	}*/

	//丁字路まで進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 80 * a);

	//速く移動
	power = fast;
	gain_rate = one_fast;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_sta_cyc(TRACE2);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);

	power = slow;
	gain_rate = one_slow;
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black || ev3_color_sensor_get_reflect(EV3_PORT_2) >= white);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE2);

	/*ここに角度で進むを入れるとうまくいくかも　いつも惰性で進んでいるように見える
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slow * -1);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -5 * a);*/

	//方向転換
	curve = 2;
	act_tsk(CURVE_TASK);
	slp_tsk();

	//container[0]まで進む
	power = slows;
	gain_rate = two_slow;
	ev3_sta_cyc(TRACE);

	//助走
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

	power = fast;
	gain_rate = two_fast;
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 410 * a);

	power = slow;
	gain_rate = two_slow;

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);

	//container[0]をよみこむ
	counter = 0;
	port = 4;
	act_tsk(SCAN_TASK);
	slp_tsk();

	power = slows;
	gain_rate = two_slow;

	//十字路まで進む
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE);

	if(container[0] != 0) {
		//container[0]があった場合
		counter_a = 0;

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//container[0]を回収
		counter = 0;
		act_tsk(CATCH_TASK);
		slp_tsk();

		if(container[0] == 3 || container[0] == 4) {
			ev3_motor_rotate(EV3_PORT_A, -90 * a, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, 90 * a, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//container[0]が黄、青だった場合
			//コンテナとコントローラーの入れ替え

			//方向転換
			act_tsk(LEFT_TASK);
			slp_tsk();

			//Foodの回収
			gain_rate = two_slow;
			power = slows;

			ev3_sta_cyc(TRACE);

			ev3_motor_reset_counts(EV3_PORT_B);
			ev3_motor_set_power(EV3_PORT_B, -100);

			do{
			}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

			power = fast;
			gain_rate = two_fast;

			ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

			//速く移動
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -250 * a);//60

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

			power = slows;
			gain_rate = two_slow;
			ev3_sta_cyc(TRACE);

			//助走
 	 		ev3_motor_reset_counts(EV3_PORT_A);
 	 		do{
 	 		}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

			power = fast;
			gain_rate = two_fast;

			if(container[0] == 4) {
				//container[0]が青の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 400 * a);
			}

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 400 * a);

			power = slow;
			gain_rate = two_slow;

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);

			//方向転換
			curve = 4;
			act_tsk(CURVE_TASK);
			slp_tsk();

			//container[3]へ
			power = slow;
			gain_rate = one_slow;
			ev3_sta_cyc(TRACE22);

			//助走
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

			power = slows;
			gain_rate = one_slow;

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE22);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			ev3_motor_set_power(EV3_PORT_D, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 138 * a);

			power = slows;
			gain_rate = one_slow;

			ev3_sta_cyc(TRACE22);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 120 * a);

			power = slow;
			gain_rate = one_slow;

			//container[3]のscan
			counter = 3;
			port = 4;
			act_tsk(SCAN_TASK);
			slp_tsk();

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -150 * a);

			ev3_stp_cyc(TRACE22);

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//方向転換
			curve = 1;
			act_tsk(CURVE_TASK);
			slp_tsk();

			//contaienr[2]を読む準備
			power = slows;
			gain_rate = two_slow;
			ev3_sta_cyc(TRACE);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 190 * a);

			ev3_stp_cyc(TRACE);

		}else{
			//container[0]が赤、緑だった場合
			//container[3]のscan
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -25 * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -150 * a);

			ev3_sta_cyc(TRACE3);
			power = slows;
			gain_rate = one_slow;

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

			power = slows;
			gain_rate = one_fast;

			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= white || ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
			ev3_speaker_play_tone(1174.66, 200);
			ev3_stp_cyc(TRACE3);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 185 * a);

			//container[3]へ
			power = slows;
			gain_rate = one_slow;

			ev3_sta_cyc(TRACE22);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -100 * a);

			power = slow;
			//container[3]のscan
			counter = 3;
			port = 4;
			act_tsk(SCAN_TASK);
			slp_tsk();

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -150 * a);

			ev3_stp_cyc(TRACE22);

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//方向転換
			curve = 2;
			act_tsk(CURVE_TASK);
			slp_tsk();

			//Foodの回収
			gain_rate = two_slow;
			power = slows;

			ev3_sta_cyc(TRACE);

			ev3_motor_reset_counts(EV3_PORT_B);
			ev3_motor_set_power(EV3_PORT_B, -100);

			do{
			}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

			power = fast;
			gain_rate = two_fast;


			ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

			//速く移動
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -550 * a);//180

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

			power = slows;
			gain_rate = two_slow;
			ev3_sta_cyc(TRACE);

			//助走
 	 		ev3_motor_reset_counts(EV3_PORT_A);
 	 		do{
 	 		}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

			power = fast;
			gain_rate = two_fast;

			if(container[0] == 2) {
				//container[0]が緑の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//container[2]を読む準備
			power = slows;
			gain_rate = two_slow;
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 260 * a);

			ev3_stp_cyc(TRACE);

		}
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);

		//container[2]のscan
		counter = 2;
		port = 1;
		act_tsk(SCAN_TASK);
		slp_tsk();

		//shipのscan container[0]のship作業
		//速く移動
		ev3_motor_set_power(EV3_PORT_A, slows * -1);
		ev3_motor_set_power(EV3_PORT_D, slows);
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 250 * a);

		//ラインまで進む
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//軸に乗せる
		ev3_motor_rotate(EV3_PORT_A, 45 * a, slow * -1, false);
		ev3_motor_rotate(EV3_PORT_D, -45 * a, slow * -1, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slows);
		ev3_motor_set_power(EV3_PORT_D, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 80) * -1 * a);

		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//壁合わせ
		ev3_motor_set_power(EV3_PORT_A, -10);
		ev3_motor_set_power(EV3_PORT_D, 10);
		tslp_tsk(1000);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}else{
		//container[0]がなかった場合
		counter_a = 3;

		//方向転換
		curve = 4;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//container[3]へ
		power = slow;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE22);

		//助走
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

		power = slows;
		gain_rate = one_slow;

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slows * -1);
		ev3_motor_set_power(EV3_PORT_D, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 138 * a);

		power = slows;
		gain_rate = one_slow;
		ev3_sta_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 140 * a);

		power = slow;
		gain_rate = two_slow;

		//container[3]のscan
		counter = 3;
		port = 4;
		act_tsk(SCAN_TASK);
		slp_tsk();

		ev3_stp_cyc(TRACE22);

		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//方向転換
		curve = 2;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//container[3]のcatch
		counter5 = 0;
		counter = 3;
		act_tsk(CATCH_TASK);
		slp_tsk();

		//コンテナとコントローラの入れ替え
		if(container[3] == 3 || container[3] == 4) {
			//container[3]が黄、青だった場合
			if(yes_container[0] == 0){
				//助走
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slows * -1);
				ev3_motor_set_power(EV3_PORT_D, slows);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, fast * -1);
				ev3_motor_set_power(EV3_PORT_D, fast);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 650 * a);

				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				//方向転換
				curve = 1;
				act_tsk(CURVE_TASK);
				slp_tsk();

				//Foodの回収
				gain_rate = two_slow;
				power = slows;

				ev3_sta_cyc(TRACE);

				ev3_motor_reset_counts(EV3_PORT_B);
				ev3_motor_set_power(EV3_PORT_B, -100);

				do{
				}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

				gain_rate = two_fast;
				power = fast;

				ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

				//速く移動
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -300 * a);//0

			}else{
				ev3_motor_reset_counts(EV3_PORT_D);
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_D) >= -25 * a);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slows * -1);
				ev3_motor_set_power(EV3_PORT_D, 0);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 150 * a);

				ev3_sta_cyc(TRACE2);
				power = slow;
				gain_rate = one_slow;

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= white || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);
				ev3_stp_cyc(TRACE2);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slows * -1);
				ev3_motor_set_power(EV3_PORT_D, slows);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 30 * a);

				ev3_motor_reset_counts(EV3_PORT_D);
				ev3_motor_set_power(EV3_PORT_A, 0);
				ev3_motor_set_power(EV3_PORT_D, slows);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_D) >= -185 * a);

				power = slows;
				gain_rate = two_slow;
				ev3_sta_cyc(TRACE);

				//助走
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

				power = fast;
				gain_rate = two_fast;

				//速く移動
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);

				power = slows;
				gain_rate = two_slow;

				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_stp_cyc(TRACE);

				//方向転換
				curve = 1;
				act_tsk(CURVE_TASK);
				slp_tsk();

				power = slows;
				gain_rate = two_slow;

				ev3_sta_cyc(TRACE);

				//助走
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

				power = fast;
				gain_rate = two_fast;

				//Foodの回収
				ev3_motor_reset_counts(EV3_PORT_B);
				ev3_motor_set_power(EV3_PORT_B, -100);

				do{
				}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

				gain_rate = two_fast;
				power = fast;

				ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

				//十字路の手前まで速く行く
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 350 * a);//0
			}

			//黄ならゆっくり
			if(container[3] == 3){
				gain_rate = two_slow;
				power = slow;
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			counter = 3;
			act_tsk(CHANGE_YB_TASK);
			slp_tsk();

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			power = slows;
			gain_rate = two_slow;
			ev3_sta_cyc(TRACE);

			//助走
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

			power = fast;
			gain_rate = two_fast;

			if(container[3] == 4) {
				//container[3]が青の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) > black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 300 * a);
			}
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 500 * a);

			power = slow;
			gain_rate = two_slow;

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);
			//左に行きcontainer[2]をscan
			//方向転換
			curve = 2;
			act_tsk(CURVE_TASK);
			slp_tsk();

			power = slows;
			gain_rate = two_slow;
			ev3_sta_cyc(TRACE);

			//助走
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

			power = fast;
			gain_rate = two_fast;

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 400 * a);

			power = slow;
			gain_rate = two_slow;
			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);
			ev3_stp_cyc(TRACE);

			//方向転換
			curve = 1;
			act_tsk(CURVE_TASK);
			slp_tsk();

			//container[2]を読む準備
			power = slows;
			gain_rate = two_slow;
			ev3_sta_cyc(TRACE);

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 190 * a);

			ev3_stp_cyc(TRACE);

	  }else{
			//contaienr[3]が赤、緑の場合
			ev3_motor_rotate(EV3_PORT_A, -200 * a, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, 200 * a, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			act_tsk(RIGHT_TASK);
			slp_tsk();

			//Foodの回収
			ev3_motor_reset_counts(EV3_PORT_B);
			ev3_motor_set_power(EV3_PORT_B, -100);

			gain_rate = two_slow;
			power = slows;
			ev3_sta_cyc(TRACE);

			do{
			}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

			power = fast;
			gain_rate = two_fast;

			ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

			//速く移動
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -350 * a);

			//container[3]が赤ならゆっくり
			if(container[3] == 1){
				gain_rate = two_slow;
				power = slow;
			}

			//十字路まで進む
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
			gain_rate = two_slow;
			power = slows;

			//助走
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

			power = fast;
			gain_rate = two_fast;

			if(container[3] == 2) {
				//container[3]が緑の場合、十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
			}

			//十字路まで進む
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//container[2]を読む準備
			power = slows;
			gain_rate = two_slow;

			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 260 * a);

			ev3_stp_cyc(TRACE);

		}
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow);

			//container[2]のscan
			counter = 2;
			port = 1;
			act_tsk(SCAN_TASK);
			slp_tsk();

			//shipのscan container[3]のship作業
			//速く移動
			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			ev3_motor_set_power(EV3_PORT_D, slows);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 250 * a);

			//ラインまで進む
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black && ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
			ev3_speaker_play_tone(1174.66, 200);

			//軸に乗せる
			ev3_motor_rotate(EV3_PORT_A, 45 * a, slow * -1, false);
			ev3_motor_rotate(EV3_PORT_D, -45 * a, slow * -1, true);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows);
			ev3_motor_set_power(EV3_PORT_D, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 80) * -1 * a);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//壁合わせ
			ev3_motor_set_power(EV3_PORT_A, -10);
			ev3_motor_set_power(EV3_PORT_D, 10);
			tslp_tsk(1000);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

	}
	//shipのscan

	//シップまで進む
	angle = 0;
	angle2 = 0;
	gain_rate = one_slow;
	power = slows;
	i = 0;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_sta_cyc(TRACE33);

	do{
		counter_s = 0;;
		//シップをよみこむ
		ht_nxt_color_sensor_measure_color(EV3_PORT_4, pHT);
		*pHT = 0;
		do{
			ev3_sta_cyc(HT4);
			if((*pHT == 1 || *pHT == 7 || *pHT == 8 || *pHT == 9 || *pHT == 10) && ship[0] == 0 && twice == 0) {
				ev3_speaker_play_tone(1975.53, 200);
				i = 0;
				ship2[j] = 1;
				j += 1;
				twice = 1;
			}else if((*pHT == 4 || *pHT == 13) && ship[1] == 0 && twice == 0) {
				ev3_speaker_play_tone(1396.91, 200);
				i = 1;
				ship2[j] = 2;
				j += 1;
				twice = 1;
			}else if((*pHT == 5 || *pHT == 6) && ship[2] == 0 && twice == 0) {
				ev3_speaker_play_tone(880.00, 200);
				i = 2;
				ship2[j] = 3;
				j += 1;
				twice = 1;
			}else if((*pHT == 2 || *pHT == 11 || *pHT == 3) && ship[3] == 0 && twice == 0) {
				ev3_speaker_play_tone(261.63, 200);
				i = 3;
				ship2[j] = 4;
				j += 1;
				twice = 1;
			}else if(*pHT == 14 || *pHT == 17){
				ev3_speaker_play_tone(1174.66, 200);
				twice = 1;
				counter_s = 1;
			}else{

			}
		}while(twice == 0);
		ev3_stp_cyc(HT4);

		twice = 0;

		if(counter6 == 0){
			angle = (ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 ;
		}else{
			angle = (ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 + angle2 ;
		}

		if(counter_s == 1){
			if(angle < 100) {
				counter_d = 1;
			}else if(angle >= 100 && angle < 300){
				counter_d = 2;
			}else if(angle >= 300 && angle < 500){
				counter_d = 3;
			}else if(angle >= 600 && angle < 800){
				counter_d = 4;
			}else if(angle >= 800 && angle < 1100){
				counter_d = 5;
			}else{
				counter_d = 6;
			}
		}else{
			if(angle < 100) {
				ship[i] = 1;
				counter_d = 1;
			}else if(angle >= 100 && angle < 300){
				ship[i] = 2;
				counter_d = 2;
			}else if(angle >= 300 && angle < 500){
				ship[i] = 3;
				counter_d = 3;
			}else if(angle >= 600 && angle < 800){
				ship[i] = 4;
				counter_d = 4;
			}else if(angle >= 800 && angle < 1100){
				ship[i] = 5;
				counter_d = 5;
			}else{
				ship[i] = 6;
				counter_d = 6;
			}
		}

		if (ship[container[counter_a] - 1] == counter_d) {
			//よみこんだシップとcontainerが等しい場合、コントローラーとフードをシップに乗せ、出航させる
			//位置調節
			power = slow;
			counter6 = 1;

			if(counter_d == 1){
				ev3_stp_cyc(TRACE33);

				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow * -1);
				tslp_tsk(500);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				angle2 = 55;
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);

				ev3_motor_set_power(EV3_PORT_A, 10);
				ev3_motor_set_power(EV3_PORT_D, -10);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 55);//55

				turn = turn - 2;
				counter_e = 2;

			}else if(counter_d == 6){
				angle2 = angle2 + (ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 + 90;//105//90

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 90);//105//90

				ev3_stp_cyc(TRACE33);

				turn = turn + 2;
				counter_e = 1;

			}else{
				angle2 = angle2 + (ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 + 115;//105//90

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 115);//105//90

				ev3_stp_cyc(TRACE33);
			}

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(300);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

			ev3_motor_set_power(EV3_PORT_A, slows);
			ev3_motor_set_power(EV3_PORT_D, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			if(counter_e == 1){
				turn = turn - 2;
			}else if(counter_e == 2){
				turn = turn + 2;
			}

			counter_e = 0;

			//シップを出航させる 1
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			tslp_tsk(400);

			//フードとコントローラーをシップに乗せ
			ev3_motor_set_power(EV3_PORT_C, -10);

			//シップを出航させる 2
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			tslp_tsk(1100);//1600

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(400);

			counter_e = 0;
			if(counter_d != 6) {
				//船が最後でなければ
				counter_e = 0;
				if(counter_d == 1){
					//進む　壁回避
					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, slow * -1 + 5);
					ev3_motor_set_power(EV3_PORT_D, slow);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -158 * a);

					//アームを元に戻す
					ev3_motor_set_power(EV3_PORT_C, 80);

					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow - 5);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 158 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					ev3_motor_stop(EV3_PORT_C, true);
					tslp_tsk(200);
				}else if(counter_d == 6){
					//進む　壁回避
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow - 5);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 158 * a);

					//アームを元に戻す
					ev3_motor_set_power(EV3_PORT_C, 80);

					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, slow * -1 + 5);
					ev3_motor_set_power(EV3_PORT_D, slow);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -158 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					ev3_motor_stop(EV3_PORT_C, true);
					tslp_tsk(200);

				}else{
					//進む
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 155 * a);

					//アームを元に戻す
					ev3_motor_set_power(EV3_PORT_C, 80);

					//進む
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 185 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					ev3_motor_stop(EV3_PORT_C, true);
					tslp_tsk(200);
				}

				if(counter_d == 3){
					//方向転換
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn * a);

				}else{
					//方向転換
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= threshold);
				}

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				ev3_sta_cyc(TRACE33);

			}else{
				//船が3つめの場合
				counter_e = 1;
				//下ろしたアームが引っかからなくなるまで進む
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 55 * a);

			}

			power = slows;

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);

		}else if(counter_s == 1){
			//サプライズの船を押し出す
			//位置調節
			power = slow;
			counter6 = 1;

			if(counter_d == 1){
				ev3_stp_cyc(TRACE33);

				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow * -1);
				tslp_tsk(500);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				angle2 = 55;
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);

				ev3_motor_set_power(EV3_PORT_A, 10);
				ev3_motor_set_power(EV3_PORT_D, -10);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 55);//55

				turn = turn - 2;
				counter_e = 2;

			}else if(counter_d == 6){
				angle2 = angle2 + (ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 + 90;//105//90

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 90);//105//90

				ev3_stp_cyc(TRACE33);

				turn = turn + 2;
				counter_e = 1;

			}else{
				angle2 = angle2 + (ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 + 115;//105//90

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 115);//105//90

				ev3_stp_cyc(TRACE33);
			}

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(300);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

			ev3_motor_set_power(EV3_PORT_A, slows);
			ev3_motor_set_power(EV3_PORT_D, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			if(counter_e == 1){
				turn = turn - 2;
			}else if(counter_e == 2){
				turn = turn + 2;
			}

			counter_e = 0;

			//シップを出航させる 1
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			tslp_tsk(400);

			//シップを出航させる 2
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			tslp_tsk(1100);//1600

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			counter_e = 0;
			if(counter_d != 6) {
				//船が最後でなければ
				counter_e = 0;
				if(counter_d == 1){
					//進む　壁回避
					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, slow * -1 + 5);
					ev3_motor_set_power(EV3_PORT_D, slow);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -158 * a);

					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1 - 6);
					ev3_motor_set_power(EV3_PORT_D, slow);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 158 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

				}else if(counter_d == 6){
					//進む　壁回避
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow - 5);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 158 * a);

					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, slow * -1 + 5);
					ev3_motor_set_power(EV3_PORT_D, slow);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -158 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

				}else{
					//進む
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 155 * a);

					//進む
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 185 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);
				}

				if(counter_d == 3){
					//方向転換
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn * a);

				}else{
					//方向転換
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= threshold);
				}

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				ev3_sta_cyc(TRACE33);

			}else{
				//船が3つめの場合
				counter_e = 1;
				//下ろしたアームが引っかからなくなるまで進む
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 55 * a);

			}

			power = slows;

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);

		}
	}while(counter_d != 6);

	ev3_stp_cyc(TRACE33);

	//contianer[1]の特定
	container[1] = (ship2[0] + ship2[1] + ship2[2]) - (container[0] + container[2] + container[3]);
	yes_container[1] = container[1];

	//no_containerの特定
	for (i = 0; i <=3; i++) {
		if (container[i] == 0) {
			no_container = i;
		}
	}

	//no_colorの特定
	no_color = 10 - (ship2[0] + ship2[1] + ship2[2]);

	//シップを出航させた、完了の0
	ship[container[counter_a] - 1] = 0;
	container[counter_a] = 0;
	yes_container[counter_a] = 0;

	angle = (ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 + angle2;


	for (k = 0; k < 2 ; k++) {
		//どのcontaienrをとるかの特定 and 壁合わせ and 方向転換
		if (k == 0 && counter_e == 0) {
			//1回目 かつ 最初の船の作業が最後の船でない
			if (container[1] == 0) {
				counter_c = 2;
				//壁合わせ and 方向転換　(左側の壁へ)
				ev3_motor_reset_counts(EV3_PORT_D);
				ev3_motor_set_power(EV3_PORT_A, -20);
				ev3_motor_set_power(EV3_PORT_D, 60);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_D) <= 30 * a);

				ev3_motor_reset_counts(EV3_PORT_D);
				ev3_motor_set_power(EV3_PORT_A, fast);
				ev3_motor_set_power(EV3_PORT_D, fast * -1);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_D) <= 245 * (ship[ship2[2] - 1] - 2) - 25);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, -60);
				ev3_motor_set_power(EV3_PORT_D, 20);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= -30 * a);

				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow * -1);
				tslp_tsk(500);

				//少し進む
				ev3_motor_reset_counts(EV3_PORT_D);
				ev3_motor_set_power(EV3_PORT_A, 10);
				ev3_motor_set_power(EV3_PORT_D, -10);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_D) >= anfw * a);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

				ev3_motor_set_power(EV3_PORT_A, slows);
				ev3_motor_set_power(EV3_PORT_D, slows);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 + 5 * a);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);


			}else{
				counter_c = 1;
				//壁合わせ and 方向転換　(右側の壁へ)
				ev3_motor_reset_counts(EV3_PORT_D);
				ev3_motor_set_power(EV3_PORT_A, 20);
				ev3_motor_set_power(EV3_PORT_D, -60);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_D) >= -30 * a);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, fast * -1);
				ev3_motor_set_power(EV3_PORT_D, fast);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 245 * (6 - ship[ship2[2] - 1]) - 45);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, 60);
				ev3_motor_set_power(EV3_PORT_D, -20);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 30 * a);

				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				tslp_tsk(800);

				//少し下がる
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, -10);
				ev3_motor_set_power(EV3_PORT_D, 10);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= anbw * a);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				//方向転換
				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

				ev3_motor_set_power(EV3_PORT_A, slows);
				ev3_motor_set_power(EV3_PORT_D, slows);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

				ev3_motor_set_power(EV3_PORT_A, slow);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 - 5 * a);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

			}



  	}else{
			//2回目
			for (i = 0; i < 4; i++) {
				if (container[i] > 0) {
					counter_c = i;
					break;
				}
			}

			if (counter_c == 0 || counter_c == 1) {
				//壁合わせ and 方向転換 (右側の壁へ)
				if (counter_d == 6){
					if(counter_s != 1){
						//アームを元に戻す
						ev3_motor_set_power(EV3_PORT_C, 80);
					}

					//うねりながら進む
					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow - 20);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -185 * a);

					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1 + 20);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);

					ev3_motor_stop(EV3_PORT_C, true);

				}else{
					//曲がりながら進む　進向方向 左向き
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, 60);
					ev3_motor_set_power(EV3_PORT_D, -20);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 520 * a);//540

					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 240 * (5 - counter_d) * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow);
					tslp_tsk(800);

					//少し下がる
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, -10);
					ev3_motor_set_power(EV3_PORT_D, 10);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= anbw * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					if(counter_s != 1){
						//アームを元に戻す
						ev3_motor_set_power(EV3_PORT_C, 80);
					}

					//方向転換
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

					ev3_motor_set_power(EV3_PORT_A, slows);
					ev3_motor_set_power(EV3_PORT_D, slows);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

					ev3_motor_set_power(EV3_PORT_A, slow);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 - 5 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_C, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);
				}

			}else{
				//壁合わせ and 方向転換 (左側の壁へ)
				if (counter_d == 1){
					//アームを元に戻す
					ev3_motor_set_power(EV3_PORT_C, 80);

					//うねりながら進む
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1 + 20);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 185 * a);

					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow - 20);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -180 * a);

					ev3_motor_stop(EV3_PORT_C, true);

				}else{
					//曲がりながら進む　進向方向 右向き
					ev3_motor_reset_counts(EV3_PORT_D);
					ev3_motor_set_power(EV3_PORT_A, 20);
					ev3_motor_set_power(EV3_PORT_D, -60);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -520 * a);//540

					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 240 * (counter_d - 2) * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow);
					tslp_tsk(800);

					//少し下がる
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, -10);
					ev3_motor_set_power(EV3_PORT_D, 10);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= anbw * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					if(counter_s != 1){
						//アームを元に戻す
						ev3_motor_set_power(EV3_PORT_C, 80);
					}

					//方向転換
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow * -1);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= turn + 5 * a);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_C, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);
				}

			}

  	}

		counter_s = 0;

		switch (counter_c) {
			case 0:
				//ラインまで進む
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				if(ev3_color_sensor_get_reflect(EV3_PORT_3) >= threshold){
					//壁に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, 0);
					ev3_motor_set_power(EV3_PORT_D, slow);
					ev3_motor_reset_counts(EV3_PORT_D);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -20 * a);

				}else{
					//内側に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, 0);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);
				}

				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 35 * a);

				//container[0]のcatch
				//速く移動
				gain_rate = two_slow;
				power =  slows;
				ev3_sta_cyc(TRACE);

				//十字路まで進む
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_stp_cyc(TRACE);

				counter = 0;
				counter5 = 1;
				act_tsk(CATCH_TASK);
				slp_tsk();

				//containerとcontrollerの入れ替え
				if(container[0] == 1 || container[0] == 2) {
					//container[0]が赤、緑の場合
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);

					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 650 * a);

					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, slow);
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//方向転換
					curve = 2;
					act_tsk(CURVE_TASK);
					slp_tsk();

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -100);

					gain_rate = two_slow;
					power = slows;

					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

					power = fast;
					gain_rate = two_fast;

					ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

					//速く移動
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_reset_counts(EV3_PORT_D);
					do{
					}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -300 * a);

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

				}else{
					//container[0]が黄、青の場合
					ev3_motor_rotate(EV3_PORT_A, -90 * a, slow * -1, false);
					ev3_motor_rotate(EV3_PORT_D, 90 * a, slow * -1, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					act_tsk(LEFT_TASK);
					slp_tsk();

					//十字路まで進む
					gain_rate = two_slow;
					power = slows;
					ev3_sta_cyc(TRACE);

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -100);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

					power = fast;
					gain_rate = two_fast;

					ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

					//速く移動
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_reset_counts(EV3_PORT_D);
					do{
					}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -250 * a);

					//container[0]が黄ならゆっくり
					if (container[0] == 3){
						gain_rate = two_slow;
						power = slow;
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//container[2]とコントローラーの入れ替え
					counter = 0;
					act_tsk(CHANGE_YB_TASK);
					slp_tsk();

					//方向転換
					act_tsk(RIGHT_TASK);
					slp_tsk();
				}

				break;

			case 1:
				//ラインまで進む
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				if(ev3_color_sensor_get_reflect(EV3_PORT_3) >= threshold){
					//壁に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, 0);
					ev3_motor_set_power(EV3_PORT_D, slow);
					ev3_motor_reset_counts(EV3_PORT_D);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -20 * a);

				}else{
					//内側に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, 0);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);
				}

				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 35 * a);

				//container[1]のcatch
				//速く移動
				gain_rate = two_slow;
				power =  slows;
				ev3_sta_cyc(TRACE);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -280 * a);

				//十字路まで進む
				power = slow;
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_stp_cyc(TRACE);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				counter = 1;
				act_tsk(CATCH_TASK);
				slp_tsk();

				if(container[1] == 1 || container[1] == 2) {
					//container[1]が赤、緑の場合
					if(yes_container[3] == 0){
						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 25 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, 0);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 150 * a);

						ev3_sta_cyc(TRACE2);
						power = slows;
						gain_rate = one_slow;

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_reset_counts(EV3_PORT_D);
						do{
						}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

						power = fast;
						gain_rate = one_fast;

						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black || ev3_color_sensor_get_reflect(EV3_PORT_2) >= white);
						ev3_speaker_play_tone(1174.66, 200);
						ev3_stp_cyc(TRACE2);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, fast);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 500 * a);

						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, 0);
						ev3_motor_set_power(EV3_PORT_D, fast);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -200 * a);

						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//方向転換
						curve = 2;
						act_tsk(CURVE_TASK);
						slp_tsk();

						//Foodの回収
						gain_rate = two_slow;
						power = slows;

						ev3_sta_cyc(TRACE);

						ev3_motor_reset_counts(EV3_PORT_B);
						ev3_motor_set_power(EV3_PORT_B, -100);

						do{
						}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

						gain_rate = two_fast;
						power = fast;

						ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

						//十字路の手前まで速く行く
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);//0

					}else if(yes_container[2] == 0){
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, slows);

						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, fast);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 650 * a);

						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//方向転換
						curve = 2;
						act_tsk(CURVE_TASK);
						slp_tsk();

						//十字路まで進む
						gain_rate = two_slow;
						power = slows;
						ev3_sta_cyc(TRACE);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						gain_rate = two_fast;
						power = fast;

						//Foodの回収
						ev3_motor_reset_counts(EV3_PORT_B);
						ev3_motor_set_power(EV3_PORT_B, -100);

						do{
						}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

						gain_rate = two_fast;
						power = fast;

						ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

						//十字路をこえる(次の十字路の手前まで速く行く補正)
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 500 * a);//100

					}else{
						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -25 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, 0);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 150 * a);

						ev3_sta_cyc(TRACE2);
						power = slow;
						gain_rate = one_slow;

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_reset_counts(EV3_PORT_D);
						do{
						}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= white || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);
						ev3_stp_cyc(TRACE2);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, slows);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 30 * a);

						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, 0);
						ev3_motor_set_power(EV3_PORT_D, slows);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -185 * a);

						power = slows;
						gain_rate = two_slow;
						ev3_sta_cyc(TRACE);

						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						power = fast;
						gain_rate = two_fast;

						//速く移動
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);

						power = slows;
						gain_rate = two_slow;

						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_stp_cyc(TRACE);

						//方向転換
						curve = 2;
						act_tsk(CURVE_TASK);
						slp_tsk();

						power = slows;
						gain_rate = two_slow;

						ev3_sta_cyc(TRACE);

						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						power = fast;
						gain_rate = two_fast;

						//Foodの回収
						ev3_motor_reset_counts(EV3_PORT_B);
						ev3_motor_set_power(EV3_PORT_B, -100);

						do{
						}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

						gain_rate = two_fast;
						power = fast;

						ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

						//十字路の手前まで速く行く
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 350 * a);//0

					}

					//container[1]が赤ならゆっくり
					if (container[1] == 1){
						gain_rate = two_slow;
						power = slow;
					}

					//十字路まで進む
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

					gain_rate = two_slow;
					power = slows;
					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					gain_rate = two_fast;
					power = fast;

					if(container[1] == 2) {
						//container[1]が緑の場合、十字路まで進む
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(L字路の手前まで進む)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 330 * a);//280

					ev3_stp_cyc(TRACE);

					//L字路の向こうの白まで進む
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);//230

					//L字路をこえる 超えてるけど減速に
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 70 * a);

					//container[1]のシップの作業
					counter = 1;
					act_tsk(SHIP_LEFT_TASK);
					slp_tsk();

				}else{
					//container[1]が黄、青の場合
					ev3_motor_rotate(EV3_PORT_A, -200 * a, slow * -1, false);
					ev3_motor_rotate(EV3_PORT_D, 200 * a, slow * -1, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					act_tsk(LEFT_TASK);
					slp_tsk();

					//十字路まで進む
					gain_rate = two_slow;
					power = slows;
					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					gain_rate = two_fast;
					power = fast;
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -100);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

					ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

					//十字路をこえる(十字路の手前まで速く行く補正)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 500 * a);//100

					//container[1]が黄ならゆっくり
					if (container[1] == 3){
						gain_rate = two_slow;
						power = slow;
					}

					//十字路まで進む
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

					gain_rate = two_slow;
					power = slows;

					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					gain_rate = two_fast;
					power = fast;

					if(container[1] == 4) {
						//container[1]が青の場合、十字路まで進む
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(L字路の手前まで進む)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 330 * a);

					ev3_stp_cyc(TRACE);

					//L字路の向こうの白まで進む
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);


					//L字路をこえる 超えてるけど減速に
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 70 * a);

					//container[1]のシップの作業
					counter = 1;
					act_tsk(SHIP_RIGHT_TASK);
					slp_tsk();

				}

				break;

			case 2:
				//ラインまで進む
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				if(ev3_color_sensor_get_reflect(EV3_PORT_2) >= threshold){
					//壁に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, 0);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);
				}else{
					//内側に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, 0);
					ev3_motor_set_power(EV3_PORT_D, slow);
					ev3_motor_reset_counts(EV3_PORT_D);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -20 * a);
				}

				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 35 * a);

				//container[2]のcatch
				//速く移動
				gain_rate = two_slow;
				power =  slows;
				ev3_sta_cyc(TRACE);

				ev3_motor_reset_counts(EV3_PORT_A);
				ev3_motor_reset_counts(EV3_PORT_D);
				do{
				}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -280 * a);

				//十字路まで進む
				power = slow;
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				ev3_stp_cyc(TRACE);

				ev3_motor_stop(EV3_PORT_A, true);
				ev3_motor_stop(EV3_PORT_D, true);
				tslp_tsk(200);

				counter = 2;
				act_tsk(CATCH_TASK);
				slp_tsk();

				//containerとcontrollerの入れ替え
				if(container[2] == 3 || container[2] == 4) {
					//container[2]が黄、青の場合
					if(yes_container[0] == 0){
						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -25 * a);

						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, 0);
						ev3_motor_set_power(EV3_PORT_D, slows);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -150 * a);

						ev3_sta_cyc(TRACE3);
						power = slows;
						gain_rate = one_slow;

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_reset_counts(EV3_PORT_D);
						do{
						}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

						power = fast;
						gain_rate = one_fast;

						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= white || ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
						ev3_speaker_play_tone(1174.66, 200);
						ev3_stp_cyc(TRACE3);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, fast);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 500 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, 0);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);//180

						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//方向転換
						curve = 1;
						act_tsk(CURVE_TASK);
						slp_tsk();

						//Foodの回収
						gain_rate = two_slow;
						power = slows;

						ev3_sta_cyc(TRACE);

						ev3_motor_reset_counts(EV3_PORT_B);
						ev3_motor_set_power(EV3_PORT_B, -100);

						do{
						}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

						power = fast;
						gain_rate = two_fast;

						ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

						//十字路の手前まで速く行く
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);//0

					}else{
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, slows);

						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, fast);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 650 * a);

						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//方向転換
						curve = 1;
						act_tsk(CURVE_TASK);
						slp_tsk();

						//十字路まで進む
						gain_rate = two_slow;
						power = slows;
						ev3_sta_cyc(TRACE);

						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						gain_rate = two_fast;
						power = fast;
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//Foodの回収
						ev3_motor_reset_counts(EV3_PORT_B);
						ev3_motor_set_power(EV3_PORT_B, -100);

						do{
						}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

						ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

						//十字路をこえる(十字路の手前まで速く行く補正)
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 500 * a);//100

					}

					//container[2]が黄ならゆっくり
					if (container[2] == 3){
						gain_rate = two_slow;
						power = slow;
					}

					//十字路まで進む
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

					power = slows;
					gain_rate = two_slow;
					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					power = fast;
					gain_rate = two_fast;

					if(container[2] == 4) {
						//container[2]が青の場合、十字路まで進む
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(L字路の手前まで進む)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 330 * a);

					ev3_stp_cyc(TRACE);

					//L字路の向こうの白まで進む
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);

					//L字路をこえる 超えてるけど減速に
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 70 * a);

					//container[2]のシップの作業
					counter = 2;
					act_tsk(SHIP_RIGHT_TASK);
					slp_tsk();


				}else{
					//container[2]が赤、緑の場合
					ev3_motor_rotate(EV3_PORT_A, -90 * a, slow * -1, false);
					ev3_motor_rotate(EV3_PORT_D, 90 * a, slow * -1, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					act_tsk(RIGHT_TASK);
					slp_tsk();

					//十字路まで進む
					gain_rate = two_slow;
					power = slows;
					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					gain_rate = two_fast;
					power = fast;
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -100);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

					ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

					//十字路をこえる(十字路の手前まで速く行く補正)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 500 * a);//100


					//container[2]が赤ならゆっくり
					if (container[2] == 1){
						gain_rate = two_slow;
						power = slow;
					}

					//十字路まで進む
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

					power = slows;
					gain_rate = two_slow;
					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					power = fast;
					gain_rate = two_fast;
					if(container[2] == 2) {
						//container[2]が緑の場合、十字路まで進む
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(L字路の手前まで進む)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 330 * a);

					ev3_stp_cyc(TRACE);

					//L字路の向こうの白まで進む
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);

					//L字路をこえる 超えてるけど減速に
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 70 * a);

					//container[2]のシップの作業
					counter = 2;
					act_tsk(SHIP_LEFT_TASK);
					slp_tsk();

				}


				break;

			case 3:
				//ラインまで進む
				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				if(ev3_color_sensor_get_reflect(EV3_PORT_2) >= threshold){
					//壁に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, slow * -1);
					ev3_motor_set_power(EV3_PORT_D, 0);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);
				}else{
					//内側に向かってしまっている時
					ev3_motor_set_power(EV3_PORT_A, 0);
					ev3_motor_set_power(EV3_PORT_D, slow);
					ev3_motor_reset_counts(EV3_PORT_D);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_D) >= -20 * a);
				}

				ev3_motor_set_power(EV3_PORT_A, slow * -1);
				ev3_motor_set_power(EV3_PORT_D, slow);
				ev3_motor_reset_counts(EV3_PORT_A);
				do{
				}while(ev3_motor_get_counts(EV3_PORT_A) <= 35 * a);

				//container[3]のcatch
				//十字路まで進む
				gain_rate = two_fast;
				power = fast;
				ev3_sta_cyc(TRACE);

				do{
				}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
				ev3_speaker_play_tone(1174.66, 200);

				counter = 3;
				counter5 = 1;
				act_tsk(CATCH_TASK);
				slp_tsk();

				//containerとcontrollerの入れ替え
				if(container[3] == 3 || container[3] == 4) {
					//container[3]が黄、青の場合
					if(yes_container[0] == 0){
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, slows);
						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						ev3_motor_set_power(EV3_PORT_A, fast * -1);
						ev3_motor_set_power(EV3_PORT_D, fast);
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 650 * a);

						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						//方向転換
						curve = 1;
						act_tsk(CURVE_TASK);
						slp_tsk();

						//Foodの回収
						gain_rate = two_slow;
						power = slows;
						ev3_sta_cyc(TRACE);

						ev3_motor_reset_counts(EV3_PORT_B);
						ev3_motor_set_power(EV3_PORT_B, -100);

						do{
						}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

						gain_rate = two_fast;
						power = fast;

						ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

						//速く移動
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 300 * a);//0

					}else{
						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, slow * -1);
						ev3_motor_set_power(EV3_PORT_D, slow);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -25 * a);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, 0);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 150 * a);

						ev3_sta_cyc(TRACE2);
						power = slow;
						gain_rate = one_slow;

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_reset_counts(EV3_PORT_D);
						do{
						}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -200 * a);

						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= white || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);
						ev3_stp_cyc(TRACE2);

						ev3_motor_reset_counts(EV3_PORT_A);
						ev3_motor_set_power(EV3_PORT_A, slows * -1);
						ev3_motor_set_power(EV3_PORT_D, slows);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 30 * a);

						ev3_motor_reset_counts(EV3_PORT_D);
						ev3_motor_set_power(EV3_PORT_A, 0);
						ev3_motor_set_power(EV3_PORT_D, slows);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_D) >= -185 * a);

						power = slows;
						gain_rate = two_slow;
						ev3_sta_cyc(TRACE);

						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						power = fast;
						gain_rate = two_fast;

						//速く移動
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 200 * a);

						power = slows;
						gain_rate = two_slow;

						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_stp_cyc(TRACE);

						//方向転換
						curve = 1;
						act_tsk(CURVE_TASK);
						slp_tsk();

						power = slows;
						gain_rate = two_slow;

						ev3_sta_cyc(TRACE);

						//助走
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

						power = fast;
						gain_rate = two_fast;

						//Foodの回収
						ev3_motor_reset_counts(EV3_PORT_B);
						ev3_motor_set_power(EV3_PORT_B, -100);

						do{
						}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

						gain_rate = two_fast;
						power = fast;

						ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

						//十字路の手前まで速く行く
						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 350 * a);//0

					}

					//container[3]が黄ならゆっくり
					if (container[3] == 3){
						gain_rate = two_slow;
						power = slow;
					}

					//十字路まで進む
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

					gain_rate = two_slow;
					power = slows;
					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					gain_rate = two_fast;
					power = fast;
					if(container[3] == 4) {
						//container[3]が青の場合、十字路まで進む
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(L字路の手前まで進む)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 330 * a);

					ev3_stp_cyc(TRACE);

					//L字路の向こうの白まで進む
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);

					//L字路をこえる 超えてるけど減速に
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 70 * a);

					//container[3]のシップの作業
					counter = 3;
					act_tsk(SHIP_RIGHT_TASK);
					slp_tsk();

				}else{
					//container[3]が赤、緑の場合
					ev3_motor_rotate(EV3_PORT_A, -200 * a, slow * -1, false);
					ev3_motor_rotate(EV3_PORT_D, 200 * a, slow * -1, true);

					ev3_motor_stop(EV3_PORT_A, true);
					ev3_motor_stop(EV3_PORT_D, true);
					tslp_tsk(200);

					//方向転換
					act_tsk(RIGHT_TASK);
					slp_tsk();

					//Foodの回収
					ev3_motor_reset_counts(EV3_PORT_B);
					ev3_motor_set_power(EV3_PORT_B, -100);

					gain_rate = two_slow;
					power = slows;
					ev3_sta_cyc(TRACE);

					do{
					}while(ev3_motor_get_counts(EV3_PORT_B) >= arm);

					power = fast;
					gain_rate = two_fast;

					ev3_motor_rotate(EV3_PORT_B, 370, 80, false);

					//速く移動
					ev3_motor_reset_counts(EV3_PORT_A);
					ev3_motor_reset_counts(EV3_PORT_D);
					do{
					}while((ev3_motor_get_counts(EV3_PORT_A) * -1 + ev3_motor_get_counts(EV3_PORT_D)) / 2 >= -350 * a);


					//container[3]が赤ならゆっくり
					if (container[3] == 1){
						gain_rate = two_slow;
						power = slow;
					}

					//十字路まで進む
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

					gain_rate = two_slow;
					power = slows;
					ev3_sta_cyc(TRACE);

					//助走
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

					gain_rate = two_fast;
					power = fast;
					if(container[3] == 2) {
						//container[2]が緑の場合、十字路まで進む
						do{
						}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
						ev3_speaker_play_tone(1174.66, 200);

						ev3_motor_reset_counts(EV3_PORT_A);
						do{
						}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);
					}

					//十字路まで進む
					do{
					}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
					ev3_speaker_play_tone(1174.66, 200);

					//十字路をこえる(L字路の手前まで進む)
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 280 * a);

					ev3_stp_cyc(TRACE);

					//L字路の向こうの白まで進む
					ev3_motor_set_power(EV3_PORT_A, fast * -1);
					ev3_motor_set_power(EV3_PORT_D, fast);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 230 * a);

					//L字路をこえる　超えてるけど減速に
					ev3_motor_set_power(EV3_PORT_A, slows * -1);
					ev3_motor_set_power(EV3_PORT_D, slows);
					ev3_motor_reset_counts(EV3_PORT_A);
					do{
					}while(ev3_motor_get_counts(EV3_PORT_A) <= 70 * a);

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

	if (counter_d >= 4) {
		counter_a = 2;
		//壁合わせ and 方向転換 (右側の壁へ)
		if (counter_d == 6){
			//アームを元に戻す
			ev3_motor_set_power(EV3_PORT_C, 80);

			//うねりながら進む
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow - 20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -185 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow * -1 + 20);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 180 * a);

			ev3_motor_stop(EV3_PORT_C, true);

		}else{
			//曲がりながら進む　進向方向 左向き
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 60);
			ev3_motor_set_power(EV3_PORT_D, -20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 540 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, fast * -1);
			ev3_motor_set_power(EV3_PORT_D, fast);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 245 * (5 - counter_d) * a);

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow);
			tslp_tsk(500);

			//少し下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -10);
			ev3_motor_set_power(EV3_PORT_D, 10);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= anbw * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//アームを元に戻す
			ev3_motor_set_power(EV3_PORT_C, 80);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 150) * -1 * a);

			ev3_motor_set_power(EV3_PORT_A, slows);
			ev3_motor_set_power(EV3_PORT_D, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= (turn - 100) * -1 * a);

			ev3_motor_set_power(EV3_PORT_A, slow);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) >= turn * -1 * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_C, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);
		}

	}else{
		counter_a = 1;
		//壁合わせ and 方向転換 (左側の壁へ)
		if (counter_d == 1){
			//アームを元に戻す
			ev3_motor_set_power(EV3_PORT_C, 80);

			//うねりながら進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow * -1 + 20);
			ev3_motor_set_power(EV3_PORT_D, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 185 * a);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow - 20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -180 * a);

			ev3_motor_stop(EV3_PORT_C, true);

		}else{
			//曲がりながら進む　進向方向 右向き
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, -60);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -540 * a);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, fast * -1);
			ev3_motor_set_power(EV3_PORT_D, fast);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 245 * (counter_d - 2) * a);

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow);
			tslp_tsk(500);

			//少し下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, -10);
			ev3_motor_set_power(EV3_PORT_D, 10);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= anbw * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//アームを元に戻す
			ev3_motor_set_power(EV3_PORT_C, 80);

			//方向転換
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 150 * a);

			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			ev3_motor_set_power(EV3_PORT_D, slows * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= turn - 100 * a);

			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= turn * a);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_C, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);
		}

	}

	//スタートエリアに戻る
	if (counter_a == 1) {
		//ラインまで進む
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		if(ev3_color_sensor_get_reflect(EV3_PORT_2) >= threshold){
			//壁に向かってしまっている時
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, 0);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);
		}else{
			//内側に向かってしまっている時
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, slow);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -20 * a);
		}

		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 35 * a);

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
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);

		//十字路まで進む
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 450 * a);

		gain_rate = two_slow;
		power = slow;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//方向転換
		curve = 2;
		act_tsk(CURVE_TASK);
		slp_tsk();

	}else{
		//ラインまで進む
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		if(ev3_color_sensor_get_reflect(EV3_PORT_3) >= threshold){
			//壁に向かってしまっている時
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, slow);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) >= -20 * a);

		}else{
			//内側に向かってしまっている時
			ev3_motor_set_power(EV3_PORT_A, slow * -1);
			ev3_motor_set_power(EV3_PORT_D, 0);
			ev3_motor_reset_counts(EV3_PORT_A);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);
		}

		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow);
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 35 * a);

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
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);

		//十字路まで進む
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		//十字路をこえる
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 100 * a);

		//十字路まで進む
		ev3_motor_reset_counts(EV3_PORT_A);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 450 * a);

		gain_rate = two_slow;
		power = slow;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//方向転換
		curve = 1;
		act_tsk(CURVE_TASK);
		slp_tsk();

	}

	//ゴールエリアまで進む
	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE);

	//助走
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 90 * a);

	gain_rate = two_fast;
	power = fast;

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 40 || ev3_color_sensor_get_reflect(EV3_PORT_3) >= 40);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE);

	//ゴールエリアにinする
	if(counter_a == 1){
		ev3_motor_rotate(EV3_PORT_A, 250 * a, slow * -1 + 5, false);
		ev3_motor_rotate(EV3_PORT_D, -250 * a, slow * -1, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}else{
		ev3_motor_rotate(EV3_PORT_A, 250 * a, slow * -1 + 5, false);
		ev3_motor_rotate(EV3_PORT_D, -250 * a, slow * -1, true);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}

	ext_tsk();
}
