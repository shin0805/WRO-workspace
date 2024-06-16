#include "ev3api.h"
#include "app.h"
#include "stdlib.h"
#include "math.h"

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

//PID制御の変数
double kp, ki, kd, gain_rate;
double delta_t = 0.004;
int diff[2];
int integral, p, i, d, sensor2, sensor3, motor, count, power, sensor1, pid;

//HTのよみこみの変数
uint8_t *pHT = 0;
uint8_t ht = 0;

rgb_raw_t *pRGB1 = 0;
rgb_raw_t rgb1;

rgb_raw_t *pRGB4 = 0;
rgb_raw_t rgb4;

uint8_t h_rgb;

//速度の変数
int fast = -80;
int slow = -20;
int slows = -30;

//gainの変数
double two_fast = 2.14;//2.14
double two_slow = 2.14;//2.14
double one_fast = 4.0;//4.0
double one_slow = 1.9;//1.9

//方向転換の変数
int right = 180;//185
int left = 180;

int right_curve = 510;//545
int left_curve = 515;//530

//センサーの閾値
int black = 20;
int white = 40;
int threshold = 30;

//三角関数
double x;
double y;
//char color_buffer1[30];

//identifierの変数
int identifier[4];  //カラーごとの場所
int identifier2[4]; //場所ごとのカラー
int twice;

//Device
int device[2] = {8, 8}; //白の位置

//ループ
int k ;

//方向転換
int pturn;

//カウンター
int direction;

//方向転換前に進む値
int angle = 150;

//RGB値
int red;
int green;
int blue;

char color_buffer1[30];

int u = 100;

int counter = 0;

int a;

//方向転換の変数
int curve;



void trace_task(intptr_t unused) {
	kp = 0.34 * gain_rate;//0.36
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
	kp = 0.33 * gain_rate;
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
	kp = 0.33 * gain_rate;
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
	kp = 0.3 * gain_rate;//0.3
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
	kp = 0.3 * gain_rate;//0.3
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


void arm_task(intptr_t unused) {
	x = ev3_motor_get_counts(EV3_PORT_C) * 0.8 / 3 / 180 * 3.14;
	y = 120 * (1 - cos(x));

	/*
	sprintf(color_buffer1, "NUMBER-%f-" ,y);
	ev3_lcd_draw_string(color_buffer1, 0,10);
	*/

	kp = 0.3 * gain_rate;//0.3
	ki = kp / 0.3;
	kd = kp * 0.075 * 0;

	diff[0] = diff[1];

	diff[1] = ev3_motor_get_counts(EV3_PORT_D) - y;

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;

	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;

	pid = p + i + d;

	if(pid >= 0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, motor * -1);
		ev3_motor_set_power(EV3_PORT_D, motor);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, motor * -1);
	}
}


void right_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, pturn * -1);
		ev3_motor_set_power(EV3_PORT_D, pturn * -1);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -50);

	do{
		ev3_motor_set_power(EV3_PORT_A, pturn * -1 - 10);
		ev3_motor_set_power(EV3_PORT_D, pturn * -1 - 10);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= 30);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, pturn * -1 + 10);
		ev3_motor_set_power(EV3_PORT_D, pturn * -1 + 10);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -70);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void left_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, pturn);
		ev3_motor_set_power(EV3_PORT_D, pturn);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 50);

	do{
		ev3_motor_set_power(EV3_PORT_A, pturn + 10);
		ev3_motor_set_power(EV3_PORT_D, pturn + 10);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= 30);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, pturn - 10);
		ev3_motor_set_power(EV3_PORT_D, pturn - 10);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 70);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
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
			}while(ev3_motor_get_counts(EV3_PORT_A) >= -50 * a);//50

			ev3_motor_set_power(EV3_PORT_A, -30 * a);
			ev3_motor_set_power(EV3_PORT_D, -30 * a);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -10 * a);
			ev3_motor_set_power(EV3_PORT_D, -10 * a);
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
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 50 * a);//50

			ev3_motor_set_power(EV3_PORT_A, 30 * a);
			ev3_motor_set_power(EV3_PORT_D, 30 * a);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 10 * a);
			ev3_motor_set_power(EV3_PORT_D, 10 * a);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 20 * a);

			wup_tsk(MAIN_TASK);

			break;

		case 3://内側への右
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 190);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 110);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 50);

			wup_tsk(MAIN_TASK);

			break;

		case 4://内側への左
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 190);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 110);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 50);

			wup_tsk(MAIN_TASK);

			break;

		case 5://外側への右 (両方黒から)
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 290);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= threshold);

			wup_tsk(MAIN_TASK);

			break;


		case 6://外側への左 (両方黒から)
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 290);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= threshold);

			wup_tsk(MAIN_TASK);

			break;


		default:
			break;
	}

}


void rotation_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, pturn * -1);//30
		ev3_motor_set_power(EV3_PORT_D, pturn * -1);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -300);

	do{
		ev3_motor_set_power(EV3_PORT_A, pturn * -1 - 10);//40
		ev3_motor_set_power(EV3_PORT_D, pturn * -1 - 10);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, pturn * -1 + 10);//30
		ev3_motor_set_power(EV3_PORT_D, pturn * -1 + 10);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -70);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	wup_tsk(MAIN_TASK);
}


void ht1_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_color(EV3_PORT_1, pHT);
}


void ht1rgb_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_1, pRGB1);
	if(h_rgb < rgb1.r){
		h_rgb = rgb1.r;
	}
}


void ht4black_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);

	if(rgb4.g > 0 && rgb4.g <= 70){
		wup_tsk(MAIN_TASK);
	}

}


void ht4color_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);

	if(rgb4.b > 0 && rgb4.b <= 230){
		wup_tsk(MAIN_TASK);
	}

}


void twist_task(intptr_t unused) {
	if(direction == 1){
		//左向きに回す
		ev3_motor_reset_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, -40);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 150);
		ev3_motor_stop(EV3_PORT_B, true);

	}else if(direction == 2){
		//前向きに回す

	}else if(direction == 3){
		//右向きに回す
		ev3_motor_reset_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, 40);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) <= 150);
		ev3_motor_stop(EV3_PORT_B, true);

	}else if(direction == 4){
		//後ろ向きに回す
		ev3_motor_reset_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, 40);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) <= 300);
		ev3_motor_stop(EV3_PORT_B, true);

	}
}


void expansion_task(intptr_t unused) {
	//リンクを前に出す
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 50);

	ev3_motor_set_power(EV3_PORT_C, -5);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 150);

	ev3_motor_stop(EV3_PORT_C, true);
}


void drop_task(intptr_t unused) {
	power = -20;

	if(direction == 1){
		//左向きに下ろす

		//下がる微調節
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 10);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(500);

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 290);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//避ける
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 60);
		ev3_motor_stop(EV3_PORT_A, true);

		//リンクをたたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 260);

		ev3_motor_set_power(EV3_PORT_C, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 560);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//元へ回転
		ev3_motor_reset_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, 50);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) <= 150);
		ev3_motor_stop(EV3_PORT_B, true);

		//戻る
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 60);

		ev3_motor_stop(EV3_PORT_C, true);
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//下がりきる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 140);
		ev3_stp_cyc(BACK);

	}else if(direction == 2){
		//前向きに下ろす

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 290);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 60);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//リンクをたたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 260);

		ev3_motor_set_power(EV3_PORT_C, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 560);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//下がりきる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 65);
		ev3_stp_cyc(BACK);

	}else if(direction == 3){
		//右向きに下ろす

		//下がる微調節
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 10);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(500);

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 290);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//避ける
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slow * -1);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) <= 60);
		ev3_motor_stop(EV3_PORT_D, true);

		//リンクをたたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 260);

		ev3_motor_set_power(EV3_PORT_C, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 560);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//元へ回転
		ev3_motor_reset_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, -50);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 150);
		ev3_motor_stop(EV3_PORT_B, true);

		//戻る
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 60);

		ev3_motor_stop(EV3_PORT_C, true);
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//下がりきる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 140);
		ev3_stp_cyc(BACK);

	}else if(direction == 4){
		//後ろ向きに下ろす

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 290);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 15);
		ev3_stp_cyc(FOWARD);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//リンクを少したたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 100);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//元へ回転しつつ避ける
		ev3_motor_reset_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, -50);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 150);
		ev3_motor_stop(EV3_PORT_B, true);

		//リンクを少したたむ
		ev3_motor_set_power(EV3_PORT_C, 60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 260);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//元へ回転しきる
		ev3_motor_reset_counts(EV3_PORT_B);
		ev3_motor_set_power(EV3_PORT_B, -50);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 150);
		ev3_motor_stop(EV3_PORT_B, true);

		//下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 45);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//リンクをたたみきる
		ev3_motor_set_power(EV3_PORT_C, 20);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 560);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);

		//下がりきる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 120);
		ev3_stp_cyc(BACK);

	}


	wup_tsk(MAIN_TASK);
}


void foward_task(intptr_t unused) {
	kp = 0.85;//0.36
	ki = 8.5;//0.1
	kd = 0.068;//0.075

	diff[0] = diff[1];
	sensor2 = ev3_motor_get_counts(EV3_PORT_D) * -1;
	sensor3 = ev3_motor_get_counts(EV3_PORT_A);

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


void back_task(intptr_t unused) {
	kp = 0.85;//0.36
	ki = 8.5;//0.1
	kd = 0.068;//0.075

	diff[0] = diff[1];
	sensor2 = ev3_motor_get_counts(EV3_PORT_D);
	sensor3 = ev3_motor_get_counts(EV3_PORT_A) * -1;

	diff[1] = sensor2 - sensor3;

	integral += (diff[0] + diff[1]) / 2.0 * delta_t;

	p = kp * diff[1];
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / delta_t;

	pid = p + i + d;

	if(pid >= 0) {
		motor = power + pid;
		ev3_motor_set_power(EV3_PORT_A, power);
		ev3_motor_set_power(EV3_PORT_D, motor * -1);
	}else{
		motor = power - pid;
		ev3_motor_set_power(EV3_PORT_A, motor);
		ev3_motor_set_power(EV3_PORT_D, power * -1);
	}

}


void main_task(intptr_t unused) {
	ev3_lcd_set_font (EV3_FONT_MEDIUM);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);	//右カラーセンサー
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);	//左カラーセンサー
	ev3_sensor_config(EV3_PORT_1, HT_NXT_COLOR_SENSOR);	//右前HTセンサー
	ev3_sensor_config(EV3_PORT_4, HT_NXT_COLOR_SENSOR);	//左前HTセンサー
	ev3_motor_config(EV3_PORT_A, MEDIUM_MOTOR);	//右タイヤ
	ev3_motor_config(EV3_PORT_D, MEDIUM_MOTOR);	//左タイヤ
	ev3_motor_config(EV3_PORT_B, MEDIUM_MOTOR);	//アーム
	ev3_motor_config(EV3_PORT_C, MEDIUM_MOTOR);	//リンク

	ev3_speaker_play_tone(1174.66, 200);

	do{
	}while(ev3_button_is_pressed(ENTER_BUTTON) == false);

	tslp_tsk(500);

	ev3_color_sensor_get_reflect(EV3_PORT_2);
	ev3_color_sensor_get_reflect(EV3_PORT_3);

	ev3_lcd_set_font(EV3_FONT_MEDIUM);

	//HT色読み さす
	pHT = &ht;

	pRGB1 = &rgb1;

	pRGB4 = &rgb4;

	/*実験
	while(1){
		curve = 4;
		act_tsk(CURVE_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);


		do{
		}while(ev3_button_is_pressed(ENTER_BUTTON) == false);

		tslp_tsk(500);

	}
	*/



	//うねる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 265);


	ev3_motor_stop(EV3_PORT_A, true);
	/*
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	*/

	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 245);

	/*
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	*/

	/*ラインにのせる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) + ev3_motor_get_counts(EV3_PORT_D) * -1) / 2 <= 0);
	*/

	//identifierをよむ
	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE22);

	twice = 0;
	k = 0;

	do{
		ht_nxt_color_sensor_measure_color(EV3_PORT_1, pHT);
		*pHT = 0;
		do{
			ev3_sta_cyc(HT1);
			if((*pHT == 1 || *pHT == 7 || *pHT == 8 || *pHT == 9 || *pHT == 10) && identifier[0] == 0 && twice == 0) {
				ev3_speaker_play_tone(1975.53, 200);
				identifier[0] = k + 1;
				identifier2[k] = 1;
				k += 1;
				twice = 1;
			}else if((*pHT == 4 || *pHT == 13) && identifier[1] == 0 && twice == 0) {
				ev3_speaker_play_tone(1396.91, 200);
				identifier[1] = k + 1;
				identifier2[k] = 2;
				k += 1;
				twice = 1;
			}else if((*pHT == 5 || *pHT == 6) && identifier[2] == 0 && twice == 0) {
				ev3_speaker_play_tone(880.00, 200);
				identifier[2] = k + 1;
				identifier2[k] = 3;
				k += 1;
				twice = 1;
			}else if((*pHT == 2 ||*pHT == 3 || *pHT == 12) && identifier[3] == 0 && twice == 0) {
				ev3_speaker_play_tone(261.63, 200);
				identifier[3] = k + 1;
				identifier2[k] = 4;
				k += 1;
				twice = 1;
			}else{

			}
		}while(twice == 0);

		ev3_stp_cyc(HT1);

		twice = 0;

	}while(identifier2[3] == 0);

	//ラインまで進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 300);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_stp_cyc(TRACE22);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_speaker_play_tone(1174.66, 200);


	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);
	ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//左に曲がる
	pturn = 30;
	act_tsk(LEFT_TASK);
	slp_tsk();

	//North Cableに向かう
	gain_rate = one_fast;
	power = fast;
	ev3_sta_cyc(TRACE2);

	for (k = 0; k < 5; k++) {
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		if(k != 4){
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);
		}

	}

	//進みながらアームを下ろす
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);

	gain_rate = two_slow;
	power = slows;

	/*
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, 80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) <= 180);
	ev3_motor_stop(EV3_PORT_C, true);
	*/

	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 840);
	ev3_motor_stop(EV3_PORT_B, true);

	/*
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, -80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);
	ev3_motor_stop(EV3_PORT_C, true);
	*/

	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 600);

	ev3_stp_cyc(TRACE2);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 70);
	ev3_motor_stop(EV3_PORT_B, true);

	//うねる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 290);


	ev3_motor_stop(EV3_PORT_A, true);
	/*
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	*/

	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 290);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//ケーブルまで進む

	gain_rate = two_slow;
	power = slow;
	ev3_sta_cyc(TRACE);


	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	//ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 120);

	ev3_stp_cyc(TRACE);

	//ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//ケーブルを拾う
	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, -20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 70);

	//ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, -80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 840);
	ev3_motor_stop(EV3_PORT_B, true);

	//方向転換
	pturn = 30;
	act_tsk(ROTATION_TASK);
	slp_tsk();

	//うねる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 295);


	ev3_motor_stop(EV3_PORT_A, true);
	/*
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	*/

	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 290);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);



	//方向転換の改良

	//進む
	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE3);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= white);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE3);

	//右にcurve
	curve = 5;
	act_tsk(CURVE_TASK);
	slp_tsk();

	//方向転換の改良


	//Device 0行2列目の読み取り
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_1, pRGB1);
	rgb1.r = 0;
	h_rgb = 0;
	ev3_sta_cyc(HT1RGB);

	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE22);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	//ラインを超える
	ev3_stp_cyc(TRACE22);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
	ev3_stp_cyc(FOWARD);

	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE22);

	if(h_rgb > 100){
		//Device 0行0列目の回収
		device[0] = 2;

		ev3_speaker_play_tone(1975.53, 200);

		//ラインまで進む
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		//ラインを超える
		ev3_stp_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE22);

		//ラインまで進む
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		//前に進む
		ev3_stp_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 185);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//右を向く
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//ラインまで下がる
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		pturn = 20;
		act_tsk(ROTATION_TASK);
		slp_tsk();

		//向きの決定 青
		if(identifier[3] == 1){
			direction = 4;
		}else if(identifier[3] == 2){
			direction = 1;
		}else if(identifier[3] == 3){
			direction = 2;
		}else if(identifier[3] == 4){
			direction = 3;
		}

		//リンクを前に出す
		act_tsk(EXPANSION_TASK);

		//回す
		act_tsk(TWIST_TASK);

		//青に入る
		//色まで進む
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.b = 250;

		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);
		tslp_tsk(500);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);



		//方向転換を改良

		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

		ev3_stp_cyc(FOWARD);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//左に曲がる
		act_tsk(LEFT_TASK);
		slp_tsk();

		//方向転換を改良



		//ラインまで進む
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//少し下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 15);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//右に曲がり向く
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}else{
		//Device 0行2列目の回収

		//前に進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 65);

		gain_rate = two_slow;
		power = slow;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 120);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//右を向く
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		pturn = 20;
		act_tsk(ROTATION_TASK);
		slp_tsk();

		//向きの決定 赤
		if(identifier[0] == 1){
			direction = 4;
		}else if(identifier[0] == 2){
			direction = 1;
		}else if(identifier[0] == 3){
			direction = 2;
		}else if(identifier[0] == 4){
			direction = 3;
		}

		//リンクを前に出す
		act_tsk(EXPANSION_TASK);

		//回す
		act_tsk(TWIST_TASK);

		//赤に入る
		//色まで進む
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.b = 250;

		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);
		tslp_tsk(500);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		//方向転換の改良

		//右にcurve
		curve = 5;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

		//Device 0行1列目の読み取り
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_1, pRGB1);
		rgb1.r = 0;
		h_rgb = 0;
		ev3_sta_cyc(HT1RGB);

		//ラインまで進む
		gain_rate = one_slow;
		power = slow;
		ev3_sta_cyc(TRACE22);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//少し下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 15);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		if(h_rgb > 100){
			device[0] = 1;
			ev3_speaker_play_tone(1975.53, 200);
		}

		ev3_stp_cyc(HT1RGB);

		//左に曲がり向く
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slows * -1);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}
	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE);

	//姿勢を整える
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 200);

	//色まで進む
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.b = 250;

	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_stp_cyc(TRACE);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_speaker_play_tone(1174.66, 200);

	//Cableを下ろす
	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 80);

	//リンクを出す
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 900);
	ev3_motor_stop(EV3_PORT_B, true);

	tslp_tsk(100);

	ev3_motor_set_power(EV3_PORT_B, -80);

	//リンクをたたむ
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) >= 0);
	ev3_motor_stop(EV3_PORT_B, true);

	tslp_tsk(200);

	//方向転換
	pturn = 30;
	act_tsk(ROTATION_TASK);
	slp_tsk();

	if(device[0] == 2){
		//Device 0行1列目の回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換の改良

		//右にcurve
		curve = 3;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

		//ラインまで進む
		gain_rate = one_slow;
		power = slow;
		ev3_sta_cyc(TRACE33);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE33);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//少し下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 15);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//右に曲がり向く
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//向きの決定 赤
		if(identifier[0] == 1){
			direction = 4;
		}else if(identifier[0] == 2){
			direction = 1;
		}else if(identifier[0] == 3){
			direction = 2;
		}else if(identifier[0] == 4){
			direction = 3;
		}

		//リンクを前に出す
		act_tsk(EXPANSION_TASK);

		//回す
		act_tsk(TWIST_TASK);

		//色まで進む
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.b = 250;

		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);
		tslp_tsk(500);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換
		pturn = 30;
		act_tsk(ROTATION_TASK);
		slp_tsk();

		//ラインまで進む
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE);

		//方向転換の改良

		//左にcurve
		curve = 4;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

		//ラインまで進む
		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE22);

		for (k = 0; k < 2; k++) {
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE22);

			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slow;
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
			ev3_stp_cyc(FOWARD);

			gain_rate = one_slow;
			power = slows;
			ev3_sta_cyc(TRACE22);

		}

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);
		ev3_stp_cyc(FOWARD);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}else{
		if(device[0] == 1){
			//Device 0行0列目の回収
			//ラインまで進む
			gain_rate = two_slow;
			power = slow;
			ev3_sta_cyc(TRACE);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);

			//方向転換の改良

			//左にcurve
			curve = 4;
			act_tsk(CURVE_TASK);
			slp_tsk();

			//方向転換の改良

			//ラインまで進む
			gain_rate = one_slow;
			power = slows;
			ev3_sta_cyc(TRACE22);
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			//前に進む
			ev3_stp_cyc(TRACE22);
			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slow;
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
			ev3_stp_cyc(FOWARD);

			gain_rate = one_slow;
			power = slows;
			ev3_sta_cyc(TRACE22);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 185);

			ev3_stp_cyc(TRACE22);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//右を向く
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//Deviceを回収
			gain_rate = two_slow;
			power = slow;
			ev3_sta_cyc(TRACE);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

			ev3_motor_reset_counts(EV3_PORT_C);
			ev3_motor_set_power(EV3_PORT_C, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

			ev3_motor_stop(EV3_PORT_C, true);

			ev3_stp_cyc(TRACE);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//ラインまで下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slows;
			ev3_sta_cyc(BACK);
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			pturn = 20;
			act_tsk(ROTATION_TASK);
			slp_tsk();
		}else{
			//Device 0行1列目の回収
			//Deviceを回収
			gain_rate = two_slow;
			power = slow;
			ev3_sta_cyc(TRACE);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

			ev3_motor_reset_counts(EV3_PORT_C);
			ev3_motor_set_power(EV3_PORT_C, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

			ev3_motor_stop(EV3_PORT_C, true);

			ev3_stp_cyc(TRACE);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//ラインまで下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slows;
			ev3_sta_cyc(BACK);
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換の改良

			//左にcurve
			curve = 4;
			act_tsk(CURVE_TASK);
			slp_tsk();

			//方向転換の改良

			//ラインまで進む
			gain_rate = one_slow;
			power = slow;
			ev3_sta_cyc(TRACE22);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE22);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slow;
			ev3_sta_cyc(BACK);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 15);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//左に曲がり向く
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

		}

		//青に入る

		//向きの決定 青
		if(identifier[3] == 1){
			direction = 4;
		}else if(identifier[3] == 2){
			direction = 1;
		}else if(identifier[3] == 3){
			direction = 2;
		}else if(identifier[3] == 4){
			direction = 3;
		}

		//リンクを前に出す
		act_tsk(EXPANSION_TASK);

		//回す
		act_tsk(TWIST_TASK);

		//色まで進む
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.b = 250;

		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);
		tslp_tsk(500);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		if(device[0] != 1 && device[0] != 2){
			device[0] = 0;

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//方向転換
			pturn = 30;
			act_tsk(ROTATION_TASK);
			slp_tsk();

			//ラインまで進む
			gain_rate = two_slow;
			power = slow;
			ev3_sta_cyc(TRACE);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//少し下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slow;
			ev3_sta_cyc(BACK);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 15);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//左に曲がり向く
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slows * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 500);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

		}else{
			//ラインまで下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slows;
			ev3_sta_cyc(BACK);
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slow;
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

			ev3_stp_cyc(FOWARD);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(200);

			//右に曲がる
			act_tsk(RIGHT_TASK);
			slp_tsk();
		}


		//ラインまで進む
		gain_rate = two_slow;
		power = slows;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		gain_rate = two_slow;
		power = slow;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

	}

	//右に曲がる
	pturn = 30;
	act_tsk(RIGHT_TASK);
	slp_tsk();

	//進みながらアームを下ろす
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);

	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE);

	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 840);
	ev3_motor_stop(EV3_PORT_B, true);

	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 500);

	ev3_stp_cyc(TRACE);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 70);
	ev3_motor_stop(EV3_PORT_B, true);

	//うねる
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 295);


	ev3_motor_stop(EV3_PORT_D, true);
	/*
	ev3_motor_stop(EV3_PORT_A, true);
	tslp_tsk(200);
	*/

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 295);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//Device 1行0列目の読み取り
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_1, pRGB1);
	rgb1.r = 0;
	h_rgb = 0;
	ev3_sta_cyc(HT1RGB);

	//ケーブルまで進む

	gain_rate = two_slow;
	power = slow;
	ev3_sta_cyc(TRACE);


	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	//power = slow;
	//ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 120);

	ev3_stp_cyc(TRACE);

	//ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	if(h_rgb > 100){
		device[1] = 0;
		ev3_speaker_play_tone(1975.53, 200);
	}

	ev3_stp_cyc(HT1RGB);

	//ケーブルを拾う
	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, -20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 70);

	//ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, -80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 840);
	ev3_motor_stop(EV3_PORT_B, true);

	//ラインまで下がる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slows;
	ev3_sta_cyc(BACK);
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);
	ev3_stp_cyc(BACK);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);


	//方向転換の改良

	//右にcurve
	curve = 3;
	act_tsk(CURVE_TASK);
	slp_tsk();

	//方向転換の改良

	if(device[1] == 0){
		//Device 1行1列目の回収
		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE33);

		for (k = 0; k < 2; k++) {
			//ラインまで進む
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE33);

			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slow;
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
			ev3_stp_cyc(FOWARD);

			gain_rate = one_slow;
			power = slows;
			ev3_sta_cyc(TRACE33);

		}

		gain_rate = two_slow;
		power = slow;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 185);

		ev3_stp_cyc(TRACE33);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//左を向く
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slows * -1);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);


		//方向転換の改良

		//右にcurve
		curve = 3;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

		//ラインまで進む
		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE33);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE33);

		//前に進む
		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

	}else{
		//Device 1行0列目の回収
		//ラインまで進む
		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE33);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE33);

		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

		gain_rate = one_slow;
		power = slow;
		ev3_sta_cyc(TRACE33);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 185);

		ev3_stp_cyc(TRACE33);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//左を向く
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slows * -1);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換の改良

		//右にcurve
		curve = 3;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE33);

		for (k = 0; k < 2; k++) {
			//ラインまで進む
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(TRACE33);

			//前に進む
			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slow;
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
			ev3_stp_cyc(FOWARD);
		}

		ev3_stp_cyc(TRACE33);

	}

	//ケーブルのところからでているラインを避ける
	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE33);

	//ラインまで進む
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE33);

	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 280);

	ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//右に曲がる
	pturn = 20;
	act_tsk(RIGHT_TASK);
	slp_tsk();

	//緑色に入る
	//進む
	gain_rate = one_slow;
	power = -40;
	ev3_sta_cyc(TRACE3);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);

	for (k = 0; k < 4; k++) {
		if(k == 3){
			gain_rate = one_slow;
			power = slow;
		}

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= white);
		ev3_speaker_play_tone(1174.66, 200);

		if(k == 3){
			gain_rate = one_slow;
			power = slow;
		}

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);

	}

	ev3_stp_cyc(TRACE3);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//右に曲がる
	pturn = 20;
	act_tsk(RIGHT_TASK);
	slp_tsk();

	//向きの決定 緑
	if(identifier[1] == 1){
		direction = 3;
	}else if(identifier[1] == 2){
		direction = 4;
	}else if(identifier[1] == 3){
		direction = 1;
	}else if(identifier[1] == 4){
		direction = 2;
	}

	//リンクを前に出す
	act_tsk(EXPANSION_TASK);

	//回す
	act_tsk(TWIST_TASK);

	//色まで進む
	gain_rate = two_slow;
	power = slow;
	ev3_sta_cyc(TRACE);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.b = 250;

	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(500);

	//Deviceを下ろす
	act_tsk(DROP_TASK);
	slp_tsk();

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//左に曲がる
	act_tsk(LEFT_TASK);
	slp_tsk();

	//ラインまで進む
	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE3);
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= white);
	ev3_speaker_play_tone(1174.66, 200);

	gain_rate = two_slow;
	power = slow;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);

	ev3_stp_cyc(TRACE3);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//右に曲がる
	pturn = 30;
	act_tsk(RIGHT_TASK);
	slp_tsk();

	//進む
	gain_rate = one_fast;
	power = slows;
	ev3_sta_cyc(TRACE3);

	for (k = 0; k < 2; k++) {
		if(k == 1){
			gain_rate = one_slow;
			power = slows;
		}

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= white);
		ev3_speaker_play_tone(1174.66, 200);

		if(k == 1){
			gain_rate = one_slow;
			power = slow;
		}

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);

	}

	ev3_stp_cyc(TRACE3);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//右に曲がる
	pturn = 30;
	act_tsk(RIGHT_TASK);
	slp_tsk();

	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE);

	//姿勢を整える
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 200);

	//色まで進む
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.b = 250;

	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_stp_cyc(TRACE);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_speaker_play_tone(1174.66, 200);

	//Cableを下ろす
	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 80);

	//リンクを出す
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 900);
	ev3_motor_stop(EV3_PORT_B, true);

	tslp_tsk(100);

	ev3_motor_set_power(EV3_PORT_B, -80);

	//リンクをたたむ
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) >= 0);
	ev3_motor_stop(EV3_PORT_B, true);

	tslp_tsk(200);

	//方向転換
	pturn = 30;
	act_tsk(ROTATION_TASK);
	slp_tsk();

	//4つ目のDeviceに向かう
	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE);

	//前に進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//方向転換
	pturn = 30;
	act_tsk(LEFT_TASK);
	slp_tsk();

	//2つラインを進む
	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE2);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	//ラインを超える
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);
	ev3_stp_cyc(TRACE2);

	//前に進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//左に曲がる
	pturn = 30;
	act_tsk(LEFT_TASK);
	slp_tsk();

	//4つ目のDeviceに向かう
	gain_rate = one_fast;
	power = fast;
	ev3_sta_cyc(TRACE2);

	for (k = 0; k < 5; k++) {
		if(k == 4){
			gain_rate = one_slow;
			power = slows;
		}

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		if(k == 4){
			break;
		}

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	}

	ev3_stp_cyc(TRACE2);

	//方向転換の改良

	//左にcurve
	curve = 4;
	act_tsk(CURVE_TASK);
	slp_tsk();

	//方向転換の改良

	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE22);

	/*
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slows;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);

	ev3_stp_cyc(FOWARD);
	*/

	//ev3_stp_cyc(TRACE3);

	//Device 1行2列目の読み取り
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_1, pRGB1);
	rgb1.r = 0;
	h_rgb = 0;
	ev3_sta_cyc(HT1RGB);


	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE22);

	//ラインを超える
	//前に進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);

	ev3_stp_cyc(FOWARD);

	ev3_stp_cyc(HT1RGB);

	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE22);

	if(h_rgb > 100 && device[1] != 0){
		//Device 1行1列目の回収
		device[0] = 2;

		ev3_speaker_play_tone(1975.53, 200);

		//ラインまで進む
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE22);

		//前に進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);

		ev3_stp_cyc(FOWARD);

		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 185);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//右を向く
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);


		//方向転換の改良

		//左にcurve
		curve = 4;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

		//ラインまで進む
		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE22);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(TRACE22);

		//進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slow;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

	}else{
		//Device 1行2列目の回収

		if(device[1] != 0){
			device[1] = 1;
		}

		//前に進む
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 65);

		gain_rate = one_slow;
		power = slow;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 120);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//右を向く
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slows);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 500);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 250);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(200);

		//方向転換の改良

		//左にcurve
		curve = 4;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

		for (k = 0; k < 2; k++) {
			//ラインまで進む
			gain_rate = one_slow;
			power = slows;
			ev3_sta_cyc(TRACE22);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		}

		ev3_stp_cyc(TRACE22);

	}

	//ラインまで進む
	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE22);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE22);

	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//左に曲がる
	pturn = 20;
	act_tsk(LEFT_TASK);
	slp_tsk();

	//黄色に入る
	//進む
	gain_rate = one_fast;
	power = fast;
	ev3_sta_cyc(TRACE2);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);

	for (k = 0; k < 4; k++) {
		if(k == 3){
			gain_rate = one_slow;
			power = slows;
		}

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		if(k == 3){
			gain_rate = one_slow;
			power = slow;
		}

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	}

	ev3_stp_cyc(TRACE2);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//右に曲がる
	pturn = 20;
	act_tsk(LEFT_TASK);
	slp_tsk();

	//向きの決定 黄色
	if(identifier[2] == 1){
		direction = 1;
	}else if(identifier[2] == 2){
		direction = 2;
	}else if(identifier[2] == 3){
		direction = 3;
	}else if(identifier[2] == 4){
		direction = 4;
	}

	//リンクを前に出す
	act_tsk(EXPANSION_TASK);

	//回す
	act_tsk(TWIST_TASK);

	//色まで進む
	gain_rate = two_slow;
	power = slow;
	ev3_sta_cyc(TRACE);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.b = 250;

	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_stp_cyc(TRACE);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_speaker_play_tone(1174.66, 200);
	tslp_tsk(500);

	//Deviceを下ろす
	act_tsk(DROP_TASK);
	slp_tsk();

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	//右に曲がる
	act_tsk(RIGHT_TASK);
	slp_tsk();

	gain_rate = one_slow;
	power = slows;
	ev3_sta_cyc(TRACE2);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	gain_rate = one_slow;
	power = slows;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(TRACE2);

	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slows;
	ev3_sta_cyc(FOWARD);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 175);

	ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);

	//うねる
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, slows);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 185);

	ev3_motor_stop(EV3_PORT_D, true);
	/*
	ev3_motor_stop(EV3_PORT_A, true);
	tslp_tsk(200);
	*/

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, slows * -1);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 185);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);

	ext_tsk();

}
