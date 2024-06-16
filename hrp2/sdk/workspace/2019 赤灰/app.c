#include "ev3api.h"
#include "app.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"

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
int slows = -40;
int slow = -20;

int start = -30;
int stop = -10;

//gainの変数
double two_fast = 2.14;//2.14
double two_slow = 1.34;//2.14
double one_fast = 4.0;//4.0
double one_slow = 1.9;//1.9

//方向転換の変数
int right = 180;//185
int left = 180;

int right_curve = 510;//545
int left_curve = 515;//530

//センサーの閾値
int black = 20;//20
int white = 45;//40
int threshold2 = 30;
int threshold3 = 25;

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
int angle = 140;//150

//RGB値
int red;
int green;
int blue;

char color_buffer1[30];

int u = 100;

//ループのカウンター
int while1;
int while2;
int while3;

//方向転換の変数
int curve;

int counter;

//カラーストップのRGBの選択
int choice;

//止まる時の時間
int timer = 100;

//start 30 20度刻み
//stop 10 30度刻み


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

	diff[1] = sensor2 - threshold2;

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

	diff[1] = sensor3 - threshold3;

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

	diff[1] = sensor2 - threshold2;

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

	diff[1] = sensor3 - threshold3;

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
		ev3_motor_set_power(EV3_PORT_A, pturn * -1);
		ev3_motor_set_power(EV3_PORT_D, pturn * -1);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -140);

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
	tslp_tsk(timer);

	wup_tsk(MAIN_TASK);
}


void left_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, pturn);
		ev3_motor_set_power(EV3_PORT_D, pturn);
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 140);

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
	tslp_tsk(timer);

	wup_tsk(MAIN_TASK);
}


void curve_task(intptr_t unused) {
	switch (curve) {
		case 1://真ん中への右
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 240);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 90);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 50);

			wup_tsk(MAIN_TASK);

			break;

		case 2://真ん中への左
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 240);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 90);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 50);

			wup_tsk(MAIN_TASK);

			break;

		case 3://内側への右
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 180);//190

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
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 180);//190

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
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 260);//290

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= threshold2);

			wup_tsk(MAIN_TASK);

			break;


		case 6://外側への左 (両方黒から)
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 260);//290

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_3) >= threshold3);

			wup_tsk(MAIN_TASK);

			break;

		case 7://真ん中への右 (両方黒から)
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, 0);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 170);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -30);
			ev3_motor_set_power(EV3_PORT_D, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 120);

			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_A, -20);
			ev3_motor_set_power(EV3_PORT_D, -20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 50);

			wup_tsk(MAIN_TASK);

			break;

		case 8://真ん中への左 (両方黒から)
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 0);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 170);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 30);
			ev3_motor_set_power(EV3_PORT_D, 30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 120);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, 20);
			ev3_motor_set_power(EV3_PORT_D, 20);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) <= 50);

			wup_tsk(MAIN_TASK);

			break;

		case 9://右を向く
			ev3_motor_stop(EV3_PORT_D, true);
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_set_power(EV3_PORT_A, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 30);

			ev3_motor_set_power(EV3_PORT_A, fast);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 450);

			ev3_motor_set_power(EV3_PORT_A, slow);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 500);

			wup_tsk(MAIN_TASK);

			break;

		case 10://左を向く
			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) <= 30);

			ev3_motor_set_power(EV3_PORT_D, fast * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) <= 450);

			ev3_motor_set_power(EV3_PORT_D, slow * -1);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_D) <= 500);

			wup_tsk(MAIN_TASK);
			break;


		default:
			break;
	}

}


void rotation_task(intptr_t unused) {
	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, -30);//pturn * -1);//30
		ev3_motor_set_power(EV3_PORT_D, -30);//pturn * -1);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -300);

	do{
		ev3_motor_set_power(EV3_PORT_A, -40);//pturn * -1 - 10);//40
		ev3_motor_set_power(EV3_PORT_D, -40);//pturn * -1 - 10);
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black);

	ev3_motor_reset_counts(EV3_PORT_A);
	do{
		ev3_motor_set_power(EV3_PORT_A, -20);//pturn * -1 + 10);//30
		ev3_motor_set_power(EV3_PORT_D, -20);//pturn * -1 + 10);
	}while(ev3_motor_get_counts(EV3_PORT_A) >= -70);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

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


void ht1color_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_1, pRGB1);
}


void ht4black_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);

	if(rgb4.g > 0 && rgb4.g <= 70){
		wup_tsk(MAIN_TASK);
	}

}


void ht4color_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);

	if(choice == 1){//赤
		if(rgb4.b > 0 && rgb4.b <= 230){
			wup_tsk(MAIN_TASK);
		}
	}else if(choice == 2){//緑
		if(rgb4.r > 0 && rgb4.r <= 170){
			wup_tsk(MAIN_TASK);
		}
	}else if(choice == 3){//黄
		if(rgb4.b > 0 && rgb4.b <= 230){//170
			wup_tsk(MAIN_TASK);
		}
	}else if(choice == 4){//青
		if(rgb4.r > 0 && rgb4.r <= 170){//170
			wup_tsk(MAIN_TASK);
		}
	}


}


void ht4red_task(intptr_t unused) {
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);

	if(rgb4.r > 0 && rgb4.r <= 230){
		wup_tsk(MAIN_TASK);
	}

}


void drop_task(intptr_t unused) {
	if(direction == 1){
		//左向きに下ろす
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		/*下がる微調節
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 10);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(500);
		*/

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 90);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 350);

		ev3_motor_stop(EV3_PORT_C, true);

		//避ける
		//押し入れる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow);
		ev3_motor_set_power(EV3_PORT_D, slow);

		//ev3_motor_reset_counts(EV3_PORT_B);
		//ev3_motor_set_power(EV3_PORT_B, -50);
		ev3_motor_rotate(EV3_PORT_B, -10, 50, false);
		//do{
		//}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 10);
		//ev3_motor_stop(EV3_PORT_B, true);

		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) * -1 <= 30);
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);

		//リンクをたたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 365);//315


		//ev3_motor_set_power(EV3_PORT_C, 20);
		ev3_motor_rotate(EV3_PORT_C, 250, 20, false);//300

		//元へ回転
		ev3_motor_rotate(EV3_PORT_B, 160, 50, false);
		/*
		ev3_motor_set_power(EV3_PORT_B, 50);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) <= 0);
		ev3_motor_stop(EV3_PORT_B, true);
		*/

		/*
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 615);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);
		*/

		//戻る
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_set_power(EV3_PORT_A, slow * -1);
		ev3_motor_set_power(EV3_PORT_D, slow * -1);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_A) <= 30);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ev3_motor_rotate(EV3_PORT_C, 315, 20, false);//300

		//下がりきる
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 130);
		ev3_stp_cyc(BACK);

	}else if(direction == 2){
		//前向きに下ろす
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 90);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 350);

		ev3_motor_stop(EV3_PORT_C, true);

		//下がる
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 60);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//リンクをたたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 315);

		//ev3_motor_set_power(EV3_PORT_C, 20);
		ev3_motor_rotate(EV3_PORT_C, 300, 20, false);
		/*
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 615);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);
		*/

		//下がりきる
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 65);
		ev3_stp_cyc(BACK);

	}else if(direction == 3){
		//右向きに下ろす
		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		/*下がる微調節
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 10);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(500);
		*/

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 90);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 350);

		ev3_motor_stop(EV3_PORT_C, true);

		//避ける
		//押し入れる
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slow * -1);
		ev3_motor_set_power(EV3_PORT_A, slow * -1);

		//ev3_motor_reset_counts(EV3_PORT_B);
		//ev3_motor_set_power(EV3_PORT_B, 50);
		ev3_motor_rotate(EV3_PORT_B, 10, 50, false);
		//do{
		//}while(ev3_motor_get_counts(EV3_PORT_B) <= 10);
		//ev3_motor_stop(EV3_PORT_B, true);

		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) <= 30);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_motor_stop(EV3_PORT_A, true);

		//リンクをたたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 365);//315

		//ev3_motor_set_power(EV3_PORT_C, 20);
		ev3_motor_rotate(EV3_PORT_C, 250, 20, false);//300

		//元へ回転
		ev3_motor_rotate(EV3_PORT_B, -160, 50, false);
		/*
		ev3_motor_set_power(EV3_PORT_B, -50);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) >= 0);
		ev3_motor_stop(EV3_PORT_B, true);
		*/

		/*
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 615);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);
		*/

		//戻る
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_motor_set_power(EV3_PORT_D, slow);
		ev3_motor_set_power(EV3_PORT_A, slow);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 30);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//下がりきる
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 130);
		ev3_stp_cyc(BACK);

	}else if(direction == 4){
		//後ろ向きに下ろす
		//進む微調節
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(TRACE);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 5);
		ev3_stp_cyc(TRACE);


		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		//リンクをおろしきる
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 90);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 350);

		ev3_motor_stop(EV3_PORT_C, true);

		//進む
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 15);
		ev3_stp_cyc(FOWARD);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//リンクを少したたむ
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, 80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 145);

		ev3_motor_stop(EV3_PORT_C, true);

		//元へ回転しつつ避ける
		ev3_motor_reset_counts(EV3_PORT_B);
		//ev3_motor_set_power(EV3_PORT_B, -50);
		ev3_motor_rotate(EV3_PORT_B, -150, 50, false);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 150);
		//ev3_motor_stop(EV3_PORT_B, true);

		//リンクを少したたむ
		ev3_motor_set_power(EV3_PORT_C, 60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 345);

		ev3_motor_stop(EV3_PORT_C, true);

		//元へ回転しきる
		ev3_motor_reset_counts(EV3_PORT_B);
		//ev3_motor_set_power(EV3_PORT_B, -50);
		ev3_motor_rotate(EV3_PORT_B, -150, 50, false);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 150);
		//ev3_motor_stop(EV3_PORT_B, true);

		//下がる
		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 45);

		//リンクをたたみきる
		ev3_motor_rotate(EV3_PORT_C, 270, 20, false);
		//ev3_motor_set_power(EV3_PORT_C, 20);
		/*
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) <= 615);

		ev3_motor_stop(EV3_PORT_C, true);
		tslp_tsk(200);
		*/

		//下がりきる
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 165);
		ev3_stp_cyc(BACK);

	}


	wup_tsk(MAIN_TASK);
}


void foward_task(intptr_t unused) {
	/*
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
	*/

	ev3_motor_set_power(EV3_PORT_A, power * -1);
	ev3_motor_set_power(EV3_PORT_D, power);

}


void back_task(intptr_t unused) {
	/*
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
	*/

	ev3_motor_set_power(EV3_PORT_A, power);
	ev3_motor_set_power(EV3_PORT_D, power * -1);

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
		//ev3_motor_stop(EV3_PORT_A, true);
		//ev3_motor_stop(EV3_PORT_D, true);
		//tslp_tsk(500);

		//Cableを下ろす
		ev3_motor_reset_counts(EV3_PORT_B);
		//ev3_motor_set_power(EV3_PORT_B, 80);
		ev3_motor_rotate(EV3_PORT_B, 1000, 80, false);

		//リンクを出す
		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);

		ev3_motor_set_power(EV3_PORT_C, -80);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);
		ev3_motor_stop(EV3_PORT_C, true);

		do{
		}while(ev3_motor_get_counts(EV3_PORT_B) <= 1000);//1000
		//ev3_motor_stop(EV3_PORT_B, true);

		//リンクをたたむ
		ev3_motor_rotate(EV3_PORT_C, 180, 20, false);

		ev3_motor_rotate(EV3_PORT_B, -1000, 80, false);
		tslp_tsk(100);


		do{
		}while(ev3_button_is_pressed(ENTER_BUTTON) == false);

		tslp_tsk(500);
	}
	*/


	ev3_motor_set_power(EV3_PORT_C, 50);

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

	ev3_motor_rotate(EV3_PORT_C, -200, 30, false);//190

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
			}else if((*pHT == 4 /*|| *pHT == 13*/) && identifier[1] == 0 && twice == 0) {
				ev3_speaker_play_tone(1396.91, 200);
				identifier[1] = k + 1;
				identifier2[k] = 2;
				k += 1;
				twice = 1;
			}else if((/**pHT == 5 || */*pHT == 6) && identifier[2] == 0 && twice == 0) {
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

		/*
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
		*/

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

	counter = 0;
	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_stp_cyc(TRACE22);
	ev3_speaker_play_tone(1174.66, 200);
	//tslp_tsk(1000000);

	//左に曲がる
	curve = 2;
	act_tsk(CURVE_TASK);
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
	//ev3_motor_set_power(EV3_PORT_B, 80);
	ev3_motor_rotate(EV3_PORT_B, 910, 80, false);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 910);
	//ev3_motor_stop(EV3_PORT_B, true);

	/*
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, -80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);
	ev3_motor_stop(EV3_PORT_C, true);
	*/

	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 510);//600//540

	ev3_stp_cyc(TRACE2);


	//ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	//tslp_tsk(200);


	/*
	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 70);
	ev3_motor_stop(EV3_PORT_B, true);
	*/

	//うねる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, 30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 165);//290//300

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, -30);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 140);//125

	ev3_motor_stop(EV3_PORT_A, true);

	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 165);//285///295

	//ケーブルまで進む
	gain_rate = two_slow;
	//power = -30;
	power = slow;
	ev3_sta_cyc(TRACE);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 80);

	//power = -20;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 110);

	//ev3_sta_cyc(FOWARD);

	power = -15;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 180);//170

	ev3_stp_cyc(TRACE);

	//ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	//ケーブルを拾う
	ev3_motor_reset_counts(EV3_PORT_B);
	//ev3_motor_set_power(EV3_PORT_B, -80);
	ev3_motor_rotate(EV3_PORT_B, -140, 80, false);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 140);
	//ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_rotate(EV3_PORT_B, -770, 80, false);//767

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

	power = -50;
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 530);

	power = slow;
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.g = 250;

	ev3_sta_cyc(HT4BLACK);
	slp_tsk();

	ev3_stp_cyc(HT4BLACK);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_stp_cyc(BACK);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	//左にcurve
	curve = 4;
	act_tsk(CURVE_TASK);
	slp_tsk();

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

	if(h_rgb > 100){
		gain_rate = one_slow;
		power = -50;
		ev3_sta_cyc(TRACE22);

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
		power = -50;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

		gain_rate = one_slow;
		power = -50;
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
		power = slow;//slows
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

		gain_rate = one_slow;
		power = slow;
		ev3_sta_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 35);//185

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		pturn = 30;
		act_tsk(RIGHT_TASK);
		slp_tsk();

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 130);//220

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ラインまで下がる
		ev3_sta_cyc(BACK);

		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

		power = slow;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//方向転換
		pturn = 20;
		act_tsk(ROTATION_TASK);
		slp_tsk();

		ev3_motor_rotate(EV3_PORT_C, -60, 30, false);

		//向きの決定 青
		ev3_motor_reset_counts(EV3_PORT_B);
		if(identifier[3] == 1){
			direction = 4;
			ev3_motor_rotate(EV3_PORT_B, 300, 20, false);
		}else if(identifier[3] == 2){
			direction = 1;
			ev3_motor_rotate(EV3_PORT_B, -150, 20, false);
		}else if(identifier[3] == 3){
			direction = 2;
		}else if(identifier[3] == 4){
			direction = 3;
			ev3_motor_rotate(EV3_PORT_B, 150, 20, false);
		}

		//青に入る(1)
		//色まで進む
		gain_rate = two_slow;
		power = -30;
		ev3_sta_cyc(TRACE);
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 90);

		power = -20;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 120);

		power = -15;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.r = 250;
		choice = 4;
		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//右を向く
		curve = 9;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 100);

		power = slow;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		//左にcurve
		curve = 8;
		act_tsk(CURVE_TASK);
		slp_tsk();

		gain_rate = two_slow;
		power = slows;
		ev3_sta_cyc(TRACE);

		//姿勢を整える
		gain_rate = two_slow;
		power = slows;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 170);

		power = -20;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 200);

		power = -10;

	}else{
		//Device 0行2列目の回収
		gain_rate = one_slow;
		power = slow;
		ev3_sta_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 35);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		pturn = 30;
		act_tsk(RIGHT_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 130);//250

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ラインまで下がる
		ev3_sta_cyc(BACK);
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

		power = slow;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//方向転換
		pturn = 20;
		act_tsk(ROTATION_TASK);
		slp_tsk();

		ev3_motor_rotate(EV3_PORT_C, -60, 30, false);

		//向きの決定 赤
		ev3_motor_reset_counts(EV3_PORT_B);
		if(identifier[0] == 1){
			direction = 4;
			ev3_motor_rotate(EV3_PORT_B, 300, 20, false);
		}else if(identifier[0] == 2){
			direction = 1;
			ev3_motor_rotate(EV3_PORT_B, -150, 20, false);
		}else if(identifier[0] == 3){
			direction = 2;
		}else if(identifier[0] == 4){
			direction = 3;
			ev3_motor_rotate(EV3_PORT_B, 150, 20, false);
		}

		//赤に入る(1)
		//色まで進む
		gain_rate = two_slow;
		power = -30;
		ev3_sta_cyc(TRACE);
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 120);//90

		power = -20;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 150);//120

		power = -10;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.b = 250;
		choice = 1;
		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 200);

		power = slow;
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

		ev3_stp_cyc(TRACE22);

		ev3_speaker_play_tone(1174.66, 200);

		//左に曲がる
		curve = 2;
		act_tsk(CURVE_TASK);
		slp_tsk();

		gain_rate = two_slow;
		power = slows;
		ev3_sta_cyc(TRACE);

		if(h_rgb > 100){
			device[0] = 1;
			ev3_speaker_play_tone(1975.53, 200);
		}

		ev3_stp_cyc(HT1RGB);

		//姿勢を整える
		gain_rate = two_slow;
		power = slows;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 230);

		power = -20;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 260);

		power = -15;

	}


	//色まで進む
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.b = 250;
	choice = 1;
	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_stp_cyc(TRACE);

	/*
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, power * -1);
	ev3_motor_set_power(EV3_PORT_D, power);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 25);
	*/

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_speaker_play_tone(1174.66, 200);

	/*
	ev3_motor_rotate(EV3_PORT_A, -20, 30, false);
	ev3_motor_rotate(EV3_PORT_D, 20, 30, false);
	*/

	//Cableを下ろす
	ev3_motor_reset_counts(EV3_PORT_B);
	//ev3_motor_set_power(EV3_PORT_B, 80);
	ev3_motor_rotate(EV3_PORT_B, 1000, 80, false);

	//リンクを出す
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);

	ev3_motor_set_power(EV3_PORT_C, -80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 1000);//1000
	//ev3_motor_stop(EV3_PORT_B, true);

	//リンクをたたむ
	ev3_motor_rotate(EV3_PORT_C, 180, 20, false);

	ev3_motor_rotate(EV3_PORT_B, -1000, 80, false);
	tslp_tsk(100);

	/*
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) <= 50);

	ev3_motor_set_power(EV3_PORT_B, -80);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) >= 0);
	ev3_motor_stop(EV3_PORT_B, true);
	*/

	if(device[0] == 2){
		//方向転換
		pturn = 30;
		act_tsk(ROTATION_TASK);
		slp_tsk();

		//Device 0行1列目の回収
		gain_rate = two_slow;
		power = slows;
		ev3_sta_cyc(TRACE);

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		power = slow;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 220);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -30);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ラインまで下がる
		ev3_sta_cyc(BACK);
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

		power = slow;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

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

		//右に曲がる
		curve = 1;
		act_tsk(CURVE_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		ev3_motor_rotate(EV3_PORT_C, -60, 30, false);

		//向きの決定 赤
		ev3_motor_reset_counts(EV3_PORT_B);
		if(identifier[0] == 1){
			direction = 4;
			ev3_motor_rotate(EV3_PORT_B, 300, 20, false);
		}else if(identifier[0] == 2){
			direction = 1;
			ev3_motor_rotate(EV3_PORT_B, -150, 20, false);
		}else if(identifier[0] == 3){
			direction = 2;
		}else if(identifier[0] == 4){
			direction = 3;
			ev3_motor_rotate(EV3_PORT_B, 150, 20, false);
		}

		//赤に入る(2)
		//色まで進む
		gain_rate = two_slow;
		power = -30;
		ev3_sta_cyc(TRACE);
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 170);

		power = -20;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 200);

		power = -10;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.b = 250;
		choice = 1;
		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//左を向く
		curve = 10;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 200);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 100);

		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 200);

		power = slow;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		//左にcurve
		curve = 8;
		act_tsk(CURVE_TASK);
		slp_tsk();

		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE3);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 40);

	}else{
		if(device[0] == 1){
			//Device 0行0列目の回収
			//左を向く
			curve = 10;
			act_tsk(CURVE_TASK);
			slp_tsk();

			//ラインまで下がる
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			power = slows;
			ev3_sta_cyc(BACK);

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 200);

			power = slow;
			do{
			}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			ev3_speaker_play_tone(1174.66, 200);

			//左にcurve
			curve = 8;
			act_tsk(CURVE_TASK);
			slp_tsk();

			gain_rate = two_slow;
			power = slows;
			ev3_sta_cyc(TRACE);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			power = slow;

			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 220);

			ev3_motor_reset_counts(EV3_PORT_C);
			ev3_motor_set_power(EV3_PORT_C, -30);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

			ev3_motor_stop(EV3_PORT_C, true);

			ev3_stp_cyc(TRACE);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(timer);

			//ラインまで下がる
			ev3_sta_cyc(BACK);
			power = slows;
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

			power = slow;
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(timer);

			//方向転換
			pturn = 20;
			act_tsk(ROTATION_TASK);
			slp_tsk();
		}else{
			//Device 0行1列目の回収
			//方向転換
			pturn = 30;
			act_tsk(ROTATION_TASK);
			slp_tsk();

			//Deviceを回収
			gain_rate = two_slow;
			power = slows;
			ev3_sta_cyc(TRACE);

			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			power = slow;
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 220);

			ev3_motor_reset_counts(EV3_PORT_C);
			ev3_motor_set_power(EV3_PORT_C, -60);
			do{
			}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

			ev3_motor_stop(EV3_PORT_C, true);

			ev3_stp_cyc(TRACE);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(timer);

			//ラインまで下がる
			ev3_sta_cyc(BACK);
			power = slows;
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

			power = slow;
			ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
			rgb4.g = 250;

			ev3_sta_cyc(HT4BLACK);
			slp_tsk();

			ev3_stp_cyc(HT4BLACK);
			ev3_speaker_play_tone(1174.66, 200);

			ev3_stp_cyc(BACK);

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(timer);

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

			//左に曲がる
			curve = 2;
			act_tsk(CURVE_TASK);
			slp_tsk();

			ev3_motor_stop(EV3_PORT_A, true);
			ev3_motor_stop(EV3_PORT_D, true);
			tslp_tsk(timer);

		}

		ev3_motor_rotate(EV3_PORT_C, -60, 30, false);

		//向きの決定 青
		ev3_motor_reset_counts(EV3_PORT_B);
		if(identifier[3] == 1){
			direction = 4;
			ev3_motor_rotate(EV3_PORT_B, 300, 20, false);
		}else if(identifier[3] == 2){
			direction = 1;
			ev3_motor_rotate(EV3_PORT_B, -150, 20, false);
		}else if(identifier[3] == 3){
			direction = 2;
		}else if(identifier[3] == 4){
			direction = 3;
			ev3_motor_rotate(EV3_PORT_B, 150, 20, false);
		}

		//青に入る(2)
		//色まで進む
		gain_rate = two_slow;
		power = -30;
		ev3_sta_cyc(TRACE);
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 170);

		power = -20;
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 200);

		power = -15;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.r = 250;
		choice = 4;
		ev3_sta_cyc(HT4COLOR);
		slp_tsk();

		ev3_stp_cyc(HT4COLOR);
		ev3_stp_cyc(TRACE);

		//Deviceを下ろす
		act_tsk(DROP_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//左を向く
		curve = 10;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//ラインまで下がる
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		power = slows;
		ev3_sta_cyc(BACK);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 200);

		power = slow;
		do{
		}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		ev3_speaker_play_tone(1174.66, 200);

		//左にcurve
		curve = 8;
		act_tsk(CURVE_TASK);
		slp_tsk();

		gain_rate = one_slow;
		power = slows;
		ev3_sta_cyc(TRACE3);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 40);

	}

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);

	ev3_motor_reset_counts(EV3_PORT_B);
	//ev3_motor_set_power(EV3_PORT_B, 80);
	ev3_motor_rotate(EV3_PORT_B, 910, 80, false);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 910);
	//ev3_motor_stop(EV3_PORT_B, true);

	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 350);//600//400

	ev3_stp_cyc(TRACE3);

	ev3_motor_stop(EV3_PORT_A, true);
	//ev3_motor_stop(EV3_PORT_D, true);
	//tslp_tsk(200);

	/*
	ev3_motor_reset_counts(EV3_PORT_B);
	ev3_motor_set_power(EV3_PORT_B, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 70);
	ev3_motor_stop(EV3_PORT_B, true);
	*/

	//うねる
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 165);//295

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, 30);
	ev3_motor_set_power(EV3_PORT_D, -30);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 140);//125

	ev3_motor_stop(EV3_PORT_D, true);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_set_power(EV3_PORT_A, 30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 165);

	//Device 1行0列目の読み取り
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_1, pRGB1);
	rgb1.r = 0;
	h_rgb = 0;
	ev3_sta_cyc(HT1RGB);

	//ケーブルまで進む
	gain_rate = two_slow;
	//power = -30;
	power = slow;
	ev3_sta_cyc(TRACE);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 80);

	//power = -20;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 110);

	//ev3_sta_cyc(FOWARD);

	power = -15;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 180);//170

	ev3_stp_cyc(TRACE);

	//ev3_stp_cyc(FOWARD);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	if(h_rgb > 100){
		device[1] = 0;
		ev3_speaker_play_tone(1975.53, 200);
	}

	ev3_stp_cyc(HT1RGB);

	//ケーブルを拾う
	ev3_motor_reset_counts(EV3_PORT_B);
	//ev3_motor_set_power(EV3_PORT_B, -80);
	ev3_motor_rotate(EV3_PORT_B, -140, 80, false);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) * -1 <= 140);
	//ev3_motor_stop(EV3_PORT_B, true);

	ev3_motor_rotate(EV3_PORT_B, -770, 80, false);//767

	//ラインまで下がる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
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
	tslp_tsk(timer);


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

			if(k == 1){
				power = slow;
			}

			//進む
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
			ev3_stp_cyc(FOWARD);

			ev3_sta_cyc(TRACE33);

		}

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 35);

		ev3_stp_cyc(TRACE33);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		pturn = 30;
		act_tsk(LEFT_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 130);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ラインまで下がる
		ev3_sta_cyc(BACK);
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

		power = slow;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);
		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//右にcurve
		curve = 3;
		act_tsk(CURVE_TASK);
		slp_tsk();

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
		power = slows;
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
		power = slow;//slows
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
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 35);//185

		ev3_stp_cyc(TRACE33);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		pturn = 30;
		act_tsk(LEFT_TASK);
		slp_tsk();

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 130);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ラインまで下がる
		ev3_sta_cyc(BACK);
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

		power = slow;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

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
			power = -50;
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
			ev3_stp_cyc(FOWARD);

			gain_rate = one_slow;
			power = -50;
			ev3_sta_cyc(TRACE33);

		}

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

	//進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slow;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 135);

	ev3_stp_cyc(TRACE33);

	//右に曲がる
	curve = 1;
	act_tsk(CURVE_TASK);
	slp_tsk();

	//緑色に入る
	//進む
	gain_rate = one_fast;
	power = fast;
	ev3_sta_cyc(TRACE3);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 2000);

	gain_rate = one_slow;
	power = slow;

	ev3_motor_rotate(EV3_PORT_C, -60, 30, false);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= white);
	ev3_speaker_play_tone(1174.66, 200);

	gain_rate = one_slow;
	power = slow;

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 90);//100

	/*
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
	*/

	ev3_stp_cyc(TRACE3);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	//右に曲がる
	pturn = 20;
	act_tsk(RIGHT_TASK);
	slp_tsk();

	//ev3_motor_rotate(EV3_PORT_C, -150, 10, false);

	//向きの決定 緑
	ev3_motor_reset_counts(EV3_PORT_B);
	if(identifier[1] == 1){
		direction = 3;
		ev3_motor_rotate(EV3_PORT_B, 150, 20, false);
	}else if(identifier[1] == 2){
		direction = 4;
		ev3_motor_rotate(EV3_PORT_B, 300, 20, false);
	}else if(identifier[1] == 3){
		direction = 1;
		ev3_motor_rotate(EV3_PORT_B, -150, 20, false);
	}else if(identifier[1] == 4){
		direction = 2;
	}

	//緑に入る
	//色まで進む
	gain_rate = two_slow;
	power = -30;
	ev3_sta_cyc(TRACE);
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 20);

	power = -20;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 50);

	power = -15;
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.r = 250;
	choice = 2;
	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_stp_cyc(TRACE);

	//Deviceを下ろす
	act_tsk(DROP_TASK);
	slp_tsk();

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	//左に曲がる
	pturn = 30;
	act_tsk(LEFT_TASK);
	slp_tsk();

	//少し進む
	gain_rate = two_slow;
	power = slows;
	ev3_sta_cyc(TRACE3);
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 300);

	ev3_stp_cyc(TRACE3);

	ev3_motor_stop(EV3_PORT_A, true);
	//ev3_motor_stop(EV3_PORT_D, true);
	//tslp_tsk(200);

	//右を向く
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 30);
	ev3_motor_set_power(EV3_PORT_D, fast);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 450);
	ev3_motor_set_power(EV3_PORT_D, slow);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_D) * -1 <= 500);

	/*
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(200);
	*/

	//ラインまで進む
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slows;
	ev3_sta_cyc(FOWARD);

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

	power = slow;
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

	ev3_stp_cyc(FOWARD);

	ev3_speaker_play_tone(1174.66, 200);

	//右にcurve
	curve = 7;
	act_tsk(CURVE_TASK);
	slp_tsk();

	/*
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
	*/

	//姿勢を整える
	gain_rate = two_slow;
	power = slow;
	ev3_sta_cyc(TRACE);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 20);//300

	/*
	power = -20;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 130);//330
	*/

	power = -15;

	//色まで進む
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.b = 250;
	choice = 3;
	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_stp_cyc(TRACE);
	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_speaker_play_tone(1174.66, 200);

	/*
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	ev3_motor_set_power(EV3_PORT_A, power * -1);
	ev3_motor_set_power(EV3_PORT_D, power);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 25);
	*/

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	//Cableを下ろす
	ev3_motor_reset_counts(EV3_PORT_B);
	//ev3_motor_set_power(EV3_PORT_B, 80);
	ev3_motor_rotate(EV3_PORT_B, 1000, 80, false);

	//リンクを出す
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, -30);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);

	ev3_motor_set_power(EV3_PORT_C, -80);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) <= 1000);//1000
	//ev3_motor_stop(EV3_PORT_B, true);

	//リンクをたたむ
	ev3_motor_rotate(EV3_PORT_C, 180, 20, false);

	ev3_motor_rotate(EV3_PORT_B, -1000, 80, false);

	/*
	ev3_motor_reset_counts(EV3_PORT_C);
	ev3_motor_set_power(EV3_PORT_C, 20);
	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) <= 50);

	ev3_motor_set_power(EV3_PORT_B, -80);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_C) <= 180);
	ev3_motor_stop(EV3_PORT_C, true);

	do{
	}while(ev3_motor_get_counts(EV3_PORT_B) >= 0);
	ev3_motor_stop(EV3_PORT_B, true);
	*/

	//左を向く
	curve = 10;
	act_tsk(CURVE_TASK);
	slp_tsk();

	//ラインまで下がる
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	power = slows;
	ev3_sta_cyc(BACK);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 200);

	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);
	ev3_speaker_play_tone(1174.66, 200);

	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 200);

	power = slow;
	do{
	}while(ev3_color_sensor_get_reflect(EV3_PORT_2) >= black || ev3_color_sensor_get_reflect(EV3_PORT_3) >= black);

	ev3_stp_cyc(BACK);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	ev3_speaker_play_tone(1174.66, 200);

	//右にcurve
	curve = 7;
	act_tsk(CURVE_TASK);
	slp_tsk();

	//4つ目のDeviceに向かう
	gain_rate = one_fast;
	power = fast;
	ev3_sta_cyc(TRACE2);

	for (k = 0; k < 5; k++) {
		if(k == 4){
			ev3_motor_reset_counts(EV3_PORT_A);
			ev3_motor_reset_counts(EV3_PORT_D);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 300);

			gain_rate = one_slow;
			power = slow;
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
	power = slow;//slows
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
		power = slow;
		ev3_sta_cyc(TRACE22);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 35);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//右を向く
		pturn = 30;
		act_tsk(RIGHT_TASK);
		slp_tsk();

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 130);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ラインまで下がる
		ev3_sta_cyc(BACK);
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

		power = slow;

		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//左にcurve
		curve = 4;
		act_tsk(CURVE_TASK);
		slp_tsk();

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
		power = slows;
		ev3_sta_cyc(FOWARD);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
		ev3_stp_cyc(FOWARD);

	}else{
		//Device 1行2列目の回収

		if(device[1] != 0){
			device[1] = 1;
		}

		gain_rate = one_slow;
		power = slow;

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 35);

		ev3_stp_cyc(TRACE22);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		pturn = 30;
		act_tsk(RIGHT_TASK);
		slp_tsk();

		//Deviceを回収
		gain_rate = two_slow;
		power = slow;
		ev3_sta_cyc(TRACE);

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 130);

		ev3_motor_reset_counts(EV3_PORT_C);
		ev3_motor_set_power(EV3_PORT_C, -60);
		do{
		}while(ev3_motor_get_counts(EV3_PORT_C) * -1 <= 120);

		ev3_motor_stop(EV3_PORT_C, true);

		ev3_stp_cyc(TRACE);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//ラインまで下がる
		ev3_sta_cyc(BACK);
		power = slows;
		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_D) - ev3_motor_get_counts(EV3_PORT_A)) / 2 <= 150);

		power = slow;
		ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
		rgb4.g = 250;

		ev3_sta_cyc(HT4BLACK);
		slp_tsk();

		ev3_stp_cyc(HT4BLACK);
		ev3_speaker_play_tone(1174.66, 200);

		ev3_stp_cyc(BACK);

		ev3_motor_stop(EV3_PORT_A, true);
		ev3_motor_stop(EV3_PORT_D, true);
		tslp_tsk(timer);

		//方向転換の改良

		//左にcurve
		curve = 4;
		act_tsk(CURVE_TASK);
		slp_tsk();

		//方向転換の改良

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
			power = -50;
			ev3_sta_cyc(FOWARD);
			do{
			}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 100);
			ev3_stp_cyc(FOWARD);

			gain_rate = one_slow;
			power = -50;
			ev3_sta_cyc(TRACE22);
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

	//左に曲がる
	curve = 2;
	act_tsk(CURVE_TASK);
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

			ev3_motor_rotate(EV3_PORT_C, -60, 30, false);
		}

		ev3_motor_reset_counts(EV3_PORT_A);
		ev3_motor_reset_counts(EV3_PORT_D);
		do{
		}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= angle);

	}

	ev3_stp_cyc(TRACE2);

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	//右に曲がる
	pturn = 20;
	act_tsk(LEFT_TASK);
	slp_tsk();

	//ev3_motor_rotate(EV3_PORT_C, -150, 10, false);

	//向きの決定 黄色
	ev3_motor_reset_counts(EV3_PORT_B);
	if(identifier[2] == 1){
		direction = 1;
		ev3_motor_rotate(EV3_PORT_B, -150, 20, false);
	}else if(identifier[2] == 2){
		direction = 2;
	}else if(identifier[2] == 3){
		direction = 3;
		ev3_motor_rotate(EV3_PORT_B, 150, 20, false);
	}else if(identifier[2] == 4){
		direction = 4;
		ev3_motor_rotate(EV3_PORT_B, 300, 20, false);
	}

	//黄に入る
	//色まで進む
	gain_rate = two_slow;
	power = -30;
	ev3_sta_cyc(TRACE);
	ev3_motor_reset_counts(EV3_PORT_A);
	ev3_motor_reset_counts(EV3_PORT_D);
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 20);

	power = -20;
	do{
	}while((ev3_motor_get_counts(EV3_PORT_A) - ev3_motor_get_counts(EV3_PORT_D)) / 2 <= 50);

	power = -15;
	ht_nxt_color_sensor_measure_rgb(EV3_PORT_4, pRGB4);
	rgb4.b = 250;
	choice = 3;
	ev3_sta_cyc(HT4COLOR);
	slp_tsk();

	ev3_stp_cyc(HT4COLOR);
	ev3_stp_cyc(TRACE);


	/*

	//リンクを前に出す
	ev3_sta_cyc(EXPANSION);

	//回す
	ev3_sta_cyc(TWIST);

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

	//周期ハンドラの停止
	ev3_stp_cyc(TWIST);
	ev3_stp_cyc(EXPANSION);

	*/

	//Deviceを下ろす
	act_tsk(DROP_TASK);
	slp_tsk();

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	//右に曲がる
	pturn = 30;
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
	}while(ev3_motor_get_counts(EV3_PORT_A) <= 175);//185

	ev3_motor_stop(EV3_PORT_A, true);
	ev3_motor_stop(EV3_PORT_D, true);
	tslp_tsk(timer);

	ext_tsk();

}
