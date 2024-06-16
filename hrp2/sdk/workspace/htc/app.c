/**
 ******************************************************************************
 ** ファイル名   : main.c
 ** 説明       : 反射値と色を測定する
 ******************************************************************************
 **/

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
/* 検知色の対応付けのためのマクロ定義 */


/* センサーの設定 */
static const sensor_port_t
    color_sensor = EV3_PORT_4;

/* 反射光値格納用の変数 */
static uint8_t *reflect ;

//PID制御の変数
double kp, ki, kd, gain_rate;
double delta_t = 0.004;
int diff[2];
int integral, p, i, d, sensor2, sensor3, motor, count, power, sensor1, pid;
int two_slow = 2.14;
int power = -30;
int threshold = 45;


void ht_task(intptr_t unused){
  ht_nxt_color_sensor_measure_color(color_sensor, reflect);
}

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

/* メインタスク */
void main_task(intptr_t unused)
{
  ev3_lcd_set_font (EV3_FONT_MEDIUM);
  ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);	//右前カラーセンサー
  ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);	//左前カラーセンサー
  ev3_sensor_config(EV3_PORT_1, HT_NXT_COLOR_SENSOR);	//右後HTセンサー
  ev3_sensor_config(EV3_PORT_4, HT_NXT_COLOR_SENSOR);	//左後HTセンサー
  ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);	//右タイヤ
  ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);	//左タイヤ

  gain_rate = two_slow;

    uint8_t a;
    reflect = &a;

    char color_buffer[30];

    ev3_lcd_set_font(EV3_FONT_MEDIUM);

    /* ループに入る前にセンサーとセンサーポートを接続する */
    ev3_sensor_config (color_sensor, HT_NXT_COLOR_SENSOR);
    //ev3_lcd_set_font (EV3_FONT_MEDIUM);
    /* 接続が完了したら合図として本体LEDをオレンジ色に変える */

    ev3_sta_cyc(HT);

    ev3_sta_cyc(TRACE2);
    /* ループの中に入る */
    while(1) {
        sprintf(color_buffer, "NUMBER:-%d-" ,a);

        /* 正面ディスプレイに文字配列の中身を描写させる */
        ev3_lcd_draw_string(color_buffer, 0,10);
    }
    ev3_stp_cyc(HT);
    ev3_stp_cyc(TRACE2);
    ext_tsk();
}
