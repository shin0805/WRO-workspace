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

static uint8_t *reflect1 ;

static uint8_t *reflect4 ;


void ht1_task(intptr_t unused){
  ht_nxt_color_sensor_measure_color(EV3_PORT_1, reflect1);
}

void ht4_task(intptr_t unused){
  ht_nxt_color_sensor_measure_color(EV3_PORT_4, reflect4);
}


/* メインタスク */
void main_task(intptr_t unused)
{
  ev3_lcd_set_font (EV3_FONT_MEDIUM);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);	//右カラーセンサー
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);	//左カラーセンサー
	ev3_sensor_config(EV3_PORT_1, HT_NXT_COLOR_SENSOR);	//右前HTセンサー
	ev3_sensor_config(EV3_PORT_4, HT_NXT_COLOR_SENSOR);	//左前カラーセンサー
	ev3_motor_config(EV3_PORT_A, MEDIUM_MOTOR);	//右タイヤ
	ev3_motor_config(EV3_PORT_D, MEDIUM_MOTOR);	//左タイヤ
	ev3_motor_config(EV3_PORT_B, MEDIUM_MOTOR);	//アーム
	ev3_motor_config(EV3_PORT_C, MEDIUM_MOTOR);	//リンク

    /* 反射光値格納用の変数 */

    int a, b;

    uint8_t c;

    uint8_t d;

    reflect1 = &c;

    reflect4 = &d;

    char color_buffer1[30];
    char color_buffer2[30];
    char color_buffer3[30];
    char color_buffer4[30];


    ev3_lcd_set_font(EV3_FONT_MEDIUM);

    /* ループの中に入る */
    /*
    ev3_motor_set_power(EV3_PORT_A, fast);
  	ev3_motor_set_power(EV3_PORT_D, fast);
    */

    ev3_sta_cyc(HT1);
    ev3_sta_cyc(HT4);

    while(1) {
        a = ev3_color_sensor_get_reflect(EV3_PORT_2);
        b = ev3_color_sensor_get_reflect(EV3_PORT_3);

        sprintf(color_buffer1, "PORT:2-%d-" ,a);
        sprintf(color_buffer2, "PORT:3-%d-" ,b);
        sprintf(color_buffer3, "PORT:1-%d-" ,c);
        sprintf(color_buffer4, "PORT:4-%d-" ,d);


        /* 正面ディスプレイに文字配列の中身を描写させる */
        ev3_lcd_draw_string(color_buffer1, 0,10);
        ev3_lcd_draw_string(color_buffer2, 0,35);
        ev3_lcd_draw_string(color_buffer3, 0,60);
        ev3_lcd_draw_string(color_buffer4, 0,85);
    }
    ext_tsk();
}
