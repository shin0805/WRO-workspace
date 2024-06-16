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

/* メインタスク */
void main_task(intptr_t unused)
{

    /* 反射光値格納用の変数 */
    static uint8_t *reflect ;
    uint8_t a;

    reflect = &a;

    char color_buffer[30];

    ev3_lcd_set_font(EV3_FONT_MEDIUM);

    /* ループに入る前にセンサーとセンサーポートを接続する */
    ev3_sensor_config (color_sensor, HT_NXT_COLOR_SENSOR);
    //ev3_lcd_set_font (EV3_FONT_MEDIUM);
    /* 接続が完了したら合図として本体LEDをオレンジ色に変える */

    /* ループの中に入る */
    while(1) {
        ht_nxt_color_sensor_measure_color(color_sensor, reflect);
        sprintf(color_buffer, "NUMBER:-%d-" ,a);

        /* 正面ディスプレイに文字配列の中身を描写させる */
        ev3_lcd_draw_string(color_buffer, 0,10);
    }
    ext_tsk();
}
