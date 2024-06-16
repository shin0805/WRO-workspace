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
#define UNKNOWN 0
#define BLACK 1
#define BLUE 2
#define GREEN 3
#define YELLOW 4
#define RED 5
#define WHITE 6
#define BROWN 7

/* センサーの設定 */
static const sensor_port_t
    color_sensor = EV3_PORT_1;

/* メインタスク */
void main_task(intptr_t unused)
{
    ev3_lcd_set_font (EV3_FONT_MEDIUM);
    /* 反射光値格納用の変数 */
    static int reflect = 0;
    static int color_id = 0;
    char color_buffer[30];

    /* ループに入る前にセンサーとセンサーポートを接続する */
    ev3_sensor_config (color_sensor, COLOR_SENSOR);
    //ev3_lcd_set_font (EV3_FONT_MEDIUM);
    /* 接続が完了したら合図として本体LEDをオレンジ色に変える */
    ev3_led_set_color(LED_ORANGE);

    /* ループの中に入る */
    while(1)
    {   /* 反射値と色を測定して表示し続ける */
        /* 反射値を測って変数に格納 */
        reflect = ev3_color_sensor_get_reflect(color_sensor);
        /* 色を読み取って変数に格納 */
        color_id = ev3_color_sensor_get_color(color_sensor);
        /* 読み取った色ごとに処理分け */
        /* それぞれの処理の中では、反射値を読み取った色を含む情報を文字配列に格納している */
        switch(color_id){
            case UNKNOWN:
                sprintf(color_buffer, "REFLECT:%d      COLOR:Unknown            ",reflect);
            break;
            case BLACK:
                sprintf(color_buffer, "REFLECT:%d      COLOR:Black              ",reflect);
            break;
            case BLUE:
                sprintf(color_buffer, "REFLECT:%d      COLOR:Blue               ",reflect);
            break;
            case GREEN:
                sprintf(color_buffer, "REFLECT:%d      COLOR:Green              ",reflect);
            break;
            case YELLOW:
                sprintf(color_buffer, "REFLECT:%d      COLOR:Yellow             ",reflect);
            break;
            case RED:
                sprintf(color_buffer, "REFLECT:%d      COLOR:Red                ",reflect);
            break;
            case WHITE:
                sprintf(color_buffer, "REFLECT:%d      COLOR:White              ",reflect);
            break;
            case BROWN:
                sprintf(color_buffer, "REFLECT:%d      COLOR:Brown              ",reflect);
            break;
        }
        /* 正面ディスプレイに文字配列の中身を描写させる */
        ev3_lcd_draw_string(color_buffer, 0,10);

    }
    ext_tsk();
}
