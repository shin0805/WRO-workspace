/*
 *  カラーセンサー制御 サンプルプログラム 2
 *  COLORモード (05_sample_color2)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// センサーのポートを設定
#define color_sensor	EV3_PORT_3

// 待機時間の設定 [msec]
const uint32_t WAIT_TIME_MS = 100;

void run_task(intptr_t unused)
{
	while (1) {
		colorid_t now_color = ev3_color_sensor_get_color(color_sensor);

		switch (now_color) {
			case COLOR_GREEN:
				ev3_led_set_color(LED_GREEN);
				break;
			case COLOR_YELLOW:
				ev3_led_set_color(LED_ORANGE);
				break;
			case COLOR_RED:
				ev3_led_set_color(LED_RED);
				break;
			case COLOR_NONE:
			case COLOR_BLACK:
			case COLOR_BLUE:
			case COLOR_WHITE:
			case COLOR_BROWN:
			default:
				ev3_led_set_color(LED_OFF);
				break;
		}

		tslp_tsk(WAIT_TIME_MS);
	}
}

void main_task(intptr_t unused) {
	// センサーの設定
	ev3_sensor_config( color_sensor , COLOR_SENSOR );

	// タスクを開始する
	act_tsk(RUN_TASK);
}
