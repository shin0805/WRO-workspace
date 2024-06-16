/*
 *	タスク処理 サンプルプログラム
 *  カラーセンサー制御 サンプルプログラム 1 ＋ 周期タスク (08_sample_cycle)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// モーターのポートを設定
#define R_motor		EV3_PORT_B
#define L_motor		EV3_PORT_C

// センサーのポートを設定
#define color_sensor	EV3_PORT_3

// 待機時間の設定 [msec]
const uint32_t WAIT_TIME_MS = 100;

// REFLECTモード用 路面の値
#define black		 6
#define white		91

void cyclic_task(intptr_t exinf)
{
	ev3_speaker_set_volume(5);
	ev3_speaker_play_tone( NOTE_C5, 300 );
	tslp_tsk(300);
}

void run_task(intptr_t unused) {
	int power = 20;
	int now_val = 0;
	static int32_t cnt = 0;
	const int threshold = (black + white) / 2;
	int back_flag = false;

	// 回転角度をリセット
	ev3_motor_reset_counts(L_motor);

	while (1) {
		now_val = ev3_color_sensor_get_reflect(color_sensor);

		if(back_flag == false) {
			if (now_val < threshold) {
				// 黒い路面の場合
				back_flag = true;
				cnt = ev3_motor_get_counts(L_motor) * -1;
				ev3_motor_rotate(L_motor, cnt, power, false );
				ev3_motor_rotate(R_motor, cnt, power, false );
			} else {
				// 白い路面の場合
				ev3_motor_steer(L_motor, R_motor, power, 0 );
			}
		} else {
			// 後進中
			if (ev3_motor_get_counts(L_motor) <= 0 ) {
				ev3_motor_reset_counts(L_motor);
				back_flag = false;
			}
		}

	tslp_tsk(WAIT_TIME_MS);
	}
}

void main_task(intptr_t unused) {
	// モーターの設定
	ev3_motor_config( L_motor , LARGE_MOTOR );
	ev3_motor_config( R_motor , LARGE_MOTOR );

	// センサーの設定
	ev3_sensor_config( color_sensor , COLOR_SENSOR );

	// タスクを開始する
	ev3_sta_cyc(CYCHDR1);
	act_tsk(RUN_TASK);
}
