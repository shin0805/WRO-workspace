/*
 *  モーター制御 サンプルプログラム（02_sample_motor）
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
#define L_motor		EV3_PORT_D

// 待機時間の設定 [msec]
const uint32_t RUN_TIME_MS = 1000;
const uint32_t WAIT_TIME_MS = 200;

void run_task(intptr_t unused) {
	int power;
	static int32_t cnt = 0;

	// 回転角度をリセット
	ev3_motor_reset_counts(L_motor);

	// 前進
	power = 20;

	ev3_motor_set_power(L_motor, 20);
	tslp_tsk(RUN_TIME_MS);
	ev3_motor_stop(L_motor, true);
	tslp_tsk(WAIT_TIME_MS);
	
	ev3_motor_set_power(R_motor, -20);
	tslp_tsk(RUN_TIME_MS);
	ev3_motor_stop(R_motor, true);
	tslp_tsk(WAIT_TIME_MS);

}

void main_task(intptr_t unused) {
	// モーターの設定
	ev3_motor_config( L_motor , LARGE_MOTOR );
	ev3_motor_config( R_motor , LARGE_MOTOR );

	// タスクを開始する
	act_tsk(RUN_TASK);
}
