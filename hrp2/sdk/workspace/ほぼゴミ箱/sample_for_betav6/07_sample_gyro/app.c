/*
 *  ジャイロセンサー制御 サンプルプログラム (07_sample_gyro)
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
#define gyro_sensor		EV3_PORT_2

// 待機時間の設定 [msec]
const uint32_t WAIT_TIME_MS = 100;

void run_task(intptr_t unused)
{
	int L_power, R_power;
	static int16_t now_val;
	static int turn_flag = false;

	// ジャイロセンサーの角度をリセット
	ev3_gyro_sensor_reset(gyro_sensor);

	while (1) {
		now_val = ev3_gyro_sensor_get_angle(gyro_sensor);

		if (turn_flag == false && now_val <= 180) {
			L_power =  20;
			R_power = -20;
		} else {
			if (turn_flag == true && now_val > 0) {
				L_power = -20;
				R_power =  20;
			} else {
				L_power = 0;
				R_power = 0;
			}
		}

		if (now_val > 180) {
			turn_flag = true;
		} else if (now_val <= 0) {
			turn_flag = false;
		}

		ev3_motor_set_power(L_motor, L_power);
		ev3_motor_set_power(R_motor, R_power);

		tslp_tsk(WAIT_TIME_MS);
	}
}

void main_task(intptr_t unused) {
	// モーターの設定
	ev3_motor_config( L_motor , LARGE_MOTOR );
	ev3_motor_config( R_motor , LARGE_MOTOR );

	// センサーの設定
	ev3_sensor_config( gyro_sensor , GYRO_SENSOR );

	// タスクを開始する
	act_tsk(RUN_TASK);
}
