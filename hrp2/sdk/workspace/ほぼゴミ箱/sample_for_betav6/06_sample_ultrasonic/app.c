/*
 *  超音波センサー制御 サンプルプログラム (06_sample_ultrasonic)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// センサーのポートを設定
#define ultraSonic_sensor	EV3_PORT_4

// 待機時間の設定 [msec]
const uint32_t WAIT_TIME_MS = 300;

void run_task(intptr_t unused) {
	int16_t now_val = 0;
	int16_t tone_val = 0;
	ev3_speaker_set_volume(1);

	while (1) {
		now_val = (int)ev3_ultrasonic_sensor_get_distance(ultraSonic_sensor);

		if( now_val > 140 ) {
			tone_val = NOTE_C6;
		} else if ( now_val > 130 ) {
			tone_val = NOTE_B5;
		} else if ( now_val > 120 ) {
			tone_val = NOTE_A5;
		} else if ( now_val > 110 ) {
			tone_val = NOTE_G5;
		} else if ( now_val > 100 ) {
			tone_val = NOTE_F5;
		} else if ( now_val > 90 ) {
			tone_val = NOTE_E5;
		} else if ( now_val > 80 ) {
			tone_val = NOTE_D5;
		} else if ( now_val > 70 ) {
			tone_val = NOTE_C5;
		} else if ( now_val > 60 ) {
			tone_val = NOTE_B4;
		} else if ( now_val > 50 ) {
			tone_val = NOTE_A4;
		} else if ( now_val > 40 ) {
			tone_val = NOTE_G4;
		} else if ( now_val > 30 ) {
			tone_val = NOTE_F4;
		} else if ( now_val > 20 ) {
			tone_val = NOTE_E4;
		} else if ( now_val > 10 ) {
			tone_val = NOTE_D4;
		} else {
			tone_val = NOTE_C4;
		}

		ev3_speaker_play_tone(tone_val, 300);
		tslp_tsk(WAIT_TIME_MS);
	}
}

void main_task(intptr_t unused) {
	// センサーの設定
	ev3_sensor_config( ultraSonic_sensor , ULTRASONIC_SENSOR );

	// タスクを開始する
	act_tsk(RUN_TASK);
}
