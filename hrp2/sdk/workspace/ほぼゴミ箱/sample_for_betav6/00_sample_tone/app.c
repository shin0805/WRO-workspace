/*
 *  環境構築確認用 サンプルプログラム (00_sample_tone)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// 待機時間の設定 [msec]
const uint32_t WAIT_TIME_MS = 2000;

void run_task(intptr_t unused) {

	ev3_speaker_set_volume(10);

	while (1) {		
		ev3_speaker_play_tone(NOTE_G4, 1000);
		tslp_tsk(WAIT_TIME_MS);

		ev3_speaker_play_tone(NOTE_C5, 1000);
		tslp_tsk(WAIT_TIME_MS);
	}

}

void main_task(intptr_t unused) {
	// タスクを開始する
	act_tsk(RUN_TASK);
}
