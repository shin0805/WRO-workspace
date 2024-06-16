/*
 *  ���\�z�m�F�p �T���v���v���O���� (00_sample_tone)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// �ҋ@���Ԃ̐ݒ� [msec]
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
	// �^�X�N���J�n����
	act_tsk(RUN_TASK);
}
