#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif


void run_task(intptr_t unused) {
	memfile_t memfile;
	ev3_memfile_load("/Connect.wav", &memfile);
	
	ev3_speaker_set_volume(10);
    ev3_speaker_play_file(&memfile, SOUND_MANUAL_STOP);
}

void main_task(intptr_t unused) {
	act_tsk(RUN_TASK);
}
