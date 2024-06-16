/*
 *  �^�b�`�Z���T�[���� �T���v���v���O����  (03_sample_touch)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// ���[�^�[�̃|�[�g��ݒ�
#define R_motor		EV3_PORT_B
#define L_motor		EV3_PORT_C

// �Z���T�[�̃|�[�g��ݒ�
#define touch_sensor	EV3_PORT_1

// �ҋ@���Ԃ̐ݒ� [msec]
const uint32_t WAIT_TIME_MS = 100;

void run_task(intptr_t unused) {
	int power;
	static bool_t	touch_pressed;

	while(1){
		touch_pressed = ev3_touch_sensor_is_pressed(touch_sensor);
		if(touch_pressed){
			power = 50;
		} else {
			power = 0;
		}

		ev3_motor_steer(L_motor, R_motor, power, 0 );
		tslp_tsk(WAIT_TIME_MS);
	}
}

void main_task(intptr_t unused) {
	// ���[�^�[�̐ݒ�
	ev3_motor_config( L_motor , LARGE_MOTOR );
	ev3_motor_config( R_motor , LARGE_MOTOR );

	// �Z���T�[�̐ݒ�
	ev3_sensor_config( touch_sensor , TOUCH_SENSOR );

	// �^�X�N���J�n����
	act_tsk(RUN_TASK);


}
