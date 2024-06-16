/*
 *  �J���[�Z���T�[���� �T���v���v���O���� 1
 *  REFLECT���[�h (04_sample_color1)
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
#define color_sensor	EV3_PORT_3

// �ҋ@���Ԃ̐ݒ� [msec]
const uint32_t WAIT_TIME_MS = 100;

// REFLECT���[�h�p �H�ʂ̒l
#define black		6
#define white		91

void run_task(intptr_t unused) {
	int power = 20;
	int now_val = 0;
	static int32_t cnt = 0;
	const int threshold = (black + white) / 2;

	// ��]�p�x�����Z�b�g
	ev3_motor_reset_counts(L_motor);

	while (1) {
		now_val = ev3_color_sensor_get_reflect(color_sensor);

		if (now_val < threshold) {
			// �����H�ʂ̏ꍇ
			cnt = ev3_motor_get_counts(L_motor) * -1;
			ev3_motor_rotate(L_motor, cnt, power, false );
			ev3_motor_rotate(R_motor, cnt, power, true );
			ev3_motor_reset_counts(L_motor);
		} else {
			// �����H�ʂ̏ꍇ
			ev3_motor_steer(L_motor, R_motor, power, 0 );
		}
		tslp_tsk(WAIT_TIME_MS);
	}
}

void main_task(intptr_t unused) {
	// ���[�^�[�̐ݒ�
	ev3_motor_config( L_motor , LARGE_MOTOR );
	ev3_motor_config( R_motor , LARGE_MOTOR );

	// �Z���T�[�̐ݒ�
	ev3_sensor_config( color_sensor , COLOR_SENSOR );

	// �^�X�N���J�n����
	act_tsk(RUN_TASK);
}
