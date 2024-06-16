/*
 *  ���C���g���[�X �T���v���v���O���� (09_sample_linetrace)
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
#define color_sensor		EV3_PORT_3
#define ultraSonic_sensor	EV3_PORT_4

// REFLECT���[�h�p �H�ʂ̒l
#define black		6
#define white		91

bool_t run_flag = true;

void cyclic_task(intptr_t exinf)
{
	int power, turn;
	int now_val = 0;
	const int threshold = (black + white) / 2;

	now_val = ev3_color_sensor_get_reflect(color_sensor);

	if (run_flag == false) {
		power = 0;
		turn = 0;
	} else {
		power = 20;

		if (now_val < threshold) {
			turn = -50;
		} else {
			turn = 50;
		}
	}

	ev3_motor_steer(L_motor, R_motor, power, turn );
}

void cyclic_ultrasonic(intptr_t exinf)
{
	if (ev3_ultrasonic_sensor_get_distance(ultraSonic_sensor) <= 10) {
		run_flag = false;
	} else {
		run_flag = true;
	}
}

void main_task(intptr_t unused) {
	// ���[�^�[�̐ݒ�
	ev3_motor_config( L_motor , LARGE_MOTOR );
	ev3_motor_config( R_motor , LARGE_MOTOR );

	// �Z���T�[�̐ݒ�
	ev3_sensor_config( color_sensor , COLOR_SENSOR );
	ev3_sensor_config( ultraSonic_sensor , ULTRASONIC_SENSOR );

	// �^�X�N���J�n����
	ev3_sta_cyc(CYCHDR1);
	ev3_sta_cyc(USONIC);
}
