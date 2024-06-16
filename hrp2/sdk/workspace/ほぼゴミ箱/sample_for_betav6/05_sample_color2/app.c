/*
 *  �J���[�Z���T�[���� �T���v���v���O���� 2
 *  COLOR���[�h (05_sample_color2)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// �Z���T�[�̃|�[�g��ݒ�
#define color_sensor	EV3_PORT_3

// �ҋ@���Ԃ̐ݒ� [msec]
const uint32_t WAIT_TIME_MS = 100;

void run_task(intptr_t unused)
{
	while (1) {
		colorid_t now_color = ev3_color_sensor_get_color(color_sensor);

		switch (now_color) {
			case COLOR_GREEN:
				ev3_led_set_color(LED_GREEN);
				break;
			case COLOR_YELLOW:
				ev3_led_set_color(LED_ORANGE);
				break;
			case COLOR_RED:
				ev3_led_set_color(LED_RED);
				break;
			case COLOR_NONE:
			case COLOR_BLACK:
			case COLOR_BLUE:
			case COLOR_WHITE:
			case COLOR_BROWN:
			default:
				ev3_led_set_color(LED_OFF);
				break;
		}

		tslp_tsk(WAIT_TIME_MS);
	}
}

void main_task(intptr_t unused) {
	// �Z���T�[�̐ݒ�
	ev3_sensor_config( color_sensor , COLOR_SENSOR );

	// �^�X�N���J�n����
	act_tsk(RUN_TASK);
}
