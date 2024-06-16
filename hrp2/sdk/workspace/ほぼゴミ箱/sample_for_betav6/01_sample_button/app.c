/*
 *  本体ボタン、LED、LCD、サウンド制御 サンプルプログラム (01_sample_button)
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

static void button_clicked_handler(intptr_t button) {
	switch (button) {
		case LEFT_BUTTON:
			ev3_led_set_color( LED_RED );
			ev3_speaker_play_tone(NOTE_C4, 500);
			ev3_lcd_draw_string("Left Pressed",0,0);
			tslp_tsk(500);
			break;
		case RIGHT_BUTTON:
			ev3_led_set_color( LED_ORANGE );
			ev3_speaker_play_tone(NOTE_D4, 500);
			ev3_lcd_draw_string("Right Pressed",0,0);
			tslp_tsk(500);
			break;
		case UP_BUTTON:
			ev3_led_set_color( LED_GREEN );
			ev3_speaker_play_tone(NOTE_E4, 500);
			ev3_lcd_draw_string("Up Pressed",0,0);
			tslp_tsk(500);
			break;
		case DOWN_BUTTON:
			ev3_led_set_color( LED_OFF );
			ev3_speaker_play_tone(NOTE_F4, 500);
			ev3_lcd_draw_string("Down Pressed",0,0);
			tslp_tsk(500);
			break;
		case ENTER_BUTTON:
			ev3_speaker_play_tone(NOTE_G4, 500);
			ev3_lcd_draw_string("Enter Pressed",0,0);
			tslp_tsk(500);
			break;
		case BACK_BUTTON:
			ev3_speaker_play_tone(NOTE_C5, 500);
			ev3_lcd_draw_string("Back Pressed",0,0);
			tslp_tsk(500);
			break;
		case TNUM_BUTTON:
			default:
			break;
	}
}

void main_task(intptr_t unused) {
	// 本体ボタンの設定
	ev3_button_set_on_clicked(LEFT_BUTTON,  button_clicked_handler, LEFT_BUTTON);
	ev3_button_set_on_clicked(RIGHT_BUTTON, button_clicked_handler, RIGHT_BUTTON);
	ev3_button_set_on_clicked(UP_BUTTON,    button_clicked_handler, UP_BUTTON);
	ev3_button_set_on_clicked(DOWN_BUTTON,  button_clicked_handler, DOWN_BUTTON);
	ev3_button_set_on_clicked(ENTER_BUTTON, button_clicked_handler, ENTER_BUTTON);
	ev3_button_set_on_clicked(BACK_BUTTON,  button_clicked_handler, BACK_BUTTON);
	ev3_button_set_on_clicked(TNUM_BUTTON,  button_clicked_handler, TNUM_BUTTON);
	ev3_speaker_set_volume(10);
	ev3_led_set_color( LED_OFF );
}
