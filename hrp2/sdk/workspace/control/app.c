#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

char color_buffer1[30];
int a;

void control_task(intptr_t unused) {
	if(ev3_button_is_pressed(DOWN_BUTTON) != false){
		ev3_motor_set_power(EV3_PORT_C, 80);

	}else if(ev3_button_is_pressed(UP_BUTTON) != false){
		ev3_motor_set_power(EV3_PORT_C, -80);

	}else{
		ev3_motor_stop(EV3_PORT_C, true);

	}

	a = ev3_motor_get_counts(EV3_PORT_C);

	sprintf(color_buffer1, "PORT:C|%d|" ,a);

	ev3_lcd_draw_string(color_buffer1, 0,60);


}


void main_task(intptr_t unused) {
	ev3_lcd_set_font (EV3_FONT_MEDIUM);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);	//右カラーセンサー
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);	//左カラーセンサー
	ev3_sensor_config(EV3_PORT_1, HT_NXT_COLOR_SENSOR);	//右前HTセンサー
	ev3_sensor_config(EV3_PORT_4, COLOR_SENSOR);	//左前カラーセンサー
	ev3_motor_config(EV3_PORT_A, MEDIUM_MOTOR);	//右タイヤ
	ev3_motor_config(EV3_PORT_D, MEDIUM_MOTOR);	//左タイヤ
	ev3_motor_config(EV3_PORT_B, MEDIUM_MOTOR);	//アーム
	ev3_motor_config(EV3_PORT_C, MEDIUM_MOTOR);	//リンク


	ev3_sta_cyc(CONTROL);



}
