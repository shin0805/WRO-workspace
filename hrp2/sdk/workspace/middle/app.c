#include "ev3api.h"
#include"app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define p_gain 0.5
#define power -30

void cyclic_task(intptr_t unused) {
	int sensorR, sensorL, sa, p;
	sensorR = ev3_color_sensor_get_reflect(EV3_PORT_2);
	sensorL = ev3_color_sensor_get_reflect(EV3_PORT_3);
	
	if (sensorR > sensorL){
		sa = sensorR - sensorL;
		p = power + p_gain * sa;
	    ev3_motor_set_power(EV3_PORT_B, power);
	    ev3_motor_set_power(EV3_PORT_D, p);
	}else{
		sa = sensorL - sensorR;
		p = power + p_gain * sa;
	    ev3_motor_set_power(EV3_PORT_B, p);
	    ev3_motor_set_power(EV3_PORT_D, power);
	}
}
	
void main_task(intptr_t unused)  {
	ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
	ev3_motor_config(EV3_PORT_D, LARGE_MOTOR);
	ev3_motor_config(EV3_PORT_C, LARGE_MOTOR);
	ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);
	ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);
	ev3_sensor_config(EV3_PORT_1, TOUCH_SENSOR);
	ev3_sensor_config(EV3_PORT_4, TOUCH_SENSOR);
	
	int touch1, touch2, tile1, tile2;
	
	ev3_sta_cyc(CYCHDR1);
    do{
    touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
    touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
    }while(touch1 == false || touch2 == false);
    ev3_stp_cyc(CYCHDR1);
    ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -40,0);
    tslp_tsk(500);
    ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_D, true);
    tslp_tsk(200);
	
	ev3_motor_rotate(EV3_PORT_B, 69, 15, false);
	ev3_motor_rotate(EV3_PORT_D, 69, 15, true);
	ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_D, true);
    tslp_tsk(200);
    
    tile1 = ev3_color_sensor_get_color(EV3_PORT_2);
    
    switch(tile1){
    	case 3:
    		tile1 = 3;
    		ev3_led_set_color(LED_GREEN);
    		break;
    	case 5:
    		tile1 = 5;
    		ev3_led_set_color(LED_RED);
    		break;
    	default:
    		tile1 = 2;
    		ev3_led_set_color(LED_OFF);
    		break;
    }
    
    ev3_motor_rotate(EV3_PORT_B, 93, 15, false);
	ev3_motor_rotate(EV3_PORT_D, 93, 15, true);
	ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_D, true);
    tslp_tsk(200);
    
    tile2 = ev3_color_sensor_get_color(EV3_PORT_2);
    
     switch(tile2){
    	case 3:
    		tile2 = 3;
    		ev3_led_set_color(LED_GREEN);
    		break;
    	case 5:
    		tile2 = 5;
    		ev3_led_set_color(LED_RED);
    		break;
    	default:
    		tile2 = 2;
    		ev3_led_set_color(LED_OFF);
    		break;
    }
	
	ev3_motor_rotate(EV3_PORT_C, -160, 30, false);
	ev3_motor_rotate(EV3_PORT_B, 485, 25, false);
	ev3_motor_rotate(EV3_PORT_D, 485, 25, true);
	
	ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_D, true);
    tslp_tsk(200);
	
	ev3_motor_rotate(EV3_PORT_C, 180, 20, true);
	
	ev3_motor_rotate(EV3_PORT_B, -308, 20, true);
	ev3_motor_stop(EV3_PORT_B, true);
	tslp_tsk(200);
	
	ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, 20,0);
    tslp_tsk(1000);
	ev3_motor_stop(EV3_PORT_B, true);
    ev3_motor_stop(EV3_PORT_D, true);
    tslp_tsk(200);
	
	switch(tile1){
		case 3:
			ev3_motor_rotate(EV3_PORT_B, -1720, 40, false);
	        ev3_motor_rotate(EV3_PORT_D, -1720, 40, true);
			ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -20,0);
			do{
            touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
            touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
            }while(touch1 == false || touch2 == false);
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
			
			ev3_motor_rotate(EV3_PORT_D, 308, 20, true);
	        ev3_motor_stop(EV3_PORT_D, true);
        	tslp_tsk(200);
			
			ev3_motor_rotate(EV3_PORT_B, 250, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, 250, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
			
			ev3_motor_rotate(EV3_PORT_B, 154, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, -154, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -30,0);
			do{
            touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
            touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
            }while(touch1 == false || touch2 == false);
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
          
            ev3_motor_rotate(EV3_PORT_D, 308, 20, true);
	        ev3_motor_stop(EV3_PORT_D, true);
        	tslp_tsk(200);
            
            ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, 30,0);
            tslp_tsk(1000);
            
            ev3_motor_rotate(EV3_PORT_B, -573, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, -573, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_rotate(EV3_PORT_C, 80, 40, true);
            
            ev3_motor_rotate(EV3_PORT_B, 444, 30, false);
	        ev3_motor_rotate(EV3_PORT_D, 444, 30, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_rotate(EV3_PORT_B, 154, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, -154, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -30,0);
            tslp_tsk(1000);
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            switch(tile2){
            	case 5:
            		ev3_motor_rotate(EV3_PORT_B, 645, 40, false);
	                ev3_motor_rotate(EV3_PORT_D, 645, 40, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
            		break;
            	case 2:
            		ev3_motor_rotate(EV3_PORT_B, 1218, 40, false);
	                ev3_motor_rotate(EV3_PORT_D, 1218, 40, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
            		break;
            }
            
            ev3_motor_rotate(EV3_PORT_B, -154, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, 154, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -40,0);
            do{
            touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
            touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
            }while(touch1 == false || touch2 == false);
           
            ev3_motor_rotate(EV3_PORT_B, 20, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, 20, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            break;
        case 5:
        	ev3_motor_rotate(EV3_PORT_B, -1720, 40, false);
	        ev3_motor_rotate(EV3_PORT_D, -1720, 40, true);
        	ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -20,0);
			do{
            touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
            touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
            }while(touch1 == false || touch2 == false);
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_rotate(EV3_PORT_D, 308, 20, true);
	        ev3_motor_stop(EV3_PORT_D, true);
        	tslp_tsk(200);
        	
        	ev3_motor_rotate(EV3_PORT_B, -170, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, -170, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
        	
        	ev3_motor_rotate(EV3_PORT_C, 80, 40, true);
        	
        	switch (tile2){
        		case 3:
        			ev3_motor_rotate(EV3_PORT_B, 430, 40, false);
	                ev3_motor_rotate(EV3_PORT_D, 430, 40, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
        			
        			ev3_motor_rotate(EV3_PORT_B, 154, 20, false);
	                ev3_motor_rotate(EV3_PORT_D, -154, 20, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
            
                    ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -30,0);
	        		do{
                    touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
                    touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
                    }while(touch1 == false || touch2 == false);
                    ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
          
                    ev3_motor_rotate(EV3_PORT_D, 308, 20, true);
	                ev3_motor_stop(EV3_PORT_D, true);
        	        tslp_tsk(200);
        	        break;
        	     case 2:
        	     	ev3_motor_rotate(EV3_PORT_B, 115, 40, false);
	                ev3_motor_rotate(EV3_PORT_D, 115, 40, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
        	     	
        	     	ev3_motor_rotate(EV3_PORT_B, 154, 20, false);
	                ev3_motor_rotate(EV3_PORT_D, -154, 20, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
                    
                    ev3_motor_rotate(EV3_PORT_B, 545, 40, false);
	                ev3_motor_rotate(EV3_PORT_D, 545, 40, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
                    
                    ev3_motor_rotate(EV3_PORT_B, -154, 20, false);
	                ev3_motor_rotate(EV3_PORT_D, 154, 20, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
                 	break;
           	}
           
           	ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -40,0);
            do{
            touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
            touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
            }while(touch1 == false || touch2 == false);
           
            ev3_motor_rotate(EV3_PORT_B, 20, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, 20, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            break;     	
        case 2:
        	ev3_motor_rotate(EV3_PORT_B, -1118, 30, false);
	        ev3_motor_rotate(EV3_PORT_D, -1118, 30, true);
        	ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_rotate(EV3_PORT_B, -154, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, 154, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_rotate(EV3_PORT_B, -86, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, -86, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_rotate(EV3_PORT_C, 80, 40, true);
            
            switch(tile2){
            	case 3:
            		ev3_motor_rotate(EV3_PORT_B, 500, 40, false);
	                ev3_motor_rotate(EV3_PORT_D, 500, 40, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
        	        break;
        	     case 5:
        	     	ev3_motor_rotate(EV3_PORT_B, 86, 20, false);
	                ev3_motor_rotate(EV3_PORT_D, 86, 20, true);
	                ev3_motor_stop(EV3_PORT_B, true);
                    ev3_motor_stop(EV3_PORT_D, true);
                    tslp_tsk(200);
                    break;
            }
            
            ev3_motor_rotate(EV3_PORT_B, 154, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, -154, 20, true);
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -30,0);
	  		do{
            touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
            touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
            }while(touch1 == false || touch2 == false);
            ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
          
            ev3_motor_rotate(EV3_PORT_D, 308, 20, true);
	        ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);
            
            ev3_motor_steer(EV3_PORT_D, EV3_PORT_B, -40,0);
            do{
            touch1 = ev3_touch_sensor_is_pressed(EV3_PORT_1);
            touch2 = ev3_touch_sensor_is_pressed(EV3_PORT_4);
            }while(touch1 == false || touch2 == false);
           
            ev3_motor_rotate(EV3_PORT_B, 20, 20, false);
	        ev3_motor_rotate(EV3_PORT_D, 20, 20, true);
	        ev3_motor_stop(EV3_PORT_B, true);
            ev3_motor_stop(EV3_PORT_D, true);
            tslp_tsk(200);	    	
            break;
    }
}
	