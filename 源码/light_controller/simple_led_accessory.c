/*
 * simple_led_accessory.c
 * Define the accessory in pure C language using the Macro in characteristics.h
 *
*  Created on: 2022-04-19
 *      Author: Ye Zhang
 */

#include <Arduino.h>
#include <homekit/types.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <stdio.h>
#include <port.h>



#define ACCESSORY_NAME  ("ESP8266_LED")
#define ACCESSORY_SN  ("SN_0123456")
#define ACCESSORY_MANUFACTURER ("西北偏北工作室")
#define ACCESSORY_MODEL  ("XBPB006")



// Global variables
float led_color_tempearture = 0;        // color_tempearture is scaled 50 to 400
float led_brightness = 0;               // brightness is scaled 0 to 100
bool led_power = false;                 //true or flase




homekit_value_t led_on_get() {
	return HOMEKIT_BOOL(led_power);
}

void led_on_set(homekit_value_t value) {
	if (value.format != homekit_format_bool) {
		printf("Invalid on-value format: %d\n", value.format);
		return;
	}
	led_power = value.bool_value;
  if (led_power) {
    if (led_brightness < 1) {
      led_brightness = 100;
    }
  }
}





homekit_value_t led_brightness_get() {
    return HOMEKIT_INT(led_brightness);
}

void led_brightness_set(homekit_value_t value) {
    if (value.format != homekit_format_int) {
        // printf("Invalid brightness-value format: %d\n", value.format);
        return;
    }
    led_brightness = value.int_value;
}



void led_color_temperature_set(homekit_value_t value) {
    if (value.format != homekit_format_uint32) {
        // printf("Invalid brightness-value format: %d\n", value.format);
        return;
    }
    led_color_tempearture = value.uint32_value;
}


homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, ACCESSORY_NAME);
homekit_characteristic_t serial_number = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, ACCESSORY_SN);
homekit_characteristic_t cha_led_on = HOMEKIT_CHARACTERISTIC_(ON, false,
		//.callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_callback),
		.getter=led_on_get,
		.setter=led_on_set
);
homekit_characteristic_t cha_led_brightness = HOMEKIT_CHARACTERISTIC_(BRIGHTNESS, 100,.getter = led_brightness_get,.setter = led_brightness_set);
homekit_characteristic_t cha_led_color_temperature = HOMEKIT_CHARACTERISTIC_(COLOR_TEMPERATURE, 400,.setter = led_color_temperature_set);

homekit_characteristic_t cha_top2bottom_motion = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, false);
homekit_characteristic_t cha_bottom2top_motion = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, false);
homekit_characteristic_t cha_left2right_motion = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, false);
homekit_characteristic_t cha_right2left_motion = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, false);


// format: bool; HAP section 9.70; write the .setter function to get the switch-event sent from iOS Home APP.
// homekit_characteristic_t cha_switch_Up_Down_on = HOMEKIT_CHARACTERISTIC_(ON, false);

// // format: string; HAP section 9.62; max length 64
// homekit_characteristic_t cha_Up_Down_name = HOMEKIT_CHARACTERISTIC_(NAME, "上下手势");

// homekit_characteristic_t cha_switch_Right_Left_on = HOMEKIT_CHARACTERISTIC_(ON, false);

// // format: string; HAP section 9.62; max length 64
// homekit_characteristic_t cha_Right_Left_name = HOMEKIT_CHARACTERISTIC_(NAME, "左右手势");


void accessory_identify(homekit_value_t _value) {
	printf("accessory identify\n");
}

homekit_accessory_t *accessories[] =
		{
                HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_bridge, .services=(homekit_service_t*[]) {
    	                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
                            &name,
						    HOMEKIT_CHARACTERISTIC(MANUFACTURER, ACCESSORY_MANUFACTURER),
						    &serial_number,
						    HOMEKIT_CHARACTERISTIC(MODEL, ACCESSORY_MODEL),
						    HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1.0"),
						    HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
                        NULL
                    }),
                NULL
                }),
				HOMEKIT_ACCESSORY(.id = 2,.category = homekit_accessory_category_lightbulb,.services=(homekit_service_t*[]){
						HOMEKIT_SERVICE(ACCESSORY_INFORMATION,.characteristics=(homekit_characteristic_t*[]){
						    HOMEKIT_CHARACTERISTIC(NAME, "LED屏幕挂灯"),
						    HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
						    NULL
						}),
						HOMEKIT_SERVICE(LIGHTBULB, .primary=true,.characteristics=(homekit_characteristic_t*[]){
						    HOMEKIT_CHARACTERISTIC(NAME, "LED屏幕挂灯"),
						    &cha_led_on,
						    &cha_led_brightness,
                            &cha_led_color_temperature, 
						    NULL
						}),
                        // HOMEKIT_SERVICE(SWITCH, .primary=true, .characteristics=(homekit_characteristic_t*[]){
			            // &cha_switch_Up_Down_on,
			            // &cha_Up_Down_name,
			            // NULL
		                // }),

                        // HOMEKIT_SERVICE(SWITCH, .primary=true, .characteristics=(homekit_characteristic_t*[]){
			            // &cha_switch_Right_Left_on,
			            // &cha_Right_Left_name,
			            // NULL
		                // }),

						NULL
						}),


                HOMEKIT_ACCESSORY(.id = 3,.category = homekit_accessory_category_sensor,.services=(homekit_service_t*[]){
					HOMEKIT_SERVICE(ACCESSORY_INFORMATION,.characteristics=(homekit_characteristic_t*[]){
						HOMEKIT_CHARACTERISTIC(NAME, "手势传感器"),
						HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
						NULL
					}),

                    HOMEKIT_SERVICE(MOTION_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
			            HOMEKIT_CHARACTERISTIC(NAME, "屏幕挂灯手势从上到下"),
			            &cha_top2bottom_motion,
			            NULL
		            }),
                    HOMEKIT_SERVICE(MOTION_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
			            HOMEKIT_CHARACTERISTIC(NAME, "屏幕挂灯手势从下到上"),
			            &cha_bottom2top_motion,
			            NULL
		            }),
                    HOMEKIT_SERVICE(MOTION_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
			            HOMEKIT_CHARACTERISTIC(NAME, "屏幕挂灯手势从左到右"),
			            &cha_left2right_motion,
			            NULL
		            }),
                    HOMEKIT_SERVICE(MOTION_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
			            HOMEKIT_CHARACTERISTIC(NAME, "屏幕挂灯手势从右到左"),
			            &cha_right2left_motion,
			            NULL
		            }),
					NULL
				}),
			NULL
		};

homekit_server_config_t config = {
		.accessories = accessories,
		.password = "771-55-104",
		//.on_event = on_homekit_event,
		.setupId = "KD9K"
};
