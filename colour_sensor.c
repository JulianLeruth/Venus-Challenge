#include <libpynq.h>
#include <iic.h>
#include "tcs3472.h"
#include <stdio.h>

#define BLACK                               0
#define WHITE                               1
#define RED                                 2
#define BLUE                                3
#define GREEN                               4

#define NUMBER_OF_MEASUREMENTS_FOR_COLOUR  25

struct n_colours {
    int black;
    int white;
    int red;
    int blue;
    int green;
};

int mostColour(struct n_colours c) {
    if (c.black > c.white && c.black > c.red && c.black > c.blue && c.black > c.green) return BLACK;
    else if (c.white > c.red && c.white > c.blue && c.white > c.green) return WHITE;
    else if (c.red > c.blue && c.red > c.green) return RED;
    else if (c.blue > c.green) return BLUE;
    else return GREEN;
}

int tcs3472Ping(tcs3472* sensor, int integration_time_ms) {
	/******** Simple connection test *******/
	uint8_t id;
	int i = tcs_ping(IIC0, &id);
	printf("Sensor Ping: ");
	if(i != TCS3472_SUCCES) {
		printf("\033[31m✘ Fail\033[0m\n");
		return EXIT_FAILURE;
	}
    printf("\033[32m✔ Succes\033[0m\n");

	/********** Preconfigure sensor ********/
	tcs_set_integration(sensor, tcs3472_integration_from_ms(integration_time_ms));
	tcs_set_gain(sensor, x4);

	//enable sensor -> loads preconfiguration
	i = tcs_init(IIC0, sensor);
	if(i != TCS3472_SUCCES) {
		printf("TCS3472 device was unsuccessfully opened.\n");
		return EXIT_FAILURE;
	}
	printf("TCS3472 device was successfully opened.\n");
	fflush(NULL);

	printf("Model ID - %#X\n\n", id);

    return EXIT_SUCCESS;
}
 
void printColour(uint16_t red, uint16_t green, uint16_t blue) {
	printf("\033[3F"); //Move cursor back 3 lines
	printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", CLAMP_255((red >> 4)), CLAMP_255((green >> 4)), CLAMP_255((blue >> 4))); //print uint8_t's
	printf("R: %hu  \n",red); //print uint16_t
	printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", CLAMP_255((red >> 4)), CLAMP_255((green >> 4)), CLAMP_255((blue >> 4))); //print uint8_t's
	printf("G: %hu  \n", green); //print uint16_t
	printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", CLAMP_255((red >> 4)), CLAMP_255((green >> 4)), CLAMP_255((blue >> 4))); //print uint8_t's
	printf("B: %hu  \n", blue); //print uint16_t
	fflush(NULL);
}

int colourSensor(tcs3472* sensor, int integration_time_ms) {
    /******** Simple connection test *******/
	uint8_t id;
	int i = tcs_ping(IIC0, &id);
	printf("---Detection: ");
	if(i != TCS3472_SUCCES) {
		printf("Fail\n");
		return EXIT_FAILURE;
	}
	printf("Succes\n");
	printf("-- ID: %#X\n", id);

	/********** Preconfigure sensor ********/
	tcs_set_integration(sensor, tcs3472_integration_from_ms(integration_time_ms));
	tcs_set_gain(sensor, x4);

	//enable sensor -> loads preconfiguration
	i = tcs_init(IIC0, sensor);
	printf("---Sensor Init: ");
	if(i != TCS3472_SUCCES) {
		printf("Fail\n");
		return EXIT_FAILURE;
	}
	printf("Succes\n");
	fflush(NULL);

    gpio_set_level(IO_AR4, GPIO_LEVEL_HIGH);

    sleep_msec(500);

    struct n_colours c = {0,0,0,0,0};
	tcsReading rgb;
	printf("\n\n\n"); //Buffer some space
    
	for (int i = 0; i < NUMBER_OF_MEASUREMENTS_FOR_COLOUR; i++) {
		if (tcs_get_reading(sensor, &rgb) == 1) return EXIT_FAILURE;

		uint16_t red, green, blue;
		if(rgb.red > 1200) red = rgb.red * 1.15;
		else red = rgb.red*1.1;
		
		blue = rgb.blue *0.85;
		green = rgb.green;
		
		uint16_t average_val = (red + blue + green)/3;

		if(average_val < 675) {
            c.black++;
			printf("Black \n");
		}
		else if(blue > 1400 && green > 1400 && red > 1400){
            c.white++;
			printf("White \n");
		}
        else if(red > green && red > blue){
            c.red++;
            printf("Red \n");
        }
		else if(blue > green && blue > red){
            c.blue++;
			printf("Blue \n");
		}
        else if(green > red && green > blue){
            c.green++;
            printf("Green \n");
        }
        
		sleep_msec(integration_time_ms + 20);

		tcs_init(IIC0, sensor);
	}
    printf("Black: %d, White: %d, Red: %d, Blue: %d, Green: %d\n", c.black, c.white, c.red, c.blue, c.green);
    
    gpio_set_level(IO_AR4, GPIO_LEVEL_LOW);
    
    return mostColour(c);
}
