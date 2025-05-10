#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

extern int vl53l0xExampleSingle();
extern int vl53l0xExampleDual();

int main(void) {
  	pynq_init();

	//Setting up the buttons & LEDs
	//Init the IIC pins
	switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
	switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
	iic_init(IIC0);

	vl53l0xExampleSingle();
	
	iic_destroy(IIC0);
	pynq_destroy();
	return EXIT_SUCCESS;
}
