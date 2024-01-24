#include <string.h>
#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include <unistd.h>
#include <stdlib.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

int main(void){
	char s[ 256 ];
	int val,n;
	uint8_t sn_touch, sn_us;	
	
	ev3_init();
	ev3_sensor_init();
	
	if(!ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch,0)){
		printf( "TOUCH sensor is NOT found");
		exit(1);
	}
	if(!ev3_search_sensor(LEGO_EV3_US, &sn_us,0)){
		printf( "US sensor is NOT found");
		exit(1);
	}

	// Initialisez le capteur gyroscopique
    if (!ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro, 0)) {
        printf("Gyro sensor not found\n");
        exit(1);
    }

	//boucle  
	n=0;
	while (n<15){
		get_sensor_value( 0, sn_touch, &val);	
		printf( "%d : %d \n",n,val);
		get_sensor_value( 0, sn_us, &val);	
		printf( "%d : %d \n",n,val);
		n++;
		Sleep(1000);
	}
	ev3_uninit();	 	
}