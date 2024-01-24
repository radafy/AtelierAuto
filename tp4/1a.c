#include <string.h>
#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <unistd.h>
#include <stdlib.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

/*
1a, "Commande par le fil":  Installez sur votre robot un moteur moyen avec une tige.
connectez par cable un Joystick (capteur gyro). Faites en sorte que l'angle de tige repete
l'angle de gyroscope
*/
uint8_t sn_touch;
uint8_t sn_gyro;
uint8_t mmotor;

void init(){
    ev3_init();
    ev3_sensor_init();
    ev3_tacho_init();

    if(!ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch,0)){
            printf( "GYRO touch is NOT found");
            exit(1);
    }

    if(!ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
            printf( "GYRO sensor is NOT found");
            exit(1);
    }

    if (!ev3_search_tacho(LEGO_EV3_M_MOTOR, &mmotor, 0)){
            printf( "M motor is NOT found");
            exit(1);
    }

    // on calibre le gyroscope
    printf("Calibration en cours...\n");
    set_sensor_mode(sn_gyro, "GYRO-CAL");
    Sleep(2000);
    set_sensor_mode(sn_gyro, "GYRO-RATE");
    Sleep(2000);
    set_sensor_mode(sn_gyro, "GYRO-ANG");
    Sleep(2000);
    printf("Calibration terminee.\n");

    printf("Demarrage en cours.\n");
    Sleep(2000);
}

void run (){
        char s[ 256 ];
        int val,n;
        n=0;
        // valeur precedente du gyroscope
        int prec;

        //valeur du capteur touch
        int touch = 0;


        while (touch != 1){
                get_sensor_value(0, sn_touch, &touch);
                printf( "%d : touch = %d \n",touch);

                prec = val;
                get_sensor_value(0, sn_gyro, &val);
                printf( "%d : val = %d, prec = %d \n",n,val, prec);

                // prendre en compte le changement de sens du gyroscope
                set_tacho_speed_sp(mmotor, (val - prec) * 30);
                set_tacho_command(mmotor, "run-forever");

                n++;
                Sleep(100);

        }
}

void finish(){
    set_tacho_command(mmotor, "stop");
    ev3_uninit();
}

int main(void){

        init();
        run();
        finish();
       
}