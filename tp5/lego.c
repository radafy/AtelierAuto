#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define TIMESTEP 100     // pas de temps
#define TIMEOUT 30       // minuteur
#define SPEED 120.0       // vitesse
#define WHEEL_D 56       // diametre des roues
#define WHEEL_B 132      // largeur du vehicule


/*
2a Suivi de mur
*/

uint8_t lmotor1, lmotor2, sn_sonar, sn_touch;
pid_t PID;

void stop_robot(){
        set_tacho_command(lmotor1, "stop");
        set_tacho_command(lmotor2, "stop");
}

// fonction pour attendre un ctrl-c
void handle_sigint(){
        printf("terminated\n");
        kill(getppid(),SIGKILL);
        stop_robot();
        ev3_uninit();
        exit(2);
}


int sign(float x) {
        int sign_bit = (*(unsigned int*)&x) >> 31;
        return 1 - ((sign_bit & 0x1) << 1);
}

float pilote(int input) {
        float w;
        float u;
        float v;
        float err = 0.5 - input;
        w = -5*err/v + u;
        u = w + 5.17*err/v;
       


        return u;
}




void run(){
        set_tacho_command(lmotor1, "reset");
        set_tacho_command(lmotor2, "reset");

        signal(2, handle_sigint);               // on ecoute si ctrl-c est execute
        ///////////////////////////////////////////
        // run corps ici 
        uint8_t sensor_value;
        while(true){
                get_sensor_value( 0, sn_sonar, &sensor_value);
                float u = pilote(sensor_value);
                set_tacho_speed_sp(lmotor1, SPEED-u);
                set_tacho_speed_sp(lmotor2, SPEED+u);
                set_tacho_command(lmotor1, "run-forever");
                set_tacho_command(lmotor2, "run-forever");
                Sleep(TIMESTEP);
        }
        //////////////////////////////////////////
        stop_robot();
        kill(getppid(),SIGKILL);        //to terminate the parrent process listening to the touch sensor
}

void init(){

    ev3_init();
    ev3_sensor_init();
    ev3_tacho_init();

    if (!ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)){
            printf("TOUCH sensor is NOT found");
            exit(1);
    }

    if (!ev3_search_sensor(LEGO_EV3_US, &sn_sonar, 0)){
            printf("US sensor is NOT found");
            exit(1);
    }

    if (!ev3_search_tacho(LEGO_EV3_L_MOTOR, &lmotor1, 0)){
            printf( "L motor 1 is NOT found");
            exit(1);
    }

    if (!ev3_search_tacho(LEGO_EV3_L_MOTOR, &lmotor2, lmotor1 + 1)){
            printf("L motor 2 is NOT found");
            exit(1);
    }

    printf("Executez Ctrl + C pour arreter le robot.\n");
    Sleep(2000);

    printf("Demarrage en cours.\n");
    Sleep(2000);
}

void finish(){
    set_tacho_command(lmotor1, "stop");
    set_tacho_command(lmotor2, "stop");
    ev3_uninit();
}

void forcedTerminason(){
    int touch = 0;
    while (touch != 1){
        get_sensor_value(0, sn_touch, &touch);
        Sleep(100);
    }
    kill(PID,SIGKILL);// quand le capteur de contact est activé on kill le process en train de run
}

int main(void){
    init();

    PID = fork();


    if (PID == 0) {
        run();
    } else if (PID ==-1) {
        printf( "error");
            exit(1);
    }
    else {
        forcedTerminason();
    }

    finish();
}