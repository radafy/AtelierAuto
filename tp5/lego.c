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
#define SPEED 200.0       // vitesse
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

void pilote(int input, float*u, float*w) {
        float err = 100.0 - (float)input;
        printf("%d erreur\n", err);
        (*u) = (*w) + 5.17*err/SPEED;
        (*w) = -5*err/SPEED + (*u)*0.83;
}

void run(){
        set_tacho_command(lmotor1, "reset");
        set_tacho_command(lmotor2, "reset");

        signal(2, handle_sigint);               // on ecoute si ctrl-c est execute
        ///////////////////////////////////////////
        // run corps ici
        int sensor_value;
        float u = 0;
        float w = 0;
        int chrono = 0;
        while(true){
                if (chrono > 1000){
                        get_sensor_value( 0, sn_sonar, &sensor_value);
                        printf("Dans if %i\n",sensor_value);
                        chrono=0;
                }
                else
                        printf("Dans else\n");
                pilote(sensor_value,&u,&w);
                set_tacho_speed_sp(lmotor1, SPEED-u);
                set_tacho_speed_sp(lmotor2, SPEED+u);
                set_tacho_command(lmotor1, "run-forever");
                set_tacho_command(lmotor2, "run-forever");
                chrono += TIMESTEP;
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