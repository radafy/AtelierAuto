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
#define NORTH 0
#define SOUTH_EAST 135
#define SOUTH 180

/*
1c. parcours avec la boussole
*/

uint8_t lmotor1, lmotor2, sn_compass, sn_touch;
pid_t PID;
float coeff = 4;
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

void move(uint8_t lmotor1, uint8_t lmotor2, float distance){ // distance en millimetres

        set_tacho_speed_sp(lmotor1, SPEED);
        set_tacho_speed_sp(lmotor2, SPEED);

        set_tacho_position_sp(lmotor1, 360 * distance / (WHEEL_D * M_PI));
        set_tacho_position_sp(lmotor2, 360 * distance / (WHEEL_D * M_PI));

        set_tacho_command(lmotor1, "run-to-rel-pos");
        set_tacho_command(lmotor2, "run-to-rel-pos");

        FLAGS_T state1, state2;
        get_tacho_state_flags(lmotor1, &state1);
        get_tacho_state_flags(lmotor2, &state2);

        while(state1 || state2){
                get_tacho_state_flags(lmotor1, &state1);
                get_tacho_state_flags(lmotor2, &state2);
                printf("running\n");
                Sleep(TIMESTEP);
        }

        set_tacho_position_sp(lmotor1, 0);
        set_tacho_position_sp(lmotor2, 0);
}

int sign(float x) {
        int sign_bit = (*(unsigned int*)&x) >> 31;
        return 1 - ((sign_bit & 0x1) << 1);
}

float pilote(float r, float x) {
        float err = fmod(r - x + 180,360) - 180;      // erreur

        // calcul de la commande PID
        float P = -coeff * err;

        printf("P: %.2f\n", P);
        return P;
}

void turn(uint8_t lmotor1, uint8_t lmotor2, uint8_t sn_compass, float dest_angle){ // angle de degres
        float curr_angle;       // angle courant
        float pre_angle;
        float u;
        get_sensor_value0(sn_compass, &curr_angle);
        while ((curr_angle != dest_angle) && (pre_angle != dest_angle)) {
                printf("curr_angle:  %2.f, dest_angle: %2.f\n", curr_angle, dest_angle);
                pre_angle = curr_angle;
                get_sensor_value0(sn_compass, &curr_angle);
                u = pilote(dest_angle, curr_angle);
                set_tacho_speed_sp(lmotor1, -u);
                set_tacho_speed_sp(lmotor2,  u);
                set_tacho_command(lmotor1, "run-forever");
                set_tacho_command(lmotor2, "run-forever");
                Sleep(TIMESTEP);
        }

        stop_robot();
}



void run(){
        set_tacho_command(lmotor1, "reset");
        set_tacho_command(lmotor2, "reset");

        signal(2, handle_sigint);               // on ecoute si ctrl-c est execute
        printf("je tourne vers le sud");
        turn(lmotor1, lmotor2, sn_compass, SOUTH);      // tourner vers le sud
        printf("j'avvance");
        move(lmotor1, lmotor2, 1000);                   // 1 metre vers le sud
        printf("je tourne vers le sud est");
        turn(lmotor1, lmotor2, sn_compass, SOUTH_EAST); // tourner vers sud-ouest
        printf("j'avance");
        move(lmotor1, lmotor2, 800);                    // 0.8 metres vers sud-ouest
        printf("je tourne vers le nord");
        turn(lmotor1, lmotor2, sn_compass, NORTH);      // tourner vers nord
        printf("j'avance");
        move(lmotor1, lmotor2, 600);                    // 0.6 m vers le nord

        stop_robot();
        kill(getppid(),SIGKILL);        //to terminate the parrent process listening to the touch sensor
}

void init(){

    ev3_init();
    ev3_sensor_init();
    ev3_tacho_init();

    if (!ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)){
            printf("COMPASS sensor is NOT found");
            exit(1);
    }

    if (!ev3_search_sensor(HT_NXT_COMPASS, &sn_compass, 0)){
            printf("COMPASS sensor is NOT found");
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
    // on calibre la boussole
    printf("Calibrage en cours...\n");
    set_sensor_mode(sn_compass, "COMPASS");
    Sleep(2000);

    printf("Calibrage termine.\n");

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