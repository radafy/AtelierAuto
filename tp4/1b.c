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
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define TIMESTEP 100
#define TIMEOUT 30

/*
1b. "robot danseur" Installez le capteur gyro a l'horizontale sur le robot. Faites en sorte que
l'angle de robot soir a sin(bt) ou bien encore a sin(bt) sin(ct) - trouvez des coefficients
raisonnables. Le faire danser une valse si possible.
*/

float I = 0.0;
float pre_err = 0.0;
float Kp = -12.0;
float Ki = 0.0;
float Kd = 0.;

void stop_robot(){
    uint8_t lmotor1, lmotor2;

    if (!ev3_search_tacho(LEGO_EV3_L_MOTOR, &lmotor1, 0)){
            printf( "L motor 1 is NOT found");
            exit(1);
    }

    if (!ev3_search_tacho(LEGO_EV3_L_MOTOR, &lmotor2, lmotor1 + 1)){
            printf("L motor 2 is NOT found");
            exit(1);
    }

    set_tacho_command(lmotor1, "stop");
    set_tacho_command(lmotor2, "stop");
}

// fonction pour attendre un ctrl-c
void handle_sigint(){
    printf("terminated\n");
    stop_robot();
    ev3_uninit();
    exit(2);
}

float get_reference(float time) {
    float a = 30.0; // amplitude de l'angle
    float b = 0.2;  // frequence du motif
    float period = 1.0; // periode de l'oscillation en secondes

    float time_sec = fmod (time / 1000.0, period);

    float reference = a * sin(b * 2.0 * M_PI * time_sec / period);

    return reference;
}

float get_output(uint8_t sens){
    int val;
    get_sensor_value(0, sens, &val);
    return val;
}

float pilote(float r, float x) {
    float err = r - x;      // erreur

    // calcul de la commande PID
    float P = Kp * err;
    I += Ki * err;
    float D = Kd * (err - pre_err);
    pre_err = err;

    float pid_out = P + I + D;

    printf("P: %.2f, I: %.2f, D: %.2f\n", P, I, D);

    return pid_out;
}

void set_control(uint8_t lmotor1, uint8_t lmotor2, float u, float K){
    set_tacho_speed_sp(lmotor1, - K * (int)u);
    set_tacho_speed_sp(lmotor2, K * (int)u);
    set_tacho_command(lmotor1, "run-forever");
    set_tacho_command(lmotor2, "run-forever");
}

void run(uint8_t lmotor1, uint8_t lmotor2, uint8_t sn_gyro, uint8_t sn_touch){
    set_tacho_command(lmotor1, "reset");
    set_tacho_command(lmotor2, "reset");
    float r, x, u, time = 0.0;
    int n = 0;
    int touch_ok = 0;

    signal(2, handle_sigint);               // on ecoute si ctrl-c est execute
    while (touch_ok != 1 || n < TIMEOUT * 10){               // on le fait durer 30 secondes
            get_sensor_value(0, sn_touch, &touch_ok);
            printf("%d : %d \n", n, touch_ok);
            if(touch_ok==1){
                    break;
            }
            r = get_reference(time);
            x = fmod(get_output(sn_gyro), 360.);
            u = pilote(r, x);
            set_control(lmotor1, lmotor2, u, 1.0);
            printf("r: %f, x: %f, u:%f\n", r, x, u);
            time += TIMESTEP;
            Sleep(TIMESTEP);
            n++;
    }
    stop_robot();
}

void init(uint8_t* sn_gyro, uint8_t* lmotor1, uint8_t* lmotor2, uint8_t* sn_touch){
    ev3_init();
    ev3_sensor_init();
    ev3_tacho_init();
    
    if (!ev3_search_sensor(LEGO_EV3_TOUCH, sn_touch, 0)) {
        printf("TOUCH sensor is NOT found");
        exit(1);
    }

    if(!ev3_search_sensor(LEGO_EV3_GYRO, sn_gyro,0)){
            printf( "GYRO sensor is NOT found");
            exit(1);
    }

    if (!ev3_search_tacho(LEGO_EV3_L_MOTOR, lmotor1, 0)){
            printf( "L motor 1 is NOT found");
            exit(1);
    }

    if (!ev3_search_tacho(LEGO_EV3_L_MOTOR, lmotor2, *lmotor1 + 1)){
            printf("L motor 2 is NOT found");
            exit(1);
    }

    printf("Executez Ctrl + C pour arreter le robot.\n");
    // on calibre le gyroscope
    printf("Calibration en cours...\n");
    set_sensor_mode(*sn_gyro, "GYRO-CAL");
    Sleep(2000);
    set_sensor_mode(*sn_gyro, "GYRO-RATE");
    Sleep(2000);
    set_sensor_mode(*sn_gyro, "GYRO-ANG");
    Sleep(2000);
    printf("Calibration terminee.\n");

    printf("Demarrage en cours.\n");
    Sleep(2000);
    I = 0.0;
    pre_err = 0.0;
}


int main(void){
    char s[ 256 ];
    int val,n;
    uint8_t sn_gyro, lmotor1, lmotor2, sn_touch;

    init(&sn_gyro, &lmotor1, &lmotor2, &sn_touch);

    

    run(lmotor1, lmotor2, sn_gyro, sn_touch);

}