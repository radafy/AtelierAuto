#include <ev3.h>

int main() {
    // Initialisation
    if (ev3_init() == -1) return 1;

    // Affichage du message "Hello World" sur l'écran
    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_lcd_draw_string("Hello World", 10, 10);

    // Gestion des capteurs
    // Exemple avec un capteur de couleur sur le port 1
    while (1) {
        int sensor_value = ev3_color_sensor_get_reflected_light_intensity(EV3_PORT_1);
        // Faites quelque chose avec la valeur du capteur...
    }

    // Gestion des moteurs
    // Exemple avec les moteurs B et C
    ev3_motor_set_power(EV3_PORT_B, 50);  // Mettez le moteur B à 50% de puissance
    ev3_motor_set_power(EV3_PORT_C, -50); // Mettez le moteur C à -50% de puissance

    // Attendez quelques secondes
    sleep(2);

    // Arrêtez les moteurs
    ev3_motor_stop(EV3_PORT_B, EV3_STOP_BRK);
    ev3_motor_stop(EV3_PORT_C, EV3_STOP_BRK);

    // Nettoyage
    ev3_uninit();

    return 0;
}
