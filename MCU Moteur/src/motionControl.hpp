#pragma once

#include <pico/stdlib.h>
#include <stdio.h>

// Programme en dev. Censé construire une courbe de vitesse en fonction de la distance à parcourir

#define MAX_SPEED   0xffff
#define DIST_BETWEEN_WHEELS 300     // En mm
#define FORWARD     1
#define BACKWARD    0

#define RISE_COEFF  1.0
#define BREAK_COEFF -1.0

#define RISE        0
#define PLATEAU     1
#define BREAK       2

struct SpeedCurve {
    uint16_t min_starting_speed = MAX_SPEED;
    uint16_t max_speed = 0;
    int64_t curves_limit[3] = { 0,0,0 };
    bool dir = FORWARD;
};

class MotionControl {
    private:

    int64_t last_abs_steps = 0;  // Nombre de pas absolus de puis la dernière construction de courbe
    SpeedCurve s_curve;          // Courbe

    int riseCurve(uint64_t x) {
        return RISE_COEFF * x + s_curve.min_starting_speed;
    }

    int breakCurve(uint64_t x) {
        return BREAK_COEFF * x + s_curve.max_speed;
    }

    int riseCurveRecursive(uint64_t y) {
        return y / RISE_COEFF + s_curve.min_starting_speed / RISE_COEFF;
    }

    int breakCurveRecursive(uint64_t y) {
        return y / BREAK_COEFF + s_curve.max_speed / BREAK_COEFF;
    }

    void buildSpeedCurve(int64_t steps) {
        last_abs_steps = steps;
        int rise_time = riseCurveRecursive(s_curve.max_speed);    // Nombre de pas à atteindre avant la vitesse max
        int break_time = breakCurveRecursive(0);       // Nombre de pas à atteindre avant l'arrêt

        if (steps < 0) {   // On s'implifie le code en gardant le signe pour plus tard
            steps *= -1;
            s_curve.dir = BACKWARD;
        }

        if (steps > (rise_time + break_time)) {
            // Si il y a suffisamment de pas pour un plateau on en prends compte
            s_curve.curves_limit[RISE] = rise_time;
            s_curve.curves_limit[PLATEAU] = steps - (rise_time + break_time);
            s_curve.curves_limit[BREAK] = break_time;
        } else {
            // Sinon il n'y a pas de plateau et les temps de monté et de freinage sont raccourci
            s_curve.curves_limit[RISE] = steps / 2;
            s_curve.curves_limit[PLATEAU] = 0;
            s_curve.curves_limit[BREAK] = steps / 2;
        }
    }

    public:

    int getSpeedFromAdvancement(int64_t abs_steps) {
        int64_t delta_steps = abs_steps - last_abs_steps;   // Avancement
        int64_t speed = 0;

        if (delta_steps < 0)   delta_steps *= -1;

        // Avant la courbe de monté (bug)
        if (delta_steps < 0) {
            speed = s_curve.min_starting_speed;
        }
        // Monté
        else if (delta_steps < s_curve.curves_limit[RISE]) {
            speed = riseCurve(delta_steps);
        }
        // Plateau
        else if (delta_steps < s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU]) {
            speed = MAX_SPEED;
        }
        // Freinage
        else if (delta_steps < s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU] + s_curve.curves_limit[BREAK]) {
            speed = breakCurve(delta_steps - (s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU]));
        }
        // Après la courbe de freinage (bug)
        else {
            speed = 0;
        }

        if (s_curve.dir == BACKWARD)    speed *= -1;

        return speed;
    }

    void move(int64_t steps) {
        buildSpeedCurve(steps);
    }

    void rotate(bool proco, float degrees) {
        //% du périmètre * périmètre de la rotation * conversion cm/pas
        int64_t steps = (degrees / 360) * 3.1415 * DIST_BETWEEN_WHEELS * 1024; // 1024 = nombres de pas par mm
        if (proco)  steps *= -1;    // En fonction du moteur : direction inverse ou non
        buildSpeedCurve(steps);
    }
};