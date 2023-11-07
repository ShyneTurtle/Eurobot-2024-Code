#pragma once

#include <pico/stdlib.h>
#include <stdio.h>

// Programme en dev. Censé construire une courbe de vitesse en fonction de la distance à parcourir

#define DIST_BETWEEN_WHEELS 300     // En mm
#define FORWARD     1
#define BACKWARD    0

#define RISE_COEFF   0.01
#define BREAK_COEFF -0.01

#define RISE        0
#define PLATEAU     1
#define BREAK       2

struct SpeedCurve {
    uint16_t min_starting_speed = 50;
    uint16_t max_speed = 0;
    int64_t curves_limit[3] = { 0, 0, 0 };
    bool dir = FORWARD;
    bool arrived = false;
    int64_t objective = 0;
};

class MotionControl {
    private:

    int64_t last_abs_steps = 0;  // Nombre de pas absolus de puis la dernière construction de courbe
    SpeedCurve s_curve;          // Courbe

    int riseCurve(uint64_t steps) {
        return RISE_COEFF * steps + s_curve.min_starting_speed;
    }

    int breakCurve(uint64_t steps) {
        return BREAK_COEFF * steps + s_curve.max_speed;
    }

    int riseRecursiveCurve(uint64_t speed) {
        return speed / RISE_COEFF - s_curve.min_starting_speed / RISE_COEFF;
    }

    int breakRecursiveCurve(uint64_t speed) {
        return speed / BREAK_COEFF + s_curve.max_speed / -BREAK_COEFF;
    }

    void buildSpeedCurve(int64_t steps_to_travel, int64_t abs_step) {
        s_curve.objective = steps_to_travel;
        s_curve.arrived = false;
        last_abs_steps = abs_step;
        int rise_time  = riseRecursiveCurve(s_curve.max_speed);    // Nombre de pas à atteindre avant la vitesse max
        int break_time = breakRecursiveCurve(0);       // Nombre de pas à atteindre avant l'arrêt

        if (steps_to_travel < 0) {   // On s'implifie le code en gardant le signe pour plus tard
            steps_to_travel *= -1;
            s_curve.dir = BACKWARD;
        }

        if (steps_to_travel > (rise_time + break_time)) {
            // Si il y a suffisamment de pas pour un plateau on en prends compte
            s_curve.curves_limit[RISE]    = rise_time;
            s_curve.curves_limit[PLATEAU] = steps_to_travel - (rise_time + break_time);
            s_curve.curves_limit[BREAK]   = break_time;
        } else {
            // Sinon il n'y a pas de plateau et les temps de monté et de freinage sont raccourci

            s_curve.curves_limit[RISE]    = steps_to_travel / 2;
            s_curve.curves_limit[PLATEAU] = 0;
            s_curve.curves_limit[BREAK]   = steps_to_travel / 2;
        }
    }

    int64_t abs(int64_t x)  {
        if (x < 0)  return -x;
        else        return  x;
    }

public:

    int getSpeedFromAdvancement(int64_t abs_steps) {
        int64_t delta_steps = abs_steps - last_abs_steps;   // Avancement
        int32_t speed = 0;

        if (s_curve.arrived)    return 0;

        if (s_curve.dir == BACKWARD)   delta_steps *= -1;

        // Avant la courbe de monté (ne devrait pas arriver)
        if (delta_steps < 0) {
            speed = s_curve.min_starting_speed;
        }
        // Monté
        else if (delta_steps < s_curve.curves_limit[RISE]) {
            speed = riseCurve(delta_steps);
        }
        // Plateau
        else if (delta_steps < s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU]) {
            speed = s_curve.max_speed;
        }
        // Freinage
        else if (delta_steps < s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU] + s_curve.curves_limit[BREAK]) {
            speed = breakCurve(delta_steps - (s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU]));
        }
        // Après la courbe de freinage (ne devrait pas arriver)
        else {
            speed = 0;
        }


        if (s_curve.dir == BACKWARD)    speed *= -1;

        return speed;
    }

    void move(int64_t steps_to_travel, int64_t actual_abs_step) {
        buildSpeedCurve(steps_to_travel, actual_abs_step);
    }

    void rotate(bool proco, float degrees, int64_t actual_abs_step) {
        //% du périmètre * périmètre de la rotation * conversion cm/pas
        int64_t steps = (degrees / 360) * 3.1415 * DIST_BETWEEN_WHEELS * 1024; // 1024 = nombres de pas par mm
        if (proco)  steps *= -1;    // En fonction du moteur : direction inverse ou non
        buildSpeedCurve(steps, actual_abs_step);
    }

    int forceStop()    {
        s_curve.arrived = true;
        return 0;
    }

    MotionControl(uint16_t max_speed) {
        s_curve.max_speed = max_speed;
    }
};