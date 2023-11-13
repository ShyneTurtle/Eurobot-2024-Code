#pragma once

#include "picoIncludes.hpp"
#include "motorControl.hpp"

// Programme en dev. Censé construire une courbe de vitesse en fonction de la distance à parcourir

#define FORWARD     1
#define BACKWARD    0

#define RISE_COEFF   0.0003
#define BREAK_COEFF -0.0003

#define SPEED_CONTROL       0
#define POSITION_CONTROL    1
#define SPEED_CURVE_CONTROL 2

#define RISE        0
#define PLATEAU     1
#define BREAK       2

struct SpeedCurve {
    float min_starting_speed = MAX_SPEED * 0.2;
    float max_speed = 0;
    int64_t curves_limit[3] = { 0, 0, 0 };
    bool dir = FORWARD;
    bool arrived = false;
    int64_t objective = 0;
};

class MotionControl {
    private:

    int64_t last_abs_steps = 0; // Nombre de pas absolus de puis la dernière construction de courbe
    float consigne = 0;       // Consigne en vitesse (tr/s) ou position (nb de pas)
    bool proco = 0;
    SpeedCurve s_curve;          // Courbe
    uint8_t motion_mode = SPEED_CONTROL;

    // === fonction des courbes : vitesse <=> position ===

    float riseCurve(uint64_t steps) {
        return RISE_COEFF * steps + s_curve.min_starting_speed;
    }

    float breakCurve(uint64_t steps) {
        return BREAK_COEFF * steps + s_curve.max_speed;
    }

    int riseRecursiveCurve(uint64_t speed) {
        return speed / RISE_COEFF - s_curve.min_starting_speed / RISE_COEFF;
    }

    int breakRecursiveCurve(uint64_t speed) {
        return speed / BREAK_COEFF + s_curve.max_speed / -BREAK_COEFF;
    }

    // === utilisation des courbes et déduction de vitesse ===

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

    float getSpeedFromCurveAdvancement(int64_t abs_steps) {
        int64_t delta_steps = abs_steps - last_abs_steps;   // Avancement
        float speed = 0;

        if (s_curve.dir == BACKWARD)   delta_steps *= -1;

        // Avant la courbe de monté (ne devrait pas arriver)
        if (delta_steps <= 0) {
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
            motion_mode = POSITION_CONTROL;
            if (s_curve.dir == FORWARD)     consigne = last_abs_steps + s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU] + s_curve.curves_limit[BREAK];
            else                            consigne = last_abs_steps - (s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU] + s_curve.curves_limit[BREAK]);
        }

        if (s_curve.dir == BACKWARD)    speed *= -1;

        return speed;
    }

public:

    // Déduis le mode de control du robot et initialise le robot au prochain déplacement
    // en fonction de l'ordre donné
    // @param order ordre sur le prochain déplacement du robot
    // @param order_value valeur associé à l'ordre, typiquement une distance (en cm ou en °)
    // @param actual_abs_step avancement du robot en nb de pas depuis le lancement
    // @param proco le core controlant le moteur
    void setOrder(uint8_t order, float order_value, int actual_abs_step) {
        last_abs_steps = actual_abs_step;   // On retient le "point de départ"
        int64_t steps = 0;
        if (proco == 0)      order_value = -order_value;    // Inverse le sens de rotation

        switch(order)   {
        case MOVE:
            motion_mode = SPEED_CURVE_CONTROL;      // Mode d'asservissement en COURBE DE VITESSE
            last_abs_steps = actual_abs_step;
            // Conversion mm en nb de pas
            // nb de pas par mm * mm
            steps = (PULSE_PER_100CM / 1000.) * order_value;
            buildSpeedCurve(order_value, actual_abs_step);
            break;
        case ROTATE:
            motion_mode = SPEED_CURVE_CONTROL;      // Mode d'asservissement en COURBE DE VITESSE
            // Conversion ° en nb de pas
            // nb de pas par cm * nb de cm pour un tour * nb de tour
            steps = (PULSE_PER_100CM / 100.) * ROBOT_PERIMETRE * (order_value / 360.);
            if (proco == 0) steps = -steps; // Inverse le sens de rotation
            buildSpeedCurve(steps, actual_abs_step);
            break;
        case FORCE_STOP:
            motion_mode = POSITION_CONTROL;         // Mode d'asservissement en POSITION
            consigne = actual_abs_step;
            break;
        case SET_POS_PID:
            motion_mode = POSITION_CONTROL;         // Mode d'asservissement en POSITION
            consigne = order_value;
            break;
        case SET_SPEED_PID:
            motion_mode = SPEED_CONTROL;            // Mode d'asservissement en VITESSE
            // Conversion milli tour/s en tr/s
            consigne = order_value / 1000.;
            break;
        }
    }

    // Calcul la vitesse moteur à appliquer selon l'ordre donnée préalablement
    // La vitesse ainsi calculé dépend de l'ordre, et de la position ou la vitesse du robot
    // @param signal signaux encodeur
    // @param motorControl données moteur et asservissement (sera modifié par la fonction)
    // @param proco n° du core controlant le moteur
    void computeSpeedFromAdvancement(SignalProcessing& signal, Motor& motorControl) {
        switch(motion_mode) {
        case SPEED_CONTROL:
            // La consigne de la VITESSE est constante et fixé par l'ordre
            motorControl.consigne = consigne;
            // Calcul de la vitesse à appliquer au moteur avec un asservissement PID en VITESSE
            pidSpeedControl(signal, motorControl);
            break;
        case POSITION_CONTROL:
            // La consigne de la POSITION est constante et fixé par l'ordre
            motorControl.consigne = consigne;
            // Calcul de la vitesse à appliquer au moteur avec un asservissement PID en POSITION
            pidPositionControl(signal, motorControl);
            break;
        case SPEED_CURVE_CONTROL:
            // On calcul la vitesse que le moteur doit avoir selon l'avancement du robot
            // (la consigne de vitesse est variable et dépend de l'avancement du robot)
            motorControl.consigne = getSpeedFromCurveAdvancement(signal.absStep);
            // Calcul de la vitesse à appliquer au moteur avec un asservissement PID en vitesse
            pidSpeedControl(signal, motorControl);
            break;
        }
    }

    MotionControl(float max_speed, bool set_proco) {
        s_curve.max_speed = max_speed;
        proco = set_proco;
    }
};