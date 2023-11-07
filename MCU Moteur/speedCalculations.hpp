#pragma once

#include "picoIncludes.hpp"
#include "lightMaths.hpp"

#define DIST_BETWEEN_WHEELS 300     // En mm
#define FORWARD     1
#define BACKWARD    0

#define RISE_COEFF   0.01
#define BREAK_COEFF -0.01

#define MARGE   20  

#define RISE        0
#define PLATEAU     1
#define BREAK       2

//On considère que la roue est capable d'aller jusqu'à 25 tr/s

// Structure pour un asservissement PID
struct PidController {
  float kp = 1;
  float ki = 0.1;
  float kd = 0;
  float error = 0;            // Différence entre la consigne et la mesure
  float deltaError = 0;       // Différence entre l'erreur précédente et l'erreur
  float sumError = 0;         // Somme des erreurs
  float previous_error = 0;   // Erreur précédente
};

// Commande et asservissement moteur
struct Motor  {
  float consigne = 0;             // Consigne en tr/s
  PidController pid;            // Structure vers des données d'asservissement
  int pwm = 0;                // Valeur PWM calculé pour le moteur
  float speed = 0;
  bool direction = FORWARD;     // Sens de rotation moteur
};

//Données de courbe
struct SpeedCurve {
    uint16_t min_starting_speed = 50;
    float max_speed = MAX_SPEED;
    int64_t curves_limit[3] = { 0, 0, 0 };
    bool dir = FORWARD;
    bool arrived = false;
    int64_t objective = 0;
};

float PID(PidController& pid)  {
  
  float correction = 0;

  pid.deltaError = pid.error - pid.previous_error;
  pid.sumError  += pid.error;

  // On évite un trop gros dépassement. Ici, si l'erreur est la plus grande et avec un coeff i=1,
  // le moteur aura la consigne maximum en un seul cycle d'exécution
  pid.sumError = constrain(pid.sumError, - PWM_WRAP_VALUE * pid.ki, PWM_WRAP_VALUE * pid.ki);
  
  correction = pid.kp * pid.error + pid.ki * pid.sumError + pid.kd * pid.deltaError;
  pid.previous_error = pid.error;

  return correction;
}

class MotionControl {
  private:

    int64_t last_abs_steps = 0;  // Nombre de pas absolus de puis la dernière construction de courbe
    SpeedCurve s_curve;          // Courbe
    bool proco = 0;

    float riseCurve(uint64_t steps) {
        return RISE_COEFF * steps + s_curve.min_starting_speed;
    }

    float breakCurve(uint64_t steps) {
        return BREAK_COEFF * steps + s_curve.max_speed;
    }

    int riseRecursiveCurve(float speed) {
        return speed / RISE_COEFF - s_curve.min_starting_speed / RISE_COEFF;
    }

    int breakRecursiveCurve(float speed) {
        return speed / BREAK_COEFF + s_curve.max_speed / -BREAK_COEFF;
    }

    void buildSpeedCurve(int64_t steps_to_travel, int64_t abs_step) {

        if (proco)  steps_to_travel *= -1;

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

public:

    float getSpeedFromAdvancement(int64_t abs_steps) {
        int64_t delta_steps = abs_steps - last_abs_steps;   // Avancement
        float speed = 0;

        if (s_curve.arrived)    return 0;

        if (s_curve.dir == BACKWARD)   delta_steps *= -1;


        if (delta_steps < 0)    {
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
        else if (delta_steps < s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU] + s_curve.curves_limit[BREAK] + 300) {
            speed = breakCurve(delta_steps - (s_curve.curves_limit[RISE] + s_curve.curves_limit[PLATEAU]));
        }
        else{
            speed = -s_curve.min_starting_speed;
        }

        if (s_curve.dir == BACKWARD)    speed *= -1;

        return speed;
    }

    void move(int64_t steps_to_travel, int64_t actual_abs_step) {
        buildSpeedCurve(steps_to_travel, actual_abs_step);
    }

    void rotate(float degrees, int64_t actual_abs_step) {
        //% du périmètre * périmètre de la rotation * conversion cm/pas
        int64_t steps = (degrees / 360) * 3.1415 * DIST_BETWEEN_WHEELS * 1024; // 1024 = nombres de pas par mm
        buildSpeedCurve(steps, actual_abs_step);
    }

    int forceStop()    {
        s_curve.arrived = true;
        return 0;
    }

    MotionControl(uint16_t max_speed, bool set_proco) {
        s_curve.max_speed = max_speed;
        proco = set_proco;
    }
};