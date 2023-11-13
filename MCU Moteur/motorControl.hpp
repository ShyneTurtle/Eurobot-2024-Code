#pragma once

#include "picoIncludes.hpp"
#include "signalProcessing.hpp"

// Structure pour un asservissement PID
struct PidController {
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float error = 0;            // Différence entre la consigne et la mesure
  float deltaError = 0;       // Différence entre l'erreur précédente et l'erreur
  float sumError = 0;         // Somme des erreurs
  float previous_error = 0;   // Erreur précédente
};

// Commande et asservissement moteur
struct Motor  {
  float consigne = 0;                   // Consigne en tr/s ou nb de pas
  PidController pid_position_control;   // Structure vers des données d'asservissement en position
  PidController pid_speed_control;      // Structure vers des données d'asservissement en vitesse
  int pwm = 0;                          // Valeur PWM calculé pour le moteur
  float speed = 0;                      // Vitesse (en tr/s)
  bool direction = FORWARD;             // Sens de rotation moteur
};

// Calcul une correction pid
// @param pid paramètre de correcction
// @return la correction
float PID(PidController& pid)  {
  
  float correction = 0;
    // L'erreur pid doit être renseigné préalablement
  pid.deltaError = pid.error - pid.previous_error;
  pid.sumError  += pid.error;

  // On évite un trop gros dépassement. Ici, si l'erreur est la plus grande et avec un coeff i=1,
  // le moteur aura la consigne maximum en un seul cycle d'exécution
  pid.sumError = constrain(pid.sumError, - PWM_WRAP_VALUE * pid.ki, PWM_WRAP_VALUE * pid.ki);
  
  correction = pid.kp * pid.error + pid.ki * pid.sumError + pid.kd * pid.deltaError;
  pid.previous_error = pid.error;

  return correction;
}

void setPIDcoeff(PidController& pid, float p, float i, float d) {
    pid.kp = p / PULSE_MULT;
    pid.ki = i / PULSE_MULT;
    pid.kd = d / PULSE_MULT;
}

// Calcul la vitesse à appliquer à un moteur selon la POSITION (odométrie) du robot
// et avec un asservissement pid
// @param signal signaux encodeur
// @param motorControl données moteur et asservissement
// @param proco n° du core controlant le moteur
void pidPositionControl(SignalProcessing& signal, Motor& motorControl) {

    motorControl.pid_position_control.error = motorControl.consigne - signal.absStep;
    motorControl.speed = PID(motorControl.pid_position_control); // Calcul de correction
    motorControl.pwm = map(motorControl.speed, -MAX_SPEED, MAX_SPEED, -PWM_WRAP_VALUE, PWM_WRAP_VALUE);   //Conversion tr/s => pwm

    //printf("abs: %ld ; s: %.3f ; er: %.3f \n", (int32_t)signal.absStep, motorControl.speed, motorControl.pid_position_control.error);

    //On tiens compte du sens de rotation moteur après la correction PID
    if (motorControl.pwm < 0) {
      motorControl.pwm *= -1;
      motorControl.direction = BACKWARD;
    }
    else  {
      motorControl.direction = FORWARD;
    }

    motorControl.pwm = constrain(motorControl.pwm, 0, PWM_WRAP_VALUE);
}

// Calcul la vitesse à appliquer à un moteur selon la VITESSE du robot
// et avec un asservissement pid. Les résultats sont enregistré dans motorControl.
// @param signal signaux encodeur
// @param motorControl données moteur et asservissement
// @param proco n° du core controlant le moteur
void pidSpeedControl(SignalProcessing& signal, Motor& motorControl) {

    motorControl.pid_speed_control.error = signal.speed_rotation - motorControl.consigne;
    motorControl.speed = motorControl.consigne - PID(motorControl.pid_speed_control); // Calcul de correction
    motorControl.pwm = map(motorControl.speed, -MAX_SPEED, MAX_SPEED, -PWM_WRAP_VALUE, PWM_WRAP_VALUE); // Conversion tr/s => pwm

    //On tiens compte du sens de rotation moteur après la correction PID
    if (motorControl.pwm < 0) {
      motorControl.pwm *= -1;
      motorControl.direction = BACKWARD;
    }
    else  {
      motorControl.direction = FORWARD;
    }

    motorControl.pwm = constrain(motorControl.pwm, 0, PWM_WRAP_VALUE);
}