#pragma once

#include "picoIncludes.hpp"
#include "signalProcessing.hpp"
#include "speedCalculations.hpp"

void routine(SignalProcessing& signal, Motor& motorControl, bool proco) {

    motorControl.pid.error = signal.speed_rotation - motorControl.consigne;
    motorControl.speed = motorControl.consigne - PID(motorControl.pid); // Calcul de correction
    motorControl.pwm = map(motorControl.speed, -MAX_SPEED, MAX_SPEED, -PWM_WRAP_VALUE, PWM_WRAP_VALUE);   //Conversion tr/s => pwm

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