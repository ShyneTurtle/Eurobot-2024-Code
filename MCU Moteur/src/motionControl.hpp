#pragma once

#include <pico/stdlib.h>
#include <stdio.h>

//Programme en dev. Censé construire une courbe de vitesse en fonction de la distance à parcourir

#define MAX_SPEED   0xffff
#define DIST_BETWEEN_WHEELS 300     //En mm
#define FORWARD     1
#define BACKWARD    0

#define RISE_COEFF  1.0
#define BREAK_COEFF -1.0

#define RISE        0
#define PLATEAU     1
#define BREAK       2

struct speedCurve    {
    uint16_t minStartingSpeed = MAX_SPEED;
    uint16_t maxSpeed = 0;
    int64_t curvesLimit[3] = {0,0,0};
    bool dir = FORWARD;
};

class motionControl {
private:

    int64_t lastAbsFoots = 0;  //Nombre de pas absolus de puis la dernière construction de courbe
    speedCurve sCurve;          //Courbe

    int RiseCurve(uint64_t x)    {
        return RISE_COEFF * x + sCurve.minStartingSpeed;
    }

    int BreakCurve(uint64_t x)   {
        return BREAK_COEFF * x + sCurve.maxSpeed;
    }

    int RiseCurve_R(uint64_t y)  {
        return y / RISE_COEFF + sCurve.minStartingSpeed / RISE_COEFF;
    }

    int BreakCurve_R(uint64_t y) {
        return y / BREAK_COEFF + sCurve.maxSpeed / BREAK_COEFF;
    }

    void BuildSpeedCurve(int64_t foots) {
        lastAbsFoots = foots;
        int riseTime = RiseCurve_R(sCurve.maxSpeed);    //Nombre de pas à atteindre avant la vitesse max
        int breakTime = BreakCurve_R(0);       //Nombre de pas à atteindre avant l'arrêt

        if (foots < 0)  {   //On s'implifie le code en gardant le signe pour plus tard
            foots *= -1;
            sCurve.dir = BACKWARD;
        }

        if (foots > (riseTime + breakTime)) {
            //Si il y a suffisamment de pas pour un plateau on en prends compte
            sCurve.curvesLimit[RISE] = riseTime;
            sCurve.curvesLimit[PLATEAU] = foots - (riseTime + breakTime);
            sCurve.curvesLimit[BREAK] = breakTime;
        }
        else    {
            //Sinon il n'y a pas de plateau et les temps de monté et de freinage sont raccourci
            sCurve.curvesLimit[RISE] = foots / 2;
            sCurve.curvesLimit[PLATEAU] = 0;
            sCurve.curvesLimit[BREAK] = foots / 2;
        }
    }

public:

    int GetSpeedFromAdvancement(int64_t absFoots)    {
        int64_t advFoots = absFoots - lastAbsFoots;   //Avancement
        int64_t speed = 0;

        if (advFoots < 0)   advFoots *= -1;

        //Avant la courbe de monté (bug)
        if (advFoots < 0) {
            speed = sCurve.minStartingSpeed;
        }
        //Monté
        else if (advFoots < sCurve.curvesLimit[RISE])  {
            speed = RiseCurve(advFoots);
        }
        //Plateau
        else if (advFoots < sCurve.curvesLimit[RISE] + sCurve.curvesLimit[PLATEAU])  {
            speed = MAX_SPEED;
        }
        //Freinage
        else if (advFoots < sCurve.curvesLimit[RISE] + sCurve.curvesLimit[PLATEAU] + sCurve.curvesLimit[BREAK]) {
            speed = BreakCurve(advFoots - (sCurve.curvesLimit[RISE] + sCurve.curvesLimit[PLATEAU]));
        }
        //Après la courbe de freinage (bug)
        else {
            speed = 0;
        }

        if (sCurve.dir == BACKWARD)    speed *= -1;

        return speed;
    }

    void Move(int64_t foots) {
        BuildSpeedCurve(foots);
    }

    void Rotate(bool proco, float degrees) {
        //% du périmètre * périmètre de la rotation * conversion cm/pas
        int64_t foots = (degrees / 360) * 3.1415 * DIST_BETWEEN_WHEELS * 1024; //1024 = nombres de pas par mm
        if (proco)  foots *= -1;    //En fonction du moteur : direction inverse ou non
        BuildSpeedCurve(foots);
    }
};