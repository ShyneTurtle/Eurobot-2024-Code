#pragma once

#include "platforme.hpp"
#include "vl53l5cx_api.hpp"
#include <math.h>



class dataProcessing {

private:

    VL53L5CX_ResultsData *data;
    uint8_t resolution = 16;
    uint8_t line_length = 4;

    uint16_t *line = new uint16_t[line_length]   {0};   // Ligne

    struct point    {
        uint16_t raw_dist = 0;  // Distance mesuré par le capteur

    };
    //45° / 8 = 5,625°
    // facteur 9,85
    //pow(dist * 0,0985, 2);

    enum Rotation {
        ROTATE_0DEG,
        ROTATE_90DEG,
        ROTATE_180DEG,
        ROTATE_270DEG
    };

    void rotate(int16_t* src, Rotation rot) {
        int16_t dst[resolution];

        for (uint8_t i = 0; i<resolution; i++) {
            if (rot == ROTATE_0DEG) {
                dst[i] = src[i];
            }
            if (rot == ROTATE_180DEG) {
                dst[i] = src[resolution-1 - i];
            }
            if (rot == ROTATE_90DEG) {
                uint8_t offset_col = (resolution-1) - i%line_length * line_length;
                uint8_t offset_line = (line_length-1) - i/line_length;
                dst[i] = src[offset_col - offset_line];
            }
            if (rot == ROTATE_270DEG) {

            }
        }

        memcpy(src, dst, resolution*2);
    }

    void draw(int16_t* buf)  {
        for (int8_t l = 0; l < line_length; l++)   {   // Pour chaque lignes
            for (uint8_t c = 0; c < line_length; c++)   {   // Pour chaque colonnes
                if (buf[l*line_length + c] != -1)   // Donnée valide
                    printf("%04d\t", buf[l*line_length + c]);
                else
                    printf("xxxx\t");
            }
            printf("\n");
        }
    }

public:

    void queryData() {
        int16_t tab[resolution];

        // Copy and remove invalid data
        for (size_t i = 0; i < resolution; i++) {
            if (data->target_status[i] == 5) 
                tab[i] = data->distance_mm[i];
            else 
                tab[i] = -1;
        }

        rotate(tab, ROTATE_90DEG);

        draw(tab);
    }



    dataProcessing(VL53L5CX_ResultsData *setData, bool setReso8x8)    {
        data = setData;

        if (setReso8x8) {
            resolution = 64;
            line_length = 8;
        }    
        else {
            resolution = 16;
            line_length = 4;
        }
    }

};
