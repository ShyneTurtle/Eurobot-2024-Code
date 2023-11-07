#pragma once

#include "picoIncludes.hpp"

// Ordre de communications entre les deux core
#define MOVE        0b00
#define ROTATE      0b01
#define FORCE_STOP  0b10

// Communication i2c :
//  reg0        : l'ordre
//  reg1 à reg4 : valeur 32 bits
// Contient les données de l'esclave i2c
struct i2cSlaveData {
    volatile uint8_t registers[256] = { 20 };
    volatile uint8_t reg_address = 0;
    volatile bool reg_address_written = false;
    volatile bool new_data = false;
};

i2cSlaveData i2c_SD;

// Un échange (32 bits) entre les 2 core comporte :
//  [31..30] : l'ordre
//  [29]     : le signe de la valeur
//  [28..0]  : la valeur

bool talk(int order, int32_t value) {
    bool succeed = true;
    bool value_sign = 0;
    if (multicore_fifo_wready())    {
        if (value < 0)  {
            value *= -1;
            value_sign = 1;
        }
        uint32_t data = (order << 30) | (value_sign<<29) | (value & 0x3fffffff);
        succeed = multicore_fifo_push_timeout_us(data, 500);
    }
    else    {
        succeed = false;
    }

    return succeed;
}

bool listen(int& order, int32_t& value)    {
    bool succeed = true;
    uint32_t data = 0;

    if (multicore_fifo_rvalid())    {
        succeed = multicore_fifo_pop_timeout_us(500, &data);
        order = (data>>30);
        if (data & 0x20000000)  value = -(data & 0x1fffffff);
        else                    value =   data & 0x1fffffff;
    }
    else    {
        succeed = false;
    }

    return succeed;
}


// Routine d'interruption i2c
void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)    {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // Ecriture de donnée par le maître
        if (!i2c_SD.reg_address_written) {
            // Le première octet correspond à l'addresse du registre
            i2c_SD.reg_address = i2c_read_byte_raw(i2c);
            i2c_SD.reg_address_written = true;
        } else {
            // Enregistrement
            i2c_SD.registers[i2c_SD.reg_address] = i2c_read_byte_raw(i2c);
            i2c_SD.reg_address++;
        }
        break;
    case I2C_SLAVE_REQUEST:
        i2c_write_byte_raw(i2c, i2c_SD.registers[i2c_SD.reg_address]);
        i2c_SD.reg_address++;
        break;
    case I2C_SLAVE_FINISH: // Fin de communication
        i2c_SD.reg_address_written = false;
        i2c_SD.new_data = true;
        break;
    }
}