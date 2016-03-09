//
// Created by vassilis on 09.03.16.
//

#ifndef SELECTIVE_SOLDERING_MOTORCONTROLLER_H
#define SELECTIVE_SOLDERING_MOTORCONTROLLER_H

#include <iostream>
#include <modbus/modbus.h>

#define M1          0
#define M2          1
#define M3          2
#define M_ENABLE    1014
#define M_DISABLE   1015
#define M_VMAX      1052
#define M_POS_ACT   1020
#define M_POS_ABS   1084
#define M_STATUS    1010
#define ON          1
#define OFF         0

#define VELOCITY 15

using namespace std;


class motorController {

public:

    motorController(const char*, int, char, int, int, int); // Constructor
    ~motorController(); // Destructor
    int initController();
    void motorsOn(bool);
    void setPosition(uint16_t, float);
    void setMaxVelocity(uint16_t, float);
    float getPosition(uint16_t);
    int getStatus(uint16_t);


private:

    int slaveAddress;
    modbus_t *thisModbusDevice;




};


#endif //SELECTIVE_SOLDERING_MOTORCONTROLLER_H
