//
// Created by vassilis on 09.03.16.
//

#include "motorController.h"

// Constructor
motorController::motorController(const char *device, int baud, char parity, int data_bit, int stop_bit, int slave) {
    thisModbusDevice = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);
    slaveAddress = slave;
}

// Destructor
motorController::~motorController() {

}


/*  NAME:               int initController(modbus_t *slave, int slaveAddress)
 *
 *  DESCRIPTION:        Initialize a slave device
 *
 *  INPUTS:
 *  -modbus_t *slave    The slave device structure of the object
 *
 *  -int slaveAddress   The slave address of the object (1 - 247 in modbus)
 *
 *  OUTPUTS:            The error code:  0 for no errors, -1 for errors and
 *                      prints the error message
 *
 *
 */

int motorController::initController() {

    if (modbus_set_slave(thisModbusDevice, slaveAddress)) {
        fprintf(stderr, "Failed to set modbus slave address\n");
        return -1;
    }


    if (modbus_connect(thisModbusDevice)) {
        fprintf(stderr, "Unable to connect to modbus server");
        return -1;
    }

    return 0;
}


/*  NAME:               motorsOn(bool OnOff)
 *
 *  DESCRIPTION:        Switch ON or OFF the 3 motors on the
 *                      slave device
 *
 *  INPUTS:
 *  -bool OnOff         true for ON, false for OFF
 *
 *
 *  OUTPUTS:            -
 *
 *  NOTES:              Bitwise operation; 7d = 0b0111 ->
 *                      M1, M2, M3 - ON
 *
 *
 */

void motorController::motorsOn(bool OnOff) {

    if (OnOff)
        modbus_write_register(thisModbusDevice, M_ENABLE, 7);
    else
        modbus_write_register(thisModbusDevice, M_DISABLE, 7);


}

/*  NAME:               setPosition(uint16_t M, float pos)
 *
 *  DESCRIPTION:        Sets a target position for a motor, which is immediately
 *                      set to motion towards it
 *
 *  INPUTS:
 *  -uint16_t M         The target motor
 *
 *  -float pos          The target position
 *
 *
 *  OUTPUTS:            -
 *
 *  PROCESS:            [1] Sets the base register - 1084-1085
 *                          for M1, 1086-1087 for M2 and 1088-1089
 *                          for M3. M1 = 0 so the correct offset
 *                          is ensured by "baseRegister = M_POS_ABS + M*2;"
 *                      [2] Convert the desired position to 2 bytes in
 *                          modbus format
 *                      [3] Write the converted value to the appropriate
 *                          register on the slave device
 */

void motorController::setPosition(uint16_t M, float pos) {

    uint16_t *pos_HEX, baseRegister;
    baseRegister = M_POS_ABS + M * 2;

    modbus_set_float(pos, pos_HEX);
    modbus_write_registers(thisModbusDevice, baseRegister, 2, pos_HEX);

}


void motorController::setMaxVelocity(uint16_t M, float vel) {

    uint16_t *pos_HEX, baseRegister;
    baseRegister = M_VMAX + M * 2;

    modbus_set_float(vel, pos_HEX);
    modbus_write_registers(thisModbusDevice, baseRegister, 2, pos_HEX);

}


/*  NAME:               getPosition(uint16_t M)
 *
 *  DESCRIPTION:        Gets the current position of a certain motor
 *                      attached to the modbus object
 *
 *
 *  INPUTS:
 *  -uint16_t M         The target motor
 *
 *  OUTPUTS:
 *    RETURN:
 *    - float           The current position of the motor as a float
 *                      value
 *
 *  PROCESS:           [1]  Sets the base register - 1020-1021
 *                          for M1, 1022-1023 for M2 and 1024-1025
 *                          for M3. M1 = 0 so the correct offset
 *                          is ensured by "baseRegister = M_POS_ACT + M*2;"
 *                     [2]  The registers are read and the value is put into
 *                          the slaveRegister variable
 *                     [3]  The 2-byte value is converted to float and returned
 *                          by the function
 */

float motorController::getPosition(uint16_t M) {
    int n;
    uint16_t slaveRegister[2], baseRegister;

    baseRegister = M_POS_ACT + M * 2;
    n = modbus_read_registers(thisModbusDevice, baseRegister, 2, slaveRegister);

    if (n <= 0) {
        fprintf(stderr, "Unable to read modbus registers\n");
        errno = -1;
    }

    return modbus_get_float(slaveRegister);
}

/*  NAME:               getStatus(uint16_t M)
 *
 *  DESCRIPTION:        Reads the status register of a motor
 *
 *
 *
 *  INPUTS:
 *  -uint16_t M         The target motor
 *
 *  OUTPUTS:
 *    RETURN:
 *    - int             The current status of the motor:
 *                      0 – drive turned off (EN signal inactive)
 *                      1 – drive turned on, no motion (EN signal active)
 *                      2 – drive in set velocity mode
 *                      3 – drive in motion to set position mode
 *                      4 – drive achieved the set position
 *                      5 – error of achieving set position (for operation with
 *                      encoder)
 *                      6 – drive in homing mode
 *                      8 – drive in position correction mode (for operation
 *                      with encoder)
 *                      9 - drive achieved limit position L while motion towards
 *                      negative position value (by program or proximity
 *                      sensor signal KL)
 *                      10 - drive achieved limit position R while motion
 *                      towards positive position value (by program or
 *                      proximity sensor signal KR)
 *
 *
 *  PROCESS:           [1]  Sets the correct register - 1010 for M1,
 *                          1011 for M2 and 1012 for M3. M1 = 0 so the
 *                          correct offset is ensured by
 *                          baseRegister = M_STATUS + M;
 *                     [2]  The register is read and the value is put into
 *                          the slaveRegister variable
 *                     [3]  The byte value is returned by the function
 */

int motorController::getStatus(uint16_t M) {
    uint16_t slaveRegister[2], baseRegister = M_STATUS + M;
    modbus_read_registers(thisModbusDevice, baseRegister, 1, slaveRegister);
    return slaveRegister[0];
}