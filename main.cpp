#include "fileParser.h"
#include <iostream>
#include <unistd.h> // sleep
#include "motorController.h"


#define MB_BITRATE 38400
#define MB_DATABITS 8
#define MB_STOPBITS 1
#define MB_PARITY 'N'

using namespace std;

int main(int argc, char *argv[]) {

    char* filename = argv[1];
    int componentSum = 1, fiducialSum = 1, i;
    float coordinates[100][2] = {0};
    float fiducials[10][2] = {0};
    
//    motorController topTable("/dev/ttyUSB0", MB_BITRATE, MB_PARITY, MB_DATABITS, MB_STOPBITS,1);  // 38400 bps, 8-N-1
//    topTable.initController();
//    topTable.motorsOn(false);
//    topTable.setMaxVelocity(M1, 15);
//    topTable.setPosition(M1, 10.6);


    getCoordinatesfromFile(filename,&componentSum, &fiducialSum, coordinates, fiducials);

    return 0;
}