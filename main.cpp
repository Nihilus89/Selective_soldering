#include "fileParser.h"
#include <iostream>
#include <unistd.h> // sleep
#include "motorController.h"
#include "inspectionCamera.h"


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
    float currentFiducials[10][2] = {0};

//    motorController topTable("/dev/ttyUSB0", MB_BITRATE, MB_PARITY, MB_DATABITS, MB_STOPBITS,1);  // 38400 bps, 8-N-1
//    topTable.initController();
//    topTable.motorsOn(false);
//    topTable.setMaxVelocity(M1, 15);
//    topTable.setPosition(M1, 10.6);

    getCoordinatesfromFile(filename,&componentSum, &fiducialSum, coordinates, fiducials);


    inspectionCamera webcam(0);
    Mat src;
    src = webcam.captureImage(30);

    for(i=0; i<fiducialSum; i++)
    {
        while(webcam.numCircles != 1)
        {
            src = webcam.captureImage(30);
            webcam.locateFiducial(src,fiducials[i][0],fiducials[i][1],3,5);
            //cout << fiducials[i][0] << endl;
            imshow("source",src);
        }


    currentFiducials[i][0] = webcam.fiducial_x;
    currentFiducials[i][1] = webcam.fiducial_y;
//    cout << webcam.fiducial_x << " " << webcam.fiducial_y << endl;

    webcam.numCircles = 0;
    }


    webcam.coordinatesTranslation(fiducials, currentFiducials, fiducialSum);
    waitKey();
    return 0;
}