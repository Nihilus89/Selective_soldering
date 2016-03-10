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
    float fiducial[10][2] = {0};
    float currentFiducial[10][2] = {0};

//    motorController topTable("/dev/ttyUSB0", MB_BITRATE, MB_PARITY, MB_DATABITS, MB_STOPBITS,1);  // 38400 bps, 8-N-1
//    topTable.initController();
//    topTable.motorsOn(false);
//    topTable.setMaxVelocity(M1, 15);
//    topTable.setPosition(M1, 10.6);

    getCoordinatesfromFile(filename,&componentSum, &fiducialSum, coordinates, fiducial);


    inspectionCamera webcam(0);
    Mat src;
    src = webcam.captureImage(30);

    for(i=0; i<fiducialSum; i++)
    {

        int detected = webcam.locateFiducial(src,fiducial[i][0],fiducial[i][1],3,5);
        while(detected)
        {
            src = webcam.captureImage(30);
            detected = webcam.locateFiducial(src,fiducial[i][0],fiducial[i][1],3,5);
        }
            imshow("source",src);
        cout << "a: " << detected << "    ";

        currentFiducial[i][0] = webcam.fiducial_x;
        currentFiducial[i][1] = webcam.fiducial_y;
        cout << webcam.fiducial_x << " " << webcam.fiducial_y << endl;

    }


    webcam.coordinatesTranslation(fiducial, currentFiducial, coordinates, fiducialSum, componentSum);

    for(int i=0; i<componentSum; i++)
        cout << "coordinates[" << i << "]: " << coordinates[i][0] << " " << coordinates[i][1] << endl;

    waitKey();
    return 0;
}