//
// Created by vassilis on 09.03.16.
//

#ifndef SELECTIVE_SOLDERING_INSPECTIONCAMERA_H
#define SELECTIVE_SOLDERING_INSPECTIONCAMERA_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

#define X_ROI 10
#define Y_ROI 10
#define WIDTH 40
#define HEIGHT 40

using namespace cv;
using namespace std;

class inspectionCamera {
public:
    float fiducial_x, fiducial_y, fiducial_r;
    int numCircles;

    inspectionCamera(int); // constructor
    Mat captureImage(int focus);
    int locateFiducial(Mat, float, float, int, int);
    void coordinatesTranslation(float correctFiducial[][2], float currentFiducial[][2], float coordinates[][2], int numFiducial, int numComp);

private:
    int devNum;
    Mat R, t;

    void rigidTransform(Mat, Mat, Mat& R, Mat& t);
};


#endif //SELECTIVE_SOLDERING_INSPECTIONCAMERA_H
