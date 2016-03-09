//
// Created by vassilis on 09.03.16.
//

#include "inspectionCamera.h"


inspectionCamera::inspectionCamera(int dev) {
    devNum = dev;

}

Mat inspectionCamera::captureImage(int focus) {

    VideoCapture stream(devNum);

    if(!stream.isOpened())
        cout << "Cannot open video camera";
    else {

        // Disable autofocus and set a manual value
        system("v4l2-ctl -d 0 -c focus_auto=0");

        // Convert the focus to string and append to the system command
        string focusCommand = "v4l2-ctl -d 0 -c focus_absolute=", result;
        ostringstream number;
        number << focus;
        result = number.str();
        focusCommand.append(result);
        const char *cstr = focusCommand.c_str();

        // Set the focus to the input value
        system(cstr);

        // Capture a frame from the stream
        Mat snap;
        stream.read(snap);
        return snap;
    }
}

int inspectionCamera::locateFiducial(Mat src, float target_x, float target_y, int minRadius, int maxRadius) {
    // The x,y coordinates of the ROI's starting point
    float x_roi = target_x - X_ROI, y_roi = target_y - Y_ROI;

    circle(src, Point(202, 392), 8, Scalar(0, 0, 255), 1, LINE_AA); // Scalar(B, G, R)
    circle(src, Point(196, 393), 8, Scalar(255, 0, 0), 1, LINE_AA); // Scalar(B, G, R)
    // Uncomment for saving the frame to disk
    //imwrite( "/home/vassilis/Dropbox/Uppsala Universitet/Thesis/Selective-soldering-PC/opencv_board_tilted.jpg", src );

    // Create the ROI and crop the frame
    Rect region_of_interest = Rect((int)x_roi,(int) y_roi, WIDTH, HEIGHT);
    Mat roi = src(region_of_interest);

    // Convert to grayscale and perform edge detection
    Mat dst, cdst;
    Canny(roi, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);

    // Perform the circles Hough transform
    vector<Vec3f> circles;
    HoughCircles(dst, circles, HOUGH_GRADIENT, 1, 100, 100, 10, minRadius, maxRadius); //image, output vector, method, dp, minDist, param1, param2, minRadius, maxRadius
    numCircles = (int)circles.size();
    // Display the circles
    for (size_t i = 0; i < circles.size(); i++) {
        Vec3i c = circles[i];
        circle(src, Point((int)c[0] + (int)x_roi, (int)c[1] + (int)y_roi), c[2], Scalar(0, 0, 255), 3, LINE_AA); // +x_roi and +y_roi for displaying using the whole's frame
        circle(src, Point((int)c[0] + (int)x_roi, (int)c[1] + (int)y_roi), 2, Scalar(0, 255, 0), 3, LINE_AA);   // coordinates rather than the ROI's ones (local)
        // Return values
        fiducial_x = c[0] + x_roi;
        fiducial_y = c[1] + y_roi;
        fiducial_r = c[2];
    }


    // Appending on the frame a rectangle to visualize the ROI
    Mat color(roi.size(), CV_8UC3, Scalar(0, 200, 200));
    double alpha = 0.5; // transparency
    addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);

}


void inspectionCamera::coordinatesTranslation(float correctFiducial[][2], float currentFiducial[][2], int numFiducials) {

    cout << currentFiducial[0][0] << " " << currentFiducial[0][1] << endl;

}


void inspectionCamera::rigidTransform(Mat A, Mat B) {

    int N; // Number of points
    Mat S,U,V; // SVD matrices
    Mat centroid_A, centroid_APrime,centroid_BPrime, centroid_B, H, temp, trans;


    reduce(A,centroid_A, 0, CV_REDUCE_AVG); // MATLAB: centroid_A = mean(A);
    reduce(B,centroid_B, 0, CV_REDUCE_AVG); // MATLAB: centroid_B = mean(B);

    N = A.rows;                             // MATLAB: N = size(A,1);

    // MATLAB:  H = (A - repmat(centroid_A, N, 1))' * (B - repmat(centroid_B, N, 1));
    temp = A - repeat(centroid_A, N, 1); // A - repmat(centroid_A, N, 1)
    transpose(temp, trans); // (A - repmat(centroid_A, N, 1))'
    H = trans * (B - repeat(centroid_B, N, 1)); //   H = (A - repmat(centroid_A, N, 1))' * (B - repmat(centroid_B, N, 1));

    // MATLAB: [U,S,V] = svd(H);
    cv::SVD s; // SVD constructor
    s.compute(H,S,U,V); // Compute SVD

    // We get S as a single column, put the values at a diagonal to match MATLAB's results
    //S = Mat::diag(S); // Commented out as it is not (currently) used

    // Negate U(:,1) to match MATLAB's results
    temp = U.col(0);
    U.col(0) = temp * -1;

    // Negate V(1,:) to match MATLAB's results
    temp = V.row(0);
    V.row(0) = temp * -1;

    // MATLAB: R = V*U';
    transpose(U, trans); // U'
    R = V * trans; // R = V*U';

    if(determinant(R)<0)
    {
        cout << "Reflection detected" << endl;
        temp = V.col(1); // MATLAB: temp = V(:,2);
        V.col(1) = temp * -1; // MATLAB: V(:,2) = temp * -1;
        R = V * trans; // MATLAB: R = V*U';
    }
    //cout << "R = "<< endl << " "  << R << endl << endl;

    // MATLAB: t = -R*centroid_A' + centroid_B'
    transpose(centroid_A, centroid_APrime); // centroid_A'
    transpose(centroid_B, centroid_BPrime); // centroid_B'
    t = -R * centroid_APrime + centroid_BPrime; // t = -R*centroid_A' + centroid_B'
    //cout << "t = "<< endl << " "  << t << endl << endl;

    float data3[1][2] = {202, 392};
    Mat test, test_tr, A2;
    test = Mat(1, 2, CV_32F, data3);

    //A2 = (ret_R*A') + repmat(ret_t, 1, n);
    transpose(A, trans);
    A2 = (R*trans) + repeat(t, 1, N);
    //test_tr = (ret_R*test') + repmat(ret_t, 1, 1);
    transpose(test,trans);
    test_tr = (R * trans) + repeat(t, 1, 1);
    transpose(test_tr,trans);
    cout << "test_tr = "<< endl << " "  << trans << endl << endl;

    transpose(A2, trans);
    cout << "A2 = "<< endl << " "  << trans << endl << endl;


}