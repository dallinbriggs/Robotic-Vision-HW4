#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    Mat image_left;
    Mat image_color_left;
    Mat image_right;
    Mat image_color_right;
    vector<Point2f> corners_left;
    vector<Point2f> corners_right;
    Size patternsize;
    string filename_left;
    string filename_right;
    string header_left;
    string header_right;
    string tail;
    Size imageSize;
    Mat distCoeffs_left;
    Mat distCoeffs_right;
    Mat cameraMatrix_left;
    Mat cameraMatrix_right;

    Mat R, T, E, F;

    cameraMatrix_left = Mat::eye(3, 3, CV_64F);
    cameraMatrix_right = Mat::eye(3, 3, CV_64F);

    vector<vector<Point3f>> objectPoints_left(0);
    vector<vector<Point2f>> imagePoints_left;
    vector<vector<Point3f>> objectPoints_right(0);
    vector<vector<Point2f>> imagePoints_right;
    distCoeffs_left = Mat::zeros(8, 1, CV_64F);
    distCoeffs_right = Mat::zeros(8, 1, CV_64F);
    imagePoints_left.clear();
    imagePoints_right.clear();
    int numCornersHor = 10;
    int numCornersVer = 7;
    int numSquares = numCornersHor * numCornersVer;

    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f(j/numCornersHor*3.88636, j%numCornersHor*3.88636, 0.0f));

    header_left = "/home/dallin/robotic_vision/HW3/Images/stereoL";
    header_right = "/home/dallin/robotic_vision/HW3/Images/stereoR";
    tail = ".bmp";
    patternsize = cvSize(10,7);
    imageSize = cvSize(640,480);

    for(int i=1; i<100; i++)
    {
        filename_left = header_left + to_string(i) + tail;
        filename_right = header_right + to_string(i) + tail;
        image_left = imread(filename_left,CV_LOAD_IMAGE_GRAYSCALE);
        image_color_left = imread(filename_left,CV_LOAD_IMAGE_COLOR);

        bool corners_detected_left = findChessboardCorners(image_left,patternsize,corners_left);
        if(corners_detected_left)
        {
            /// Set the neeed parameters to find the refined corners
            Size winSize = Size( 5, 5 );
            Size zeroZone = Size( -1, -1 );
            TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
            cornerSubPix(image_left,corners_left,winSize,zeroZone,criteria);
            imagePoints_left.push_back(corners_left);
            objectPoints_left.push_back(obj);

        }
        drawChessboardCorners(image_color_left,patternsize,corners_left,corners_detected_left);
        imshow("Left", image_color_left);

        image_right = imread(filename_right,CV_LOAD_IMAGE_GRAYSCALE);
        image_color_right = imread(filename_right,CV_LOAD_IMAGE_COLOR);

        bool corners_detected_right = findChessboardCorners(image_right,patternsize,corners_right);
        if(corners_detected_right)
        {
            /// Set the neeed parameters to find the refined corners
            Size winSize = Size( 5, 5 );
            Size zeroZone = Size( -1, -1 );
            TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
            cornerSubPix(image_right,corners_right,winSize,zeroZone,criteria);
            imagePoints_right.push_back(corners_right);
            objectPoints_right.push_back(obj);

        }
        drawChessboardCorners(image_color_right,patternsize,corners_right,corners_detected_right);
        imshow("Right", image_color_right);
        waitKey(50);

    }

    FileStorage fs_left("Intrinsic_calibration_baseball_left.xml", FileStorage::READ);

    fs_left["CameraMatrix"] >> cameraMatrix_left;
    fs_left["DistortionCoefficients"] >> distCoeffs_left;
    FileStorage fs_right("Intrinsic_calibration_baseball_right.xml", FileStorage::READ);
    fs_right["CameraMatrix"] >> cameraMatrix_right;
    fs_right["DistortionCoefficients"] >> distCoeffs_right;
    TermCriteria criteria_stereo=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6);
    stereoCalibrate(objectPoints_right,imagePoints_left,imagePoints_right,cameraMatrix_left,distCoeffs_left,cameraMatrix_right,distCoeffs_right,imageSize,R,T,E,F,CALIB_FIX_INTRINSIC,criteria_stereo);

    cout << E << endl;
    cout << F << endl;
    cout << T << endl;
    FileStorage fs("Extrinsic_calibration_baseball.xml", FileStorage::WRITE);
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;

    waitKey(0);

    return 0;
}
