#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        vector<Point2f> *pts = (vector<Point2f>*)userdata;
        pts->push_back(Point2f(x,y));
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }

}


int main(int argc, char *argv[])
{
    Mat image_left;
    Mat image_color_left;
    Mat image_right;
    Mat image_color_right;
    Mat image_undistort_left;
    Mat image_undistort_right;
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
    Mat fundamental_matrix;
    vector<Vec3f> lines_left;
    vector<Vec3f> lines_right;
    vector<vector<Point3f>> objectPoints_left(0);
    vector<vector<Point3f>> objectPoints_right(0);

    vector<Point2f> point_left;
    vector<Point2f> point_right;
    Mat corners_left;
    Mat corners_right;
    vector<Point2f> corners_outer_left;
    vector<Point2f> corners_outer_left_rect;
    vector<Point2f> corners_outer_right;
    vector<Point2f> corners_outer_right_rect;
    vector<Point3f> corners_diff_left;
    vector<Point3f> corners_diff_right;
    vector<Point3f> perspective_left;
    vector<Point3f> perspective_right;

    Mat R1, P1, R2, P2, Q;

    Mat R, T, E, F;

    cameraMatrix_left = Mat::eye(3, 3, CV_64F);
    cameraMatrix_right = Mat::eye(3, 3, CV_64F);


    vector<vector<Point2f>> imagePoints_left;
    vector<vector<Point2f>> imagePoints_rect_left;

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

    header_left = "/home/dallin/robotic_vision/HW3/Images/stereoL1";
    header_right = "/home/dallin/robotic_vision/HW3/Images/stereoR1";
    tail = ".bmp";
    patternsize = cvSize(10,7);
    imageSize = cvSize(640,480);


    filename_left = header_left + tail;
    filename_right = header_right + tail;

    image_left = imread(filename_left,CV_LOAD_IMAGE_GRAYSCALE);
    image_color_left = imread(filename_left,CV_LOAD_IMAGE_COLOR);

    image_right = imread(filename_right,CV_LOAD_IMAGE_GRAYSCALE);
    image_color_right = imread(filename_right,CV_LOAD_IMAGE_COLOR);

    FileStorage fs_left("Intrinsic_calibration_baseball_left.xml", FileStorage::READ);
    fs_left["CameraMatrix"] >> cameraMatrix_left;
    fs_left["DistortionCoefficients"] >> distCoeffs_left;
    FileStorage fs_right("Intrinsic_calibration_baseball_right.xml", FileStorage::READ);
    fs_right["CameraMatrix"] >> cameraMatrix_right;
    fs_right["DistortionCoefficients"] >> distCoeffs_right;
    FileStorage fs_rectify("Rectification_baseball.xml", FileStorage::READ);
    fs_rectify["R1"] >> R1;
    fs_rectify["P1"] >> P1;
    fs_rectify["R2"] >> R2;
    fs_rectify["P2"] >> P2;
    fs_rectify["Q"] >> Q;

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
//        drawChessboardCorners(image_color_left,patternsize,corners_left,corners_detected_left);
        corners_outer_left = {corners_left.at<Point2f>(0), corners_left.at<Point2f>(9),corners_left.at<Point2f>(60), corners_left.at<Point2f>(69)};
        undistortPoints(corners_outer_left,corners_outer_left_rect,cameraMatrix_left,distCoeffs_left,R1,P1);
        circle(image_color_left,corners_outer_left[0],5,Scalar(0,0,255),2,LINE_8,0);
        circle(image_color_left,corners_outer_left[1],5,Scalar(0,0,255),2,LINE_8,0);
        circle(image_color_left,corners_outer_left[2],5,Scalar(0,0,255),2,LINE_8,0);
        circle(image_color_left,corners_outer_left[3],5,Scalar(0,0,255),2,LINE_8,0);

        circle(image_color_left,corners_outer_left_rect[0],5,Scalar(0,255,0),2,LINE_8,0);
        circle(image_color_left,corners_outer_left_rect[1],5,Scalar(0,255,0),2,LINE_8,0);
        circle(image_color_left,corners_outer_left_rect[2],5,Scalar(0,255,0),2,LINE_8,0);
        circle(image_color_left,corners_outer_left_rect[3],5,Scalar(0,255,0),2,LINE_8,0);
    }
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
        //        drawChessboardCorners(image_color_right,patternsize,corners_right,corners_detected_right);
        corners_outer_right = {corners_right.at<Point2f>(0), corners_right.at<Point2f>(9),corners_right.at<Point2f>(60), corners_right.at<Point2f>(69)};
        undistortPoints(corners_outer_right,corners_outer_right_rect,cameraMatrix_right,distCoeffs_right,R2,P2);
        circle(image_color_right,corners_outer_right[0],5,Scalar(0,0,255),2,LINE_8,0);
        circle(image_color_right,corners_outer_right[1],5,Scalar(0,0,255),2,LINE_8,0);
        circle(image_color_right,corners_outer_right[2],5,Scalar(0,0,255),2,LINE_8,0);
        circle(image_color_right,corners_outer_right[3],5,Scalar(0,0,255),2,LINE_8,0);

        circle(image_color_right,corners_outer_right_rect[0],5,Scalar(0,255,0),2,LINE_8,0);
        circle(image_color_right,corners_outer_right_rect[1],5,Scalar(0,255,0),2,LINE_8,0);
        circle(image_color_right,corners_outer_right_rect[2],5,Scalar(0,255,0),2,LINE_8,0);
        circle(image_color_right,corners_outer_right_rect[3],5,Scalar(0,255,0),2,LINE_8,0);
    }


    for(int i=0; i < 4; i++)
    {
        corners_diff_left.push_back(Point3f(corners_outer_left_rect[i].x, corners_outer_left_rect[i].y,
                                       corners_outer_left_rect[i].x-corners_outer_right_rect[i].x));
    }

    for(int i=0; i < 4; i++)
    {
        corners_diff_right.push_back(Point3f(corners_outer_right_rect[i].x, corners_outer_right_rect[i].y,
                                       corners_outer_left_rect[i].x-corners_outer_right_rect[i].x));
    }

    perspectiveTransform(corners_diff_left,perspective_left,Q);
    perspectiveTransform(corners_diff_right,perspective_right,Q);

    cout << perspective_left << endl;
    cout << perspective_right << endl;

    imshow("Left", image_color_left);
    imshow("right", image_color_right);
    moveWindow("right",650,30);


    waitKey(0);

    return 0;
}

