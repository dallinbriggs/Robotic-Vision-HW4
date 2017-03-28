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
    Mat image_right;
    Mat image_left_color;
    Mat image_right_color;
    Mat roi_first_left;
    Mat roi_first_right;
    Mat frame_thresh_left;
    Mat frame_thresh_right;
    Mat motion_right;
    Mat motion_left;
    Mat first_frame_left;
    Mat first_frame_right;
    Mat roi_current_left;
    Mat roi_current_right;
    Mat roi_previous_left;
    Mat roi_previous_right;
    string filename_left;
    string filename_right;
    string header;
    string tail;
    int x_left = 333;
    int y_left = 88;
    int x_right = 233;
    int y_right = 88;
    int width = 70;
    int height = 70;
    Rect rectangle_left = Rect(x_left,y_left,width,height);
    Rect rectangle_right = Rect(x_right,y_right,width,height);


    vector<Point3f> corners_diff_left;
    vector<Point3f> corners_diff_right;
    vector<Point3f> perspective_left;
    vector<Point3f> ball_left;
    vector<Point3f> ball_right;
    vector<Point3f> perspective_right;
    vector<Point2f> point_left, point_right;

    Mat R1, P1, R2, P2, Q;
    Mat distCoeffs_left;
    Mat distCoeffs_right;
    Mat cameraMatrix_left;
    Mat cameraMatrix_right;


    vector<vector<Point> > contours_left;
    vector<vector<Point> > contours_right;
    vector<Vec4i> hierarchy_left;
    vector<Vec4i> hierarchy_right;


    header = "/home/dallin/robotic_vision/HW4/ImagesDallin/Ball_test";
    tail = ".bmp";
    first_frame_left = imread(header + "L14" + tail, CV_LOAD_IMAGE_GRAYSCALE);
    first_frame_right = imread(header + "R14" + tail, CV_LOAD_IMAGE_GRAYSCALE);

    for(int i = 15; i < 40; i++)
    {

        filename_left = header + "L" + to_string(i) + tail;
        filename_right = header + "R" + to_string(i) + tail;

        image_left = imread(filename_left,CV_LOAD_IMAGE_GRAYSCALE);
        image_right = imread(filename_right,CV_LOAD_IMAGE_GRAYSCALE);
        image_left_color = imread(filename_left,CV_LOAD_IMAGE_COLOR);
        image_right_color = imread(filename_right,CV_LOAD_IMAGE_COLOR);

        roi_current_left = image_left(rectangle_left);
        roi_current_right = image_right(rectangle_right);
        roi_first_left = first_frame_left(rectangle_left);
        roi_first_right = first_frame_right(rectangle_right);

        roi_current_left.copyTo(roi_previous_left);
        roi_current_right.copyTo(roi_previous_right);


        if(roi_previous_left.empty() || roi_previous_right.empty())
        {
            roi_previous_left = Mat::zeros(roi_current_left.size(), roi_current_left.type()); // prev frame as black
            roi_previous_right = Mat::zeros(roi_current_right.size(), roi_current_right.type()); // prev frame as black
            //signed 16bit mat to receive signed difference
        }
        absdiff(roi_current_left, roi_first_left, motion_left);
        absdiff(roi_current_right, roi_first_right, motion_right);
        GaussianBlur(motion_left,motion_left, Size(11,11), 0,0);
        GaussianBlur(motion_right,motion_right, Size(11,11), 0,0);
        threshold(motion_left, frame_thresh_left,4,255,1);
        threshold(motion_right, frame_thresh_right,4,255,1);

        findContours(frame_thresh_left,contours_left,hierarchy_left, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        findContours(frame_thresh_right,contours_right,hierarchy_right, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        drawContours(frame_thresh_left,contours_left,0,Scalar(200,200,200),2,LINE_8,hierarchy_left,0x7fffffff,Point(0,0));
        imshow("Thresh", frame_thresh_left);
        vector<Moments> contour_moments_left(contours_left.size());
        vector<Moments> contour_moments_right(contours_right.size());
        for( int i = 0; i < contours_left.size(); i++ )
        { contour_moments_left[i] = moments( contours_left[i], true ); }
        for( int i = 0; i < contours_right.size(); i++ )
        { contour_moments_right[i] = moments( contours_right[i], true ); }

        ///  Get the mass centers:
        vector<Point2f> mc_left( contours_left.size() );
        vector<Point2f> mc_right( contours_right.size() );
        for( int i = 0; i < contours_left.size(); i++ )
        { mc_left[i] = Point2f( contour_moments_left[i].m10/contour_moments_left[i].m00 , contour_moments_left[i].m01/contour_moments_left[i].m00 ); }
        for( int i = 0; i < contours_right.size(); i++ )
        { mc_right[i] = Point2f( contour_moments_right[i].m10/contour_moments_right[i].m00 , contour_moments_right[i].m01/contour_moments_right[i].m00 ); }


        if (!contours_left.empty())
        {
            x_left = (int)mc_left[1].x + x_left - width/2;
            y_left = (int)mc_left[1].y + y_left - height/2;
            rectangle_left = Rect(x_left,y_left,width,height);
            circle(image_left_color,Point2f(x_left + width/2,y_left + height/2),5,Scalar(0,255,0),2,LINE_8,0);

        }
        if (!contours_right.empty())
        {
            x_right = (int)mc_right[1].x + x_right - width/2;
            y_right = (int)mc_right[1].y + y_right - height/2;
            rectangle_right = Rect(x_right,y_right,width,height);
            circle(image_right_color,Point2f(x_right + width/2,y_right + height/2),5,Scalar(0,255,0),2,LINE_8,0);
        }

        rectangle(image_left_color,rectangle_left,Scalar(0,0,255),2,LINE_8,0);
        rectangle(image_right_color,rectangle_right,Scalar(0,0,255),2,LINE_8,0);
        imshow("Left", image_left_color);
//        imshow("right", image_right_color);
//        moveWindow("right",643,23);

        FileStorage fs_left("calib_left.yaml", FileStorage::READ);
        fs_left["CameraMatrix"] >> cameraMatrix_left;
        fs_left["DistortionCoefficients"] >> distCoeffs_left;
        FileStorage fs_right("calib_right.yaml", FileStorage::READ);
        fs_right["CameraMatrix"] >> cameraMatrix_right;
        fs_right["DistortionCoefficients"] >> distCoeffs_right;
        FileStorage fs_rectify("Rectification_baseball_2.xml", FileStorage::READ);
        fs_rectify["R1"] >> R1;
        fs_rectify["P1"] >> P1;
        fs_rectify["R2"] >> R2;
        fs_rectify["P2"] >> P2;
        fs_rectify["Q"] >> Q;

        if(!contours_left.empty() && !contours_right.empty())
        {
            point_left = {Point2f(x_left + width/2,y_left + height/2)};
            undistortPoints(point_left,point_left,cameraMatrix_left,distCoeffs_left,R1,P1);

            point_right = {Point2f(x_right + width/2,y_right + height/2)};
            undistortPoints(point_right,point_right,cameraMatrix_right,distCoeffs_right,R2,P2);

            corners_diff_left.push_back(Point3f(point_left[0].x, point_left[0].y,
                    point_left[0].x - point_right[0].x));

            corners_diff_right.push_back(Point3f(point_right[0].x, point_right[0].y,
                    point_left[0].x - point_right[0].x));

            perspectiveTransform(corners_diff_left,perspective_left,Q);
            perspectiveTransform(corners_diff_right,perspective_right,Q);
            ball_left.push_back(Point3f(perspective_left[0].x, perspective_left[0].y, perspective_left[0].z));
            ball_right.push_back(Point3f(perspective_right[0].x, perspective_right[0].y, perspective_right[0].z));
        }
        waitKey(500);


    }

    FileStorage fs_perspective("Perspective_baseball.csv", FileStorage::WRITE);
    fs_perspective << "ball_left" << perspective_left;

    fs_perspective << "ball_right" << perspective_right;
    cout << perspective_left << endl;

    waitKey(0);
    return 0;
}
