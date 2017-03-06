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
    Mat current_frame_left;
    Mat previous_frame_left;
    Mat current_frame_right;
    Mat previous_frame_right;
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
    string filename_write;
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
    std::vector<KeyPoint> keypoints_left_old;
    int keypoint_count = 0;

    keypoints_left_old = vector<KeyPoint>{KeyPoint(Point2f(0,0),1,-1,0,0,-1)};

//    VideoWriter VOut; // Create a video write object.
//    // Initialize video write object (only done once). Change frame size to match your camera resolution.
//    VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(640, 480), 1);
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 255;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = .3;

#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

    // Set up detector with params
    SimpleBlobDetector detector(params);

    // You can use the detector this way
    // detector.detect( im, keypoints);

#else

    // Set up detector with params
    Ptr<SimpleBlobDetector> blobby = SimpleBlobDetector::create(params);

    // SimpleBlobDetector::create creates a smart pointer.
    // So you need to use arrow ( ->) instead of dot ( . )
    // detector->detect( im, keypoints);

#endif

    header = "/home/dallin/robotic_vision/HW4/ImagesDallin/Ball_test";
    tail = ".bmp";
    first_frame_left = imread(header + "L14" + tail, CV_LOAD_IMAGE_GRAYSCALE);
    first_frame_right = imread(header + "R14" + tail, CV_LOAD_IMAGE_GRAYSCALE);

    for(int i = 14; i < 58; i++)
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
        GaussianBlur(motion_left,motion_left, Size(7,7), 1.5, 1.5);
        GaussianBlur(motion_right,motion_right, Size(7,7), 1.5, 1.5);
        threshold(motion_left, frame_thresh_left,10,255,1);
        threshold(motion_right, frame_thresh_right,10,255,1);


//        // Detect blobs.
        std::vector<KeyPoint> keypoints_left;
        std::vector<KeyPoint> keypoints_right;
        blobby->detect(frame_thresh_left, keypoints_left);
        blobby->detect(frame_thresh_right, keypoints_right);

//        // Draw detected blobs as red circles.
//        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob

//        drawKeypoints(image_left, keypoints_left, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        if (!keypoints_left.empty())
        {
            x_left = (int)keypoints_left[0].pt.x + x_left - width/2;
            y_left = (int)keypoints_left[0].pt.y + y_left - height/2;
            rectangle_left = Rect(x_left,y_left,width,height);
            cout << x_left << " " << y_left << endl;
//            keypoint_count = keypoint_count +1;
            circle(image_left_color,Point2f(x_left + width/2,y_left + height/2),5,Scalar(0,255,0),2,LINE_8,0);

        }
        if (!keypoints_right.empty())
        {
            x_right = (int)keypoints_right[0].pt.x + x_right - width/2;
            y_right = (int)keypoints_right[0].pt.y + y_right - height/2;
            rectangle_right = Rect(x_right,y_right,width,height);
            cout << x_right << " " << y_right << endl;
            keypoint_count = keypoint_count +1;
            circle(image_right_color,Point2f(x_right + width/2,y_right + height/2),5,Scalar(0,255,0),2,LINE_8,0);
        }


        rectangle(image_left_color,rectangle_left,Scalar(0,0,255),2,LINE_8,0);
        rectangle(image_right_color,rectangle_right,Scalar(0,0,255),2,LINE_8,0);
        imshow("Left", image_left_color);
        imshow("right", image_right_color);
        moveWindow("right",643,23);

        cout << "Keypoints: " << keypoint_count << endl;
        if (keypoint_count == 1 || keypoint_count == 5 || keypoint_count == 10 || keypoint_count == 15 || keypoint_count == 20)
        {
            imwrite("ROI_left_" + to_string(keypoint_count) + ".jpg",image_left_color);
            imwrite("ROI_right_" + to_string(keypoint_count) + ".jpg",image_right_color);
        }


        waitKey(1);

    }

    waitKey(0);
    return 0;
}
