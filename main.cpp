//  main.cpp
//  arucoDetector

//  RUN COMMAN EXAMPLE  (NOT VALID IN THIS CURRENT VERSION)
// ./detectTest --d=0 --ci=0 --c=out_camera_data.xml --l=0.05 --dp=detector_param.xml --r="false"
//
//  Created by Kosmas Kritsis on 25/04/16.
//  Copyright © 2016 Kosmas Kritsis. All rights reserved.
//

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include "bowTrack.hpp"

#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace aruco;

/**
namespace {
    const char* about = "Basic marker detection";
    const char* keys  =
    "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
    "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
    "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
    "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
    "{v        |       | Input from video file, if ommited, input comes from camera }"
    "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
    "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
    "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
    "{dp       |       | File of marker detector parameters }"
    "{r        |       | show rejected candidates too }";
}

**/


///// HERE IS THE COLOR TRACKING

// file stream for storing data in csv format
ofstream myfile;

// video capture object to acquire webcam feed
VideoCapture capture;
VideoWriter outputVideo;

// initial min and max HSV filter values.
// these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 179;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

//default capture width and height (only for camera)
const int FRAME_WIDTH = 960; // predefined 1280
const int FRAME_HEIGHT = 540; // predifined 720

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=5; // predefined was 50

//minimum and maximum object area
const int MIN_OBJECT_AREA = 4*4; // predifined was 10*10
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/3.0;

//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
const string trackbarValueWindowName = "Trackbars Values";

//image for displaying the trackbar values
Mat trackbarValImg = Mat::zeros(200, 300, CV_8UC3);


//coordinates of the detected markers of the previous image for detecting speed and current string
Point2d previousA;
Point2d previousB;

float previousVelocityA = 0.0;
float previousVelocityB = 0.0;

double maxBowLength = 380.0; // Max bow pixels: 369.177464 with video bowSize.mp4

//video playback trackbar
int videoTrackbarPos = 0;


//method for handling the calibration trackbars
void on_trackbar( int, void* )
{
    //Method when trackbar position is changed
    
    trackbarValImg = Mat::zeros(200, 300, CV_8UC3);
    
    char hMin[15];
    char hMax[15];
    char sMin[15];
    char sMax[15];
    char vMin[15];
    char vMax[15];
    
    //getTrackbarPos("", trackbarWindowName)
    sprintf( hMin, "H_MIN %i", H_MIN); // 20
    sprintf( hMax, "H_MAX %i", H_MAX); // 35
    sprintf( sMin, "S_MIN %i", S_MIN); // 50
    sprintf( sMax, "S_MAX %i", S_MAX); // 255
    sprintf( vMin, "V_MIN %i", V_MIN); // 50
    sprintf( vMax, "V_MAX %i", V_MAX); // 255
    
    putText(trackbarValImg, hMin, Point(10,20), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
    putText(trackbarValImg, hMax, Point(10,40), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
    putText(trackbarValImg, sMin, Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
    putText(trackbarValImg, sMax, Point(10,80), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
    putText(trackbarValImg, vMin, Point(10,100), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
    putText(trackbarValImg, vMax, Point(10,120), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
    
    imshow(trackbarValueWindowName, trackbarValImg);
}


//method for handling the video playback trackbars

void on_trackbar2(int current_frame, void*)
{
    current_frame = videoTrackbarPos;
    capture.set(CV_CAP_PROP_POS_FRAMES, current_frame);
}


string intToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void createTrackbars(){
    //create window for trackbars
    namedWindow(trackbarValueWindowName,WINDOW_AUTOSIZE);
    namedWindow(trackbarWindowName,0);
    
    //create memory to store trackbar name on window
    char TrackbarName1[50];
    char TrackbarName2[50];
    char TrackbarName3[50];
    char TrackbarName4[50];
    char TrackbarName5[50];
    char TrackbarName6[50];
    
    sprintf( TrackbarName1, "H_MIN %i", H_MIN);
    sprintf( TrackbarName2, "H_MAX %i", H_MAX);
    sprintf( TrackbarName3, "S_MIN %i", S_MIN);
    sprintf( TrackbarName4, "S_MAX %i", S_MAX);
    sprintf( TrackbarName5, "V_MIN %i", V_MIN);
    sprintf( TrackbarName6, "V_MAX %i", V_MAX);
    
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->

    createTrackbar( TrackbarName1, trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
    createTrackbar( TrackbarName2, trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
    createTrackbar( TrackbarName3, trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
    createTrackbar( TrackbarName4, trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
    createTrackbar( TrackbarName5, trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
    createTrackbar( TrackbarName6, trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
    
    imshow( trackbarValueWindowName, trackbarValImg);
}


void drawObject(vector<bowTrack> bow, Mat &frame, Mat &final, Mat &tracking){
    
    for(int i =0; i<bow.size(); i++){
        
        //cv::circle(frame,cv::Point(theFruits.at(i).getXPos(),theFruits.at(i).getYPos()),10,cv::Scalar(0,0,255));
        cv::putText(final,intToString(bow.at(i).getXPos())+ " , " + intToString(bow.at(i).getYPos()),cv::Point(bow.at(i).getXPos(),bow.at(i).getYPos()+20),1,1,Scalar(0,255,0));
        //cv::putText(frame,theFruits.at(i).getType(),cv::Point(theFruits.at(i).getXPos(),theFruits.at(i).getYPos()-30),1,2,theFruits.at(i).getColour());
    }
    if(bow.size()==2){
        cv::Point2d pointA(bow.at(0).getXPos(),bow.at(0).getYPos());
        cv::Point2d pointB(bow.at(1).getXPos(),bow.at(1).getYPos());
        
        cv::line(final,pointA,pointB,bow.at(0).getColour(),5);
        //cv::line(tracking,pointA,pointB,bow.at(0).getColour(),5);
    }
}


// Method for detecting the pixel displacement per frame for the two markers
void speedDetect(vector<bowTrack> bow, Mat &frame, Mat &final, Mat &tracking, Point2d &previousA, Point2d &previousB){
    // check if there are two points detected
    
    
    if (bow.size()==2){
        
        cv::Point2d pointA;
        cv::Point2d pointB;
        
        // specify the pointA to be always the upper marker
        // and pointB the lower marker.
        // Evaluation based on their height coordinates.
        if (bow.at(0).getYPos() < bow.at(1).getYPos()){
            cv::Point2d A(bow.at(0).getXPos(),bow.at(0).getYPos());
            cv::Point2d B(bow.at(1).getXPos(),bow.at(1).getYPos());
            pointA = A;
            pointB = B;
            
        }
        // (bow.at(0).getYPos() > bow.at(1).getYPos())
        else {
            cv::Point2d A(bow.at(1).getXPos(),bow.at(1).getYPos());
            cv::Point2d B(bow.at(0).getXPos(),bow.at(0).getYPos());
            pointA = A;
            pointB = B;
        }
        
        // euclidean distance between the previous and the current point positions
        double dPointA = cv::norm(previousA-pointA);
        double dPointB = cv::norm(previousB-pointB);
        
        printf("Point 1 distance: %f\n", dPointA);
        printf("Point 2 distance: %f\n", dPointB);
        
        // euclidean distance for determining the length of the bow
        double bowLength = norm(pointA-pointB);
        printf("Bow Length: %f\n", bowLength);
        
        // displacement in x axis
        int x1 = bow.at(0).getXPos();
        int x2 = bow.at(1).getXPos();
        
        double xDisplacement= abs(x1-x2);
        printf("X Projection Length: %f\n", xDisplacement);
            
        // displacement in y axis
        int y1 = bow.at(0).getYPos();
        int y2 = bow.at(1).getYPos();
            
        double yDisplacement = abs(y1-y2);
        printf("Y Projection Length: %f\n", yDisplacement);
        
        // velocity vector line
        arrowedLine(final, previousA, pointA, Scalar(255,0,0), 3, 8, 0, 0.4);
        arrowedLine(final, previousB, pointB, Scalar(255,0,0), 3, 8, 0, 0.4);
        
        //float angle = atan2(point1.y - p2.y, p1.x - p2.x);
        
        float angle = fastAtan2(pointB.y-pointA.y, pointB.x-pointA.x);
        
        
        //putText(frame, to_string(angle), Point2d(pointB.x+50, (pointA.y+pointB.y)/2), 1,1,Scalar(0,255,0));
        
        // Relative speed
        
        float rspeedA = ((0.75*dPointA)/bowLength)/0.0333667;
        float rspeedB = ((0.75*dPointB)/bowLength)/0.0333667;
        
        // Actual Speed
        
        float pixelDisplacementA = dPointA*(maxBowLength/bowLength);
        float pixelDisplacementB = dPointB*(maxBowLength/bowLength);
        
        float cmDisplacementA = pixelDisplacementA*(75.0/maxBowLength);
        float cmDisplacementB = pixelDisplacementB*(75.0/maxBowLength);
        
        // Angle from camera in degrees
        float angle2 = bowLength/maxBowLength;
        double angleReal = acos(angle2) * (180.0 / PI);
        
        putText(final, to_string(angleReal), Point2d(pointB.x-100, (pointA.y+pointB.y)/2), 1,1,Scalar(0,255,0));

        printf("Angle Real: %f\n", angleReal);
        
        // Acceleration Computation
        float accelarationA = (rspeedA-previousVelocityA)/0.0333667;
        float accelarationB = (rspeedB-previousVelocityB)/0.0333667;
        
        float absAccelarationA = abs(accelarationA);
        float absAccelarationB = abs(accelarationB);
        
        // Writing to the csv file
       
        myfile<<dPointA<<","<<dPointB<<","<<cmDisplacementA<<","<<cmDisplacementB<<","<<rspeedA<<","<<rspeedB<<","<<absAccelarationA<<","<<absAccelarationB<<","<<angleReal<<"\n";
        
        //myfile<<"\n";
        
        // for extracting the maximum length for the bow
        //if (maxBowLength<bowLength){
        //    maxBowLength = bowLength;
        //}
        
        // swapping the values for the next frame
        previousA = pointA;
        previousB = pointB;
        
        previousVelocityA = rspeedA;
        previousVelocityB = rspeedB;
        
    }
    else{
        myfile<<0.0<<","<<0.0<<","<<0.0<<","<<0.0<<"\n";
    }
}

void morphOps(Mat &thresh){
    
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(6,6));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(10,10));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);

    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}


void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed,Mat &final, Mat &tracking, double &framenum){
    vector <bowTrack> apples;
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {
                
                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                
                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA){
                    
                    bowTrack apple;
                    
                    apple.setXPos(moment.m10/area);
                    apple.setYPos(moment.m01/area);
                    
                    apples.push_back(apple);
                    
                    objectFound = true;
                    
                }
                else objectFound = false;
            }
            //let user know you found an object
            
            if(objectFound ==true){
                //draw object location on screen
                drawObject(apples,final,cameraFeed,tracking);

                // Check if is the first frame of the video
                // in order to initialize the bow points
                if (framenum==1){
                    if (apples.at(0).getYPos() < apples.at(1).getYPos()){
                        previousA = cv::Point2d(apples.at(0).getXPos(),apples.at(0).getYPos());
                        previousB = cv::Point2d(apples.at(1).getXPos(),apples.at(1).getYPos());
                    }
                    // (bow.at(0).getYPos() > bow.at(1).getYPos())
                    else {
                        previousA = cv::Point2d(apples.at(1).getXPos(),apples.at(1).getYPos());
                        previousB = cv::Point2d(apples.at(0).getXPos(),apples.at(0).getYPos());
                    }
                }
                else{
                    speedDetect(apples,cameraFeed,final,tracking,previousA,previousB);
                }
                
                    
            }
            
        }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
}



void trackFilteredObject(bowTrack theBow, Mat threshold, Mat HSV, Mat &cameraFeed, Mat &final, Mat &tracking, double &framenum){
    vector <bowTrack> apples;
    
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    //TODO: create algorithm to keep only the 2 biggest
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {
                
                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                
                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA){
                    
                    bowTrack apple;
                    
                    apple.setXPos(moment.m10/area);
                    apple.setYPos(moment.m01/area);
                    apple.setType(theBow.getType());
                    apple.setColour(theBow.getColour());
                    
                    apples.push_back(apple);
                    
                    objectFound = true;
                    
                }else objectFound = false;
                
                
            }
            //let user know you found an object
            if(objectFound ==true){
                //draw object location on screen
                drawObject(apples,cameraFeed,final,tracking);
                
                // Check if is the first frame of the video
                // in order to initialize the bow points
                if (framenum==1){
                    if (apples.at(0).getYPos() < apples.at(1).getYPos()){
                        previousA = cv::Point2d(apples.at(0).getXPos(),apples.at(0).getYPos());
                        previousB = cv::Point2d(apples.at(1).getXPos(),apples.at(1).getYPos());
                        
                    }
                    // (bow.at(0).getYPos() > bow.at(1).getYPos())
                    else {
                        previousA = cv::Point2d(apples.at(1).getXPos(),apples.at(1).getYPos());
                        previousB = cv::Point2d(apples.at(0).getXPos(),apples.at(0).getYPos());
                    }
                }
                else{
                    speedDetect(apples,cameraFeed,final,tracking,previousA,previousB);
                }
            }
        }
        else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
}

//For The Aruco markers
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

// Aruco detector

pair<int, int> arucoDetect(Mat &videoImg, Mat &final, Mat &tracking, Ptr<Dictionary> diction, Ptr<DetectorParameters> detectorParams, bool estimatePose, float markerLen, Mat camMatrix, Mat distCoeffs, int totalTime, int totalIterations, float markerLength) {
    
    ///HERE THE LOOP STARTS SO AS THE ARUCO FINDING
    
    //Mat imageCopy;
            
    double tick = (double)getTickCount();
            
    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    vector< Vec3d > rvecs, tvecs;
        
    // detect markers and estimate pose
    aruco::detectMarkers(videoImg, diction, corners, ids, detectorParams);
        
    if(estimatePose && ids.size() > 0)
        aruco::estimatePoseSingleMarkers(corners, markerLen, camMatrix, distCoeffs, rvecs,
                                                 tvecs);
            
    double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
        
    if(totalIterations % 30 == 0) {
        cout << "Detection Time = " << currentTime * 1000 << " ms "
        << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
    }
            
    // draw results
    //videoImg.copyTo(imageCopy);
    if(ids.size() > 0) {
        aruco::drawDetectedMarkers(final, corners, ids);
                
    if(estimatePose) {
        for(unsigned int i = 0; i < ids.size(); i++)
            aruco::drawAxis(final, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
        //aruco::drawAxis(<#InputOutputArray image#>, <#InputArray cameraMatrix#>, <#InputArray distCoeffs#>, <#InputArray rvec#>, <#InputArray tvec#>, <#float length#>)
        }
    }
    
    //imshow("out", videoImg);
    
    
    
    return make_pair(totalTime, totalIterations);
}


/*
void drawFingerboard(cv::Mat &Image, Marker &m, const CameraParameters &CP) {
    
    float size = m.ssize * 3;
    Mat objectPoints(4, 3, CV_32FC1);
    objectPoints.at< float >(0, 0) = 0;
    objectPoints.at< float >(0, 1) = 0;
    objectPoints.at< float >(0, 2) = 0;
    objectPoints.at< float >(1, 0) = size;
    objectPoints.at< float >(1, 1) = 0;
    objectPoints.at< float >(1, 2) = 0;
    objectPoints.at< float >(2, 0) = 0;
    objectPoints.at< float >(2, 1) = size;
    objectPoints.at< float >(2, 2) = 0;
    objectPoints.at< float >(3, 0) = 0;
    objectPoints.at< float >(3, 1) = 0;
    objectPoints.at< float >(3, 2) = size;
    
    vector< Point2f > imagePoints;
    cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
    // draw lines of different colours
    cv::line(Image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255, 255), 1, CV_AA);
    cv::line(Image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0, 255), 1, CV_AA);
    cv::line(Image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0, 255), 1, CV_AA);
    putText(Image, "x", imagePoints[1], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255, 255), 2);
    putText(Image, "y", imagePoints[2], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0, 255), 2);
    putText(Image, "z", imagePoints[3], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0, 255), 2);
}
*/

//**********************************
//**********************************
//Main
//**********************************
//**********************************

int main(int argc, char* argv[])
{
    
    //if we would like to calibrate our filter values, set to true.
    bool calibrationMode = false;
    bool paused = false;
    
    //Matrix to store each frame of the webcam feed
    Mat cameraFeed1;
    Mat cameraFeed2;
    Mat final;
    Mat cameraInitial;
    Mat threshold;
    Mat HSV;
    
    //cv::Size resize(FRAME_WIDTH,FRAME_HEIGHT);
    
    if(calibrationMode){
        //create slider bars for HSV filtering
        createTrackbars();
    }
    
    //video capture object to acquire webcam feed
    //VideoCapture capture;
    //open capture object at location zero (default location for webcam)
    //capture.open(0); // video input
    capture.open(argv[1]); // alfonso1-string1.mp4 test3.mp4  testG.mp4 alfonso2(edit).mp4 alfonos01-string4-crop.mp4 bowSize.mp4 alfonos01-string4-edit.mp4
    //set height and width of capture frame
    //capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    //capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    
    //Acquire input size and adjust outputVideo properties
    Size frameSize = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH),
                  (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    printf("Capture Width: %i\n", (int)capture.get(CV_CAP_PROP_FRAME_WIDTH));
    printf("Capture Height: %i\n", (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    printf("Total number of frames: %i\n", (int)capture.get(CAP_PROP_FRAME_COUNT));
    
    //capture.get(6);'S','V','1','0'
    //'S','V','Q','3'
    //outputVideo.open("DEMO_PRESENTATION.mp4", VideoWriter::fourcc('S','V','Q','3'), capture.get(5), frameSize, true);
    
    
    
    
    //SART OF ARUCO PARAMETERS
    int dictionaryId = 0;
    bool showRejected = false;
    bool estimatePose = true;
    float markerLength = 0.05;
    
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    
    bool readOk = readDetectorParameters("detector_param_copy.xml", detectorParams);
    if(!readOk) {
        cerr << "Invalid detector parameters file" << endl;
        return 0;
    }
    
    // do corner refinement in markers
    detectorParams->doCornerRefinement = true;
    
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    
    Mat camMatrix, distCoeffs;
    
    if(estimatePose) {
        bool readOk = readCameraParameters("out_camera_data.xml", camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    double totalTime = 0;
    int totalIterations = 0;
    // END OF ARUCO PARAMETERS
    
    /*
    // Creation of the playback trackbar
    int totalVideoFrames = (int) capture.get(CV_CAP_PROP_FRAME_COUNT);
    char playbackTrackbar[50];
    
    if( totalVideoFrames!= 0 )
    {
        
        namedWindow(windowName);
        sprintf( playbackTrackbar, "Current Frame %i", totalVideoFrames);
        
        createTrackbar(playbackTrackbar, windowName, &videoTrackbarPos, totalVideoFrames, on_trackbar2);
        
    }
    */

    
    // Open file stream
    myfile.open(argv[2]); //alfonso1-string1.csv
    myfile<<"PixelDispA,PixelDispB,cmDisplacementA,cmDisplacementB,VelocityA,VelocityB,AccelarationA,AccelarationB,Inclination\n";
    
    
    //start an infinite loop where webcam feed is copied to cameraFeed matrix
    //all of our operations will be performed within this loop
    while(1){
        Mat tracking(frameSize, CV_8UC3, Scalar(0,0,0));

        //create slider bars for HSV filtering
        if(calibrationMode){
            
            createTrackbars();
        }
        
        //pause and unpause the playback
        if (!paused) {
            

            //store image to matrix
            capture.read(cameraFeed1);
            cameraFeed1.copyTo(final);
            // Check for the end of the video
            
            if (cameraFeed1.empty()){
                printf("END OF VIDEO FILE!\n");
                printf("Max bow pixels: %f\n", maxBowLength);
                myfile.close();
                // In order to concatenate the video csv and the sudio csv files
                // i use the bash command paste.
                // paste -d "," loudnessString4.csv bow_data.csv > concat1.csv
                //
                // for filtering out the zero values from the lost frames
                // i use awk
                // awk -F',' '{if ($1!=0 && $2!=0 && $3!=0 && $4!=0) print}' concat1.csv > concat1-filtered.csv
                //
                // remove character from excel
                // tr '\r' '\n' < 111.csv > 112.csv
                break;
            }
        
            //Debbuging Prints
            printf("Video FPS: %f\n", capture.get(5));
            printf("Current Frame Number: %f\n", capture.get(1));
            printf("Current Frame MSEC: %f\n", capture.get(0));
            
            if (capture.get(1)){
                
            }
        
            //capture.read(cameraInitial);
            //cv::resize(cameraInitial, cameraFeed1, resize);
        
            //capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
            //capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
            
            //convert frame from BGR to HSV colorspace
            //cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
            double currentframe = capture.get(1);
            if(calibrationMode==true){
                //if in calibration mode, we track objects based on the HSV slider values.
                cvtColor(cameraFeed1,HSV,COLOR_BGR2HSV);
                inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
                morphOps(threshold);
                imshow(windowName2,threshold);
                trackFilteredObject(threshold,HSV,cameraFeed1,final,tracking,currentframe);
            }
            else{
                //create our object of interest
                bowTrack track("bow");
            
                //first find apples
                cvtColor(cameraFeed1,HSV,COLOR_BGR2HSV);
                inRange(HSV,track.getHSVmin(),track.getHSVmax(),threshold);
                morphOps(threshold);
                trackFilteredObject(track,threshold,HSV,cameraFeed1,final,tracking, currentframe);
            
            }
        
            //Aruco Detect
       
            pair<int, int> totals = arucoDetect(cameraFeed1, final, tracking, dictionary, detectorParams, estimatePose, markerLength, camMatrix, distCoeffs, totalTime, totalIterations, markerLength);
        
            totals.first = totalTime;
            totals.second= totalIterations;
            
            
        
            //show frames
        
            //imshow(windowName2,threshold);
            //cv::setWindowProperty(windowName,CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
            
            
            //imshow(windowName,cameraFeed1);
            
            
            //imshow("Tracking",tracking);
            imshow("Final",final);
            //imshow(windowName1,HSV);
            
            // Playback trackbar
            //setTrackbarPos(playbackTrackbar, windowName, ++videoTrackbarPos);
        
            
            
            // save video file
            
        }
        // Ideally delay should be t/fps = 1000msec/30fps = 33,333333333333333
        // delay 30ms so that screen can refresh.
        // image will not appear without this waitKey() command

        //1000.0/capture.get(5);
        
        //outputVideo.write(final);
        
        
        
        char c = (char)waitKey(1);
        
        switch(c){
            case 27:
                //outputVideo.release();
                myfile.close();
                return 0;
            case 'p':
                paused = !paused;
                break;
            case 'c':
                calibrationMode = !calibrationMode;
                break;
        }

    }
    return 0;
}
