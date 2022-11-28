#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringSerial.h>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/videoio.hpp>
#include <iostream>
// Default speed of the motors, this is modified by the PID output
#define leftMotorBaseSpeed 26
#define rightMotorBaseSpeed 26

// Speed limits of the motors
#define min_speed 10
#define max_speed 70


using namespace cv;
using namespace std;
int white;

float errorr, errorSum, errorOld;  // Variables for the PID loop
int leftMotorSpeed, rightMotorSpeed; // Variables to hold the current motor speed (+-100%)

float Kp, Ki, Kd; // Variables to store the PID constants
int robot;
char speedstring[50];

void stop(void);
//void runForward(int left_speed, int right_speed);
//void turnLeft(int left_speed, int right_speed);
//void turnRight(int left_speed, int right_speed);
int constrain(int speed);
float PID(float lineDist);
int calculate_error(VideoCapture cap,Mat S);
Point findContourCentre(std::vector<cv::Point> contour);

int main(){

    wiringPiSetup();
    robot=serialOpen("/dev/ttyAMA0",57600);

    leftMotorSpeed = 0;  // Initialise Speed variables
    rightMotorSpeed = 0;

    errorr = 0;    // Initialise error variables
    errorSum = 0;
    errorOld = 0;
    Kp = 0.33;
    Ki = 0
    ;
    Kd = 0;

    //runForward(leftMotorBaseSpeed, rightMotorBaseSpeed);
    int pixel_error;

    VideoCapture cap;
    cap.open(0);

    Mat S;

    while(1){

        int pixel_error=calculate_error(cap,S);

        float output = PID(-pixel_error); // Calculate the PID output.

        leftMotorSpeed = leftMotorBaseSpeed - output;     // Calculate the modified motor speed
        rightMotorSpeed = rightMotorBaseSpeed + output;


        if(leftMotorSpeed<10&&leftMotorSpeed>0)
        {
            rightMotorSpeed = constrain(rightMotorSpeed);
            leftMotorSpeed = constrain(leftMotorSpeed);
            sprintf(speedstring, "#Baffff00%d,00%d,0%d,0%d", leftMotorSpeed, leftMotorSpeed, rightMotorSpeed, rightMotorSpeed);
            serialPrintf(robot, speedstring);
        }

        else if(-10<leftMotorSpeed&&leftMotorSpeed<=0)
        {
            rightMotorSpeed = constrain(rightMotorSpeed);
            leftMotorSpeed = constrain(leftMotorSpeed);
            sprintf(speedstring, "#Barrff00%d,00%d,0%d,0%d", -leftMotorSpeed, -leftMotorSpeed, rightMotorSpeed, rightMotorSpeed);
            serialPrintf(robot, speedstring);
        }
        else if(leftMotorSpeed<=-10)
        {
            rightMotorSpeed = constrain(rightMotorSpeed);
            leftMotorSpeed = constrain(leftMotorSpeed);
            sprintf(speedstring, "#Barrff0%d,0%d,0%d,0%d", -leftMotorSpeed, -leftMotorSpeed, rightMotorSpeed, rightMotorSpeed);
            serialPrintf(robot, speedstring);
        }
        else if(rightMotorSpeed<10&&rightMotorSpeed>0)
        {
            leftMotorSpeed = constrain(leftMotorSpeed);
            rightMotorSpeed = constrain(rightMotorSpeed);
            sprintf(speedstring, "#Baffff0%d,0%d,00%d,00%d",  leftMotorSpeed, leftMotorSpeed,rightMotorSpeed, rightMotorSpeed);
            serialPrintf(robot, speedstring);
        }

        else if(-10<rightMotorSpeed&&rightMotorSpeed<=0)
        {
            leftMotorSpeed = constrain(leftMotorSpeed);
            rightMotorSpeed = constrain(rightMotorSpeed);
            sprintf(speedstring, "#Baffrr0%d,0%d,00%d,00%d", leftMotorSpeed, leftMotorSpeed,-rightMotorSpeed, -rightMotorSpeed);
            serialPrintf(robot, speedstring);
        }
        else if(rightMotorSpeed<=-10)
        {
            leftMotorSpeed = constrain(leftMotorSpeed);
            rightMotorSpeed = constrain(rightMotorSpeed);
            sprintf(speedstring, "#Baffrr0%d,0%d,0%d,0%d", leftMotorSpeed, leftMotorSpeed, -rightMotorSpeed, -rightMotorSpeed);
            serialPrintf(robot, speedstring);
        }
        else
        {
            leftMotorSpeed = constrain(leftMotorSpeed);
            rightMotorSpeed = constrain(rightMotorSpeed);
            sprintf(speedstring, "#Baffff0%d,0%d,0%d,0%d", leftMotorSpeed, leftMotorSpeed, rightMotorSpeed, rightMotorSpeed);
            serialPrintf(robot, speedstring);

        }
        cout<<output<<endl;
        cout<<speedstring<<endl;


        // Apply new speed and direction to each motor
       /* if((leftMotorSpeed-rightMotorSpeed)>30)
            {
                leftMotorSpeed = constrain(leftMotorSpeed);
                rightMotorSpeed = constrain(rightMotorSpeed);
                turnRight(leftMotorSpeed, rightMotorSpeed);
            }
        else if((leftMotorSpeed-rightMotorSpeed)<-30)
            {
                leftMotorSpeed = constrain(leftMotorSpeed);
                rightMotorSpeed = constrain(rightMotorSpeed);
                turnLeft(leftMotorSpeed, rightMotorSpeed);
            }
        else
        {
            leftMotorSpeed = constrain(leftMotorSpeed);
            rightMotorSpeed = constrain(rightMotorSpeed);
            runForward(leftMotorSpeed, rightMotorSpeed);
        }*/

        }
        serialClose(robot);
        return 0;
}


int calculate_error(VideoCapture cap,Mat S)
{

    cap>>S;
    Mat SHSV;
	Rect rect(0, 0, 640, 180);
    S = S(rect);
    //resize(S,S,Size(600,800));

    cvtColor(S,SHSV,CV_BGR2HSV);
    inRange(SHSV,Scalar(0,0,0),Scalar(180,255,65),SHSV);

    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    morphologyEx(SHSV, morphed, MORPH_OPEN, kernel);

    Mat edge;




    Canny(morphed, edge, 100, 100*3, 3);




    vector<vector<Point>> contours;    //
    vector<Vec4i> hierarchy;

    findContours(morphed, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);    //
    if(contours.size()>0)
    {

    Mat linePic = Mat::zeros(morphed.rows, morphed.cols, CV_8UC3);

    /*for (int index = 0; index < contours.size(); index++)
        {
            drawContours(linePic, contours, index, Scalar(0,0,255), 1, 8//hierarchy
                         );
        }*/


    vector<vector<Point>> polyContours(contours.size());
    int maxArea = 0;
    for (int index = 0; index < contours.size(); index++)
        {
            if (contourArea(contours[index]) > contourArea(contours[maxArea]))
            {
                maxArea = index;
            }

            approxPolyDP(contours[index], polyContours[index], 5, true);
        }

    Mat polyPic = Mat::zeros(SHSV.size(), CV_8UC3);

    //drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);

    Point centre;
    centre=findContourCentre(polyContours[maxArea]);
    //cout<<centre.x-320<<endl;

   //imshow("5",linePic);
    //imshow("3",S);

    imshow("1",polyPic);

    int b=centre.x-320;
    imshow("0",SHSV);
   // waitKey(10);
    white =b;
    return b;
    }

    else{return white;}
}






Point findContourCentre(std::vector<cv::Point> contour)
{
    Moments foundRegion;    // Variables to store the region moment and the centre point
    Point centre;
    foundRegion = moments(contour, false);      // Calculate the moment for the contour
    centre.x = (foundRegion.m10/foundRegion.m00);  //Calculate the X and Y positions
    centre.y = (foundRegion.m01/foundRegion.m00);

    return centre;
}




// Function to calculate the PID output.
float PID(float lineDist)
{
  // PID loop
  errorOld = errorr;        // Save the old error for differential component
  errorr = lineDist;  // Calculate the error in position
  errorSum += errorr;

  float proportional = errorr * Kp;  // Calculate the components of the PID

  float integral = errorSum * Ki;

  float differential = (errorr - errorOld) * Kd;

  float output = proportional + integral + differential;  // Calculate the result

  return output;
}


void stop(void)
{
    serialPrintf(robot,"#Ha");
}
/*void runForward(int left_speed, int right_speed)
{
    sprintf(speedstring, "#Baffff0%d,0%d,0%d,0%d", left_speed, left_speed, right_speed, right_speed);
    serialPrintf(robot, speedstring);
    cout<<speedstring<<endl;
}
void turnLeft(int left_speed, int right_speed)
{
    sprintf(speedstring, "#Barrff0%d,0%d,0%d,0%d", left_speed, left_speed, right_speed, right_speed);
    serialPrintf(robot, speedstring);
    printf("l\n");
}
void turnRight(int left_speed, int right_speed)
{
    sprintf(speedstring, "#Baffrr0%d,0%d,0%d,0%d", left_speed, left_speed, right_speed, right_speed);
    serialPrintf(robot, speedstring);
    printf("r\n");
}*/
int constrain(int speed)
{
    if(speed>max_speed) speed = max_speed;
    else if(speed<-60) speed = -60;

    else speed = speed;
    return speed;
}
