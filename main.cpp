#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <lcd.h>
#include <sys/time.h>
#include <cstdlib>
#include <iostream>


#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/videoio.hpp>
#include <iostream>
// Default speed of the motors, this is modified by the PID output
int leftMotorBaseSpeed=25;
int rightMotorBaseSpeed=25;


// Speed limits of the motors
#define min_speed 10
#define max_speed 80

#define servopin 29
#define Trig 21
#define Echo 22

#define LCD_RS  3               //Register select pin
#define LCD_E   0               //Enable Pin
#define LCD_D4  6               //Data pin 4
#define LCD_D5  1               //Data pin 5
#define LCD_D6  5               //Data pin 6
#define LCD_D7  4               //Data pin 7

float disMeasure();
using namespace cv;
using namespace std;

int white;
float errorr, errorSum, errorOld;  // Variables for the PID loop
int leftMotorSpeed, rightMotorSpeed; // Variables to hold the current motor speed (+-100%)

float Kp, Ki, Kd; // Variables to store the PID constants
int robot;
char speedstring[50];

int constrain(int speed);
float PID(float lineDist);
void stop(void);

int colordetection(VideoCapture cap,Mat S);
int calculate_error(VideoCapture cap,Mat S,int,int,int,int,int,int);
int templatematching(Mat outPic);
int Symbol(VideoCapture cap,Mat S);

int line=0;

void servoHeadUp();
void servoHeadDown();

Point findContourCentre(std::vector<cv::Point> contour);

Point2f srcPoints[4], dstPoints[4];


int Trafficlight(VideoCapture cap);
float disMeasure();

int colordetection_blue(VideoCapture cap,Mat S);
int colordetection_green(VideoCapture cap,Mat S);
int colordetection_yellow(VideoCapture cap,Mat S);
int colordetection_red(VideoCapture cap,Mat S);

int main(){

    //int leftMotorBaseSpeed 26;
    //int rightMotorBaseSpeed 26;

    wiringPiSetup();
    robot=serialOpen("/dev/ttyAMA0",57600);

    leftMotorSpeed = 0;  // Initialise Speed variables
    rightMotorSpeed = 0;

    errorr = 0;    // Initialise error variables
    errorSum = 0;
    errorOld = 0;
    Kp = 0.3
    ;
    Ki = 0;
    Kd = 0.08;

    int pixel_error;
    int lcd;

    if (lcd = lcdInit (2, 16, 4, LCD_RS, LCD_E ,LCD_D4 , LCD_D5, LCD_D6,LCD_D7,0,0,0,0))
    {
        printf ("lcdInit failed! \n");
        return -1 ;
    }




    pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);


    //runForward(leftMotorBaseSpeed, rightMotorBaseSpeed);

    int triangle=0;
    int circle=0;
    int square=0;

    int counter=0;
    int line=1;


    VideoCapture cap;
    cap.open(0);
    Mat S;


    while(1){


        if(line==1)
        {
            int color_line=colordetection_blue(cap,S);
            cout<<"color_line"<<color_line<<endl;
            if(color_line==2)
            {
                Kp = 0.4;
                pixel_error=calculate_error(cap,S,70,110,55,150,255,120);
                Kp = 0.28;
            }

            else if (color_line==3)
            {
                pixel_error=calculate_error(cap,S,0,0,0,180,255,66);
            }
        }


        if(line==2)
        {
            int color_line=colordetection_green(cap,S);
            cout<<"color_line"<<color_line<<endl;



            if(color_line==2)
            {
                Kp = 1;
                pixel_error=calculate_error(cap,S,0,140,0,80,255,120);
                rightMotorBaseSpeed=25;
                leftMotorBaseSpeed=25;
                Kp = 0.28;
            }

            else if (color_line==3)
            {
                pixel_error=calculate_error(cap,S,0,0,0,180,255,66);
                rightMotorBaseSpeed=25;
                leftMotorBaseSpeed=25;
            }
        }


        if(line==3)
        {
            int color_line=colordetection_red(cap,S);
            cout<<"color_line"<<color_line<<endl;
            if(color_line==2)
            {
                Kp = 0.35;
                pixel_error=calculate_error(cap,S,130,200,110,180,255,173);
                leftMotorBaseSpeed=24;
                rightMotorBaseSpeed=24;
                Kp = 0.28;
            }


            else if (color_line==3)
            {
                pixel_error=calculate_error(cap,S,0,0,0,180,255,66);
                rightMotorBaseSpeed=25;
                leftMotorBaseSpeed=25;
            }
        }


        if(line==4)
        {
            int color_line=colordetection_yellow(cap,S);



            if(color_line==2)
            {
                Kp = 2;
                pixel_error=calculate_error(cap,S,0,40,100,50,255,255);
                Kp = 0.28;
            }

            else if (color_line==3)
            {

                pixel_error=calculate_error(cap,S,0,0,0,180,255,66);
            }
        }



        int color=colordetection(cap,S);
        if(color==2&&counter==0)
        {
            stop();
            servoHeadUp();
            int symbol=Symbol(cap,S);

            cout<<"symbol is "<<symbol<<endl;


            if(symbol==10)
            {
                lcdPosition(lcd,0,0);
                lcdPrintf(lcd,"Traffic light");
                delay(2000);
                lcdClear(lcd);
                cout<<symbol<<endl;
                while(1)
                {
                    int trafficlight=Trafficlight(cap);
                    if (trafficlight>=1)
                    {
                        break;
                    }
                }

            }


            servoHeadDown();
            for(int s=0;s<30;s++)
            {
                cap>>S;
            }

            if(symbol==999)
            {
                counter++;
                continue;
            }




            else if(symbol==5)
            {
                float dis = disMeasure();
                lcdPosition(lcd,0,0);
                lcdPuts(lcd, "Distance :");
                lcdPosition(lcd,0,1);
                lcdPrintf(lcd, "%0.2f", dis);
                delay(3000);
                lcdClear(lcd);
            }

            else if(symbol==6)
            {
                lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
                lcdPuts(lcd, "blue");  //Print the text on the LCD at the current cursor postion
                delay(2000);
                lcdClear(lcd);
                line=1;
            }

            else if(symbol==7)
            {
                lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
                lcdPuts(lcd, "green");  //Print the text on the LCD at the current cursor postion
                delay(2000);
                lcdClear(lcd);

                line=2;
            }

            else if(symbol==8)
            {
                lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
                lcdPuts(lcd, "red");  //Print the text on the LCD at the current cursor postion
                delay(2000);
                lcdClear(lcd);
                line=3;
            }

            else if(symbol==9)
            {
                lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
                lcdPuts(lcd, "yellow");  //Print the text on the LCD at the current cursor postion
                delay(2000);
                lcdClear(lcd);


                line=4;
            }


            else if(symbol==11)
            {
                lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
                lcdPuts(lcd, "Turn L");  //Print the text on the LCD at the current cursor postion
                delay(3000);
                lcdClear(lcd);

                sprintf(speedstring, "#Baffff0%d,0%d,0%d,0%d", 20,20,20,20);

                serialPrintf(robot, speedstring);
                cout<<speedstring<<endl;
                delay(1000);
                sprintf(speedstring, "#Barrff0%d,0%d,0%d,0%d", 80,80,80,80);

                serialPrintf(robot, speedstring);
                cout<<speedstring<<endl;
                delay(1500);
            }

            else if(symbol==12)
            {
                lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
                lcdPuts(lcd, "Turn R");  //Print the text on the LCD at the current cursor postion
                delay(3000);
                lcdClear(lcd);
                sprintf(speedstring, "#Baffff0%d,0%d,0%d,0%d", 20,20,20,20);
                serialPrintf(robot, speedstring);
                cout<<speedstring<<endl;
                delay(1000);
                sprintf(speedstring, "#Baffrr0%d,0%d,0%d,0%d", 80,80,80,80);
                serialPrintf(robot, speedstring);
                cout<<speedstring<<endl;
                delay(1500);
            }

            //cout<<symbol<<endl;
            counter++;
            continue;


        }
        else if(color==2&&counter>0)
        {
            pixel_error=calculate_error(cap,S,0,0,0,180,255,70);
            //cout<<"RETURN COLOR"<<color<<endl;
        }
        else
        {
            //pixel_error=calculate_error(cap,S,0,0,0,180,255,70);
            counter=0;
            //cout<<"return color"<<color<<endl;
        }

        //cout<<"pixel_error "<<pixel_error<<endl;
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
       // cout<<"output "<<output<<endl;
       // cout<<speedstring<<endl;




        }

        serialClose(robot);
        return 0;
}



int colordetection(VideoCapture cap,Mat S)
{
    cap>>S;
    Mat SHSV,SHSV2;
	Rect rect(0, 0, 640, 240);
    S = S(rect);
    cvtColor(S,SHSV,CV_BGR2HSV);
    inRange(SHSV,Scalar(155,140,140),Scalar(167,237,185),SHSV2);
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    morphologyEx(SHSV2, morphed, MORPH_OPEN, kernel);

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

    //Mat polyPic = Mat::zeros(SHSV.size(), CV_8UC3);

    //drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);

    Point centre;
    //printf("H=%d\t",SHSV.at<Vec3b>(centre)[0]);
    //printf("S=%d\t",SHSV.at<Vec3b>(centre)[1]);
    //printf("V=%d\t",SHSV.at<Vec3b>(centre)[2]);
    //centre=findContourCentre(polyContours[maxArea]);
    //if(SHSV.at<Vec3b>(centre)[0]>157&&SHSV.at<Vec3b>(centre)[1]>45&&SHSV.at<Vec3b>(centre)[2]>43)
    if(SHSV.at<Vec3b>(centre)[0]>=0)
    {
        return 2;
    }

   //imshow("5",linePic);
    //imshow("3",S);

    //imshow("1",polyPic);


    //imshow("0",SHSV);
   // waitKey(10);

    else {return 3;}

    }

    else{return 3;}
}

 int Symbol(VideoCapture cap,Mat S)
 {



    for(int s=0;s<30;s++)
    {
        cap>>S;
    }

    cap.set(CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CAP_PROP_FRAME_HEIGHT,960);

    imshow("s",S);
    //waitKey(50);

    cap>>S;

    //cout<<"capture!!!"<<endl;
    if(!cap.isOpened())
    {
        cout<<"camera is not opened"<<endl;
        return -1;
    }

    if(S.empty())
    {
        cout<<"S is nothing"<<endl;
        return -1;
    }
    cap.set(CAP_PROP_FRAME_WIDTH,640);
    cap.set(CAP_PROP_FRAME_HEIGHT,480);
    Mat SHSV;

    imwrite("S.jpg", S);

    //resize(S,S,Size(600,800));

    cvtColor(S,SHSV,CV_BGR2HSV);
    inRange(SHSV,Scalar(160,134,60),Scalar(180,255,255),SHSV);

    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    morphologyEx(SHSV, morphed, MORPH_OPEN, kernel);

    Mat edge;

    Canny(morphed, edge, 100, 100*3, 3);

    vector<vector<Point>> contours;    //´¢´æÂÖÀª
    vector<Vec4i> hierarchy;

    findContours(morphed, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);    //»ñÈ¡ÂÖÀª

    if(contours.size()>0)
 {

    Mat linePic = Mat::zeros(morphed.rows, morphed.cols, CV_8UC3);

    for (int index = 0; index < contours.size(); index++)
        {
            drawContours(linePic, contours, index, Scalar(0,0,255), 1, 8);
        }

    vector<vector<Point>> polyContours(contours.size());
    int maxArea = 0;
    for (int index = 0; index < contours.size(); index++)
        {
            if (contourArea(contours[index]) > contourArea(contours[maxArea]))
            {
                maxArea = index;
            }

            approxPolyDP(contours[index], polyContours[index], 50, true);
        }

    if(polyContours.size()>0){
    Mat polyPic = Mat::zeros(SHSV.size(), CV_8UC3);

    drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);


    imshow("2",polyPic);


    vector<int>  hull;
    convexHull(polyContours[maxArea], hull, false);    //¼ì²â¸ÃÂÖÀªµÄÍ¹°ü


    if(polyContours[maxArea].size()==4)
    {

    //Point2f srcPoints[4], dstPoints[4];

    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(SHSV.cols, 0);
    dstPoints[2] = Point2f(SHSV.cols, SHSV.rows);
    dstPoints[3] = Point2f(0, SHSV.rows);

    /*for (int i = 0; i < 4; ++i)
        {
            circle(polyPic, polyContours[maxArea][i], 10, Scalar(rand() & 255, rand() & 255, rand() & 255), 3);
        }*/

    //addWeighted(polyPic, 0.5, S, 0.5, 0, S);

    //srcPoints[0]=polyContours[maxArea][1];
    //srcPoints[1]=polyContours[maxArea][0];

    //srcPoints[2]=polyContours[maxArea][3];
    //srcPoints[3]=polyContours[maxArea][2];






    bool sorted = false;
    int n = 4;
    while (!sorted)
    {
        for (int i = 1; i < n; i++)
        {
            sorted = true;
            if (polyContours[maxArea][i-1].x > polyContours[maxArea][i].x)
            {
                swap(polyContours[maxArea][i-1], polyContours[maxArea][i]);
                sorted = false;
            }
        }

        n--;
    }

    if (polyContours[maxArea][0].y < polyContours[maxArea][1].y){
        srcPoints[0] = polyContours[maxArea][0];
        srcPoints[3] = polyContours[maxArea][1];
    }
    else{
        srcPoints[0] = polyContours[maxArea][1];
        srcPoints[3] = polyContours[maxArea][0];
    }

    if (polyContours[maxArea][2].y < polyContours[maxArea][3].y) {
        srcPoints[1] = polyContours[maxArea][2];
        srcPoints[2] = polyContours[maxArea][3];
    }
    else {
        srcPoints[1] = polyContours[maxArea][3];
        srcPoints[2] = polyContours[maxArea][2];
    }


    Mat transMat = getPerspectiveTransform(srcPoints, dstPoints); //µÃµ½±ä»»¾ØÕó
    Mat outPic;
    warpPerspective(S, outPic, transMat,S.size());

    resize(outPic,outPic,Size(320,240));
    imshow("3",outPic);
    waitKey(1000);

    cvtColor(outPic,outPic,CV_BGR2GRAY);


    threshold(outPic, outPic, 125, 255, CV_THRESH_BINARY);

    int q=templatematching(outPic);
    cout<<"q is "<<q<<endl;
    imwrite("output.jpg", outPic);

    if(q==13){return 999;}
    else{return q;}
    }

    else{return 999;}
    }

    else{return 999;}
    }

    else{return 999;}
 }


int calculate_error(VideoCapture cap,Mat S,int Hmin, int Smin, int Vmin, int Hmax, int Smax, int Vmax)
{

    cap>>S;
    Mat SHSV;
	Rect rect(0, 0, 640, 180);
    S = S(rect);
    //resize(S,S,Size(600,800));

    cvtColor(S,SHSV,CV_BGR2HSV);

    inRange(SHSV,Scalar(Hmin,Smin,Vmin),Scalar(Hmax,Smax,Vmax),SHSV);

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

    drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);

    Point centre;
    centre=findContourCentre(polyContours[maxArea]);


    imshow("5",linePic);
    //imshow("3",S);

//    imshow("1",polyPic);

    int b=centre.x-320;
//    imshow("0",SHSV);
 //   waitKey(10);
    white = b;
    cout<<"return cal "<<b<<endl;
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

int templatematching(Mat outPic)
{
    double Similarity[12];
    Mat S1=imread("Symbols/CountShape1.png");
    Mat S2=imread("Symbols/CountShape2.png");
    Mat S3=imread("Symbols/CountShape3.png");
    Mat S4=imread("Symbols/Football.png");
    Mat S5=imread("Symbols/MeasureDistance.png");
    Mat S6=imread("Symbols/ShortCutBlue.png");
    Mat S7=imread("Symbols/ShortcutGreen.png");
    Mat S8=imread("Symbols/ShortcutRed.png");
    Mat S9=imread("Symbols/ShortcutYellow.png");
    Mat S10=imread("Symbols/TrafficLight.png");
    Mat S11=imread("Symbols/TurnLeft.png");
    Mat S12=imread("Symbols/TurnRight.png");

    Mat SS[12]={S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12};
    float eeee[12];

    for(int i=0;i<12;i++)
    {
        cv::Mat dstImg;

        cvtColor(SS[i],SS[i],CV_BGR2GRAY);
        threshold(SS[i], SS[i], 200, 255, CV_THRESH_BINARY);
        //inRange(SS[i],Scalar(0,0,0),Scalar(360,251,255),SS[i]);

      //  imshow("a",SS[i]);
      //  waitKey(1000);

        dstImg.create(outPic.dims,outPic.size,outPic.type());
        matchTemplate(outPic, SS[i], dstImg, 5);


        float a;

        a = dstImg.at<float>(0,0);

        //Scalar ss = sum(dstImg);
       // eeee[i]=ss[0];
        eeee[i]=a;
        cout << eeee[i] << endl;

    }

    float highest=0;
    int b;
        for (int i = 0; i < 12; i++)
    {
        if (eeee[i] > highest)
            {
                highest = eeee[i];
                b=i;
            }
    }
    cout<<"highest is "<<highest<<endl;
    cout<<"b+1= "<<b+1<<endl;
    if(highest>=0.2)
    {
        return b+1;
    }

    else if (highest<0.2)
    {
        return 13;
    }
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

int constrain(int speed)
{
    if(speed>max_speed) speed = max_speed;
    else if(speed<-80) speed = -80;

    else speed = speed;
    return speed;
}

 void servoHeadUp(){
     softPwmCreate(servopin, 15, 200);
     softPwmWrite(servopin,16);
     delay(700);
     softPwmStop(servopin);
 }

 void servoHeadDown(){
     softPwmCreate(servopin, 20, 200);
     softPwmWrite(servopin,24);
     delay(700);
     softPwmStop(servopin);
 }

  float disMeasure()
 {
     struct timeval tv1;
     struct timeval tv2;
     long start, stop;
     float dis;

     digitalWrite(Trig, LOW);
     delayMicroseconds(2);

     digitalWrite(Trig, HIGH);
     delayMicroseconds(10);
     digitalWrite(Trig, LOW);

     while(!(digitalRead(Echo) == 1));
     gettimeofday(&tv1, NULL);

     while(!(digitalRead(Echo) == 0));
     gettimeofday(&tv2, NULL);

     start = tv1.tv_sec * 1000000 + tv1.tv_usec;
     stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;

     dis = 7*(float)(stop - start) / 1000000 * 3400 / 2;

     return dis;
 }


 int Trafficlight(VideoCapture cap)
 {
        int LowH = 0;
	int HighH = 180;

	int LowS = 0;
	int HighS = 255;

	int LowV = 0;
	int HighV = 70;

    Mat test, HSV, mask1, mask2,intr_HSV, bright_area;
    double area=0;
    double max_area=0;
    int max_ID=0;
    vector<vector<Point>> contours;
    vector<vector<Point>> contour_green;

    cap>>test;
    resize(test,test,Size(640,480));
    imshow("2",test);

    mask1=test;

    mask1 = mask1(Rect(srcPoints[0].x/2-80,srcPoints[0].y/2, 80, 120));
    imwrite("mask.jpg",mask1);
    imshow("mask1",mask1);

    Mat gray;
    cvtColor(mask1,gray,COLOR_BGR2GRAY);

    Point minLoc, maxLoc;
    double minValue, maxValue;
    int number=0;
    minMaxLoc(gray, &minValue, &maxValue, &minLoc, &maxLoc);

    cvtColor(mask1,HSV,COLOR_BGR2HSV);
    inRange(HSV, Scalar(44,55,55), Scalar(72,255,255), mask2);
    imshow("HSV thres", mask1);
    imshow("origin", mask2);
    findContours(mask2, contour_green, RETR_TREE, CHAIN_APPROX_SIMPLE);//CHAIN_APPROX_NO
    for(int i=0;i<contour_green.size();i++)
    {
        	double min_x=10000;
            double min_y=10000;
            double max_x=0;
            double max_y=0;
        for(int j=0;j<contour_green[i].size();j++)
        {
            if (contour_green[i][j].x>max_x)
            {
                max_x=contour_green[i][j].x;
            }
            if (contour_green[i][j].y>max_y)
            {
                max_y=contour_green[i][j].y;
            }
            if (contour_green[i][j].x<min_x)
            {
                min_x=contour_green[i][j].x;
            }
            if (contour_green[i][j].y<min_y)
            {
                min_y=contour_green[i][j].y;
            }
        }
        if (maxLoc.x<=max_x&&maxLoc.x>=min_x&&maxLoc.y>=min_y&&maxLoc.y<=max_y)
        {
            number++;
        }
    }
    if(number>=1){
        cout << "Green" << endl;
    }else{
        cout << "Red" << endl;
    }
waitKey(20);
    return number;
 }


 int colordetection_blue(VideoCapture cap,Mat S)
{
    cap>>S;
    Mat SHSV,SHSV2;
	Rect rect(0, 0, 640, 180);
    S = S(rect);
    cvtColor(S,SHSV,CV_BGR2HSV);
    inRange(SHSV,Scalar(80,120,61),Scalar(140,255,110),SHSV2);
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    morphologyEx(SHSV2, morphed, MORPH_OPEN, kernel);

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

    drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);

    Point centre;
    printf("H=%d\t",SHSV.at<Vec3b>(centre)[0]);
    printf("S=%d\t",SHSV.at<Vec3b>(centre)[1]);
    printf("V=%d\t",SHSV.at<Vec3b>(centre)[2]);
    centre=findContourCentre(polyContours[maxArea]);
    if(SHSV.at<Vec3b>(centre)[0]>0&&SHSV.at<Vec3b>(centre)[1]>0&&SHSV.at<Vec3b>(centre)[2]<255)
    {
        return 2;
           //imshow("5",linePic);
    //imshow("3",S);

    //imshow("1",polyPic);


    //imshow("0",SHSV);
    //waitKey(10);
    }


    else {return 3;}

    }


    else{return 3;}
}


int colordetection_green(VideoCapture cap,Mat S)
{
    cap>>S;
    Mat SHSV,SHSV2;
	Rect rect(0, 0, 640, 180);
    S = S(rect);
    cvtColor(S,SHSV,CV_BGR2HSV);
    inRange(SHSV,Scalar(0,152,0),Scalar(75,255,105),SHSV2);
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    morphologyEx(SHSV2, morphed, MORPH_OPEN, kernel);

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

    drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);

    Point centre;
    printf("H=%d\t",SHSV.at<Vec3b>(centre)[0]);
    printf("S=%d\t",SHSV.at<Vec3b>(centre)[1]);
    printf("V=%d\t",SHSV.at<Vec3b>(centre)[2]);
    centre=findContourCentre(polyContours[maxArea]);
    if(SHSV.at<Vec3b>(centre)[0]<181&&SHSV.at<Vec3b>(centre)[1]>0&&SHSV.at<Vec3b>(centre)[2]<256)
    {
        return 2;
           //imshow("5",linePic);
    //imshow("3",S);

    //imshow("1",polyPic);


    //imshow("0",SHSV);
    //waitKey(10);
    }


    else {return 3;}

    }


    else{return 3;}
}


int colordetection_red(VideoCapture cap,Mat S)
{
    cap>>S;
    Mat SHSV,SHSV2;
	Rect rect(0, 0, 640, 180);
    S = S(rect);
    cvtColor(S,SHSV,CV_BGR2HSV);
    inRange(SHSV,Scalar(130,200,110),Scalar(180,255,173),SHSV2);
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    morphologyEx(SHSV2, morphed, MORPH_OPEN, kernel);

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

    drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);

    Point centre;
    printf("H=%d\t",SHSV.at<Vec3b>(centre)[0]);
    printf("S=%d\t",SHSV.at<Vec3b>(centre)[1]);
    printf("V=%d\t",SHSV.at<Vec3b>(centre)[2]);
    centre=findContourCentre(polyContours[maxArea]);
    if(SHSV.at<Vec3b>(centre)[0]>0&&SHSV.at<Vec3b>(centre)[1]>0&&SHSV.at<Vec3b>(centre)[2]>0)
    {
        return 2;
           //imshow("5",linePic);
    //imshow("3",S);

    //imshow("1",polyPic);


    //imshow("0",SHSV);
    //waitKey(10);
    }


    else {return 3;}

    }


    else{return 3;}
}


int colordetection_yellow(VideoCapture cap,Mat S)
{
    cap>>S;
    Mat SHSV,SHSV2;
	Rect rect(0, 0, 640, 180);
    S = S(rect);
    cvtColor(S,SHSV,CV_BGR2HSV);
    inRange(SHSV,Scalar(0,64,120),Scalar(27,255,255),SHSV2);
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
    morphologyEx(SHSV2, morphed, MORPH_OPEN, kernel);

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

    //Mat polyPic = Mat::zeros(SHSV.size(), CV_8UC3);

    //drawContours(polyPic, polyContours, maxArea, Scalar(0,0,255/*rand() & 255, rand() & 255, rand() & 255*/), 2);

    Point centre;
    printf("H=%d\t",SHSV.at<Vec3b>(centre)[0]);
    printf("S=%d\t",SHSV.at<Vec3b>(centre)[1]);
    printf("V=%d\t",SHSV.at<Vec3b>(centre)[2]);
    centre=findContourCentre(polyContours[maxArea]);
    if(SHSV.at<Vec3b>(centre)[0]<180&&SHSV.at<Vec3b>(centre)[1]>0&&SHSV.at<Vec3b>(centre)[2]>0)
    {
        return 2;
    }

   //imshow("5",linePic);
    //imshow("3",S);

    //imshow("1",polyPic);


    //imshow("0",SHSV);
   // waitKey(10);

    else {return 3;}

    }

    else{return 3;}
}
