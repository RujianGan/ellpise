#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Aria.h"
#include <termios.h>	// POSIX terminal control definitionss
#include <pthread.h>
#include <string.h>
#include <unistd.h>		// UNIX standard function definitions
#include <fcntl.h>		// File control definitions
#include <fstream>

using namespace cv;
using namespace std;

#ifndef PI
#define PI 3.1415926f
#endif // !PI

Mat mFrame;
Mat src_gray;
int thresh = 40;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void *Ellipse_thread(void* ha);

// open camera
VideoCapture cap(0);

//设置控制标志
bool fstop = false;

//检测参数
float sub_width[2],sub_height[2],sub_pos_real[2];
float avg_width,avg_height,pos_real,dis_real,ang_real;
unsigned int Ellipse_count = 0;

//参考参数
float height_real = 0.045;  //圆的实际高度
float dis_ref = 0.5,ang_ref = 0.52,pos_ref = 360;

//求得的参数，在过程中使用
float offset_real;  //实际中领导者与小车
float th1,th2;      //跟随者与领导者连线之间夹角，跟随者分别与领导者，虚拟领导者之间的夹角
float xe,ye,the;    //以跟随者为参考系下的虚拟领导者的坐标
float dis_virtual;  //跟随者与虚拟领导者时间的距离

float kx=0.25,ky=1,kth=1,kd=3;  //系数，可调
float v_control,w_control;  //控制输入

//data saving
ofstream oData;

/** @function main */
int main(int argc, char** argv)
{
    //判断摄像头开启与否
    if(!cap.isOpened())
    {
        return -1;
    }

    ArRobot robot;
    //amigo连接初始化
    Aria::init();

    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArLog::log(ArLog::Terse, "Be careful! You can not afford it!!!");
    // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
    // and then loads parameter files for this robot.
    ArRobotConnector robotConnector(&parser, &robot);
    if (!robotConnector.connectRobot())
    {
      ArLog::log(ArLog::Terse, "Could not connect to the robot.");
      if (parser.checkHelpAndWarnUnparsed())
      {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
      }
    }
    if (!Aria::parseArgs())
    {
      Aria::logOptions();
      Aria::exit(1);
      return 1;
    }

	//oData.open("data.txt", ios::out | ios::trunc);
	//oData << "dis_error\tpos_error\theight\twidth" << endl;
    // add pthread
    pthread_t pthread_camera;
    pthread_create(&pthread_camera,NULL,Ellipse_thread,NULL);


  	ArLog::log(ArLog::Normal, "Connected.");
  	// Start the robot processing cycle running in the background.
  	// True parameter means that if the connection is lost, then the
  	// run loop ends.
  	robot.runAsync(true);
  	// Print out some data from the SIP.
  	// We must "lock" the ArRobot object
  	// before calling its methods, and "unlock" when done, to prevent conflicts
  	// with the background thread started by the call to robot.runAsync() above.
  	// See the section on threading in the manual for more about this.
  	// Make sure you unlock before any sleep() call or any other code that will
  	// take some time; if the robot remains locked during that time, then
  	// ArRobot's background thread will be blocked and unable to communicate with
  	// the robot, call tasks, etc.
  	robot.lock();
  	ArLog::log(ArLog::Normal, "Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
  	    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  	robot.unlock();

  	// Sleep for 3 seconds.
  	ArLog::log(ArLog::Normal, "Will start driving in 3 seconds...");
  	ArUtil::sleep(3000);

  	// The main part!!
  	ArLog::log(ArLog::Normal, "Go test...");
  	robot.lock();
  	robot.enableMotors();
  	while(fabsf(dis_real-dis_ref)>0.02||fabsf(pos_ref-pos_real)>10)  //误差10度，2厘米
  	{
        //offset_real = height_real * (pos_ref-pos_real) / avg_height;
        //th1 = asinf(offset_real/dis_real);
        //dis_virtual = sqrtf(dis_real*dis_real+dis_ref*dis_ref-2*dis_ref*dis_real*cosf(ang_real));
        //th2 = asinf(sinf(ang_real)*dis_ref/dis_virtual);
        //the = th1-th2;
        //xe = dis_virtual * cosf(the);
        //ye = dis_virtual * sinf(the);
        //v_control = kx * (1-expf(-0.5*xe))/(1+expf(-0.5*xe)) + kd * (1-expf(-0.5*(dis_real-dis_ref)))/(1+expf(-0.5*(dis_real-dis_ref)));
        //w_control = ky * (1-expf(-0.5*ye))/(1+expf(-0.5*ye)) + kth * (1-expf(-0.5*the))/(1+expf(-0.5*the));
			v_control = kd * (1-expf(-0.5*(dis_real-dis_ref)))/(1+expf(-0.5*(dis_real-dis_ref)));
			w_control = kx * (1-expf(-0.5*(pos_ref-pos_real)/100))/(1+expf(-0.5*(pos_ref-pos_real)/100));
			//if(fabsf(pos_ref-pos_real) < 100)
			//	w_control += kth * (1-expf(-0.5*(ang_real-ang_ref)))/(1+expf(-0.5*(ang_real-ang_ref)));
			robot.setVel(v_control*1000);//v_control*1000
			robot.setRotVel(w_control*180/PI);//w_control*180/PI
	
			robot.unlock();
			ArUtil::sleep(10);
    		robot.lock();
  	}
	
  	robot.setVel(0);
  	robot.setRotVel(0);
  	robot.unlock();
  	ArUtil::sleep(1000);

  	ArLog::log(ArLog::Normal, "Stopping.");
  	robot.lock();
  	robot.stop();
  	robot.unlock();
  	ArUtil::sleep(1000);
	oData.close();

  	ArLog::log(ArLog::Normal, "Ending robot thread...");
  	robot.stopRunning();
  	// wait for the thread to stop
  	robot.waitForRunExit();
  	// exit
  	ArLog::log(ArLog::Normal, "Exiting...");
  	Aria::exit(0);

    return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void*)
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// 阈值化检测边界
    threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);
    /// 寻找轮廓
    findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    /*cout << hierarchy[7][3] << contours.size();*/
    /// 对每个找到的轮廓创建可倾斜的边界框和椭圆
    vector<RotatedRect> minRect(contours.size());
    vector<RotatedRect> minEllipse(4);    //o o
    vector<RotatedRect> Ellipse_temp(contours.size());

    for (unsigned int j,i = 0; i < contours.size(); i++)
    {
        minRect[i] = minAreaRect(Mat(contours[i]));
        if (contours[i].size() > 20)
        {
            Ellipse_temp[i] = fitEllipse(Mat(contours[i]));
            for(j = 0; j < i; j++)
            {
					//Ellipse_temp[i||j].center.y < 80 according to height to change the circle
                if (fabsf(Ellipse_temp[i].center.x - Ellipse_temp[j].center.x) < 5 && fabsf(Ellipse_temp[i].center.y - Ellipse_temp[j].center.y) < 5)
                    break;
            }
            if(j < i)
            {
                minEllipse[Ellipse_count] = fitEllipse(Mat(contours[i]));
                minEllipse[Ellipse_count+1] = fitEllipse(Mat(contours[j]));
                Ellipse_count += 2;
                if(Ellipse_count > 2)
                    break;
            }
        }
    }
    // 判断摄像头中两组圆的位置关系
    if (Ellipse_count == 2)
    {
        if(minEllipse[0].center.x < pos_ref)
        {
            sub_height[1] = (minEllipse[0].size.height + minEllipse[1].size.height)/2;
            sub_width[1] = (minEllipse[0].size.width + minEllipse[1].size.width)/2;
            sub_pos_real[1] = (minEllipse[0].center.x + minEllipse[1].center.x)/2;
        }
        else
        {
            sub_height[0] = (minEllipse[0].size.height + minEllipse[1].size.height)/2;
            sub_width[0] = (minEllipse[0].size.width + minEllipse[1].size.width)/2;
            sub_pos_real[0] = (minEllipse[0].center.x + minEllipse[1].center.x)/2;
        }
    }
    if (Ellipse_count == 4)
    {
        if(minEllipse[2].center.x > minEllipse[0].center.x)
        {
            sub_height[0] = (minEllipse[0].size.height + minEllipse[1].size.height)/2;
            sub_width[0] = (minEllipse[0].size.width + minEllipse[1].size.width)/2;
            sub_pos_real[0] = (minEllipse[0].center.x + minEllipse[1].center.x)/2;
            sub_height[1] = (minEllipse[2].size.height + minEllipse[3].size.height)/2;
            sub_width[1] = (minEllipse[2].size.width + minEllipse[3].size.width)/2;
            sub_pos_real[1] = (minEllipse[2].center.x + minEllipse[3].center.x)/2;
        }
        else
        {
            sub_height[0] = (minEllipse[2].size.height + minEllipse[3].size.height)/2;
            sub_width[0] = (minEllipse[2].size.width + minEllipse[3].size.width)/2;
            sub_pos_real[0] = (minEllipse[2].center.x + minEllipse[3].center.x)/2;
            sub_height[1] = (minEllipse[0].size.height + minEllipse[1].size.height)/2;
            sub_width[1] = (minEllipse[0].size.width + minEllipse[1].size.width)/2;
            sub_pos_real[1] = (minEllipse[0].center.x + minEllipse[1].center.x)/2;
        }
    }
    avg_height = sub_height[0]>sub_height[1]?sub_height[0]:sub_height[1];
    avg_width = sub_width[0]>sub_width[1]?sub_width[0]:sub_width[1];
	if(avg_width > avg_height) avg_width = avg_height;
    if(avg_height > 30 && avg_width > 30)
    {
        pos_real = (sub_pos_real[0]+sub_pos_real[1])/2;    //椭圆在摄像头图像中的位置（无y方向上的偏移）
        dis_real = 36.0/(avg_height+10);  //meters
        ang_real = acosf(avg_width/avg_height);// * (sub_height[0]-sub_height[1])/fabsf(sub_height[0]-sub_height[1]);    //leader-follower's angle
    }
		printf("dis_error: %f\tpos_error: %f\tang_error: %f\n", dis_real-dis_ref,pos_ref-pos_real,ang_real-ang_ref);
		//oData << dis_real-dis_ref << "\t" << pos_ref-pos_real << "\t" << avg_height << "\t" << avg_width << endl;
    //printf("dis: %f\tang: %f\tposition: %f\n", dis_real,ang_real,pos_real);
    Ellipse_count = 0;

    /// 绘出轮廓及其可倾斜的边界框和边界椭圆
    Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        // contour
        drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
        // ellipse
        // ellipse(drawing, minEllipse[i], color, 2, 8);
        // rotated rectangle
        // Point2f rect_points[4]; minRect[i].points(rect_points);
        // for (int j = 0; j < 4; j++)
        //     line(drawing, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8);
    }

    /// 结果在窗体中显示
    namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
}


void *Ellipse_thread(void* ha=NULL)
{
    while (!fstop)
    {
        cap >> mFrame;
        /// 转为灰度图并模糊化
        cvtColor(mFrame, src_gray, CV_BGR2GRAY);
        blur(src_gray, src_gray, Size(3, 3));

        /// 创建窗体
        namedWindow("Source", CV_WINDOW_AUTOSIZE);
        imshow("Source", mFrame);

        // createTrackbar(" Threshold:", "Source", &thresh, max_thresh, thresh_callback);
        thresh_callback(0, 0);

        if(waitKey(30) >=0)
            fstop = true;
    }
    return 0;
}
