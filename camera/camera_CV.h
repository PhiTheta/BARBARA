#include <opencv/cv.h> // usr/include/opencv
#include <opencv/highgui.h>
#include<iostream>
#include <math.h>
#include <vector>


using namespace std;
using namespace cv;

struct Camera_Target_Alarm{

	bool Red_Obstacle_Flag;
	bool Blue_Target_Flag;
	float c_x;
	float c_y;

};

struct World_frame{

float x_rob_frame;
float y_rob_frame;
bool Alarm;
int centerX;
int centerY;
int radius;
};

World_frame array_Object( void* buff, int width);
World_frame Find_Object_file( const char* filename);
