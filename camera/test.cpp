#include <opencv/cv.h> // usr/include/opencv
#include <opencv/highgui.h>
#include<iostream>
#include <math.h>
#include <vector>
#include "camera_CV.h"

using namespace std;
using namespace cv;

int main(int, char**)
{
	World_frame my_frame;
	void* image = NULL;
	int width =0;
	World_frame my_frame2;
	my_frame2 = array_Object_buff(iamge,width);
	 my_frame = Find_Object_file("Image2.jpg");
	cout<<"\nmy_frame_x:%d"<<my_frame.x_rob_frame;
	cout<<"\nmy_frame_y:%d"<<my_frame.y_rob_frame;
	cvWaitKey(); //Wait till the user presses a key or cvWaitKey(10) will wait for 10ms
	return 0;
}
