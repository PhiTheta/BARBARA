#include <opencv/cv.h> // usr/include/opencv
#include <opencv/highgui.h>
#include<iostream>
#include <math.h>
#include <vector>
#include "camera_CV.h"

using namespace std;
using namespace cv;

int Red_found;
float center_x, center_y, extra_x, extra_y, rad;
IplImage* load;
IplImage* copy_load; 
IplImage* edges;
IplImage* src_gray;
IplImage* gaussian_blur;

void load_img_CV(void** buff, int width){
	
	cvSetData(load, *buff, 3*width);
	cvNamedWindow( "Input", CV_WINDOW_AUTOSIZE ); //Create a window “Input”
	cvShowImage( "Input", load ); //Display src in a window named “Input”

}
void load_img_CV_file (const char* filename){

        load = cvLoadImage( filename, 1 );
	cvNamedWindow( "Input", CV_WINDOW_AUTOSIZE ); //Create a window “Input”
	cvShowImage( "Input", load ); //Display src in a window named “Input”
}
void color_filter(){

	copy_load=cvCreateImage(cvGetSize(load),8,3); 
	CvScalar s,c;
	for(int i=0;i<(load->height);i++)// go thru the columns
	{
		for(int j=0;j<(load->width);j++)// go thru the rows 

		{
			s=cvGet2D(load,i,j);
			if((s.val[2]>120)&&(s.val[1]<80)&&(s.val[0]<80))
			{
				c.val[2]=255;//Set R to 0
				c.val[1]=255;//Set G to 255
				c.val[0]=255;//Set B to 0
				cvSet2D(copy_load,i,j,c); //Change the pixel value of copy img to pure green(G=255 R=0 B=0)
			}else //Set all other pixels in copy to white
			{
				c.val[2]=0; // Red
				c.val[1]=0;// Green
				c.val[0]=0;// Blue
				cvSet2D(copy_load,i,j,c); // Now set the scalar c(now white) to the pixel in i,j in copy
			}


		}
	}

	
	cvNamedWindow( "Output", CV_WINDOW_AUTOSIZE ); //Create a window “Output”
	cvShowImage( "Output", copy_load ); //Display copy in a window named “Output”
}

void edges_found(){
		edges =cvCreateImage(cvGetSize(copy_load),8,0); 
		src_gray =cvCreateImage(cvGetSize(copy_load),8,0); 
		cvCvtColor( copy_load, src_gray, CV_RGB2GRAY  );
		cvSmooth(src_gray, edges, CV_BLUR, 3, 3, 3, 3);
		cvCanny(edges, edges, 0, 0, 3);

		cvNamedWindow( "Edges", CV_WINDOW_AUTOSIZE ); //Create a window “Output”
		cvShowImage( "Edges", edges ); //Display copy in a window named “Output”

}

Camera_Target_Alarm circle_found(){

	Camera_Target_Alarm alarm_1;
	alarm_1.Red_Obstacle_Flag = false;
	gaussian_blur =cvCreateImage(cvGetSize(copy_load),8,0); 
	src_gray =cvCreateImage(cvGetSize(copy_load),8,0); 
	cvCvtColor( copy_load, src_gray, CV_RGB2GRAY  );
	cvSmooth(src_gray, gaussian_blur, CV_GAUSSIAN, 3, 3, 3, 3);
	vector<Vec3f> circles;
	 HoughCircles(gaussian_blur, circles, CV_HOUGH_GRADIENT,2, gaussian_blur->width/4, 200, 100 );

	if(circles.size() > 0){
		alarm_1.Red_Obstacle_Flag = true;
	}
	alarm_1.c_x=0;
	alarm_1.c_y=0;
	for( size_t i = 0; i < circles.size(); i++ )
    	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		rad = cvRound(circles[i][2]);
		alarm_1.c_x=0;
		alarm_1.c_y=0;
		alarm_1.c_x=  cvRound(circles[i][0]);
		alarm_1.c_y = cvRound(circles[i][1]);
		center_x = cvRound(circles[i][0]);
		center_y = cvRound(circles[i][1]);
		
		
		
		//cout<<"\nc_x1:%f"<<center_x;
		//cout<<"\nc_y1:%f"<<center_y; 
	}
	return alarm_1;
}

World_frame coordinate_transformation( Camera_Target_Alarm alarm){
	// K_matrix = [535 0 311; 0 528 231; 0 0 1]; 

	World_frame frame1;
	frame1.Alarm = false; 
	frame1.Alarm = alarm.Red_Obstacle_Flag;
	frame1.x_rob_frame = 0; 
	frame1.y_rob_frame = 0; 

	//float C_calib_x = (535*alarm.c_x) + (0* alarm.c_y) + 311; 
	//float C_calib_y = (0*alarm.c_x) + (528* alarm.c_y) + 231; 
	
	float C_calib_x = (0.0019*center_x) + (0* center_y) + (-0.5814); 
	float C_calib_y = (0*center_x) + ((0.0019)* center_y) + (-0.4375); 

	float X_world_calculation, Y_world_calculation; 
	
	X_world_calculation = 0.0032 + 0.6834 *( exp((-1*(C_calib_y))/0.6796));
	Y_world_calculation = 0.6068*C_calib_x*((fabs((-C_calib_x*(-C_calib_y+0.471)*0.2847)/0.5974)+0.5974)/0.5974);

	//cout<<"\nc_x:%f"<<alarm.c_x;
	//cout<<"\nc_y:%f"<<alarm.c_y; 

	//cout<<"\ncamera_x:%f"<<C_calib_x;
	//cout<<"\ncamera_y:%f"<<C_calib_y; 

	frame1.x_rob_frame = -1*Y_world_calculation;
	frame1.y_rob_frame = X_world_calculation;
	frame1.centerX = center_x;
	frame1.centerY = center_y;
	frame1.radius = rad;
	//cout<<"\nw_x:%f"<<X_world_calculation;
	//cout<<"\nw_y:%f"<<Y_world_calculation; 
	return frame1; 
}

World_frame array_Object_buff( void** buff, int width){
	
	load_img_CV(buff, width); 
	color_filter(); 
	edges_found();
	Camera_Target_Alarm alarm_actual = circle_found();
	World_frame frame_actual = coordinate_transformation(alarm_actual); 

	return frame_actual; 
}

World_frame Find_Object_file( const char* filename){
	
	load_img_CV_file(filename); 
	color_filter(); 
	edges_found();
	Camera_Target_Alarm alarm_actual = circle_found();
	World_frame frame_actual = coordinate_transformation(alarm_actual); 

	return frame_actual; 
}

