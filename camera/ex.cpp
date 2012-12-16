/*
 *  test.cpp
 *  camera_CV
 *
 *  Created by Jaime Hernandez on 13/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */
#include <opencv/cv.h> // usr/include/opencv
#include <opencv/highgui.h>
#include<iostream>
#include <math.h>
#include <vector>

using namespace std;
using namespace cv;

int main(int, char**)
{
	IplImage* src = cvLoadImage( "Image.jpg", 1 );
	IplImage* copy=cvCreateImage(cvGetSize(src),8,3); 

	CvScalar s,c;

	// searching for red color 

	for(int i=0;i<(src->height);i++)// go thru the columns
	{
		for(int j=0;j<(src->width);j++)// go thru the rows 

		{
			s=cvGet2D(src,i,j);
			if((s.val[2]>120)&&(s.val[1]<80)&&(s.val[0]<80)
)
			{
				c.val[2]=255;//Set R to 0
				c.val[1]=255;//Set G to 255
				c.val[0]=255;//Set B to 0
				cvSet2D(copy,i,j,c); //Change the pixel value of copy img to pure green(G=255 R=0 B=0)

			}else //Set all other pixels in copy to white
			{
				c.val[2]=0; // Red
				c.val[1]=0;// Green
				c.val[0]=0;// Blue
				cvSet2D(copy,i,j,c); // Now set the scalar c(now white) to the pixel in i,j in copy
			}


		}
	}
	
	IplImage* edges =cvCreateImage(cvGetSize(src),8,0); 
	IplImage* src_gray =cvCreateImage(cvGetSize(src),8,0); 
	cvCvtColor( copy, src_gray, CV_RGB2GRAY  );
	cvSmooth(src_gray, edges, CV_BLUR, 3, 3, 3, 3);
	cvCanny(edges, edges, 0, 0, 3);
	IplImage* gaussian_blur =cvCreateImage(cvGetSize(src),8,0); 
	Mat img;
	cvSmooth(src_gray, gaussian_blur, CV_GAUSSIAN, 3, 3, 3, 3);
	vector<Vec3f> circles;
	 HoughCircles(gaussian_blur, circles, CV_HOUGH_GRADIENT,
                 2, gaussian_blur->width/4, 200, 100 );
	cout<<"Color found...\n";
	cvNamedWindow( "Input", CV_WINDOW_AUTOSIZE ); //Create a window “Input”
	cvShowImage( "Input", src ); //Display src in a window named “Input”
	cvNamedWindow( "Output", CV_WINDOW_AUTOSIZE ); //Create a window “Output”
	cvShowImage( "Output", copy ); //Display copy in a window named “Output”
	cvNamedWindow( "Edges", CV_WINDOW_AUTOSIZE ); //Create a window “Output”
	cvShowImage( "Edges", edges ); //Display copy in a window named “Output”
	for( size_t i = 0; i < circles.size(); i++ )
    	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		 int radius = cvRound(circles[i][2]);
		cout<<"center_x:%f\n"<<cvRound(circles[i][0]);
		cout<<"center_y:%f\n"<<cvRound(circles[i][1]);
		cout<<"radious:%f\n"<<cvRound(circles[i][2]);
		// draw the circle center
         	circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
         	// draw the circle outline
         	circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
	}
	cvNamedWindow( "circles", 1 );
    	//imshow( "circles", img );
	cvWaitKey(); //Wait till the user presses a key or cvWaitKey(10) will wait for 10ms
	cvReleaseImage( &src );

        return 0;
}
