#include "/usr/include/SDL/SDL.h"
#include "/usr/include/SDL/SDL_image.h"
#include <string>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include<math.h>
#include "/usr/include/SDL/SDL_gfxPrimitives.h"
#include <armadillo>
#include "camera.h"

using namespace std;
using namespace arma;

//Screen attributes
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 960;
const int SCREEN_BPP = 32;

//The surfaces
Uint32 RMask =0x00000000, BMask = 0x00000000, GMask = 0x00000000, AMask = 0, temp, n_temp, pixel_array, n_pixel, test_pixel, Rm, Bm, Gm, Alm,n_Rm, n_Bm, n_Gm, n_Alm,buffer;
SDL_PixelFormat *fmt;
SDL_Color *color;
Uint8 red, green, blue, alpha, n_red, n_green, n_blue, n_alpha, bit_pix, byt_pix,new_alpha,new_red,new_blue,new_green;
int prev_brightness, max_brightness, brightness,c_x, c_y, f_pixel_x, f_pixel_y, count_pixels,count_pixels_red, radius, area, color_flag,color_filter;
int edgeDir[SCREEN_WIDTH*2][SCREEN_HEIGHT*2]; // 623x474
float gradient[SCREEN_WIDTH*2][SCREEN_HEIGHT*2]; // 623x474
int newRow;
int newCol;
bool edgeEnd = false;
int Hough_accum[SCREEN_WIDTH][SCREEN_HEIGHT];
int radius_algorithm;
int center_x, center_y;
union convert {
						unsigned char image;
						int value;
};

void filter_pixel32(unsigned char *image, int width, int height, int x, int y, int Flag) 
{
 unsigned char image_read_1 = image[ ( y) + (x) ];
 unsigned char image_read_2 = image[ (y) + (x+1) ];
 unsigned char image_read_3 = image[ ( y) + (x+2) ];

		 		if (Flag == 1)
				{
					
					union convert Rd1; 
					union convert G1;
					union convert B1;
				
					Rd1.image = image_read_1; 
					G1.image = image_read_2;
					B1.image = image_read_3;
					
				//red = Rd1.value;

				//green = G1.value;

				//blue = B1.value;
				
				red = image_read_1;
				green = image_read_2;
				blue  = image_read_3;
				
				// RGB filtering
				// Red
			
					if ((red > 140)&&(((green-blue) <80) && (((red +blue)/2)<(red -70))))
					{
						if(count_pixels ==0){
							f_pixel_x = x;
							f_pixel_y = y;						
						}
						n_red = 255;//((red-blue) + (red-green))/2;
						n_blue = 0;
						n_green = 0;
						n_alpha = 0;

						
						count_pixels_red++;
						//int brightness = (0.02126*red)+(0.7152*green)+(0.0722*blue);
						brightness = (red+blue+green)/3;
						if(brightness > prev_brightness){
							max_brightness = brightness;
							prev_brightness =brightness;
							c_x = x; 
							c_y = y;
							
							count_pixels++;
							
						}
						
					}else{
						n_red = 0;
						n_blue = 0;
						n_green = 0;
						n_alpha = 0 ;
					}
			
				}

				if (Flag == 2){

				if (((((red+green)/2)<(blue-30))&&(blue>90)&&((green-red)<40))||((((red+green)/2)<(blue-60))&&(blue>120)&&((green-red)<80)))
				{
						if(count_pixels ==0){
							f_pixel_x = x;
							f_pixel_y = y;						
						}
						n_red = ((blue-green) + (blue-red));
						n_blue = 0;
						n_green = 0;
						n_alpha = 0;
					
						brightness = (red+blue+green)/3;
						if(brightness > prev_brightness){
						max_brightness = brightness;
						prev_brightness =brightness;
						c_x = x; 
						c_y = y;
						count_pixels++;
						
						}
				}else{
						n_red = 0;
						n_blue = 0;
						n_green = 0;
						n_alpha = 0 ;

				}
								}
						
}


// Calculate the distance to the center of the detected circle 

float Distance_Calculation(int X_center, int Y_center, int width, int height){

	float distance,distance_center; 
	float z_dis, pix_to_m_x, pix_to_m_y;
	float Xdiff, Ydiff;
	float image_height_m, image_width_m;
	int image_center_x, image_center_y;
	z_dis = 0.78;
	distance_center= tan(M_PI/4)*z_dis;
	
	image_height_m = 1.097;//0.297;
	image_width_m = 1.010;//0.210;
	
	image_center_x = (int)width/2;
	image_center_y = (int)height/2;
	
	pix_to_m_x= image_width_m/width;
	pix_to_m_y= image_height_m/height;

	if((X_center == 0)&&(Y_center == 0)){
		
		distance =  distance_center + image_height_m/2;
	}else if ((X_center == width) &&(Y_center == 0)){
				
		distance =  distance_center + image_height_m/2;
	}else if ((X_center == 0) &&(Y_center == height)){
				
		distance =  distance_center - image_height_m/2;
	}else if ((X_center == width) &&(Y_center == height)){
				
		distance =  distance_center - image_height_m/2;
	}else if (( X_center == image_center_x)&&(Y_center == image_center_y)){
		
		distance = distance_center;
	}else if (( X_center == 0)&&(Y_center == image_center_y)){
		
		distance = distance_center;
	}else if (( X_center == image_center_x)&&(Y_center == 0)){
		
		distance = distance_center;
	}
	
	else {
				
		Ydiff = ((float)image_center_y - (float)Y_center)*pix_to_m_y;
		
		Xdiff = (((float)image_center_x) - (float)X_center)*pix_to_m_x;
		
		distance = +sqrt((pow(Xdiff,2))+(pow(Ydiff,2))); //(distance_center-((float)image_center_y*pix_to_m_y))
	}
	

return distance;
}

// Calculate the angle to the center of the detected circle 

float Angle_Calculation(int X_center, int Y_center, int width, int height, int distance){
	float Y_diff, X_diff;
	float angle;
	float pix_to_m_x, pix_to_m_y;	
	float image_height_m, image_width_m;
	
	image_height_m = 1.097;
	image_width_m = 1.010;
	
	pix_to_m_x= image_width_m/width;
	pix_to_m_y= image_height_m/height;
			
				Y_diff = (height) - Y_center;
			
				X_diff = (width/2) - X_center;
			
			angle = atan2(Y_diff, X_diff);

return angle;
}

// Find the object of an specific form and color

Camera_Obstacle_Alarm Find_Object(unsigned char *image, int width, int height, int local_color_flag)
{
    
    Camera_Obstacle_Alarm alarm_1;
    alarm_1.Red_Target_Flag = false;
    alarm_1.Blue_Target_Flag = false;

    //Go through columns
    for( int y = 0; y < height; y++ )
    {
        //Go through rows
        for( int x = 0; x < width; x++)
        {
            //Get pixel
		filter_pixel32( image, width, height, x, y, local_color_flag );
		if(local_color_flag ==1){
			if(count_pixels >100) {//5){
				alarm_1.Red_Target_Flag = true;
			}
		}
	        if(local_color_flag== 2){
			if(count_pixels >= 10){
			alarm_1.Blue_Target_Flag = true;
			}
		}
            x = x+2;
        }
    }

	
	alarm_1.c_x = c_x;
	alarm_1.c_y = c_y;

    return alarm_1;
}

// Get Position of the detected object

Camera_Distance Postion_Object(int X_center, int Y_center, int width, int height){

		Camera_Distance Distance1; 
		Distance1.distance = fabs(Distance_Calculation(X_center, Y_center, width, height)); 
		Distance1.angle = Angle_Calculation(X_center, Y_center, width, height, Distance1.distance); 

		return Distance1;
}

