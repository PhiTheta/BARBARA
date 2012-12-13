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
SDL_Surface *filtered = NULL;
SDL_Surface *smoothed = NULL;
SDL_Surface *bright = NULL;
Uint32 RMask =0x00000000, BMask = 0x00000000, GMask = 0x00000000, AMask = 0, temp, n_temp, pixel_array, n_pixel, test_pixel, Rm, Bm, Gm, Alm,n_Rm, n_Bm, n_Gm, n_Alm,buffer;
SDL_PixelFormat *fmt;
SDL_Color *color;
Uint8 red, green, blue, alpha, n_red, n_green, n_blue, n_alpha, bit_pix, byt_pix,new_alpha,new_red,new_blue,new_green;
int prev_brightness, max_brightness, brightness,c_x, c_y, f_pixel_x, f_pixel_y, count_pixels, radius, area, color_flag,color_filter;
int edgeDir[SCREEN_WIDTH*2][SCREEN_HEIGHT*2]; // 623x474
float gradient[SCREEN_WIDTH*2][SCREEN_HEIGHT*2]; // 623x474
int newRow;
int newCol;
bool edgeEnd = false;
int Hough_accum[SCREEN_WIDTH][SCREEN_HEIGHT];
SDL_Surface *detected = NULL;
int radius_algorithm;
int center_x, center_y;

Uint32 get_pixel32( SDL_Surface *surface, int x, int y )
{
    //Convert the pixels to 32 bitcenter_x = x; 
	center_y = y-radius;
    Uint32 *pixels = (Uint32 *)surface->pixels;

    //Get the requested pixel
    return pixels[ ( y * surface->w ) + x ];
}
Uint32 get_pixel32_1( SDL_Surface *surface, int x, int y )
{
    //Convert the pixels to 32 bit
    Uint32 *pixels = (Uint32 *)surface->pixels;

    //Get the requested pixel
    return pixels[ (( y * surface->w ) + x) +1];
}
Uint32 get_pixel32_2( SDL_Surface *surface, int x, int y )
{
    //Convert the pixels to 32 bit
    Uint32 *pixels = (Uint32 *)surface->pixels;

    //Get the requested pixel
    return pixels[ (( y * surface->w ) + x) +2];
}
Uint32 filter_pixel32(SDL_Surface *surface, int x, int y, int Flag) 
{
Uint32 requested_pixel = get_pixel32( surface, x, y);

		 		if (Flag == 1)
				{
 
				temp = requested_pixel & surface->format->Rmask;  
				temp = temp >> surface->format->Rshift; 
				temp = temp << surface->format->Rloss;  
				red = (Uint8)temp;

				
				temp = requested_pixel & surface->format->Gmask;  
				temp = temp >> surface->format->Gshift; 
				temp = temp << surface->format->Gloss; 
				green = (Uint8)temp;

			 
				 temp = requested_pixel & surface->format->Bmask; 
				 temp = temp >> surface->format->Bshift; 
				 temp = temp << surface->format->Bloss;  
				 blue = (Uint8)temp;

				temp = requested_pixel & surface->format->Amask;  
				temp = temp >> surface->format->Ashift; 
				temp = temp << surface->format->Aloss;  
				alpha = (Uint8)temp;
				
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
			
				//Red buffer
				
				buffer = buffer^n_red;
				buffer = buffer << surface->format->Rshift;
				buffer = buffer >>surface->format->Rloss;
				}

				if (Flag == 2){

				if (((((red+green)/2)<(blue-30))&&(blue>90)&&((green-red)<40))||((((red+green)/2)<(blue-60))&&(blue>120)&&((green-red)<80)))
				{
						n_red = ((blue-green) + (blue-red));
						n_blue = 0;
						n_green = 0;
						n_alpha = 0;
				}else{
						n_red = 0;
						n_blue = 0;
						n_green = 0;
						n_alpha = 0 ;

				}
				// Blue Buffer

				buffer = n_red;
				}
				
return buffer;
						
}

// putting pixels back on the image

	void put_pixel32( SDL_Surface *surface, int x, int y, Uint32 pixel )
	{
	    //Convert the pixels to 32 bit
	    Uint32 *pixels = (Uint32 *)surface->pixels;

	    //Set the pixel
	  // cout << "x " << x << ", y " << y << " w, " << surface->w << " h, " << surface->h << " pointer " << surface << endl;
	    pixels[ ( y * surface->w ) + x ] = pixel;
	}
	void put_pixel32_1( SDL_Surface *surface, int x, int y, Uint32 pixel )
	{
	    //Convert the pixels to 32 bit
	    Uint32 *pixels = (Uint32 *)surface->pixels;

	    //Set the pixel
	    pixels[ (( y * surface->w ) + x )+1] = pixel;
	}
	void put_pixel32_2( SDL_Surface *surface, int x, int y, Uint32 pixel )
	{
	    //Convert the pixels to 32 bit
	    Uint32 *pixels = (Uint32 *)surface->pixels;

	    //Set the pixel
	    pixels[ (( y * surface->w ) + x )+2] = pixel;
	}

// Find the Edges on the image 

void findEdge(int rowShift, int colShift, int row, int col, int dir, int lowerThreshold, SDL_Surface* surface){
	int W = (surface->w);
	int H = (surface->h);
	Uint32 i;
//printf("Edge dir: %d, dir: %d, row: %d, col: %d\n",edgeDir[row][col], dir, row, col);
SDL_LockSurface( surface );
	/* Find the row and column values for the next possible pixel on the edge */
	if (colShift < 0) {
		if (col > 0)
			newCol = col + colShift;
		else
			edgeEnd = true;
	} else if (col < W - 1) {
		newCol = col + colShift;
	} else
		edgeEnd = true;		// If the next pixel would be off image, don't do the while loop
	if (rowShift < 0) {
		if (row > 0)
			newRow = row + rowShift;
		else
			edgeEnd = true;
	} else if (row < H - 1) {
		newRow = row + rowShift;
	} else
		edgeEnd = true;	
		
	/* Determine edge directions and gradient strengths */
	while ( (edgeDir[newRow][newCol]==dir) && !edgeEnd && (gradient[newRow][newCol] > lowerThreshold) ) {
		/* Set the new pixel as white to show it is an edge */
		i = (Uint32)(newRow*3*W + 3*newCol);
		Uint32 p_pix_1 = SDL_MapRGB(surface->format, 255,255,255);
		put_pixel32( surface, newCol, newRow, p_pix_1);
		put_pixel32_1( surface, newCol, newRow, p_pix_1);
		put_pixel32_2( surface, newCol, newRow, p_pix_1);
		//printf("pixel: %u\n",p_pix_1);
		if (colShift < 0) {
			if (newCol > 0)
				newCol = newCol + colShift;
			else
				edgeEnd = true;	
		} else if (newCol < W - 1) {
			newCol = newCol + colShift;
		} else
			edgeEnd = true;	
		if (rowShift < 0) {
			if (newRow > 0)
				newRow = newRow + rowShift;
			else
				edgeEnd = true;
		} else if (newRow < H - 1) {
			newRow = newRow + rowShift;
		} else
			edgeEnd = true;	
	}	
SDL_UnlockSurface( surface );
}

// Non supress Non Max gradient 

void suppressNonMax(int rowShift, int colShift, int row, int col, int dir, int lowerThreshold, SDL_Surface *surface){
int W = surface->w;
int H = surface->h;
 newRow = 0;
 newCol = 0;
unsigned long i;
 edgeEnd = false;
float nonMax[SCREEN_WIDTH][3];		// Temporarily stores gradients and positions of pixels in parallel edges
int pixelCount = 0;		// Stores the number of pixels in parallel edges
int count;				
int max[3];			// Maximum point in a wide edge
Uint32 Edgepixel;
if (colShift < 0) {
		if (col > 0)
			newCol = col + colShift;
		else
			edgeEnd = true;
	} else if (col < W - 1) {
		newCol = col + colShift;
	} else
		edgeEnd = true;		// If the next pixel would be off image, don't do the while loop
	if (rowShift < 0) {
		if (row > 0)
			newRow = row + rowShift;
		else
			edgeEnd = true;
	} else if (row < H - 1) {
		newRow = row + rowShift;
	} else
		edgeEnd = true;	
	i = (Uint32)(newRow*3*W + 3*newCol);
	/* Find non-maximum parallel edges tracing up */
	Edgepixel = get_pixel32(surface, newCol, newRow);
	while ((edgeDir[newRow][newCol] == dir) && !edgeEnd && (Edgepixel == SDL_MapRGB(surface->format, 255,255,255))) {
		if (colShift < 0) {
			if (newCol > 0)
				newCol = newCol + colShift;
			else
				edgeEnd = true;	
		} else if (newCol < W - 1) {
			newCol = newCol + colShift;
		} else
			edgeEnd = true;	
		if (rowShift < 0) {
			if (newRow > 0)
				newRow = newRow + rowShift;
			else
				edgeEnd = true;
		} else if (newRow < H - 1) {
			newRow = newRow + rowShift;
		} else
			edgeEnd = true;	
		nonMax[pixelCount][0] = newRow;
		nonMax[pixelCount][1] = newCol;
		nonMax[pixelCount][2] = gradient[newRow][newCol];
		pixelCount++;
		i = (Uint32)(newRow*3*W + 3*newCol);
	}

	/* Find non-maximum parallel edges tracing down */
	edgeEnd = false;
	colShift *= -1;
	rowShift *= -1;
	if (colShift < 0) {
		if (col > 0)
			newCol = col + colShift;
		else
			edgeEnd = true;
	} else if (col < W - 1) {
		newCol = col + colShift;
	} else
		edgeEnd = true;	
	if (rowShift < 0) {
		if (row > 0)
			newRow = row + rowShift;
		else
			edgeEnd = true;
	} else if (row < H - 1) {
		newRow = row + rowShift;
	} else
		edgeEnd = true;	
	i = (Uint32)(newRow*3*W + 3*newCol);
	Edgepixel = get_pixel32(surface, newCol, newRow);
	while ((edgeDir[newRow][newCol] == dir) && !edgeEnd && (Edgepixel == SDL_MapRGB(surface->format, 255,255,255))) {
		if (colShift < 0) {
			if (newCol > 0)
				newCol = newCol + colShift;
			else
				edgeEnd = true;	
		} else if (newCol < W - 1) {
			newCol = newCol + colShift;
		} else
			edgeEnd = true;	
		if (rowShift < 0) {
			if (newRow > 0)
				newRow = newRow + rowShift;
			else
				edgeEnd = true;
		} else if (newRow < H - 1) {
			newRow = newRow + rowShift;
		} else
			edgeEnd = true;	
		nonMax[pixelCount][0] = newRow;
		nonMax[pixelCount][1] = newCol;
		nonMax[pixelCount][2] = gradient[newRow][newCol];
		pixelCount++;
		i = (Uint32)(newRow*3*W + 3*newCol);
	}

	/* Suppress non-maximum edges */
	max[0] = 0;
	max[1] = 0;
	max[2] = 0;
	for (count = 0; count < pixelCount; count++) {
		if (nonMax[count][2] > max[2]) {
			max[0] = nonMax[count][0];
			max[1] = nonMax[count][1];
			max[2] = nonMax[count][2];
		}
	}
	for (count = 0; count < pixelCount; count++) {
		i = (Uint32)(nonMax[count][0]*3*W + 3*nonMax[count][1]);
		Uint32 p_pix_1 = SDL_MapRGB(surface->format, 255,255,255);;
		put_pixel32( surface, nonMax[count][0], nonMax[count][1], p_pix_1);
		put_pixel32_1( surface, nonMax[count][0], nonMax[count][1], p_pix_1);
		put_pixel32_2( surface, nonMax[count][0], nonMax[count][1], p_pix_1);
	}

}

// Acummulate pixel 

bool accum_pixel(SDL_Surface *surface, int x , int y){
// Checking image bounds
	if( x <0 || x >= surface->w || y < 0 || y >= surface-> h)
	{
		return false;
	}
		return true;
}

// Acumulate circles 

void accum_circle(SDL_Surface *surface, int x, int y, int localRadius)
{
	int f=1-localRadius;
	int ddF_x = 1; 
	int ddF_y = -2*localRadius;
	int x1 = 0;
	int y1 = localRadius;
	
	
if (accum_pixel ( surface, x, y+ localRadius))
{ 
	put_pixel32(surface, x, y+localRadius, SDL_MapRGB(surface->format, 255,255,255));
	
}
if (accum_pixel ( surface, x, y - localRadius))
{ 
	put_pixel32(surface, x, y - localRadius, SDL_MapRGB(surface->format, 255,255,255));
	
}
if (accum_pixel ( surface, x - localRadius, y))
{ 
	put_pixel32(surface, x + localRadius , y, SDL_MapRGB(surface->format, 255,255,255));
	
}
if (accum_pixel ( surface, x + localRadius, y))
{ 
	put_pixel32(surface, x - localRadius, y, SDL_MapRGB(surface->format, 255,255,255));
	
}

while (x1<y1)
{
	if(x1 < y1)
	{
		y1--;
		ddF_y += 2;
		f += ddF_y;
	}

	x1++;
	ddF_x += 2;
	f += ddF_x;

	if (accum_pixel ( surface, x + x1, y+ y1))
	{ 
		put_pixel32(surface, x + x1, y+ y1, SDL_MapRGB(surface->format, 255,255,255));
	}

	if (accum_pixel ( surface, x - x1, y+ y1))
	{ 
		put_pixel32(surface, x - x1, y+ y1, SDL_MapRGB(surface->format, 255,255,255));
	}

	if (accum_pixel ( surface, x + x1, y- y1))
	{ 
		put_pixel32(surface, x + x1, y- y1, SDL_MapRGB(surface->format, 255,255,255));
	}
	if (accum_pixel ( surface, x - x1, y- y1))
	{ 
		put_pixel32(surface, x - x1, y- y1, SDL_MapRGB(surface->format, 255,255,255));
	}
}

	if(center_x >= x1){
		center_x = x1;
	}
	
	if(center_y >= y1){
		center_y = y1;
	}
	
}

// find the circle 

void Circle_Find(SDL_Surface *Edges, int min_rad, int max_rad, SDL_Surface *ColorSurface){
Uint32 pixel_edge_test;
if (min_rad == 0)
{
	min_rad = 5;
}
if(max_rad == 0)
{
	max_rad= min(Edges->w, Edges->h)/2;	
}
SDL_Surface*surfaces[max_rad-min_rad];
//std::vector<SDL_Surface*> hough_array (max_rad , min_rad);
SDL_Surface *hough = NULL;

// Create Hough array for detecting the circles 
for(int i = min_rad; i< max_rad; i++)
{
	hough = SDL_CreateRGBSurface( SDL_SWSURFACE, Edges->w, Edges->h, Edges->format->BitsPerPixel, RMask, GMask, BMask, 0 );
	surfaces[i - min_rad] = hough;


// find edges from image


	for(int x = 0; x <Edges->w; x++)
	{
		for(int y =0; y <Edges->h; y++)
		{
			pixel_edge_test = get_pixel32(Edges, x , y );
			if(pixel_edge_test == SDL_MapRGB(Edges->format, 255,255,255))
			{
				accum_circle(surfaces[i - min_rad], x , y, i);
			}
		}
	
	}




detected = SDL_CreateRGBSurface( SDL_SWSURFACE, Edges->w, Edges->h, Edges->format->BitsPerPixel, RMask, GMask, BMask, 0 );
SDL_LockSurface(detected );
unsigned int threshold = 100;

				
			for(int x = 0; x < Edges->w; x++)
   			 {
      				for(int y = 0; y < Edges->h; y++)
      					{
					
					temp = get_pixel32(ColorSurface,x,y) & ColorSurface->format->Rmask;  
					temp = temp >> ColorSurface->format->Rshift; 
					temp = temp << ColorSurface->format->Rloss;  
					red = (Uint8)temp;

				
					temp = get_pixel32(ColorSurface,x,y) & ColorSurface->format->Gmask;  
					temp = temp >> ColorSurface->format->Gshift; 
					temp = temp << ColorSurface->format->Gloss; 
					green = (Uint8)temp;

			 
				 	temp = get_pixel32(ColorSurface,x,y) & ColorSurface->format->Bmask; 
				 	temp = temp >> ColorSurface->format->Bshift; 
				 	temp = temp << ColorSurface->format->Bloss;  
				 	blue = (Uint8)temp;

					temp = get_pixel32(ColorSurface,x,y) & ColorSurface->format->Amask;  
					temp = temp >> ColorSurface->format->Ashift; 
					temp = temp << ColorSurface->format->Aloss;  
					alpha = (Uint8)temp;
					unsigned int Brightness = (red+green+blue)/3;
        				if(Brightness > threshold)
        				{
         					put_pixel32(detected, x, y , get_pixel32(surfaces[i - min_rad], x , y ));
        				}

					/*
					temp = get_pixel32(surfaces[i - min_rad],x,y) & surfaces[i - min_rad]->format->Rmask;  
					temp = temp >> surfaces[i - min_rad]->format->Rshift; 
					temp = temp << surfaces[i - min_rad]->format->Rloss;  
					red = (Uint8)temp;

				
					temp = get_pixel32(surfaces[i - min_rad],x,y) & surfaces[i - min_rad]->format->Gmask;  
					temp = temp >> surfaces[i - min_rad]->format->Gshift; 
					temp = temp << surfaces[i - min_rad]->format->Gloss; 
					green = (Uint8)temp;

			 
				 	temp = get_pixel32(surfaces[i - min_rad],x,y) & surfaces[i - min_rad]->format->Bmask; 
				 	temp = temp >> surfaces[i - min_rad]->format->Bshift; 
				 	temp = temp << surfaces[i - min_rad]->format->Bloss;  
				 	blue = (Uint8)temp;

					temp = get_pixel32(surfaces[i - min_rad],x,y) & surfaces[i - min_rad]->format->Amask;  
					temp = temp >> surfaces[i - min_rad]->format->Ashift; 
					temp = temp << surfaces[i - min_rad]->format->Aloss;  
					alpha = (Uint8)temp;
					int Brightness = (red+green+blue)/3;*/
        				if(Brightness >= threshold)
        				{
         					put_pixel32(detected, x, y , get_pixel32(surfaces[i - min_rad], x , y ));
        				}
      				}
    			}
}
			
			//circleRGBA(detected, center_x, center_y, min_rad, 255, 255, 255, 255);
			
SDL_UnlockSurface( detected );



}

// Smoothed the surface and find the edges on the image 

SDL_Surface *Smoothed_image(SDL_Surface *surface){
SDL_Surface *Smoothed = NULL;// Smoothed image
unsigned int W,H; // width and height of the image
unsigned int row, col; // pixel row and columns positions
Uint32 i; // Dummy variable for row-column vector
int upperThreshold =100; // Gradient necessary to start a edge
int lowerThreshold =10; //Minimum gradient strengh to continue the edge
Uint32 iOffset; // Variable to offset row-column vector during sobel mask
int rowOffset; // row offset of the current pixel
int colOffset; //Col offset from the current pixel
int rowTotal = 0;		// Row position of offset pixel
int colTotal = 0;		// Col position of offset pixel
int Gx;				// Sum of Sobel mask products values in the x direction
int Gy;				// Sum of Sobel mask products values in the y direction
float thisAngle;		// Gradient direction based on Gx and Gy
int newAngle;			// Approximation of the gradient direction
int GxMask[3][3];		// Sobel mask in the x direction
int GyMask[3][3];		// Sobel mask in the y direction
Uint32 newPixel;		// Sum pixel values for gaussian
Uint32 oldPixel;		// old pixel
Uint32 blurPixel;		// blur pixel
Uint32 changePixel;		// change pixel
Uint32 changePixel1;		// change pixel
Uint32 changePixel2;		// change pixel
Uint32 changePixel3;		// change pixel
int gaussianMask[5][5];		// Gaussian mask

W = surface-> w;
H = surface-> h;

	for (row = 0; row < H*2; row++) {
		for (col = 0; col < W*2; col++) {
			edgeDir[row][col] = 0;
		}
	}

/* Declare Sobel masks */
	GxMask[0][0] = -1; GxMask[0][1] = 0; GxMask[0][2] = 1;
	GxMask[1][0] = -2; GxMask[1][1] = 0; GxMask[1][2] = 2;
	GxMask[2][0] = -1; GxMask[2][1] = 0; GxMask[2][2] = 1;

	GyMask[0][0] =  1; GyMask[0][1] =  2; GyMask[0][2] =  1;
	GyMask[1][0] =  0; GyMask[1][1] =  0; GyMask[1][2] =  0;
	GyMask[2][0] = -1; GyMask[2][1] = -2; GyMask[2][2] = -1;

/* Declare Gaussian mask */
	gaussianMask[0][0] = 1;	 gaussianMask[0][1] = 4;  gaussianMask[0][2] = 7;  gaussianMask[0][3] = 4;  gaussianMask[0][4] = 1;	
	gaussianMask[1][0] = 4;	 gaussianMask[1][1] = 16;  gaussianMask[1][2] = 26; gaussianMask[1][3] = 16;  gaussianMask[1][4] = 4;	
	gaussianMask[2][0] = 7;	 gaussianMask[2][1] = 26; gaussianMask[2][2] = 41; gaussianMask[2][3] = 26; gaussianMask[2][4] = 7;	
	gaussianMask[3][0] = 4;	 gaussianMask[3][1] = 16;  gaussianMask[3][2] = 26; gaussianMask[3][3] = 16;  gaussianMask[3][4] = 4;	
	gaussianMask[4][0] = 1;	 gaussianMask[4][1] = 4;  gaussianMask[4][2] = 7;  gaussianMask[4][3] = 4;  gaussianMask[4][4] = 1;	




Smoothed = SDL_CreateRGBSurface( SDL_SWSURFACE, surface->w, surface->h, surface->format->BitsPerPixel, RMask, GMask, BMask, 0 );
cout<<"lock smoothed"<<endl;
SDL_LockSurface( surface );

/* Gaussian Blur */
	for (row = 2; row < H-2; row++) {
		for (col = 2; col < W-2; col++) {
			newPixel = 0;
			for (rowOffset=-2; rowOffset<=2; rowOffset++) {
				for (colOffset=-2; colOffset<=2; colOffset++) {
					rowTotal = row + rowOffset;
					colTotal = col + colOffset;
					iOffset = (Uint32)(rowTotal*3*W + colTotal*3);
					oldPixel = get_pixel32(surface, colTotal, rowTotal);
					newPixel += ((Uint32) oldPixel * (Uint32)gaussianMask[rowOffset+2][colOffset+2]);
				}
			}
			i = (Uint32)(row*3*W + col*3);
			put_pixel32( Smoothed, col, row,(newPixel)/(256));//(16777216)
		}
	}
 
/* Determine edge directions and gradient strengths */
	for (row = 1; row < H-1; row++) {
		for (col = 1; col < W-1; col++) {
			i = (Uint32)(row*3*W + col*3);
			Gx = 0;
			Gy = 0;
			/* Calculate the sum of the Sobel mask times the nine surrounding pixels in the x and y direction */
			for (rowOffset=-1; rowOffset<=1; rowOffset++) {
				for (colOffset=-1; colOffset<=1; colOffset++) {
					rowTotal = row + rowOffset;
					colTotal = col + colOffset;
					iOffset = (Uint32)(rowTotal*3*W + colTotal*3);
					blurPixel = get_pixel32(Smoothed, colTotal, rowTotal);
					Gx = Gx + (blurPixel * GxMask[rowOffset+1][colOffset+1]);
					Gy = Gy + (blurPixel * GyMask[rowOffset+1][colOffset+1]);
					//printf("Gx: %d, Gy: %d\n",Gx, Gy);
				}
			}

			gradient[row][col] = sqrt(pow(Gx,2.0) + pow(Gy,2.0));	// Calculate gradient strength			
			thisAngle = (atan2(Gx,Gy)/3.14159) * 180.0;		// Calculate actual direction of edge
			//printf("gradient: %f, thisAngle: %f, col: %d, row: %d\n",gradient[row][col], thisAngle, row, col);

			/* Convert actual edge direction to approximate value */
			if ( ( (thisAngle < 22.5) && (thisAngle > -22.5) ) || (thisAngle > 157.5) || (thisAngle < -157.5) )
				newAngle = 0;
			if ( ( (thisAngle > 22.5) && (thisAngle < 67.5) ) || ( (thisAngle < -112.5) && (thisAngle > -157.5) ) )
				newAngle = 45;
			if ( ( (thisAngle > 67.5) && (thisAngle < 112.5) ) || ( (thisAngle < -67.5) && (thisAngle > -112.5) ) )
				newAngle = 90;
			if ( ( (thisAngle > 112.5) && (thisAngle < 157.5) ) || ( (thisAngle < -22.5) && (thisAngle > -67.5) ) )
				newAngle = 135;
				
			edgeDir[row][col] = newAngle;		// Store the approximate edge direction of each pixel in one array
		}
	}

/* Trace along all the edges in the image */
	for (row = 1; row < H - 1; row++) {
		for (col = 1; col < W - 1; col++) {
			edgeEnd = false;
			if (gradient[row][col] > upperThreshold) {	
				/* Switch based on current pixel's edge direction */
				switch (abs(edgeDir[row][col])){		
					case 0:
						findEdge(0, 1, row, col, 0, lowerThreshold, Smoothed);
						break;
					case 45:
						findEdge(1, 1, row, col, 45, lowerThreshold, Smoothed);
						break;
					case 90:
						findEdge(1, 0, row, col, 90, lowerThreshold, Smoothed);
						break;
					case 135:
						findEdge(1, -1, row, col, 135, lowerThreshold, Smoothed);
						break;
					default :
						i = (Uint32)(row*3*W + 3*col);
						Uint32 p_pix =SDL_MapRGB(surface->format, 0,0,0);
						put_pixel32( Smoothed, col, row, p_pix);
				    		put_pixel32_1( Smoothed, col, row, p_pix);
						put_pixel32_2( Smoothed, col, row, p_pix);
						break;
					}
				}
			else {
				i = (Uint32)(row*3*W + 3*col);
					Uint32 p_pix_2 = SDL_MapRGB(surface->format, 0,0,0);
						put_pixel32( Smoothed, col, row, p_pix_2);
				    		put_pixel32_1( Smoothed, col, row, p_pix_2);
						put_pixel32_2( Smoothed, col, row, p_pix_2);
			}	
		}
	}

/* Suppress any pixels not changed by the edge tracing */

	for (row = 0; row < H; row++) {
		for (col = 0; col < W; col++) {	
			// Recall each pixel is composed of 3 bytes
			i = (Uint32)(row*3*W + 3*col);
			changePixel = get_pixel32(Smoothed, 3*col, 3*row);
			changePixel1 = get_pixel32_1(Smoothed, 3*col, 3*row);
			changePixel2 = get_pixel32_2(Smoothed, 3*col, 3*row);
			//printf("change_pixel 1: %u, 2: %u, 3: %u\n", changePixel, changePixel1, changePixel2);
			// If a pixel's grayValue is not black or white make it black
			if( (((changePixel) != SDL_MapRGB(surface->format, 255,255,255)) && ((changePixel) != SDL_MapRGB(surface->format, 0,0,0))) 
			 || (((changePixel1) != SDL_MapRGB(surface->format, 255,255,255)) && ((changePixel1)!= SDL_MapRGB(surface->format, 0,0,0))) 
			 || (((changePixel2) != SDL_MapRGB(surface->format, 255,255,255)) && ((changePixel2) != SDL_MapRGB(surface->format, 0,0,0))) ) {
				put_pixel32(Smoothed, col, row,SDL_MapRGB(surface->format, 0,0,0));  
				put_pixel32_1(Smoothed, col, row,SDL_MapRGB(surface->format, 0,0,0));  
				put_pixel32_2(Smoothed, col, row,SDL_MapRGB(surface->format, 0,0,0));  
			}
		}
	}


	/* Non-maximum Suppression */
	for (row = 1; row < H - 1; row++) {
		for (col = 1; col < W - 1; col++) {
			i = (Uint32)(row*3*W + 3*col);
			changePixel3 = get_pixel32(Smoothed, row, col);
			if (changePixel3 == 255) {		// Check to see if current pixel is an edge
				/* Switch based on current pixel's edge direction */
				switch (edgeDir[row][col]) {		
					case 0:
						suppressNonMax( 1, 0, row, col, 0, lowerThreshold, Smoothed);
						break;
					case 45:
						suppressNonMax( 1, -1, row, col, 45, lowerThreshold, Smoothed);
						break;
					case 90:
						suppressNonMax( 0, 1, row, col, 90, lowerThreshold, Smoothed);
						break;
					case 135:
						suppressNonMax( 1, 1, row, col, 135, lowerThreshold, Smoothed);
						break;
					default :
						break;
				}
			}	
		}
	}
	cout <<"unlock smoothed"<<endl;
SDL_UnlockSurface( surface );

 
return Smoothed;
}

// Calculate the distance to the center of the detected circle 

float Distance_Calculation(int X_center, int Y_center){
float center_M [3];

float camera[3][4]; 

center_M[0] = X_center; center_M[1] = Y_center; center_M[2] = 1; 

camera[0][0] = 1.454e-15; camera[0][1] = -3.1233e-17; camera[0][2] = -3.2949e-19; camera[0][3] = 0.7071;
camera[1][0] = 2.7704e-18; camera[1][1] = 3.2835e-16; camera[1][2] = -3.2654e-17; camera[1][3] = -1.0181e-19;
camera[2][0] = 1.1490e-19; camera[2][1] = 1.1469e-15; camera[2][2] = -0.7071; camera[2][3] = 7.3943e-18; 


float Camera_center [3]; 

Camera_center[0] = 4.8619; Camera_center[1] = 4.1021; Camera_center[2] = -0.0066; 

float c1, c2, c3, c4;

c1 = (camera[0][0]*center_M[0])+(camera[1][0]*center_M[1])+(camera[2][0]*center_M[2]);
c2 = (camera[0][1]*center_M[0])+(camera[1][1]*center_M[1])+(camera[2][1]*center_M[2]);
c3 = (camera[0][2]*center_M[0])+(camera[1][2]*center_M[1])+(camera[2][2]*center_M[2]);
c4 = (camera[0][3]*center_M[0])+(camera[1][3]*center_M[1])+(camera[2][3]*center_M[2]);

float distance; 

distance = c3 - Camera_center[2]; 

return distance;
}

// Calculate the angle to the center of the detected circle 

float Angle_Calculation(int X_center, int Y_center){
float center_M [3];

float camera[3][4]; 

center_M[0] = X_center; center_M[1] = Y_center; center_M[2] = 1; 

camera[0][0] = 1.454e-15; camera[0][1] = -3.1233e-17; camera[0][2] = -3.2949e-19; camera[0][3] = 0.7071;
camera[1][0] = 2.7704e-18; camera[1][1] = 3.2835e-16; camera[1][2] = -3.2654e-17; camera[1][3] = -1.0181e-19;
camera[2][0] = 1.1490e-19; camera[2][1] = 1.1469e-15; camera[2][2] = -0.7071; camera[2][3] = 7.3943e-18; 

float Camera_center [3]; 

Camera_center[0] = 4.8619; Camera_center[1] = 4.1021; Camera_center[2] = -0.0066; 

float c1, c2, c3, c4;

c1 = (camera[0][0]*center_M[0])+(camera[1][0]*center_M[1])+(camera[2][0]*center_M[2]);
c2 = (camera[0][1]*center_M[0])+(camera[1][1]*center_M[1])+(camera[2][1]*center_M[2]);
c3 = (camera[0][2]*center_M[0])+(camera[1][2]*center_M[1])+(camera[2][2]*center_M[2]);
c4 = (camera[0][3]*center_M[0])+(camera[1][3]*center_M[1])+(camera[2][3]*center_M[2]);

float angle; 

angle = tan(c3/Camera_center[2]); 

return angle;
}

// Find the object of an specific form and color

Camera_Obstacle_Alarm Find_Object(SDL_Surface *surface, int local_color_flag)
{

    Camera_Obstacle_Alarm alarm_1;
    //If the image is color keyed
    if( surface->flags & SDL_SRCCOLORKEY )
    {
        filtered = SDL_CreateRGBSurface( SDL_SWSURFACE, surface->w, surface->h, surface->format->BitsPerPixel, RMask, GMask, BMask, 0 );
    }
    //Otherwise
    else
    {
         filtered = SDL_CreateRGBSurface( SDL_SWSURFACE, surface->w, surface->h, surface->format->BitsPerPixel, RMask, GMask, BMask, AMask );
    }
 
    if( surface->flags & SDL_SRCCOLORKEY )
    {
        smoothed = SDL_CreateRGBSurface( SDL_SWSURFACE, surface->w, surface->h, surface->format->BitsPerPixel, RMask, GMask, BMask, 0 );
    }
    //Otherwise
    else
    {
         smoothed = SDL_CreateRGBSurface( SDL_SWSURFACE, surface->w, surface->h, surface->format->BitsPerPixel, RMask, GMask, BMask, AMask );
    }

    if( surface->flags & SDL_SRCCOLORKEY )
    {
        bright = SDL_CreateRGBSurface( SDL_SWSURFACE, surface->w, surface->h, surface->format->BitsPerPixel, RMask, GMask, BMask, 0 );
    }
    //Otherwise
    else
    {
         bright = SDL_CreateRGBSurface( SDL_SWSURFACE, surface->w, surface->h, surface->format->BitsPerPixel, RMask, GMask, BMask, AMask );
    }
    //If the surface must be locked
    if( SDL_MUSTLOCK( surface ) )
    {
        //Lock the surface
        cout << "lock"<< endl;
        SDL_LockSurface( surface );
    }

    //Go through columns
    for( int x = 0, rx = filtered->w - 1; x < filtered->w; x++, rx-- )
    {
        //Go through rows
        for( int y = 0, ry = filtered->h - 1; y < filtered->h; y++, ry-- )
        {
            //Get pixel
		Uint32 pixel = filter_pixel32( surface, x, y, local_color_flag );
		if(local_color_flag ==1){
			if(pixel == 16711680){
				area++;
				alarm_1.Red_Obstacle_Flag = true;
			}
		}
	        if(local_color_flag== 2){
			if(pixel == 255){
			area++;
			alarm_1.Blue_Target_Flag = true;
			}
		}
            
		put_pixel32(filtered, x ,y, pixel);
        }
    }
	smoothed = Smoothed_image(surface);
	Circle_Find(smoothed, 0,0,surface);
	bright = detected;
	alarm_1.c_x = c_x;
	alarm_1.c_x = c_y;
	
	
    //Unlock surface
    if( SDL_MUSTLOCK( surface ) )
    {
		cout << "unlock"<< endl;
        SDL_UnlockSurface( surface );
    }

   
    //Copy color key
    if( surface->flags & SDL_SRCCOLORKEY )
    {
        SDL_SetColorKey( filtered, SDL_RLEACCEL | SDL_SRCCOLORKEY, surface->format->colorkey );
    }

    if( surface->flags & SDL_SRCCOLORKEY )
    {
        SDL_SetColorKey( smoothed, SDL_RLEACCEL | SDL_SRCCOLORKEY, surface->format->colorkey );
    }

    if( surface->flags & SDL_SRCCOLORKEY )
    {
        SDL_SetColorKey( bright, SDL_RLEACCEL | SDL_SRCCOLORKEY, surface->format->colorkey );
    }

    alarm_1.Filtered = filtered;
    alarm_1.Smoothed = smoothed;
    alarm_1.Brightness = bright;

    return alarm_1;
}

// Get Position of the detected object

Camera_Distance Postion_Object(int X_center, int Y_center){

		Camera_Distance Distance1; 
		Distance1.distance = Distance_Calculation(X_center, Y_center); 
		Distance1.angle = Angle_Calculation(X_center, Y_center); 

		return Distance1;
}

