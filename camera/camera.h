#include "/usr/include/SDL/SDL.h"
#include "/usr/include/SDL/SDL_image.h"
#include "/usr/include/SDL/SDL_gfxPrimitives.h"

using namespace std;

struct Camera_Obstacle_Alarm{

	bool Red_Obstacle_Flag; 
	bool Blue_Target_Flag;
	int c_x;
	int c_y;
	SDL_Surface *Smoothed; 
	SDL_Surface *Filtered; 
	SDL_Surface *Brightness;

};

struct Camera_Distance{

	float distance;
	float angle;

};
Camera_Obstacle_Alarm Find_Object(SDL_Surface *surface, int color_flag);
Camera_Distance Postion_Object(int X_center, int Y_center);

