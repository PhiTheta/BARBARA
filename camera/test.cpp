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

int Desired_Flag = 0; 
Camera_Obstacle_Alarm COA; 
Camera_Distance D1;
//The event structure
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 960;
const int SCREEN_BPP = 32;
SDL_Surface *screen = NULL;
SDL_Surface *image = NULL;
SDL_Event event;
// Load and optimize the given image to the size of the set screen; 

SDL_Surface *load_image( std::string filename )
{
    //The image that's loaded
    SDL_Surface* loadedImage = NULL;

    //The optimized surface that will be used
    SDL_Surface* optimizedImage = NULL;

    //Load the image
    loadedImage = IMG_Load( filename.c_str() );

    //If the image loaded
    if( loadedImage != NULL )
    {
        //Create an optimized surface
        optimizedImage = SDL_DisplayFormat( loadedImage );

        //Free the old surface
        SDL_FreeSurface( loadedImage );

        //If the surface was optimized
        if( optimizedImage != NULL )
        {
            //Color key surface
            SDL_SetColorKey( optimizedImage, SDL_SRCCOLORKEY, SDL_MapRGB( optimizedImage->format, 0, 0xFF, 0xFF ) );
        }
    }

    //Return the optimized surface
    return optimizedImage;
}

bool init()
{
    //Initialize all SDL subsystems
    if( SDL_Init( SDL_INIT_EVERYTHING ) == -1 )
    {
        return false;
    }

    //Set up the screenput_pixel32( Smoothed, col, row, p_pix_1);
    screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );

    //If there was an error in setting up the screen
    if( screen == NULL )
    {
        return false;
    }

    //Set the window caption
    SDL_WM_SetCaption( "Flip Test", NULL );

    //If everything initialized fine
    return true;
}

bool load_files(string name)
{
    //Load the image
    image = load_image( name );

    //If there was an problem loading the image
    if( image == NULL )
    {
        return false;
    }

    //If eveything loaded fine
    return true;
}


void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip = NULL)
{
    //Holds offsets
    SDL_Rect offset;

    //Get offsets
    offset.x = x;
    offset.y = y;

    //Blit
    SDL_BlitSurface( source, clip, destination, &offset );
}

int main( int argc, char* args[] )
{
	//Quit flag
    bool quit = false;

    //Initialize
    if( init() == false )
    {
        return 1;
    }

    //Load the files
    if( load_files("scn.png") == false )
    {
        return 1;
    }

	COA = Find_Object(image, 1); 
	D1 = Postion_Object(COA.c_x, COA.c_y);

	printf("center_x: %d, center_y: %d, distance: %f, angle: %f\n",  COA.c_x, COA.c_y,D1.distance,D1.angle);
	apply_surface( 0, 0, image, screen, NULL);
	apply_surface( 640, 0, COA.Filtered, screen,NULL);
	apply_surface( 0, 480, COA.Smoothed, screen,NULL);
	apply_surface( 640, 480, COA.Brightness, screen, NULL);

	//Update the screen
   	 if( SDL_Flip( screen ) == -1 )
    	{
       		return 1;
    	}

    	//While the user hasn't quit
    	while( quit == false )
    	{
        	//While there's event to handle
       		while( SDL_PollEvent( &event ) )
        	{
            	//If the user has Xed out the window
            	if( event.type == SDL_QUIT )
            	{
                	//Quit the program
                	quit = true;
            	}
        }
    }

    //Clean up
    //Free the surfaces
    SDL_FreeSurface( image );
    SDL_FreeSurface( COA.Filtered );
    SDL_FreeSurface( COA.Smoothed );
    SDL_FreeSurface( COA.Brightness );

    //Quit SDL
    SDL_Quit();
    return 0;
}
