/**
 * \file
 * \brief J2B2 Control Demonstration using the J2B2ClientInterface.
 * \author Antti Maula <antti.maula@tkk.fi>
 */
#include "J2B2Demo.hpp"
#include "owndebug.h"
#include <math.h>
#include <signal.h>
#include "gimi.h"
#include <float.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <sys/stat.h>


using namespace std;
bool skip_window=false;

#ifdef ENABLE_SERIALLINK
# include "SerialLink_Client.hpp"
#endif

// SDL includes
#ifndef _DONT_USE_SDL_
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_thread.h>
#include <SDL/SDL_gfxPrimitives.h>
#endif

#define DIST_MARGIN		0.1
#define ANGLE_MARGIN	0.1
#define MIN_WSPEED		0.5
#define MAX_WSPEED		1.5
#define MAX_SPEED		0.4
#define MIN_SPEED		0.1
#define MAGIC_CNST		1
#define WEIGHT_DATA     0.12
#define WEIGHT_SMOOTH	0.12
#define A_TOLERANCE 	0.00001
#define CAM_LIDAR_DIST	0.09
#define GRIPPER_OPEN_ANGLE	0
#define GRIPPER_CLOSED_ANGLE (M_PI/3-M_PI/20)
#define CAMERA_TILT_ANGLE	(-M_PI/10)
#define CORRIDOR_RADIUS	0.3
#define CORRIDOR_DEPTH	0.85
#define GRID_WALL_DEPTH	4

inline float truncate(float val)
{
	if(val > M_PI) {
		val -= 2*M_PI;
	}
	else if(val <= -M_PI) {
		val += 2*M_PI;
	}
	return val;
}
	

	int jx, jy, jr;

//*****************************************************************************

CJ2B2Demo::CJ2B2Demo(CJ2B2Client &aInterface)
  : CSync(1,1), // One ConditionLock, one Mutex.
    CThread(5), // Maximum of 5 threads.
    iInterface(aInterface), // Pass in & store Interface reference
    iDemoActive(false), // Mark demo to be inactive at begin.
    iPTUDemoActive(false),
    iSensorsThreadActive(false),
    iInfoThreadActive(false),
    iMotionThreadActive(false),
    iCameraThreadActive(false),
    iSDLThreadActive(false),
    iCurrentSensorEvent(KSensorEventAllClear), // Sensors seem clear at begin.
    iSmallestDistanceToObject(),
    iLastCameraImage(),
    iLastLaserDistanceArray(),
    iLastLaserTimestamp(),
    iLastOdometryTimestamp(),
    iFirstSLAMAttempt(true),
    iGripperOpen(true),
    iMap(),
	iSmoothAstarPath(),
	iAstarPath(),
	iLaserScans(),
    iNextWaypoint(),
    iLidarPoint(),
    iPauseOn(true),
    iHasPlan(false),
    iObstacleHazard(false),
    iIter(0),
    iNavigationStep(0),
    iMapGrid(),
    iBasePoint(),
    iRobotGridPoint(),
    iWaypointGridPoint(),
    iPreviousAvoidanceDirection(DirectionForward),
    iPreviousNavigationDirection(DirectionUnknown),
    iMotionState(MotionStateIdle),
    iRobotState(RobotStateIdle),
    iPreviousRobotState(RobotStateIdle)
{
}
//*****************************************************************************

CJ2B2Demo::~CJ2B2Demo()
{
  Terminate();
}
//*****************************************************************************

void CJ2B2Demo::Execute()
{
  // Execute only if demo not active.
  if (!iDemoActive) {
    // Flag to mark 'Master' DemoActive (controls threads)
    iDemoActive = true;
        
    const MaCI::Position::TPose2D *mypos = new MaCI::Position::TPose2D::TPose2D(0,0,0);//1.5,1.5,0);
	
	 iInterface.iPositionOdometry->SetPosition(*mypos);
	 
	 iLidarPoint.x = 0;
	 iLidarPoint.y = 0;
	 
	int posSeq = -1;
	MaCI::Position::CPositionData pd;
	if (iInterface.iPositionOdometry) {
		iInterface.iPositionOdometry->GetPositionEvent(pd, &posSeq, 1000);
		const MaCI::Position::TPose2D *pose = pd.GetPose2D();
		iBasePoint.x = pose->x;
		iBasePoint.y = pose->y;
	}
	 //EKF = new odoEKF();
	 //EKF->setDt(0.2);
    
    
    // Start execution of parallel thread(s) (Base class name is not
    // required, it is just added in the demo for clarification
    // purposes)
    dPrint(1,"Executing demo threads...");
    CThread::RunThread(KThreadSensorsDemo);
    CThread::RunThread(KThreadInfoDemo);
    CThread::RunThread(KThreadCameraDemo);
    CThread::RunThread(KThreadSDLDemo);
  }
}
//*****************************************************************************

void CJ2B2Demo::Terminate()
{
  if (iDemoActive) {
    iDemoActive = false;
    dPrint(1,"Terminating threads...");
    if (iCameraThreadActive)  CThread::WaitThread(KThreadCameraDemo);
    if (iMotionThreadActive)  CThread::WaitThread(KThreadMotionDemo);
    if (iInfoThreadActive)    CThread::WaitThread(KThreadInfoDemo);
    if (iSensorsThreadActive) CThread::WaitThread(KThreadSensorsDemo);
    if (iSDLThreadActive)     CThread::WaitThread(KThreadSDLDemo);
    dPrint(1,"All Threads terminated succesfully.");
  }
}
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

int CJ2B2Demo::RunInfoDemo(int aIterations)
{
  //int iterations = 0;
  //int posSeq = -1;
  //ownTime_ms_t tbegin = ownTime_get_ms();
  using namespace MaCI::Position;
  using namespace MaCI::Map;
  using namespace gimi;

  if (iInfoThreadActive) {
    dPrint(1,"InfoDemo already active! Will not start again.");
    return -1;

  } else {
    dPrint(3,"InfoDemoThread starting...");
    iInfoThreadActive = true;

  }

/*
  // This prints Odometry out./J2B2-UI-example localhost 40022 FSRSim.J2B2put every 5 seconds.
  if (iInterface.iPositionOdometry) {
    while(iDemoActive && 
          iInfoThreadActive && 
          (aIterations == -1 || iterations < aIterations)) {
      CPositionData pd;
      if (iInterface.iPositionOdometry->GetPositionEvent(pd,
                                                     &posSeq,
                                                     1000)) {
        const TPose2D *pose = pd.GetPose2D();
        if (pose && ownTime_get_ms_since(tbegin) > 1000) {
          dPrint(1,"Odometry position now at: x:%f, y:%f, a: %frad",
                 pose->x, pose->y, pose->a);
          tbegin = ownTime_get_ms();
        }
      } else {
        dPrint(1,"No event available!");
        
      }
      
      ++iterations;
    }
  } else {
    dPrint(1,"No PositionOdometry available. Nothing to do. Abort Infothread.");

  }
  //*/
  // Mark thread terminated.
  iInfoThreadActive = false;

  dPrint(3,"InfoDemoThread terminated.");
  return 0;
}
//*****************************************************************************

int CJ2B2Demo::RunSDLDemo(int aIterations)
{ 
  if (iSDLThreadActive) {
    dPrint(1,"SDLDemo already active! Will not start again.");
    return -1;

  } else {
    dPrint(3,"SDLDemoThread starting...");
    iSDLThreadActive = true;

  }

#ifdef ENABLE_SERIALLINK
  SerialLink::CSerialLinkClient *slc = new SerialLink::CSerialLinkClient();
  if (slc->Connect("robogw.dyndns.tv", 40002, "J2B2.ttyACM0", false)) {
    slc->Close();
    if (slc->Open()) {
      dPrint(ODTEST,"Succesfully Open():ed SerialLink connection");

    } else {
      dPrint(ODTEST,"SerialLi./J2B2-UI-example localhost 40022 FSRSim.J2B2nk connection not available.");
      delete slc;
      slc = NULL;

    }
  }
#endif

#ifndef _DONT_USE_SDL_
  int iterations = 0;
  SDL_Surface *screen = NULL;
  const int window_width = 1100;
  const int window_height = 768;

  // Manual Speed control
  ownTime_ms_t speedcommand_last_sent = ownTime_get_ms();
  float r_speed = 0.00, r_wspeed = 0.00, r_acc = 0.20;

  // Manual PTU control
  ownTime_ms_t ptucommand_last_sent =  ownTime_get_ms();
  float ptu_pan = 0.00, ptu_tilt = 0.00;
  float ptu_pan_delta = 0.0, ptu_tilt_delta = 0.0;

  /////////////////////////////////////////////////////////////////////////////
  // Initialize SDL
  if(SDL_Init(SDL_INIT_VIDEO) == -1){
    dPrint(1,"Failed to initialize SDL Video: %s\n", SDL_GetError());
    exit(1);
  }
  
  // Register cleanup function for SDL
  atexit(SDL_Quit);
  
  // Initialize SDL-Video in mode WW*WH, 32bpp, & some flags.
  if ((screen = SDL_SetVideoMode(window_width, window_height, 32,
				 SDL_HWSURFACE|SDL_DOUBLEBUF|SDL_RESIZABLE)) == NULL) {
    dPrint(1, "Unable to set video mode for SDL Context: %s\n", SDL_GetError());
    abort();
  }
  
  // Clear the screen
  SDL_FillRect(screen, NULL, 0);

  // Set imporant info :)
  SDL_WM_SetCaption("Simple J2B2 Demo application","J2B2Demo");
  

  // Now, run the loop of processing as long as the demo is alive.
  while(iDemoActive && 
        iSDLThreadActive && 
        (aIterations == -1 || iterations < aIterations)) {
    SDL_Event event;
    
    // Wait for incoming event
    while (SDL_PollEvent(&event)) {
      
      // Switch based on event type.
      switch (event.type) {
        
        // QUIT ('Window close' clicked)
      case SDL_QUIT: {
        // close button clicked
        dPrint(1,"'Window close' clicke./J2B2-UI-example localhost 40022 FSRSim.J2B2d.");
        iDemoActive = false;
        ownSleep_ms(1000);
        raise(SIGINT);
        raise(SIGINT);
        raise(SIGTERM);
        ownSleep_ms(1000);
        raise(SIGKILL);
        break;
      }
        
        // Any key down
      case SDL_KEYDOWN: {
        // Switch based on the key that was pressed
        switch (event.key.keysym.sym) {
          
          /////////////////////////////////////////////////////////////
          // Toggle DEBUG messages
          /////////////////////////////////////////////////////////////
        case SDLK_t:
          iPauseOn = !iPauseOn;
        break;
          
          /////////////////////////////////////////////////////////////
          // Perform simple robot control (just set the current values
          ///////////////////////////iX_RES//////////////////////////////////
        case SDLK_UP:
          // 'w' pressed - set speed to 'forward 0.3m/s'
          r_speed = 0.30;
          break;
        case SDLK_LEFT:
          // 'a' pressed - set speed 'turn left M_PI rad/s'
          r_wspeed = M_PI;
          break;
        case SDLK_DOWN:
          // 's' pressed - set speed to 'backwards 0.3m/s'
          r_speed = -0.30;
          break;
        case SDLK_RIGHT:
          // 'd' pressed - set speed 'turn right M_PI rad/s'
          r_wspeed = -M_PI;
          break;
          
          ///////////////////////////////////////////////////////////
          // Perform PTU control
          ///////////////////////////////////////////////////////////
        case SDLK_w:
          // 'w' pressed
          ptu_tilt_delta = 1.0;
          break;
        case SDLK_a:
          // 'a' pressed
          ptu_pan_delta = 1.0;
          break;
        case SDLK_s:
          // 's' pressed
          ptu_tilt_delta = -1.0;
          break;
        case SDLK_d:
          // 'd'-pressed
          ptu_pan_delta = -1.0;
          break;

         // Behaviour control example
        case SDLK_e:
          dPrint(1,"Enable Emergency behaviours");
          iInterface.iBehaviourCtrl->SetStart();
          break;
          
        case SDLK_r:
          dPrint(1,"Disable Emergency behaviours");
          iInterface.iBehaviourCtrl->SetStop();
          break;

          // Control Motion demo
        case SDLK_m:
          if (iMotionThreadActive) {
            dPrint(1,"Terminating MotionDemo...");
            iMotionThreadActive = false;
            iPreviousRobotState = iRobotState = RobotStateIdle;
			iObstacleHazard = false;
            iNextWaypoint.x = iNextWaypoint.y = 0;
            CThread::WaitThread(KThreadMotionDemo);
            dPrint(1,"Terminated.");

          } else {
            dPrint(1,"Executing MotionDemo..");
            CThread::RunThread(KThreadMotionDemo);
          }
          break;
        
        //Control gripper 
        case SDLK_g:
		 if (!iMotionThreadActive) {
			 bool worked = false;
			 while(!worked) {
				 worked = iInterface.iServoCtrl->SetPosition(iGripperOpen ? GRIPPER_CLOSED_ANGLE : GRIPPER_OPEN_ANGLE, KServoUserServo_0);
			 }
			 ownSleep_ms(200);
			 iGripperOpen = !iGripperOpen;
		 }
		 break;

          // Control PTU demo.
        case SDLK_p:
          if (iPTUDemoActive) {
            dPrint(1,"Terminating PTUDemo...");
            iPTUDemoActive = false;
            dPrint(1,"Terminated.");
            
          } else if (iCameraThreadActive) {
            dPrint(1,"Executing PTUDemo..");
            iPTUDemoActive = true;

          } else {
            dPrint(1,"Executing CameraThread & PTUDemo..");
            iPTUDemoActive = true;
            CThread::RunThread(KThreadCameraDemo);
            
          }
          break;

        case SDLK_1: {
          // Set All outputs HIGH.
          // Calling SetPin for all pins SEPARATELY. If you need more
          // synchronous & faster control between different outputs,
          // use the 'SetDigitalMask()' function instead.
          if (iInterface.iIOBoardESC != NULL) {
            for(unsigned int pin = 8; pin < 16; ++pin) {
              iInterface.iIOBoardESC->SetDigitalPin(pin, true);
            }
          }
          dPrint(1,"Set all outputs on ESC to HIGH state");
          break;
        }

        case SDLK_0: {
          // Set all outputs LOW.
          // Calling SetPin for all pins SEPARATELY. If you need more
          // synchronous & faster control between different outputs,
          // use the 'SetDigitalMask()' function instead.
          if (iInterface.iIOBoardESC != NULL) {
            for(unsigned int pin = 8; pin < 16; ++pin) {
              iInterface.iIOBoardESC->SetDigitalPin(pin, false);
            }
          }
          dPrint(1,"Set all outputs on ESC to LOW state");
          break;
        }

#ifdef ENABLE_SERIALLINK
          /** This block demonstrates how to use the SeriaLink module.
           * The module itself is constructed in the beginning of this
           * function. The module tries to emulate the interface of a
           * standard serial port. See more from the documentation of
           * the CSerialLinkClient, linked from the project work
           * tutorial pages.
           */
        case SDLK_i: {
          if (slc) {
            char msg[] = "Hello, world!";
            char reply[100] = "";
            
            slc->Write((const unsigned char*)msg, strlen(msg)+1);

            // Give it a while to go through (Only for demonstration purposes!)
            ownSleep_ms(500);

            // Now, Wait for Read event, ma./J2B2-UI-example localhost 40022 FSRSim.J2B2ximum of 500ms more
            if (slc->WaitToRead(500)) {
              if (slc->Read((uint8_t*)reply, sizeof(reply)) > 0) {
                dPrint(ODTEST,"Got reply: '%s'", reply);
              }
            } else {
              dPrint(ODTEST,"No reply within timeout :(");
              
            }
          } else {
            dPrint(ODTEST,"SerialLink not available (slc == NULL)");
            
          }
          break;
        }
#endif
          
          ///////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////

        default:
          // Default, print events
          dPrint(1,"SDL returned KEYDOWN event for '%s'" , SDL_GetKeyName(event.key.keysym.sym));
          break;
        }
        break; // SDL_KEYDOWN - end
      }
        
      case SDL_KEYUP: {
        switch (event.key.keysym.sym) {
        case SDLK_UP:
        case SDLK_DOWN:
          r_speed = 0.00;
          break;
        case SDLK_LEFT:
        case SDLK_RIGHT:
          // In every case of the controls, UP == SetSpeed(0.0, 0.0);
          r_wspeed = 0.00;
          break;
          
        case SDLK_w:
        case SDLK_s:
          ptu_tilt_delta = 0.00;
          break;

        case SDLK_a:
        case SDLK_d:
          ptu_pan_delta = 0.00;
          break;

        case SDLK_0:
        case SDLK_1:
          // No action on IO ups.
          break;

        case SDLK_e:
        case SDLK_r:
        case SDLK_m:
        case SDLK_p:
          // No action for Command keys on UP.
          break;

        default:
          dPrint(1,"SDL returned KEYUP event for '%s'" , SDL_GetKeyName(event.key.keysym.sym));
          break;
        }
        break; // SDL_KEYUP - end
      }
        
        // Default handler. Nothing done.
      default:
        break; // default - end
      }
    }
    

    // Then, send the current speed commands to Robot (it needs
    // refreshing). But ONLY; if the demo is not active, and last
    // command was sent more than 200 ms ago (too rapid sending rate
    // will clutter the connection and everything will start to go
    // wrong)
    

    if (iMotionThreadActive == false && 
        ownTime_get_ms_since(speedcommand_last_sent) > 200) {
      // Only send if the motion demo is not active (it is a
      // standalone demo controlling the robot, if it is active,
      // sending commands to robot would only mess up things)
      iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
      speedcommand_last_sent = ownTime_get_ms();
    }


    // Then, apply the PTU manual control parameters.
    if (iMotionThreadActive == false && iPTUDemoActive == false &&
        ownTime_get_ms_since(ptucommand_last_sent) > 100) {
      // Do movement
      ptu_pan += ptu_pan_delta * M_PI/20;
      ptu_tilt += ptu_tilt_delta * M_PI/20;
      
      // Do value checking and limiting.
      if (ptu_pan > M_PI/2) ptu_pan = M_PI/2;
      else if (ptu_pan < -M_PI/2) ptu_pan = -M_PI/2;
      if (ptu_tilt > M_PI/2) ptu_tilt = M_PI/2;
      else if (ptu_tilt < -M_PI/2) ptu_tilt = -M_PI/2;

      // Do control
      iInterface.iServoCtrl->SetPosition(ptu_pan, KServoCameraPTUPan);
      iInterface.iServoCtrl->SetPosition(ptu_tilt, KServoCameraPTUTilt);
      ptucommand_last_sent = ownTime_get_ms();
    }

    // Lock class mutex.
    Lock();


    // Clear the screen.
    SDL_FillRect(screen, NULL, 0);

	SDL_Rect rectImg = {450, 0, 640, 481};
	SDL_Rect frameRectImg = {450, 0, 640, 480};
	SDL_FillRect(screen , &rectImg , SDL_MapRGB(screen->format , 0,200,0));
	SDL_FillRect(screen , &frameRectImg , SDL_MapRGB(screen->format , 0,0,0));
	
	
    // Now, draw the last received Image.
    if (iLastCameraImage.GetImageDataType() == MaCI::Image::KImageDataJPEG &&
        iLastCameraImage.GetImageDataPtr() != NULL) {
      
      /////////////////////////////////////////////////////////////////////////////
      // Draw the last imageimage! (Don't care whether its new or not)
      /////////////////////////////////////////////////////////////////////////////
      SDL_Surface *image;
      SDL_RWops *rw;
      SDL_Rect rcDest = {450,0,0,0};
      
      rw = SDL_RWFromMem((void*)iLastCameraImage.GetImageDataPtr(), 
                         iLastCameraImage.GetImageDataSize());
      image = IMG_LoadJPG_RW(rw);
      SDL_BlitSurface(image, NULL, screen, &rcDest);
      SDL_FreeSurface(image);
    }
      
    
    // Draw the Map
    if (iMap.size() > 0) {
		
		MaCI::Position::CPositionData pd;
		if (iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastLaserTimestamp.GetGimTime())) {
			const MaCI::Position::TPose2D *pose = pd.GetPose2D();
			
			SDL_Rect rect = {0,0,450,450};
			SDL_Rect frameRect = {0,0,449,449};
			SDL_FillRect(screen , &rect , SDL_MapRGB(screen->format , 0,200,0));
			SDL_FillRect(screen , &frameRect , SDL_MapRGB(screen->format , 0,0,0));
			float center_x = rect.x+rect.w/2;
			float center_y = rect.y+rect.h/2;
		  
			//Draw home rect
			SDL_Rect homeFrameRect = {center_x+iBasePoint.x-10, center_y+iBasePoint.y-10, 20, 20};
			SDL_Rect homeRect = {center_x+iBasePoint.x-9, center_y+iBasePoint.y-9, 18, 18};
			SDL_FillRect(screen , &homeFrameRect , SDL_MapRGB(screen->format , 255,255,255) );
			SDL_FillRect(screen , &homeRect , SDL_MapRGB(screen->format , 0,0,0) );
			
			TPoint posePoint;
			posePoint.x = pose->x;
			posePoint.y = pose->y;
			
			TPoint robotSDL = SDLPoint(posePoint);
			TPoint lidarSDL = SDLPoint(robotToWorldPoint(iLidarPoint, pose));
			
			//Draw laser lines
			vector<TPoint> laserPoints;
			unsigned int size = iLastLaserDistanceArray.size();
			Sint16 lvx[size], lvy[size];
			for(EACH_IN_i(iLastLaserDistanceArray)) {
				TPoint scanPoint = worldPoint(i->distance, i->angle, iLidarPoint.y, pose);
				TPoint scanSDL = SDLPoint(scanPoint);
				int idx = std::distance(iLastLaserDistanceArray.begin(), i);
				lvx[idx] = scanSDL.x;
				lvy[idx] = scanSDL.y;
				laserPoints.push_back(scanSDL);
			}
			filledPolygonRGBA(screen, lvx, lvy, size, 255, 155, 155, 50);
			
			//Draw safe zone
			if (iMotionThreadActive) {
				TPoint close_left, close_right, far_left, far_right;
				close_left.x = -CORRIDOR_RADIUS, close_left.y = 0;
				close_right.x = CORRIDOR_RADIUS, close_right.y = 0;
				far_left.x = -CORRIDOR_RADIUS, far_left.y = CORRIDOR_DEPTH;
				far_right.x = CORRIDOR_RADIUS, far_right.y = CORRIDOR_DEPTH;
				close_left = robotToWorldPoint(close_left, pose);
				close_right = robotToWorldPoint(close_right, pose);
				far_left = robotToWorldPoint(far_left, pose);
				far_right = robotToWorldPoint(far_right, pose);
				TPoint farLeftSDL = SDLPoint(far_left);
				TPoint farRightSDL = SDLPoint(far_right);
				TPoint closeLeftSDL = SDLPoint(close_left);
				TPoint closeRightSDL = SDLPoint(close_right);
				const Sint16 vx[4] = {closeLeftSDL.x, farLeftSDL.x, farRightSDL.x, closeRightSDL.x};
				const Sint16 vy[4] = {closeLeftSDL.y, farLeftSDL.y, farRightSDL.y, closeRightSDL.y};
				filledPolygonRGBA(screen, vx, vy, 4, iObstacleHazard ? 255 : 0, iObstacleHazard ? 0 : 255, 0, 100);
			}
			
			
			//Draw a navigation pointer
			char mystr[255];
			if (iRobotState == RobotStateGoHome || iRobotState == RobotStateGoToStone || (iRobotState == RobotStateAvoidObstacle && (iPreviousRobotState == RobotStateGoHome || iPreviousRobotState == RobotStateGoToStone))) { 
				TPoint waypointSDL = SDLPoint(iNextWaypoint);
				lineRGBA(screen, robotSDL.x, robotSDL.y, waypointSDL.x, waypointSDL.y, 255, 255, 0, 255);
				sprintf(mystr, "Going to: %f, %f", iNextWaypoint.x,iNextWaypoint.y);
				stringRGBA(screen, 480, 540,  mystr, 0, 255, 0, 150);
			}
			
			//Draw a star waypoints
			if (iMotionThreadActive) {
				if (iSmoothAstarPath.size() > 0) {
					for (int i = iNavigationStep; i < (int)iSmoothAstarPath.size()-1; i++) {
						MaCI::Position::TPose2D waypoint1 = iSmoothAstarPath.at(i);
						MaCI::Position::TPose2D waypoint2 = iSmoothAstarPath.at(i+1);
						TPoint nodePoint1, nodePoint2;
						nodePoint1.x = waypoint1.x*X_RES, nodePoint1.y = waypoint1.y*Y_RES;
						nodePoint2.x = waypoint2.x*X_RES, nodePoint2.y = waypoint2.y*Y_RES;
						TPoint nodeSDL1 = SDLPoint(nodePoint1);
						TPoint nodeSDL2 = SDLPoint(nodePoint2);
						lineRGBA(screen, nodeSDL1.x, nodeSDL1.y, nodeSDL2.x, nodeSDL2.y, 255, 0, 255, 255);
					}
				}
				else {
					stringRGBA(screen, 480, 560, "No smooth astar path", 255, 255, 0, 150);
				}
			}
			
			
			//Draw robot, heading and laser device position
			TPoint pointerPoint1, pointerPoint2, pointerPoint3;
			pointerPoint1.x = 0, pointerPoint1.y = 0.2;
			pointerPoint2.x = -0.2, pointerPoint2.y = 0;
			pointerPoint3.x = 0.2, pointerPoint3.y = 0;
			pointerPoint1 = robotToWorldPoint(pointerPoint1, pose);
			pointerPoint2 = robotToWorldPoint(pointerPoint2, pose);
			pointerPoint3 = robotToWorldPoint(pointerPoint3, pose);
			TPoint pointerSDL1 = SDLPoint(pointerPoint1);
			TPoint pointerSDL2 = SDLPoint(pointerPoint2);
			TPoint pointerSDL3 = SDLPoint(pointerPoint3);
			
			filledCircleRGBA(screen, robotSDL.x, robotSDL.y, (int)10, 0, 255, 0, 255);
			filledTrigonRGBA(screen, pointerSDL1.x, pointerSDL1.y, pointerSDL2.x, pointerSDL2.y, pointerSDL3.x, pointerSDL3.y, 255, 0, 0, 255);
			filledCircleRGBA(screen, lidarSDL.x, lidarSDL.y, (int)2, 255, 255, 0, 255);
			circleRGBA(screen, robotSDL.x, robotSDL.y, (int)10, 0, 0, 0, 255);
			
			// draw detected circle 
			
			circleRGBA(screen, 450 + jx, jy, jr, 255, 255, 255, 255);
			//Draw obstacles
			//Lock();
			for (vector<TPoint>::iterator iterator = iMap.begin(); iterator < iMap.end(); iterator++) {
				TPoint pointSDL = SDLPoint(*iterator);
				filledCircleRGBA(screen, pointSDL.x, pointSDL.y, (int)1, 100, 100, 255, 255);
			}
			//Unlock(); 
			
			//Draw laser points
			for (vector<TPoint>::iterator iterator = laserPoints.begin(); iterator < laserPoints.end(); iterator++) {
				TPoint laser = *iterator;
				filledCircleRGBA(screen, laser.x, laser.y, (int)2, 255, 0, 0, 255);
			}
			
			
			//Draw shortest distance
			TPoint closestPoint = robotPoint(iSmallestDistanceToObject.distance, iSmallestDistanceToObject.angle, iLidarPoint.y);
			closestPoint = robotToWorldPoint(closestPoint, pose);
			TPoint closestSDL = SDLPoint(closestPoint);
			lineRGBA(screen, lidarSDL.x, lidarSDL.y, closestSDL.x, closestSDL.y, 100, 100, 255, 255);
			circleRGBA(screen, closestSDL.x, closestSDL.y, (int)3, 100, 100, 255, 255);
			
			
			const char stateStr[9][60] = {
				"State: Idle",
				"State: Wandering",
				"State: Avoiding obstacle",
				"State: Navigating to a ball",
				"State: Navigating home",
				"State: Openning gripper",
				"State: Closing gripper",
				"State: Moving away from the base",
				"State: Self destruct"
			};
			
			const char gripperStr[2][60] = {
				"Gripper: Closed",
				"Gripper: Open"
			};
			
			const char directionStr[4][60] = {
				"Direction: Unknown",
				"Direction: Forward",
				"Direction: Left",
				"Direction: Right"
			};
			
			//Draw odometry
			sprintf(mystr, "Odometry: %f, %f, %f", pose->x, pose->y, pose->a);
			stringRGBA(screen, 480, 500,  mystr, 0, 255, 0, 150);
			stringRGBA(screen, 480, 510,  stateStr[iRobotState], 0, 255, 0, 150);
			stringRGBA(screen, 480, 520,  gripperStr[iGripperOpen], 0, 255, 0, 150);
			if (iRobotState == RobotStateAvoidObstacle) {
				stringRGBA(screen, 700, 510,  directionStr[iPreviousAvoidanceDirection], 0, 255, 0, 150);
			}
				
			
			
			//Draw grid
			SDL_Rect rectGrid = {0, 450, 450, 370};
			SDL_Rect frameRectGrid = {0, 450, 449, 369};
			SDL_FillRect(screen , &rectGrid , SDL_MapRGB(screen->format , 0,200,0));
			SDL_FillRect(screen , &frameRectGrid , SDL_MapRGB(screen->format , 0,0,0));
			
			if (iAstarPath.size() > 0) {//iRobotState == RobotStateGoHome || iRobotState == RobotStateGoToStone || (iRobotState == RobotStateAvoidObstacle && (iPreviousRobotState == RobotStateGoHome || iPreviousRobotState == RobotStateGoToStone))) {
				
				SDL_Rect cellRect = {0, 450, rectGrid.w/MAP_COLS, rectGrid.h/MAP_ROWS};
				for (int i = 0; i < MAP_ROWS; i++) {
					for (int j = 0; j < MAP_COLS; j++) {
						if (iMapGrid[i*MAP_COLS+j] == 0) {
							cellRect.x = rectGrid.w-cellRect.w*(j+1);
							cellRect.y = rectGrid.y+cellRect.h*i;
							SDL_FillRect(screen, &cellRect, SDL_MapRGB(screen->format, 100, 100, 255));
						}
					}
				}
				for (EACH_IN_i(iAstarPath)) {
					cellRect.x = rectGrid.w-cellRect.w*(i->x+1);
					cellRect.y = rectGrid.y+cellRect.h*i->y;
					SDL_FillRect(screen, &cellRect, SDL_MapRGB(screen->format, 255, 0, 255));
				}
				cellRect.x = rectGrid.w-cellRect.w*(iRobotGridPoint.x+1);
				cellRect.y = rectGrid.y+cellRect.h*iRobotGridPoint.y;
				SDL_FillRect(screen, &cellRect, SDL_MapRGB(screen->format, 0, 255, 0));
				cellRect.x = rectGrid.w-cellRect.w*(iWaypointGridPoint.x+1);
				cellRect.y = rectGrid.y+cellRect.h*iWaypointGridPoint.y;
				SDL_FillRect(screen, &cellRect, SDL_MapRGB(screen->format, 255, 255, 0));
				
				//Check if mouse over
				int x, y;
				SDL_GetMouseState(&x, &y);
				if (x >= rectGrid.x && x <= rectGrid.x+rectGrid.w && y >= rectGrid.y && y <= rectGrid.y+rectGrid.h) {
					int grid_x = round((x-rectGrid.x)/cellRect.w);
					int grid_y = round((y-rectGrid.y)/cellRect.h);
					SDL_Rect captionRect = {x+10, y, 30, 10};
					SDL_FillRect(screen, &captionRect, SDL_MapRGB(screen->format, 0, 0, 0));
					sprintf(mystr, "%d,%d", grid_x, grid_y);
					stringRGBA(screen, x+10, y, mystr, 255, 255, 255, 255);
				}
			}
		}
	}
	
    if (iMotionThreadActive) {
      stringRGBA(screen, 480, 550, "Driving autonomously", 255, 0, 0, 150);
    }
    
    // print status reports
    char diststr[255];
    TPoint closestPoint = robotPoint(iSmallestDistanceToObject.distance, iSmallestDistanceToObject.angle, iLidarPoint.y);
    sprintf(diststr, "Closest point: %.2fm at %.3f rad (%.2fm, %.2fm cartesian)", 
            iSmallestDistanceToObject.distance, 
            iSmallestDistanceToObject.angle,
            closestPoint.x, closestPoint.y);
    
    stringRGBA(screen, 480, 530, 
               diststr,
               0, 255, 255, 150);

    if (iInterface.iIOBoardESC != NULL) {
      int m = iInterface.iIOBoardESC->GetDigitalMask(0, 1000);
      char bitstr[255];
      sprintf(bitstr, "IO port state now: %08x", m);
      
      stringRGBA(screen, 
                 480, 570, 
                 bitstr,
                 0, 255, 255, 150);
    }
    
    if (iPTUDemoActive) {
      stringRGBA(screen, 
                 480, 580, 
                 "PTUDemo is active!", 
                 0, 255, 0, 150);
    }

    // Flip the double buffered screen. (Display the newly rendered screen)
    SDL_Flip(screen);

    // Unlock whole class mutex.
    Unlock();

    // Cut some slack :)
    ownSleep_ms(10);
  }

#endif

  dPrint(3,"SDLDemoThread terminated.");
  return 0;
}
//*****************************************************************************

int CJ2B2Demo::RunCameraDemo(int aIterations)
{
  bool r;
  using namespace MaCI::Image;
  using namespace MaCI::Map;
  CImageData imgData;
  unsigned int imgSeq         = 0;
  float panAngleRad           = -M_PI/2.0;
  float tiltAngleRad          = -M_PI/10.0;
  int panStepCurrent          = 0;
  const int panSteps          = 10;
  float panStepSizeRad  = ( M_PI / 10 );

  if (iCameraThreadActive) {
    dPrint(1,"CameraDemo already active! Will not start again.");
    return -1;
    
  } else {
    dPrint(3,"CameraDemoThread starting...");
    iCameraThreadActive = true;

  }

  int iterations = 0;
  while(iDemoActive && 
        iCameraThreadActive && 
        (aIterations == -1 || iterations < aIterations)) {
    
    // Is ServoCtrl available, and is ServoDemo active?
    if (iInterface.iServoCtrl && 
        iPTUDemoActive) {

      // We have a fixed number of steps. 
      if (panStepCurrent < panSteps) {
        panAngleRad += panStepSizeRad;
        panStepCurrent++;
        
      } else { 
        panStepCurrent = 0;
        panStepSizeRad = -panStepSizeRad;
      }
      
      
      // Command Servo to move. (Move servos attached to Camera PTU)
      r = iInterface.iServoCtrl->SetPosition(panAngleRad, KServoCameraPTUPan);
      r &= iInterface.iServoCtrl->SetPosition(tiltAngleRad, KServoCameraPTUTilt);

      if (r) {
        float ppos,tpos;
        iInterface.iServoCtrl->GetPosition(KServoCameraPTUPan, ppos, 500);
        iInterface.iServoCtrl->GetPosition(KServoCameraPTUTilt, tpos, 500);
        dPrint(3,"Camera now pointing to pan:%.3f, tilt:%.3f",
               ppos, tpos);
        
        // Wait to stabilize
        ownSleep_ms(200);

      } else {
        // Some failure?
        ownSleep_ms(1000);

      }
      
    } else {
      dPrint(8,"No ServoCtrl available - skipping");

    }
    

    if (iInterface.iImageCameraFront) {
      
      // Read last available Image. This function reads a image from
      // the camera and stores it to the provided 'imgData'
      // object. The 'imgSeq' is the number to expect. In normal
      // processing, user should never change value of this variable
      // after starting to receive images. Note: the 'imgSeq' variable
      // is modified BY the 'GetImageData' function to reflect the
      // latest received imagedata sequence. See description of
      // 'GetImageData' function for more details.
      r = iInterface.iImageCameraFront->GetImageData(imgData, &imgSeq);

      if (r) {

        // COPY the received imagedata to class variable; for (possibly)
        // using it from another thread.
        
        // Lock the whole J2B2Demo instance. We are only using a single
        // lock for all shared operations for simplicity. You should
        // consider using multiple Locks depending on your application
        // implementation
        Lock();
        
        // The 'GetImage' function call of the CImageData class returns
        // the REAL data element from the CImageData container. The
        // resulting data is stored to image container class names
        // 'CImageContainer'. This class is a multi-function container
        // which is for example; able to convert the given data between
        // various formats. For example, the J2B2 always sends the data
        // in JPG format, but by using the CImageContainer classes
        // 'ConvertTo()' function, you can attempt to convert the data
        // to different format before using it.
        if (imgData.GetImage(iLastCameraImage, NULL, true) != true) {
          dPrint(1,"Failed to read ImageContainer data?");
        }
        
        // Release the previously locked class instance.
        Unlock();
      }
    } else {
      dPrint(5,"No ImageCameraFront available - skipping (sleep 1000ms)");
      ownSleep_ms(1000);
      
    }
    
    // Increment.
    iterations++;
  }

  dPrint(3,"CameraDemoThread terminated.");
  return 0;
}
//******************************* EKF Function *********************************////




//*****************************************************************************///

int CJ2B2Demo::RunMotionDemo(int aIterations){
  using namespace MaCI::Position;
  using namespace MaCI;
 
  int posSeq = -1;
  float K_alpha = 0.05;
  
  
  if (iMotionThreadActive) {
    dPrint(1,"MotionDemo already active! Will not start again.");
    return -1;
    
  } else {
    dPrint(3,"MotionDemoThread starting...");
    iMotionThreadActive = true;

	MaCI::Position::CPositionData pd;
    const MaCI::Position::TPose2D *mypos = new MaCI::Position::TPose2D::TPose2D(0,0,0);
	iInterface.iPositionOdometry->SetPosition(*mypos);
	iRobotState = RobotStateWander;
	int cnt = 60;
	float r_acc = 0.2;
	float r_speed = 0.0;
	float r_wspeed = 0.5;
	if (iInterface.iPositionOdometry) {
		iInterface.iPositionOdometry->GetPositionEvent(pd, &posSeq, 1000);
		const TPose2D *pose = pd.GetPose2D();
		iBasePoint.x = pose->x;
		iBasePoint.y = pose->y;
	}
	
	iInterface.iBehaviourCtrl->SetStart();
   
	for (int i = 0; i < MAP_COLS*MAP_ROWS; i++) {
		iMapGrid[i] = 1;
	}
   
    bool tilted = false;
    while (!tilted) {
		tilted = iInterface.iServoCtrl->SetPosition(CAMERA_TILT_ANGLE, KServoCameraPTUTilt);
	}
	ownSleep_ms(20);
	tilted = false;
	while (!tilted && !iGripperOpen) {
		tilted = iInterface.iServoCtrl->SetPosition(GRIPPER_OPEN_ANGLE, KServoUserServo_0);
	}
	iGripperOpen = true;
	iNavigationStep = 0;
	iSmoothAstarPath.clear();
	ownSleep_ms(1000);
	dPrint(1, "Camera tilted");
	
	ownTime_ms_t lastObstacleOccurance = 0;

	int iterations = 0;
	while(iDemoActive && iMotionThreadActive && (aIterations == -1 || iterations < aIterations) && iRobotState != RobotStateShutdown) {
		if (iInterface.iMotionCtrl) {
			if (iInterface.iPositionOdometry) {
				iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastLaserTimestamp.GetGimTime());
				const TPose2D *pose = pd.GetPose2D();
				//iPose = pd.GetPose2D();
				//dPrint(1, "ODO %f %f %f", pose->x, pose->y, pose->a);
					
				
				r_acc = 0.1;
				r_speed = 0.0;
				r_wspeed = 0.0;
				
				//Check for obstacles;
				iObstacleHazard = false;
				for (EACH_IN_i(iLaserScans)) {
					if (fabs(i->x) < CORRIDOR_RADIUS && i->y < CORRIDOR_DEPTH) {
						iObstacleHazard = true;
						break;
					}
				}
								
				if (iObstacleHazard) {
					lastObstacleOccurance = ownTime_get_ms();
				}
					
				if (iObstacleHazard && iRobotState != RobotStateAvoidObstacle) {
					iInterface.iMotionCtrl->SetStop();
					ownSleep_ms(20);
					iPreviousRobotState = iRobotState;
					dPrintLCYellow(1, "Avoiding obstacle");
					iRobotState = RobotStateAvoidObstacle;
					continue;
				}
				else {
					if (iRobotState == RobotStateMoveAway) {
						r_acc = 0.1;
						r_speed = -0.15;
						r_wspeed = 0;
						iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
						ownSleep_ms(20);
						if (fabs(iBasePoint.x-pose->x) > 0.2 || fabs(iBasePoint.y-pose->y) > 0.2) {
							iInterface.iMotionCtrl->SetStop();
							iPreviousRobotState = iRobotState = RobotStateShutdown;
							iMotionThreadActive = false;
							iObstacleHazard = false;
				            iNextWaypoint.x = iNextWaypoint.y = 0;
							dPrintLCGreen(1, "Mission complete. Good bye!");
				            CThread::WaitThread(KThreadMotionDemo);
				            dPrint(1,"Terminated.");
						}
					}
					else if (iRobotState == RobotStateOpenGripper) {
						bool open = true;
						if (!iGripperOpen) {
							open = iInterface.iServoCtrl->SetPosition(GRIPPER_OPEN_ANGLE, KServoUserServo_0);
							ownSleep_ms(200);
						}
				        if (open) {
							dPrintLCYellow(1, "Moving away from the base");
							iRobotState = RobotStateMoveAway;
							iGripperOpen = true;
							continue;
						}
					}	
					else if (iRobotState == RobotStateCloseGripper) {
						bool closed = true;
						if (iGripperOpen) {
							closed = iInterface.iServoCtrl->SetPosition(GRIPPER_CLOSED_ANGLE, KServoUserServo_0);
							ownSleep_ms(200);	
						}
				        if (closed) {
							iNextWaypoint = iBasePoint;
							dPrintLCYellow(1, "Going home");
							iRobotState = RobotStateGoHome;
							iGripperOpen = false;
							continue;
						}
					}
					else if (iRobotState == RobotStateAvoidObstacle) {
						if (iObstacleHazard) {
							TurnDirection direction = iPreviousAvoidanceDirection != DirectionUnknown && iPreviousAvoidanceDirection != DirectionForward ? iPreviousAvoidanceDirection : iSmallestDistanceToObject.angle > 0 ? DirectionRight : DirectionLeft;
							
							//Turn by random angle
							r_wspeed = 0.3;
							r_wspeed *= direction == DirectionLeft ? 1 : -1;
							iPreviousAvoidanceDirection = direction;
							iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
							ownSleep_ms(20);
						}
						else {
							iInterface.iMotionCtrl->SetStop();
							ownSleep_ms(20);
							if (iPreviousRobotState == RobotStateGoHome) dPrintLCYellow(1, "Going home");
							else if (iPreviousRobotState == RobotStateGoToStone) dPrintLCYellow(1, "Going to a ball");
							else if (iPreviousRobotState == RobotStateWander) dPrintLCYellow(1, "Wandering");
							
							//Rotate another direction when exiting obstacle avoidance, so that it does not enter it again
							if (iPreviousRobotState == RobotStateGoHome || iPreviousRobotState == RobotStateGoToStone) {
								iPreviousNavigationDirection = iPreviousAvoidanceDirection;
							}
							
							iPreviousAvoidanceDirection = DirectionUnknown;
							
							iRobotState = iPreviousRobotState;
							continue;
						}
					}
					else if (iRobotState == RobotStateWander) {
						
						if (cnt++ >= 60) {
							cnt = 0;
							//Stop before processing image.
							//Will set speed on the following iteration
							iInterface.iMotionCtrl->SetStop();
							ownSleep_ms(20);				
							analyzeCamera();
							continue;
						}
						
						r_acc = 0.1;
						r_wspeed = 0.0;
						r_speed = 0.15;
						
						//Don't change direction faster than 1s
						//if (ownTime_get_ms() - lastObstacleOccurance > 1000 && iPreviousAvoidanceDirection != DirectionUnknown) {
							iPreviousAvoidanceDirection = DirectionUnknown;
						//}
						iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
						ownSleep_ms(20);
						
					} else if (iRobotState == RobotStateGoHome || RobotStateGoToStone) {
						
						iPreviousAvoidanceDirection = DirectionUnknown;
						
					     ////Run A*
						if (!iHasPlan) {
						
							//Stop before thinking a lot
							iInterface.iMotionCtrl->SetStop();
							ownSleep_ms(20);
							
							updateMapGrid();
							
							//Bias the map because it contains negative values also
							//But AStar algorithm needs only non-negative values
							
							//Current robot position in grid coordinates
							TPoint posePoint;
							posePoint.x = pose->x, posePoint.y = pose->y;
							iRobotGridPoint = biasedPoint(posePoint);
							
							//Waypoint position in grid coordinates
							iWaypointGridPoint = biasedPoint(iNextWaypoint);
							
							dPrint(1, "On the map navigating from %d, %d to %d, %d", iRobotGridPoint.x,iRobotGridPoint.y,iWaypointGridPoint.x,iWaypointGridPoint.y);
						
							pathplan2 plan;
							iAstarPath = plan.get_graph(iMapGrid,MAP_COLS,MAP_ROWS,iRobotGridPoint.x,iRobotGridPoint.y,iWaypointGridPoint.x,iWaypointGridPoint.y);
																		
							iSmoothAstarPath = smooth(iAstarPath,WEIGHT_DATA,WEIGHT_SMOOTH,A_TOLERANCE);
							
							//Unbias nodes
							for (EACH_IN_i(iSmoothAstarPath)) {
								i->x -= MAP_COLS/2;
								i->y -= MAP_ROWS/2;
							}
							
							iHasPlan = true;
							
							//Start with node 1, because node 0 is current position;
							iNavigationStep = 1;
						}
			
		                if (iNavigationStep < (int)iSmoothAstarPath.size()-1) {
							       
							dPrint(1,"Driving to a intermediate waypoint %d of %d", iNavigationStep, iSmoothAstarPath.size());
							
							TPose2D next_stop = iSmoothAstarPath.at(iNavigationStep+1);
							float x_next_stop_meters = next_stop.x * X_RES;
							float y_next_stop_meters = next_stop.y * Y_RES;
							
							if (iMotionState == MotionStateIdle) { 
								dPrint(1,"Motion Control State: Idle");
							}
							else if (iMotionState == MotionStateDriving) { 
								dPrint(1,"Motion Control State: Driving");
							}
							else if (iMotionState == MotionStateTurning) { 
								dPrint(1,"Motion Control State: Turning");
							}
							
							dPrint(1, "Intermediate waypoint %f, %f. Now at %f, %f",x_next_stop_meters, y_next_stop_meters,pose->x,pose->y);
							
							float dx = x_next_stop_meters - pose->x;
							float dy = y_next_stop_meters - pose->y;
							
							float alpha = atan2(dy, dx)-pose->a;
		                    alpha = truncate(alpha);
							dPrint(1, "Angle error %f", alpha);
									
							double rho = sqrt(dx*dx + dy*dy);
							if (rho <= DIST_MARGIN) {
	                            iNavigationStep++; 
	                            continue;
							}
							
							switch (iMotionState) {
								case MotionStateDriving:
								{
									//If angle is to large, rotate at one place instead of driving
									if (fabs(alpha) > 1.3) {
										iInterface.iMotionCtrl->SetStop();
										ownSleep_ms(20);
										iMotionState = MotionStateTurning;
										continue;
									}
									
									iPreviousNavigationDirection = DirectionForward;
									r_speed = 0.05;
									r_wspeed = K_alpha * alpha;
									
									if (fabs(alpha) > M_PI_2) {
										r_speed *= -1;
									}
									
									if (r_speed >= 0.0 || r_speed == -0.0) {
										r_speed = MAX(MIN(r_speed, MAX_SPEED), MIN_SPEED);
									}
									else {
										r_speed = MIN(MAX(r_speed, -MAX_SPEED), -MIN_SPEED);	
									}									
									r_wspeed = MAX(MIN(r_wspeed, MAX_WSPEED), -MAX_WSPEED);
									           
									iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
									ownSleep_ms(20);
								}
								break;
								
								
								
								case MotionStateTurning:
								{
										    
								    r_speed = 0;                    
									r_wspeed = MAGIC_CNST*alpha;
									
									if ((iPreviousNavigationDirection == DirectionLeft && r_wspeed < 0) || (iPreviousNavigationDirection == DirectionRight && r_wspeed > 0)) {
										r_wspeed *= -1;
									}
									
									if (r_wspeed >= 0.0 || r_wspeed == -0.0) {
										iPreviousNavigationDirection = DirectionLeft;
										r_wspeed = MAX(MIN(r_wspeed, MAX_WSPEED), MIN_WSPEED);
									}
									else {
										iPreviousNavigationDirection = DirectionRight;
										r_wspeed = MIN(MAX(r_wspeed, -MAX_WSPEED), -MIN_WSPEED);	
									}
									
			                        if(fabs(alpha) < ANGLE_MARGIN) {
			                            iInterface.iMotionCtrl->SetStop();
			                            ownSleep_ms(20);
			                            iMotionState = MotionStateDriving;
										continue;
			                        } else {
			                            iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
			                            ownSleep_ms(20);
			                        }
								}
					            break;                
					                
				                case MotionStateIdle:
									iPreviousNavigationDirection = DirectionUnknown;
									iMotionState = MotionStateTurning;
								break;
								}
						}
						else {
							dPrintLCGreen(1,"Waypoint reached");
							iHasPlan = false;
							iMotionState = MotionStateIdle;
							if (iRobotState == RobotStateGoToStone) {
								dPrintLCYellow(1, "Closing gripper");
								iRobotState = RobotStateCloseGripper;
							}
							else if (iRobotState == RobotStateGoHome) {
								dPrintLCYellow(1, "Opening gripper");
								iRobotState = RobotStateOpenGripper;
							}
						}
					}
				}
			}  
        } else {
            dPrint(1,"No MotionCtrl available - Terminating MotionDemo thread.");
            break;
        }
        
        
        // 5. Increment iteration counter
        ++iterations;
        
        // 6. Round complete, we should be near an obstacle now, so enter
        // random again and Redo!
        //dPrint(1,"Round complete. Robot is stopped. %d iterations executed",
         //      iterations);
    }
      
      
      dPrint(3,"MotionDemoThread terminated.");
      return 0;
  }
}

//*****************************************************************************

int CJ2B2Demo::RunSensorsDemo(int aIterations)
{
  using namespace MaCI;
  using namespace MaCI::Ranging;
  int iterations = 0;


  TDistanceArray laserDistanceData;
  TDistanceArray bumperDistanceData;
  TDistanceHeader laserHeader;
  TDistanceHeader bumperHeader;
  Common::TTimestamp laserTimestamp;
  Common::TTimestamp bumperTimestamp;
  int bumperSeq = -1;
  int laserSeq = -1;

  // This is an example sequence only - it doesn't correctly use the
  // bumper and laser together, as there can be event on both
  // simultaneously.
  while(iDemoActive && (aIterations == -1 || iterations < aIterations)) {

    bool r;
    if (iInterface.iRangingBumpers) {
      r = iInterface.iRangingBumpers->GetDistanceArray(bumperDistanceData, 
                                                       &bumperHeader, &bumperTimestamp, &bumperSeq, 
                                                       0);
    } else {
      r = false;
      dPrint(8,"No iRangingBumpers available - skipping");

    }

    // Check status from bumpers
    if (r) {
      // If this branch is reached, there was a bumperevent available.
      for(EACH_IN_i(bumperDistanceData)) {
        if (i->distance >= 0.00) {
          dPrint(2,"Bumpers hit: %f meters @ %f rad", i->distance, i->angle);
          iSmallestDistanceToObject = *i;
        }
      }
    }
    
    // Waiting for Laser measure for maximum of 100ms. If you are
    // only reading data from Laser, you could make this longer -
    // too fast polling consumes processing time and gains nothing.
    
    if (iInterface.iRangingLaser) {
		 MaCI::Ranging::TDeviceInformationPosition laserPosition;
		iInterface.iRangingLaser->GetDevicePosition (laserPosition);
		iLidarPoint.y = laserPosition.x;
		
      r = iInterface.iRangingLaser->GetDistanceArray(laserDistanceData, 
                                                     &laserHeader, &laserTimestamp, &laserSeq, 
                                                     100);
      // Check laser status
      if (r) {
        // Copy distances
        Lock();
        iLastLaserDistanceArray = laserDistanceData;
        iLastLaserTimestamp = laserTimestamp;
        Unlock();
        
        runSLAM();
        
        // Check distances received.
        //
        // The "EACH_IN_i" is a macro using C++ iterators which operates
        // so that 'i' variable is set to point to each element in the
        // provided array in turn. When using it as
        // 'for(EACH_IN_i(<array>))' you can easily access all the
        // elements through the 'i' variable.
        iSmallestDistanceToObject.distance = 1000;
        iLaserScans.clear();
        for(EACH_IN_i(laserDistanceData)) {
          if (i->distance < iSmallestDistanceToObject.distance &&
              i->distance > 0) iSmallestDistanceToObject = *i;
          iLaserScans.push_back(robotPoint(i->distance, i->angle, iLidarPoint.y));
        }
      }
    } else {
      r = false;
      dPrint(8,"No iRangingLaser available - skipping (sleep 100ms)");
      ownSleep_ms(100);
    }
    
    // Increment
    ++iterations;
  }
  return 0;
}

//*****************************************************************************

int CJ2B2Demo::ThreadFunction(const int aThreadNumber)
{
  int result = 0;
  switch(aThreadNumber) {
  case KThreadSensorsDemo:
    result = RunSensorsDemo(-1);
    break;

  case KThreadInfoDemo:
    result = RunInfoDemo(-1);
    break;

  case KThreadMotionDemo:
    result = RunMotionDemo(-1);
    break;
    
  case KThreadCameraDemo:
    result = RunCameraDemo(-1);
    break;

  case KThreadSDLDemo:
    result = RunSDLDemo(-1);
    break;

  default:
    dPrint(1,"WARNING: Invalid thread Number specified! (No implementation!)");
    abort();
    break;
  }
  return result;
}
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************




void CJ2B2Demo::analyzeCamera()
{
	using namespace MaCI::Image;
	using namespace MaCI::Position;
	
	dPrint(1, "Analyzing camera image");
    if (iInterface.iImageCameraFront != NULL && iLastCameraImage.GetImageDataType() == KImageDataJPEG &&
        iLastCameraImage.GetImageDataPtr() != NULL) {
			
		CImageContainer container;
		int res = mkdir("vision", S_IRWXU|S_IRGRP|S_IXGRP);
		if (res == -1) {
			//Folder exists
		}
		else if (res == 0) {
			dPrint(1, "Folder 'vision' created. Check the camera images there");
		}
		ownTime_ms_t time = ownTime_get_ms();
		Lock();
		container.Copy(iLastCameraImage);
		Unlock();
		char mystr[255];
		sprintf(mystr, "vision/Image_%lld.jpg", time);
		container.WriteImageToFile(mystr);
		container.ConvertTo(KImageDataRGB);
		dPrint(1, "Converted JPG image to RGB raw");
		TImageInfo info = container.GetImageInfoRef();
		
		if (info.imagedatatype != KImageDataRGB) {
			dPrintLCRed(1,"Image data is not RGB!!!");
			return;
		}
		else {
			//int size = (int)container.GetImageDataSize();
			//dPrintLCYellow(1, "Image size %d", size);
			//for (int i = 0; i < 5; i++) {
				//dPrintLCYellow(1, "%X", (unsigned char)container.GetImageDataPtr()[i]);
			//}
		}
		
		// aplication with the camera code on Open CV
       World_frame frame;
       frame = Find_Object_file(mystr);
       if (frame.Alarm) {
		   dPrint(1, "Found a ball (c_x %f; c_y %f)", frame.x_rob_frame, frame.y_rob_frame);
		   float angle= 0, distance=0;
		   angle = atan2f(frame.x_rob_frame,frame.y_rob_frame);
		   distance = fabs(sqrtf(pow(frame.x_rob_frame,2)+pow(frame.y_rob_frame,2)));
		   dPrint(1, "Found a ball (distance %f; angle %f)", distance, angle);
		   jx = frame.centerX;
		   jy = frame.centerY; 
		   jr = frame.radius;
		   //In Universal frame
				CPositionData pd;
				
				if (iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastLaserTimestamp.GetGimTime())) {
					const TPose2D *pose1 = pd.GetPose2D();
					iNextWaypoint = worldPoint(distance, angle, iLidarPoint.y+CAM_LIDAR_DIST, pose1);
					dPrintLCYellow(1, "Going to a ball");
					iRobotState = RobotStateGoToStone;
		}else {
			dPrint(1, "Did not find anything");
		}
		}
      /*
		int search_color_flag = 1; //1 - red, 2 - blue
		Camera_Obstacle_Alarm answer = Find_Object((unsigned char *)container.GetImageDataPtr(), info.imagewidth, info.imageheight, search_color_flag);
		
		if (answer.Red_Target_Flag) {
			dPrint(1, "Found something red");
			Camera_Distance distance = Postion_Object(answer.c_x, answer.c_y, info.imagewidth, info.imageheight);
			dPrint(1, "Found a ball (distance %f; angle %f)", distance.distance, distance.angle);
			
			//Create a waypoint here
			//In Universal frame
			CPositionData pd;
			
			if (iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastLaserTimestamp.GetGimTime())) {
				const TPose2D *pose = pd.GetPose2D();
				iNextWaypoint = worldPoint(distance.distance, distance.angle, iLidarPoint.y+CAM_LIDAR_DIST, pose);
				dPrintLCYellow(1, "Going to a ball");
				iRobotState = RobotStateGoToStone;
			}
		}
		else {
			dPrint(1, "Did not find anything");
		}
		* */
	}
	else {
		//dPrint(1, "Image not available");
		
		//Assuming that we are in the simulator
		CPositionData pd;
		if (iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastLaserTimestamp.GetGimTime())) {
			const TPose2D *pose = pd.GetPose2D();
			dPrintLCRed(1, "ACHTUNG!!!! SIMULATED WAYPOINT!!!");
			iNextWaypoint.x = 1.14;//iBasePoint.x+sin(pose->a)*1.5;
			iNextWaypoint.y = -1.17;//iBasePoint.y+cos(pose->a)*0.5;
			dPrintLCYellow(1, "Going to a ball");
			iRobotState = RobotStateGoToStone;
		}
		return;
	}
}

void CJ2B2Demo::updateMapGrid()
{
	for (int i = GRID_WALL_DEPTH; i < MAP_ROWS-GRID_WALL_DEPTH; i++) {
		for (int j = GRID_WALL_DEPTH; j < MAP_COLS-GRID_WALL_DEPTH; j++) {
			int idx = i*MAP_COLS+j;
			bool set = false;
			for (EACH_IN_k(iMap)) {
				//Biased by half of the map
				TGridPoint obstacle = biasedPoint(*k);
				if (obstacle.x == j && obstacle.y == i) {
					set = true;
					//Set the cell and 16 adjacent cells around it as obstacles					
					for (int ii = i-GRID_WALL_DEPTH; ii <= i+GRID_WALL_DEPTH; ii++) {
						for (int jj = j-GRID_WALL_DEPTH; jj < j+GRID_WALL_DEPTH; jj++) {
							int idxx = ii*MAP_COLS+jj;
							iMapGrid[idxx] = 0;
						}
					}
					break;
				}
			}
			if (!set) {
				iMapGrid[idx] = 1;
			}
		}
	}
}

vector<MaCI::Position::TPose2D> CJ2B2Demo::smooth(vector<node> astar_path, float weight_data, float weight_smooth, float tolerance)
{
	using namespace MaCI::Position;
	
	vector<TPose2D> smooth_astar_path, original_astar_path;
	
	//Make a Copy of X and Y
	for(unsigned int i = 0; i < astar_path.size(); i++) {
		node originalNode = astar_path.at(i);
		TPose2D pose;
		pose.x = (float)originalNode.x;
		pose.y = (float)originalNode.y;
		smooth_astar_path.push_back(pose);
		original_astar_path.push_back(pose);
	}
	
	float change = tolerance;
	TPose2D aux_i, new_point, aux_i1, aux_i2, initial_XY;
	
	while(change >= tolerance) {
		change = 0.0;					
		for(int i = 1; i < (int)smooth_astar_path.size()-1; i++) {
			aux_i = smooth_astar_path.at(i);
			aux_i1 = smooth_astar_path.at(i-1);
			aux_i2 = smooth_astar_path.at(i+1);
			new_point = aux_i;
			initial_XY = original_astar_path.at(i);
			
			new_point.x += weight_data * (initial_XY.x - new_point.x);
			new_point.x += weight_smooth * ( ( aux_i1.x + aux_i2.x) - ( 2.0 * new_point.x) );
			change += fabs(aux_i.x - new_point.x);
			smooth_astar_path.at(i) = new_point;
			
			new_point.y += weight_data * (initial_XY.y - new_point.y);
			new_point.y += weight_smooth * ( ( aux_i1.y + aux_i2.y) - ( 2.0 * new_point.y) );
			change += fabs(aux_i.y - new_point.y);
			smooth_astar_path.at(i) = new_point;
		}
	}
	
	return smooth_astar_path;
}

void CJ2B2Demo::runSLAM()
{	
	using namespace MaCI::Position;
	using namespace MaCI;
	
	if (iInterface.iPositionOdometry) {
		if (iFirstSLAMAttempt) {
			//Get the readings and just save them without any SLAM
			
			//int posSeq = -1;
			//CPositionData pd;
			//if (iInterface.iPositionOdometry->GetPositionEvent(pd, &posSeq, 1000)) {
				//const TPose2D *pose = pd.GetPose2D();
				//if (pose) {
					//iRobotPose = gridPoseFromTPose(pose);
					iFirstSLAMAttempt = false;
					//dPrint(1, "Getting very first odometry %d,%d,%f", iRobotPose.x,iRobotPose.y,iRobotPose.angle);
					
					//vector<ISGridPoint> euclideanLaserData = getEuclideanLaserData();
					//updateMap(iRobotPose, euclideanLaserData);
					//iPreviousLaserData = euclideanLaserData;
				//}
			//}
			
			return;
		}
		
		//Get predicted position from odometry
		MaCI::Position::CPositionData pd;
		if (iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastOdometryTimestamp.GetGimTime())) {
			
			const TPose2D *pose1 = pd.GetPose2D();	
			//ISGridPose2D previousPose = gridPoseFromTPose(pose1);
			
			if (iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastLaserTimestamp.GetGimTime())) {
				const TPose2D *pose2 = pd.GetPose2D();
				
				
				if (fabs(pose1->a-pose2->a) < 0.00001) {
					updateMap(pose2, true);
					iLastOdometryTimestamp = iLastLaserTimestamp;
				}
				
				
				/*
				if (!iPauseOn) dPrint(1,"Previous time %f Current time %f", iLastOdometryTimestamp.GetGimTime().getTimeInSeconds(), iLastLaserTimestamp.GetGimTime().getTimeInSeconds());
				
				iPreviousOdometryPose = previousPose;
				iOdometryPose = gridPoseFromTPose(pose2);
				iPreviousRobotPose = iRobotPose;
				
				if (!iPauseOn) dPrint(1, "Previous odometry %d,%d,%f; Current odometry %d,%d,%f", iPreviousOdometryPose.x,iPreviousOdometryPose.y,iPreviousOdometryPose.angle, iOdometryPose.x,iOdometryPose.y,iOdometryPose.angle);
			
			
				ISGridPose2D poseDifference;
				poseDifference.x = iOdometryPose.x-iPreviousOdometryPose.x;
				poseDifference.y = iOdometryPose.y-iPreviousOdometryPose.y;
				poseDifference.angle = iOdometryPose.angle-iPreviousOdometryPose.angle;
				
				if ((poseDifference.x != 0 && poseDifference.y != 0) || poseDifference.angle > 0.01) {
					iLastOdometryTimestamp = iLastLaserTimestamp;
				}
				
				if (!iPauseOn) dPrint(1, "Difference %d,%d,%f", poseDifference.x,poseDifference.y,poseDifference.angle);
		
				ISGridPose2D predictedPose;
				predictedPose.x = iRobotPose.x+poseDifference.x;
				predictedPose.y = iRobotPose.y+poseDifference.y;
				predictedPose.angle = iRobotPose.angle+poseDifference.angle;
				
				if (!iPauseOn) dPrint(1, "Previous pose %d,%d,%f; Predicted pose %d,%d,%f", iPreviousRobotPose.x,iPreviousRobotPose.y,iPreviousRobotPose.angle, predictedPose.x,predictedPose.y,predictedPose.angle);
				
				vector<ISGridPoint> euclideanLaserData = getEuclideanLaserData();
			
				//Generate different poses around predicted position
				float maxRadius = 0.3;
				int numCircles = 3;
				int numPositionsInCircle = 8;
				float maxAngleDeviation = 0.03;
				int numAngles = 11;
				
				
				vector<ISGridPose2D> generatedPoses = generateGridPoses(predictedPose, 
																		maxRadius, 
																		numCircles,
																		numPositionsInCircle,
																		maxAngleDeviation,
																		numAngles,
																		X_RES,
																		Y_RES);
				  
				  
				  
				  
				  
				//Calculate transformed laser readings for each of the pose				
			    int minDifference = INT_MAX;
			    int minPoseDeviation = INT_MAX;
			    int index = -1;
				
				//if (!iPauseOn) dPrint(1,"Real Position (%f,%f,%f) diff  %f", predictedPose.x, predictedPose.y, predictedPose.angle, fabs(sumDifferences(scanDistances, scanDistances)));
			    for (unsigned int i = 0; i < generatedPoses.size(); i++) {
					ISGridPose2D generatedPose = generatedPoses.at(i);
					
					
					vector<ISGridPoint> transformedPoints = transformGridPoints(iPreviousRobotPose, generatedPose, iPreviousLaserData);
					//vector<ISGridPoint> transformedPoints = transformGridPoints(predictedPose, generatedPose, euclideanLaserData);
					int difference = sumGridDifferences(transformedPoints, euclideanLaserData, false);
					int poseDeviation = getPoseDifference(generatedPose, predictedPose);
					
					if (poseDeviation == 0) {
						if (!iPauseOn) dPrint(1,"Detected Predicted Pose (%d,%d,%f), diff  %d (pose diff %d)",generatedPose.x, generatedPose.y, generatedPose.angle, difference, poseDeviation);
					}
						
					
					//if (!iPauseOn) dPrint(1,"Pose (%d,%d,%f), diff  %d (pose d./J2B2-UI-example localhost 40022 FSRSim.J2B2iff %d)",generatedPose.x, generatedPose.y, generatedPose.angle, difference, poseDeviation);
					
					//Choose pose with smallest scans error and also the pose closest to the predicted one
					if ((difference < minDifference) || (difference == minDifference && poseDeviation < minPoseDeviation)) {
						index = i;
						minDifference = difference;
						minPoseDeviation = poseDeviation;
					}
				}
				
				
				//Correct current pose
				ISGridPose2D correctedPose = predictedPose; 
				if (index >= 0 && index < (int)generatedPoses.size()) {
					correctedPose = generatedPoses.at(index);
					
					if (!iPauseOn) dPrint(1,"Best pose [%d,%d,%f] Diff: %d, Dev: %d", correctedPose.x,correctedPose.y,correctedPose.angle, minDifference, minPoseDeviation);
					
					//Only if not rotating too much
					if (fabs(poseDifference.angle) < 0.001) {
						 updateMap(correctedPose, euclideanLaserData);
					}
				}
				
				iRobotPose = correctedPose;
				iPreviousLaserData = euclideanLaserData;
				
				 
		 */ 
			 }
		 }
	}
}

TPoint CJ2B2Demo::worldPoint(float distance, float angle, float y_peripheral_offset, const MaCI::Position::TPose2D *pose)
{
	TPoint point = robotPoint(distance, angle, y_peripheral_offset);
	
	//World coordinates
	return robotToWorldPoint(point, pose);
}

TPoint CJ2B2Demo::robotToWorldPoint(TPoint point, const MaCI::Position::TPose2D *pose)
{
	TPoint res;
	float worldAngle = pose->a+M_PI_2;
	res.x = point.x*cos(worldAngle)+point.y*sin(worldAngle)+pose->x;
	res.y = point.x*sin(worldAngle)-point.y*cos(worldAngle)+pose->y;
	return res;
}
	

TPoint CJ2B2Demo::robotPoint(float distance, float angle, float y_peripheral_offset)
{
	TPoint point;
	
	//Laser origin coordinates
	point.x = distance*sin(angle);
	point.y = distance*cos(angle);
	
	//Robot origin coordinates
	point.y += y_peripheral_offset;
	
	return point;
}

TPoint CJ2B2Demo::SDLPoint(TPoint point)
{
	TPoint center;
	center.x = 225;
	center.y = 225;
	
	TPoint res;
	res.x = center.x-(point.x-iBasePoint.x)*50; //50 - multiplier for euclidean coordinates,
	res.y = center.y+(point.y-iBasePoint.y)*50; //that is 1m is 50 px
	res.x = MIN(450, res.x);
	res.y = MIN(450, res.y);
	return res;
}

TGridPoint CJ2B2Demo::biasedPoint(TPoint point)
{
	TGridPoint res;
	res.x = round(point.x/X_RES)+MAP_COLS/2;
	res.y = round(point.y/Y_RES)+MAP_ROWS/2;
	return res;
}

void CJ2B2Demo::updateMap(const MaCI::Position::TPose2D *pose, bool eraseUntrustedPoints)
{
	TPoint lidarWorldPoint = robotToWorldPoint(iLidarPoint, pose);
	
	for(EACH_IN_i(iLaserScans)) {
		TPoint obstaclePoint = robotToWorldPoint(*i, pose);
		
		bool exists = false;
		for (vector<TPoint>::iterator iterator = iMap.begin(); iterator < iMap.end(); iterator++) {
			TPoint mapPoint = *iterator;
			if (fabs(obstaclePoint.x-mapPoint.x) < 0.01 && fabs(obstaclePoint.y-mapPoint.y) < 0.01) {
				exists = true;
				break;
			}
		}
		
		if (!exists) {
			Lock();
			iMap.push_back(obstaclePoint);
			
			if (eraseUntrustedPoints) {
				//If obstacle lies on the laser point, eliminate it
				for (vector<TPoint>::iterator iterator = iMap.begin(); iterator < iMap.end(); iterator++) {
					TPoint mapPoint = *iterator;
					if (fabs(obstaclePoint.x-lidarWorldPoint.x) > 0.00001) {
						float b = (obstaclePoint.x*lidarWorldPoint.y-lidarWorldPoint.x*obstaclePoint.y)/(obstaclePoint.x-lidarWorldPoint.x);
						float k = (lidarWorldPoint.y-b)/lidarWorldPoint.x;
						if ((k*mapPoint.x+b)-mapPoint.y < 0.000001 &&
						   ((mapPoint.x-lidarWorldPoint.x > 0.01 && mapPoint.x-obstaclePoint.x < -0.01) || 
						    (mapPoint.x-lidarWorldPoint.x < -0.01 && mapPoint.x-obstaclePoint.x > 0.01)) &&
						   ((mapPoint.y-lidarWorldPoint.y > 0.01 && mapPoint.y-obstaclePoint.y < -0.01) ||
						    (mapPoint.y-lidarWorldPoint.y < -0.01 && mapPoint.y-obstaclePoint.y > 0.01))
						 ) {
							iMap.erase(iterator);
						}
						//float obstacleDis=sqrt(obstaclePoint.x*obstaclePoint.x + obstaclePoint.y*obstaclePoint.y);
						//float mapPointDis=sqrt(mapPoint.x*mapPoint.x + mapPoint.y*mapPoint.y);
						//float lidarPointDis=sqrt(lidarWorldPoint.x*lidarWorldPoint.x + lidarWorldPoint.y*lidarWorldPoint.y);
						//float mapPointAng= atan2(mapPoint.y,mapPoint.x);
						//float lidarPointAng=atan2(lidarWorldPoint.y,lidarWorldPoint.x);
						//if ((k*mapPoint.x+b)-mapPoint.y < 0.0001 && 
						    //(( obstacleDis  <  mapPointDis) && (mapPointDis < lidarPointDis )) &&
						    //( fabs( mapPointAng - lidarPointAng)<0.0001)
						    //){
						    //iMap.erase(iterator); 
						//}						
						
						
					}
				}
			}
			
			Unlock();
		}
		
	}
}
