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
//#include "astar/pathplan2.h"

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

/* Steps to take in the Matrix
30,27---Start
35,27
35,11
35,5
21,5
21,14
25,14
15,6
9,6
5,6
5,23
15,23
22,23
22,27
30,27---End


*/
struct pose2D {
	int x;
	int y;
	float theta;
};

pose2D iPose;
pose2D wayPoint;

int iPreviousDirection = 0;

int w = 45;
int h = 37;
float real_w = 4.5;
float real_h = 3.7;
bool has_plan = false;
unsigned int step = 0;
//vector<node> path;
int iMap[] = {	  
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,
    0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

  
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
    iLastLaserDistanceArray()
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
    
    iPreviousDirection = 0;
    iPose.x = 30;
    iPose.y = 27;
    iPose.theta = 0;
    
    wayPoint.x = 36;
    wayPoint.y = 12;
    wayPoint.theta = 270;
    
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
  int iterations = 0;
  int posSeq = -1;
  ownTime_ms_t tbegin = ownTime_get_ms();
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

  // This prints Odometry output every 5 seconds.
  if (iInterface.iPositionOdometry) {
    while(iDemoActive && 
          iInfoThreadActive && 
          (aIterations == -1 || iterations < aIterations)) {
      CPositionData pd;
      //if (iInterface.iPositionOdometry->GetPositionEvent(pd,
                                                     //&posSeq,
                                                     //1000)) {
        //const TPose2D *pose = pd.GetPose2D();
        //if (pose && ownTime_get_ms_since(tbegin) > 5000) {
          //dPrint(1,"Odometry position now at: x:%f, y:%f, a: %frad",
                 //pose->x, pose->y, pose->a);
          //tbegin = ownTime_get_ms();
        //}
      //} else {
        //dPrint(1,"No event available!");
        
      //}
      
      ++iterations;
    }
  } else {
    dPrint(1,"No PositionOdometry available. Nothing to do. Abort Infothread.");

  }
  
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
      dPrint(ODTEST,"SerialLink connection not available.");
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
        dPrint(1,"'Window close' clicked.");
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
          // Perform simple robot control (just set the current values
          /////////////////////////////////////////////////////////////
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
            CThread::WaitThread(KThreadMotionDemo);
            dPrint(1,"Terminated.");

          } else {
            dPrint(1,"Executing MotionDemo..");
            CThread::RunThread(KThreadMotionDemo);

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

            // Now, Wait for Read event, maximum of 500ms more
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
    if (iPTUDemoActive == false &&
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

    // Now, draw the last received Image.
    if (iLastCameraImage.GetImageDataType() == MaCI::Image::KImageDataJPEG &&
        iLastCameraImage.GetImageDataPtr() != NULL) {
      
      /////////////////////////////////////////////////////////////////////////////
      // Draw the last imageimage! (Don't care whether its new or not)
      /////////////////////////////////////////////////////////////////////////////
      SDL_Surface *image;
      SDL_RWops *rw;
      SDL_Rect rcDest = {0,0,0,0};
      
      rw = SDL_RWFromMem((void*)iLastCameraImage.GetImageDataPtr(), 
                         iLastCameraImage.GetImageDataSize());
      image = IMG_LoadJPG_RW(rw);
      SDL_BlitSurface(image, NULL, screen, &rcDest);
      SDL_FreeSurface(image);
    }
    
    // Draw the LaserScan
    if (iLastLaserDistanceArray.size()) {
      float min_d = 1000;
      float scale = 160; ///< scales from meters to screen pixels
      int min_x_end = 0;
      int min_y_end = 0;
      const int x_origin = window_width/2; ///< origin of laser measurement
      const int y_origin = window_height; ///< origim of laser meas
      const int dasize = iLastLaserDistanceArray.size();
      for(int i = 0; i < dasize; ++i) {
        const MaCI::Ranging::TDistance &measurement = iLastLaserDistanceArray[i];
        int pix_x = x_origin - (int)(scale * measurement.distance * sin(measurement.angle)); 
        int pix_y = y_origin - (int)(scale * measurement.distance * cos(measurement.angle)); 
        if (pix_x >= 0 && pix_x < window_width && 
            pix_y >= 0 && pix_y < window_height) {
          pixelRGBA(screen, pix_x, pix_y, 255, 0, 0, 150);
          if (measurement.distance < min_d) {
            min_d = measurement.distance;
            min_x_end = pix_x;
            min_y_end = pix_y;
          }
        }
      }
      filledCircleRGBA(screen, x_origin, y_origin,
                       (int)(0.2 * scale), 0, 255, 255, 255);
      
      //Example for drawing a line using SDL: (The -1 in X origin is
      //to lift the line above border, so lines drawn directly towards
      //X axis would be visible too)
      lineRGBA(screen,
               x_origin, 
               y_origin-1,
               min_x_end,
               min_y_end,
               255, 255, 0, 255);
      
    }
    
    // Print help
#define HELPSTRCOUNT 7
    const char helpstr[HELPSTRCOUNT][60] = { 
      "m - Toggle MotionDemo",
      "p - Toggle PTU demo",
      "e - Enable Emergency actions",
      "r - Disable Emergency actions",
      "1,0 - Set all IO outputs on ESC to HIGH/LOW",
      "w,a,s,d - Control PTU manually (PTUDemo must be OFF)",
      "arrows - Drive around manually (MotionDemo must be OFF)",
    };
    int help_i = 0;
    int text_x = 650;
    int text_y = 5;
    const int text_y_delta = 10;

    for(help_i = 0; 
        help_i < HELPSTRCOUNT;
        text_y += text_y_delta, ++help_i) {
      stringRGBA(screen, 
                 text_x, text_y, 
                 helpstr[help_i], 
                 255, 0, 0, 150);
    }

    // print status reports
    text_y += text_y_delta;
    char diststr[255];
    sprintf(diststr, "Smallest distance to object: %.2fm at %.3f rad", 
            iSmallestDistanceToObject.distance, 
            iSmallestDistanceToObject.angle);
    
    stringRGBA(screen, 
               text_x, text_y, 
               diststr,
               0, 255, 255, 150);
    text_y += text_y_delta;

    if (iInterface.iIOBoardESC != NULL) {
      int m = iInterface.iIOBoardESC->GetDigitalMask(0, 1000);
      char bitstr[255];
      sprintf(bitstr, "IO port state now: %08x", m);
      
      stringRGBA(screen, 
                 text_x, text_y, 
                 bitstr,
                 0, 255, 255, 150);
      text_y += text_y_delta;
    }
    
    if (iMotionThreadActive) {
      stringRGBA(screen, 
                 text_x, text_y, 
                 "MotionDemo is active!", 
                 0, 255, 0, 150);
      text_y += text_y_delta;
    }
    if (iPTUDemoActive) {
      stringRGBA(screen, 
                 text_x, text_y, 
                 "PTUDemo is active!", 
                 0, 255, 0, 150);
      text_y += text_y_delta;
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
  const float angspeed = M_PI / 4.0;
  const unsigned int turn_duration = 5000;
  bool dirLeft = true;
  ownTime_ms_t tbegin;
  
  //variables midterm
  float r_acc = 0.2;
  float r_speed = 0.0;
  float r_wspeed = 0.5;
  int posSeq = -1;
  step = 0;
  
  float x_present;
  float y_present;
  float a_present, a_present_abs;
  float a_next, a_present_2pi;
  
  float x_next ;
  float y_next ;
  int state_r=0; 
  bool check_flag = false, motion_flag = false; // for debugging 
  
  //Control Loop from Lecture Slides
  float rho=1, alpha, beta;
  float delta_x=0.1, delta_y=0.1;
  float K_rho= 0.3, K_alpha=0.7, K_beta=(-0.15);
  //float K_rho= 1, K_alpha=2.66, K_beta=(-0.5);

  if (iMotionThreadActive) {
    dPrint(1,"MotionDemo already active! Will not start again.");
    return -1;
    
  } else {
    dPrint(3,"MotionDemoThread starting...");
    iMotionThreadActive = true;

   

	iInterface.iBehaviourCtrl->SetStart();

	int iterations = 0;
	while(iDemoActive && iMotionThreadActive && (aIterations == -1 || iterations < aIterations)) {
    
		// Got MotionCtrl?
		if (iInterface.iMotionCtrl) {
		
			dPrint(1,"x next is: %f",x_next);
			dPrint(1,"x present is: %f",x_present);
			dPrint(1,"y next is: %f",y_next);
			dPrint(1,"y present is: %f",y_present);
			dPrint(1,"delta_x is: %f",delta_x);       
			dPrint(1,"delta_y is: %f",delta_y);
			dPrint(1,"rho is: %f",rho);
			dPrint(1,"alpha is: %f",alpha);
			dPrint(1,"beta is: %f",beta);
			dPrint(1,"a_present is: %f",a_present);
			//dPrint(1,"a_prsent_2pi %f",a_present_2pi);
			dPrint(1,"r_wspeed is: %f",r_wspeed);
			dPrint(1,"r_speed is: %f",r_speed);
			dPrint(1,"r_acc is: %f",r_acc);
			dPrint(1,"a_next is: %f",a_next);
			dPrint(1,"state_r is: %d",state_r);
			dPrint(1,"Step: %d", step);
			//dPrint(1,"Check Flag: %d", check_flag);
			//dPrint(1,"Motion Flag: %d ", motion_flag);
			//dPrint(1,"Has Plan Flag: %d", has_plan);
        
      
////////////////IVAN'S MID TERM CODE//////////////////////////////////////////////

		//Run A*
	
			//if (!has_plan) {
				//pathplan2 plan;
				//path = plan.get_graph(iMap,w,h,iPose.x,iPose.y,wayPoint.x,wayPoint.y);
				//for(unsigned int i = 0; i < path.size(); i++) {
					//node aaa = path.at(i);
					//dPrint(1, "x: %d y: %d F: %f G: %f H: %f parentx: %d parenty: %d", aaa.x, aaa.y,  aaa.F, aaa.G, aaa.H, aaa.px, aaa.py);			
				//}
				//has_plan = true;
				//step =0;
			//}
		
			
			if (iInterface.iPositionOdometry) {
			
				state_r = 1;
				//if ( (step) < path.size()) {
				if ( (step) <= 14) {	
				//state_r = 7;
					MaCI::Position::CPositionData pd;
					iInterface.iPositionOdometry->GetPositionEvent(pd, &posSeq, 1000);
					const TPose2D *pose = pd.GetPose2D();
				// Need EKF
					//node plan = path.at(step);
					//node next_stp = path.at(step+1);
					
				
					x_present = pose->x;
					y_present = pose->y;
					a_present = pose->a;
					dPrint(1,"Update Pose");
					
					if(step == 0){
						x_next = 3.6;
						y_next = 2.7;
						a_next = 3*(M_PI/2);
					}else if(step == 1){
						x_next = 3.6;
						y_next = 1.5;
						a_next = 3*(M_PI/2);				
					}else if(step == 2){
						x_next = 3.6;
						y_next = 0.5;
						a_next = (M_PI);				
					}else if(step == 3){
						x_next = 2.7;
						y_next = 0.5;
						a_next = (M_PI/2);			
					}else if(step == 4){
						x_next = 2.7;
						y_next = 1.4;
						a_next = (M_PI);				
					}else if(step == 5){
						x_next = 2.0;
						y_next = 1.4;
						a_next = (M_PI);				
					}else if(step == 6){
						x_next = 1.5;
						y_next = 1.4;
						a_next = 3*(M_PI/2);				
					}else if(step == 7){
						x_next = 1.5;
						y_next = 0.65;
						a_next = (M_PI);				
					}else if(step == 8){
						x_next = 1.0;
						y_next = 0.65;
						a_next = (M_PI);				
					}else if(step == 9){
						x_next = 0.5;
						y_next = 0.65;
						a_next = (M_PI/2);				
					}else if(step == 10){
						x_next = 0.9;
						y_next = 2.4;
						a_next = (0);				
					}else if(step == 11){
						x_next = 1.6;
						y_next = 2.4;
						a_next = (0);				
					}else if(step == 12){
						x_next = 2.4;
						y_next = 2.4;
						a_next = (M_PI/2);				
					}else if(step == 13){
						x_next = 2.4;
						y_next = 2.7;
						a_next = (0);				
					}else if(step == 14){
						x_next = 3;
						y_next = 2.7;
						a_next = (0);				
					}		
						
					
					//A Star Starts
					//x_next = next_stp.x;
					//y_next = next_stp.y;
				
					////Set Up a_next
					
					//if( plan.x == next_stp.x  ){
						//if((next_stp.y - plan.y) > 0){	
								//a_next = (M_PI/2);
						//} else if((next_stp.y - plan.y) < 0){
								//a_next = ((3*M_PI/2));
						//}
					//}
					
					//if( plan.y == next_stp.y  ){
						//if((next_stp.x - plan.x) > 0){	
								//a_next = 0;
						//} else if((next_stp.x - plan.x) < 0){
								//a_next = M_PI;							
						//}
					//}
									
					//x_next = (next_stp.x * real_w)/w;
					//y_next = (next_stp.y * real_h)/h;						
					
					//A Star Ends
					
						
					delta_x = (x_next - x_present);
					delta_y = (y_next - y_present);	
					
					rho = sqrt(delta_x*delta_x + delta_y*delta_y);
					alpha = atan2(delta_y, delta_x)-a_present;
				
					if(alpha > M_PI){
						
						alpha = 2*M_PI - alpha;
						alpha = -1 * alpha;
					}
				
					if(alpha <= -1 * M_PI){
					
						alpha = 2*M_PI + alpha;
					}
				
				
				
					beta = -1*(a_present + alpha);
				
					if(beta > M_PI){
					
						beta = 2*M_PI - beta;
						beta = -1 * beta;
					}
				
					if(beta < -1 * M_PI){
						
						beta = 2*M_PI + beta;
					}
				
					if( (alpha>=(-1*(M_PI/2))) && (alpha<=(M_PI/2)) ){
					
						r_speed = K_rho * rho;
						r_wspeed = K_alpha * alpha + K_beta * beta;	
					}
				
					else if( ( (alpha>(-1*M_PI)) && (alpha<(-1*(M_PI/2))) ) || ( (alpha>(M_PI/2)) && (alpha<=(M_PI)) ) ){
					
						r_speed = -1* K_rho * rho;
						r_wspeed = K_alpha * alpha + K_beta * beta;	
					}
					
				
				
					if(r_speed >= 0.2){
					
						r_speed = 0.2;
					
					}
				
					if(r_wspeed >= 0.4){
					
						r_wspeed = 0.4;
					
					}
				
					if(r_wspeed <= -0.4){
					
						r_wspeed = -0.4;
					
					}
				
				
				
				//if( (r_wspeed < 0.03) && (r_wspeed >= 0) ) {
					
					//r_wspeed = 0.035;
				//}
				
				//if( (r_wspeed > -0.03) && (r_wspeed < 0) ) {
					
					//r_wspeed = 0.035;
				//}
				
				
				
					if( (rho <= 0.01) ) {	
						iInterface.iMotionCtrl->SetStop();
						ownSleep_ms(MIN(200,ownTime_get_ms_left(turn_duration, tbegin)));	
						state_r = 2;
						dPrint(1,"state_r is: %d",state_r);
						r_wspeed = 0.5;
						r_speed = 0;
						
						if(a_present<0){
								a_present_abs = -1 * a_present;
						}
								
						if((a_present == -0.00 ) || (a_present == 2 * M_PI)){
								
								a_present_abs = 0;
						}
					    dPrint(1,"a_present Absolute calculated checking, to do a_next");
					    
					    
						if(a_next == (M_PI/2)){
						
							if( (a_present_abs >= (M_PI/2)) && (a_present_abs <= M_PI) ){
					
								r_wspeed = -1 * r_wspeed;
								state_r=11;
							}else{
								r_wspeed = r_wspeed;
								state_r=12;
							}
						}else if( a_next == (3*(M_PI/2)) ){
					
							if( (a_present_abs > (M_PI/2)) && ((a_present_abs) < (M_PI) ) ){
					
								r_wspeed = -1 * r_wspeed;
								
								state_r=13;
								
								}else{
					
								r_wspeed = r_wspeed; 
								state_r=14;
							}
						}else if( a_next == M_PI ){
					
								if((a_present > 0)) {
					
									r_wspeed = r_wspeed;					
									state_r=15;
					
								}else if((a_present < 0)) {
					
									r_wspeed = -1 * r_wspeed;
									state_r=16;
								}
						}else if( a_next == 0 ){
					
								if((a_present > 0)) {
					
								r_wspeed = -1 * r_wspeed;
								state_r=17;
								}else if((a_present < 0)) {
					
								r_wspeed = r_wspeed;
								state_r=18;
							
								}
						}
						dPrint(1,"state_r is: %d",state_r);
					    dPrint(1,"r_wspeed set to: %f",r_wspeed);
					    
						if(a_present <= 0.00){
							a_present_2pi = a_present + (2 * M_PI);
						}else if(a_present>0.00){
								a_present_2pi = a_present;
						}
		
						if((a_present == -0.00 ) || (a_present == 2 * M_PI)){
								a_present_2pi = 0;
						}
					
						if( (a_present_2pi >= (a_next-0.1) ) && (a_present_2pi <= (a_next+0.1)) ){
						
							iInterface.iMotionCtrl->SetStop();
							ownSleep_ms(MIN(200,ownTime_get_ms_left(turn_duration, tbegin)));
							state_r = 19;
							step++;						
						
						}else{
						
							iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
							ownSleep_ms(MIN(200,ownTime_get_ms_left(turn_duration, tbegin)));					
							state_r = 20;
						}
					
										
											
					}else{
						iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
						ownSleep_ms(MIN(200,ownTime_get_ms_left(turn_duration, tbegin)));	
						state_r = 3;					
					}
				}else{
					//has_plan = false;
					step = 0;
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
			dPrint(1,"Round complete. Robot is stopped. %d iterations executed",
            iterations);
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
  const float proximityAlertLimit = 0.30;
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
      r = iInterface.iRangingLaser->GetDistanceArray(laserDistanceData, 
                                                     &laserHeader, &laserTimestamp, &laserSeq, 
                                                     100);
      // Check laser status
      if (r) {
        // Copy distances
        Lock();
        iLastLaserDistanceArray = laserDistanceData;
        Unlock();
        
        // Check distances received.
        //
        // The "EACH_IN_i" is a macro using C++ iterators which operates
        // so that 'i' variable is set to point to each element in the
        // provided array in turn. When using it as
        // 'for(EACH_IN_i(<array>))' you can easily access all the
        // elements through the 'i' variable.
        iSmallestDistanceToObject.distance = 1000;
        for(EACH_IN_i(laserDistanceData)) {
          if (i->distance < iSmallestDistanceToObject.distance &&
              i->distance > 0) iSmallestDistanceToObject = *i;

          if (i->distance <= proximityAlertLimit && i->distance >= 0) {
            // Proximity alert.
            dPrint(3,"Proximity alert at direction %.3frad CCW from fw direction, %.3f meters free!",
                   i->angle,
                   i->distance);
          }
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
