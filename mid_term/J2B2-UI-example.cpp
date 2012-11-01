/**
 * \file
 * \brief Example of simple operation using the J2B2 Client interface.
 * \author Antti Maula <antti.maula@tkk.fi>
 */

#include "J2B2-API.hpp"
#include "J2B2Demo.hpp"
#include "owndebug.h"
#include <stdio.h>

int main(int argc, char *argv[])
{
  // Initialize Debugging library to see the dPrint():s
  debugInit();
  debugSetGlobalDebugLvl(1);

  // Construct instance of J2B2 Client
  CJ2B2Client j2b2;
  std::string machineGroup = "J2B2";

  // Just print.
  printf("\nJ2B2-UI-example - v1.1\n\n");

  // Check that required number of parameters are present, otherwise
  // print usage and exit.
  if (argc < 3 ) {
    printf("Usage:\n%s <GIMnetAP address> <GIMnetAP port> [Machine group]\n\n", argv[0]);
    exit(1);
  }

  // Open the client, store the return result. (Use command line
  // positional args directly)
  const bool open_result = j2b2.Open(argv[1], atoi(argv[2]), "", argc>3?argv[3]:"J2B2");

  // Check the Open() result
  if (open_result) {
    printf("J2B2 Open() was succesfull!\n\n");

  } else {
    printf("J2B2 Open() failed!\n\n");

  }

  // Check presence of each interface
  printf("CameraFront\t %spresent!\n", j2b2.iImageCameraFront?"":"NOT "); 
  printf("ServoCtrl\t %spresent!\n", j2b2.iServoCtrl?"":"NOT "); 
  printf("MotionCtrl\t %spresent!\n", j2b2.iMotionCtrl?"":"NOT ");
  printf("PositionOdometry %spresent!\n", j2b2.iPositionOdometry?"":"NOT ");
  printf("BehaviourCtrl\t %spresent!\n", j2b2.iBehaviourCtrl?"":"NOT "); 
  printf("RangingLaser\t %spresent!\n", j2b2.iRangingLaser?"":"NOT ");
  printf("RangingBumpers\t %spresent!\n", j2b2.iRangingBumpers?"":"NOT ");
  printf("IOBoardAD5414\t %spresent!\n", j2b2.iIOBoardAD5414?"":"NOT "); 
  printf("IOBoardESC\t %spresent!\n", j2b2.iIOBoardESC?"":"NOT "); 
  printf("TextToSpeech\t %spresent!\n", j2b2.iTextToSpeech?"":"NOT ");


  // Check for minimum setup for the Demo to work at some
  // level. Remember, that if some services are missing, the features
  // they provide will not be usable in the demo. For example, without
  // camera - no image is displayed. Also, these work as a test
  // whether _NOTHING_ was found. - in which case there is no sense to
  // run the demo.
  if (j2b2.iServoCtrl && j2b2.iMotionCtrl && 
      j2b2.iPositionOdometry &&
      j2b2.iBehaviourCtrl) {
    
    // Call demo :)
    CJ2B2Demo demo(j2b2);
    demo.Execute();
    

    // Not the cleanest way to wait. Using a signal handler (Ctrl-C
    // catcher) is recommended, but left out here to keep the code
    // more readable)
    while(1) {
      dPrint(8,"Still waiting...");
      ownSleep_ms(5000);
    }
    
    // Terminate the demostration. This will attempt to gracefully close
    // and terminate all the demonstration threads before returning
    // executing to this module.
    demo.Terminate();

  } else {
    dPrint(ODTEST,"");

  } 

  // Guess.
  printf("\nExit.\n\n");
  
  return 0;
}
//*****************************************************************************
