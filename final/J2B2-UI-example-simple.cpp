/**
 * \file
 * \brief Very simple example for using the J2B2 API
 * \author Antti Maula <antti.maula@tkk.fi>
 */

#include "J2B2-API.hpp"
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
  printf("\nJ2B2-UI-example-simple - v1.1\n\n");

  // Check that required number of parameters are present, otherwise
  // print usage and exit.
  if (argc < 3 ) {
    printf("Usage:\n%s <GIMnetAP address> <GIMnetAP port> [Machine group]\n", argv[0]);
    printf("(Machine group should be 'FSRSim.J2B2' for the simulator, and 'J2B2' for the real robot\n\n");
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

  // Check and print presence of each interface
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


  /////////////////////////////////////////////////////////////////////////////
  // At this point, perform any checks for required interfaces, and
  // dispatch control to your own application.

  
  // Print notification that the application is about to exit.
  printf("\nExit.\n\n");
  
  return 0;
}
