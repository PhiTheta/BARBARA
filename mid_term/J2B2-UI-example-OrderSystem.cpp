/**
 * \file
 * \brief Example of simple operation using the J2B2 Client interface.
 * \author Antti Maula <antti.maula@tkk.fi>
 */

#include "J2B2-API.hpp"
#include "J2B2Demo.hpp"
#include "owndebug.h"
#include <stdio.h>

#include "FSR2011OrderSystemClient.hpp"

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


  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// OrderSystem Setup ////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  bool r;
  using namespace FSR2011OrderSystem;
  
  // Instantiate the Client
  CFSR2011OrderSystemClient osc;

  // Connect the Client to service.
  if (!osc.Connect(argv[1], atoi(argv[2]), "FSR2011OrderSystemServer")) {
    dPrintLCRed(ODERROR,"FATAL ERROR; Failed to connect to service!");
    exit(1);
  }
  
  // Test Ping()
  uint32_t diff = 0;
  r = osc.Ping(diff);
  assert(r == true);
  dPrint(ODTEST,"Ping to OrderSystemServer took: %u microseconds", diff);

  // Test GetOrderList()
  dPrint(ODTEST,"Requesting Orders...");
  TOrderList ol;
  r = osc.GetOrderList(ol);
  if (r) {
    dPrint(ODTEST,"OrderList received, contains %u orders", ol.size());
    for(EACH_IN_i(ol)) {
      printf("Order %u, items: [ ", i->order_id);
      for(EACH_IN_j(i->items)) {
        printf("%uxP%u ", j->item_count, j->slot_id); 
      }
      printf("], delivered to D%u", i->delivery_slot);
      printf("\n");
    }
  }
  dPrint(ODTEST,"OK. (Orders processed)");


  // Test Reporting (Multiple Report* functions)
  // Iterate over every item in 'ol', having each item in variable 'i' at a time...
  for(EACH_IN_i(ol)) {
    // Report beginning of an order
    osc.ReportBeginOrder(*i);

    // Iterate over every item in 'items' of an order... (Item in 'j')
    for(EACH_IN_j(i->items)) {
      // Report pickup begin
      osc.ReportBeginPickup(*i, *j);

      // Report pickup end
      osc.ReportEndPickup(*i, *j);
    }
    
    // Report end of an order.
    osc.ReportEndOrder(*i);
  }



  // Guess.
  printf("\nExit.\n\n");
  
  return 0;
}
//*****************************************************************************
