/**
 * \file
 * \brief Header of J2B2 Demo class.
 * \author Antti Maula <antti.maula@tkk.fi>
 */
#ifndef _J2B2_DEMO_HPP_
#define _J2B2_DEMO_HPP_

#include "thread.hpp"
#include "sync.hpp"
#include "J2B2-API.hpp"
#include "../simpleSLAM/slam.h"
#include "astar/pathplan2.h"

#define MAP_WIDTH		4.5
#define MAP_HEIGHT		3.7
#define MAP_ROWS		(37*3)
#define MAP_COLS		(45*3)
#define X_RES			(MAP_WIDTH/MAP_COLS)
#define Y_RES			(MAP_HEIGHT/MAP_ROWS)

class CJ2B2Demo : private gim::CSync, 
                  private gim::CThread
{
public:
  CJ2B2Demo(CJ2B2Client &aClient);
  ~CJ2B2Demo();

  /** Execute hard-co(re|ded) demonstration.
   */
  void Execute(void);

  
  /** Terminate demonstration.
   */
  void Terminate(void);

private:
  /** Types describing current thread handlers.
   */
  enum EThreadImplementation { 
    KThreadSensorsDemo = 0,   ///< Sensors demo
    KThreadInfoDemo    = 1,   ///< Info demo
    KThreadMotionDemo  = 2,   ///< Motion demo
    KThreadCameraDemo  = 3,   ///< Camera demo
    KThreadSDLDemo     = 4    ///< SDL demo
  };


  /** Types describing event types.
   */
  enum ESensorEvent {
    KSensorEventAllClear = 0,       ///< No active sensor events.
    KSensorEventForwardBlocked = 1  ///< Forward dir. blocked.
  };

  CJ2B2Client &iInterface;
  int ThreadFunction(const int aThreadNumber); ///< Incoming data handler function
  int RunSensorsDemo(int aIterations = 1);
  int RunInfoDemo(int aIterations = 1);
  int RunCameraDemo(int aIterations = 1);
  int RunMotionDemo(int aIterations = 1);
  int RunSDLDemo(int aIterations = 1);

  volatile bool iDemoActive;
  volatile bool iPTUDemoActive;

  volatile bool iSensorsThreadActive;
  volatile bool iInfoThreadActive;
  volatile bool iMotionThreadActive;
  volatile bool iCameraThreadActive;
  volatile bool iSDLThreadActive;

  ESensorEvent iCurrentSensorEvent;
  MaCI::Ranging::TDistance iSmallestDistanceToObject;

  MaCI::Image::CImageContainer iLastCameraImage;
  MaCI::Ranging::TDistanceArray iLastLaserDistanceArray;
  MaCI::Common::TTimestamp iLastLaserTimestamp;
  MaCI::Common::TTimestamp iLastOdometryTimestamp;
  MaCI::Ranging::TDeviceInformationPosition iLaserPosition;
  bool iFirstSLAMAttempt;
  ISGridPose2D iRobotPose;
  ISGridPose2D iPreviousRobotPose;
  ISGridPose2D iOdometryPose;
  ISGridPose2D iPreviousOdometryPose;
  vector<ISGridPoint> iGridMap;
  vector<ISGridPoint> iPreviousLaserData;
  vector<ISGridPose2D> iSmoothAstarPath;
  vector<node> iAstarPath;
  ISGridPoint iNextWaypoint;
  volatile bool iPauseOn;
  volatile bool iHasPlan;
  int iIter;
  void updateMap(ISGridPose2D pose, vector<ISGridPoint> scans);
  void runSLAM();
  ISGridPose2D gridPoseFromTPose(const MaCI::Position::TPose2D *pose);
  vector<ISGridPose2D> smooth(vector<node> astar_path, float weight_data, float weight_smooth, float tolerance);
  vector<ISGridPoint> getEuclideanLaserData();
  void mapFromGridMap(vector<ISGridPoint> map, int **output);
};

#endif
