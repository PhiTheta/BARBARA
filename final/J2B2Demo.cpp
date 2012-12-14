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

using namespace std;
bool skip_window=false;

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


SDL_Surface *mainScreen = NULL;



int iPreviousDirection = 0;

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

ISGridPose2D CJ2B2Demo::gridPoseFromTPose(const MaCI::Position::TPose2D *pose)
{
	ISGridPose2D res;
	//double xind = pose->x;
	//double yind = pose->y;
	//xind /= (X_RES*2);
	//yind /= (Y_RES*2);
	//res.x = round(xind);
	//res.y = round(yind);
	res.x = round(pose->x/X_RES);
	res.y = round(pose->y/Y_RES);
	res.angle = (double)pose->a;
	
	//if (!iPauseOn) dPrint(1,"%f,%f -> %f,%f -> %d,%d [%f,%f]", pose->x, pose->y, xind, yind, res.x, res.y, X_RES, Y_RES);
	
	return res;
}

void CJ2B2Demo::analyzeCamera()
{
	dPrintLCRed(1, "ACHTUNG!!!! SIMULATED WAYPOINT!!!");
	iNextWaypoint.x = 1.5;
	iNextWaypoint.y = 0;
	iRobotState = RobotStateGoToStone;
	return;
	
	
	dPrint(1, "Analyzing camera image");
    if (iLastCameraImage.GetImageDataType() == MaCI::Image::KImageDataJPEG &&
        iLastCameraImage.GetImageDataPtr() != NULL) {
		SDL_Surface *surface = NULL;
		SDL_RWops *rw = NULL;
		
		MaCI::Image::CImageContainer container;
		iLastCameraImage.WriteImageToFile("Image.jpg");
		Lock();
		container.Copy(iLastCameraImage);
		container.WriteImageToFile ("CopiedImage.jpg");
		
		Unlock();
		exit(1);
		rw = SDL_RWFromMem((void*)container.GetImageDataPtr(), container.GetImageDataSize());               
						 
		surface = IMG_LoadJPG_RW(rw);
		if (surface == NULL) {
			dPrint(1, "Could not get an image");
			return;
		}
		//SDL_BlitSurface(surface, NULL, mainScreen, &rcDest);
		
		dPrint(1, "Got an image");
      
		Camera_Obstacle_Alarm answer = Find_Object(surface, 1);	//1 - red, 2 - blue
		
		if (answer.Red_Obstacle_Flag) {
			Camera_Distance distance = Postion_Object(answer.c_x, answer.c_y);
			distance.distance = fabs(distance.distance);
			dPrint(1, "Found something (distance %f; angle %f)", distance.distance, distance.angle);
			
			//Create a waypoint here
			//In Universal frame
			//iNextWaypoint = laserToWorld(distance.distance, distance.angle, iRobotPose, iLaserPosition.x, X_RES, Y_RES);
			iRobotState = RobotStateGoToStone;
		}
		//SDL_FreeSurface(surface);
		
	}
	else {
		dPrint(1, "Image not available");
	}
}

vector<ISGridPoint> CJ2B2Demo::getEuclideanLaserData()
{
	vector<ISGridPoint> euclideanLaserData;
	int index = 0;
	for(EACH_IN_i(iLastLaserDistanceArray)) {
		ISGridPoint point = euclideanLaserPoint(i->distance, i->angle, iLaserPosition.x, X_RES, Y_RES, index++);
		if (!setContainsPoint(euclideanLaserData, point)) {
			euclideanLaserData.push_back(point);
		}
	}
	return euclideanLaserData;
}

void CJ2B2Demo::mapFromGridMap(vector<ISGridPoint> map, int **output)
{
	for (int i = 0; i < MAP_ROWS; i++) {
		for (int j = 0; j < MAP_COLS; j++) {
			int idx = i*MAP_COLS+j;
			(*output)[idx] = 1;
			for (vector<ISGridPoint>::iterator iterator = map.begin(); iterator < map.end(); iterator++) {
				ISGridPoint point = *iterator;
				if (point.x == j && point.y == i) {
					(*output)[idx] = 0;
					break;
				}
			}
		}
	}
}

TPoint CJ2B2Demo::mapMatrixRepresentation(vector<TPoint> map, int **output)
{
	//Bias the map because it contains negative values also
	//But AStar algorithm needs only non-negative values
	float min_x = FLT_MAX;
	float min_y = FLT_MAX;
	for (vector<TPoint>::iterator iterator = map.begin(); iterator < map.end(); iterator++) {
		TPoint point = *iterator;
		if ((min_x-point.x) > 0.0001) min_x = point.x;
		if ((min_y-point.y) > 0.0001) min_y = point.y;
	}
	//min_x = ceil(min_x);
	//min_y = ceil(min_y);
	
	for (int i = 0; i < MAP_ROWS; i++) {
		for (int j = 0; j < MAP_COLS; j++) {
			int idx = i*MAP_COLS+j;
			(*output)[idx] = 1;
			for (vector<TPoint>::iterator iterator = map.begin(); iterator < map.end(); iterator++) {
				TPoint point = *iterator;
				//Biased by min_x, min_y
				int x = round((point.x-min_x)/X_RES);
				int y = round((point.y-min_y)/Y_RES);
				if (x == j && y == i) {
					(*output)[idx] = 0;
				//	dPrintLCYellow(1,"Settign cell %d,%d as an obstacle", x,y);
					break;
				}
			}
		}
	}
	TPoint bias;
	bias.x = min_x;
	bias.y = min_y;
	return bias;
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
			dPrint(1,"Smooth astar path %d %d", i, smooth_astar_path.size());
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
		//dPrint(1,"ToleranceX: %f", change);
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

void CJ2B2Demo::updateMap(const MaCI::Position::TPose2D *pose, bool eraseUntrustedPoints)
{
	TPoint me;
	me.x = pose->y;
	me.y = pose->x;
		
	TPoint lidarPoint;
	lidarPoint.x = iLaserPosition.x*sin(pose->a)+me.x;
	lidarPoint.y = iLaserPosition.x*cos(pose->a)+me.y;
	
	for(EACH_IN_i(iLastLaserDistanceArray)) {
		TPoint point;
		
		//Laser origin coordinates
		point.x = i->distance*sin(i->angle);
		point.y = i->distance*cos(i->angle);
		
		//Robot origin coordinates
		point.y += iLaserPosition.x;
		
		//World coordinates
		TPoint worldPoint;
		worldPoint.x = point.x*cos(pose->a)+point.y*sin(pose->a)+me.x;
		worldPoint.y = -point.x*sin(pose->a)+point.y*cos(pose->a)+me.y;
		
		bool exists = false;
		for (vector<TPoint>::iterator iterator = iMap.begin(); iterator < iMap.end(); iterator++) {
			TPoint mapPoint = *iterator;
			if (fabs(worldPoint.x-mapPoint.x) < 0.01 && fabs(worldPoint.y-mapPoint.y) < 0.01) {
				exists = true;
				break;
			}
		}
		
		if (!exists) {
			Lock();
			iMap.push_back(worldPoint);
			
			if (eraseUntrustedPoints) {
				//If obstacle lies on the laser point, eliminate it
				for (vector<TPoint>::iterator iterator = iMap.begin(); iterator < iMap.end(); iterator++) {
					TPoint mapPoint = *iterator;
					if (fabs(worldPoint.x-lidarPoint.x) > 0.0001) {
						float b = (worldPoint.x*lidarPoint.y-lidarPoint.x*worldPoint.y)/(worldPoint.x-lidarPoint.x);
						float k = (lidarPoint.y-b)/lidarPoint.x;
						if ((k*mapPoint.x+b)-mapPoint.y < 0.0001 &&
						   ((mapPoint.x-lidarPoint.x > 0.001 && mapPoint.x-worldPoint.x < -0.001) || 
						    (mapPoint.x-lidarPoint.x < -0.001 && mapPoint.x-worldPoint.x > 0.001)) &&
						   ((mapPoint.y-lidarPoint.y > 0.001 && mapPoint.y-worldPoint.y < -0.001) ||
						    (mapPoint.y-lidarPoint.y < -0.001 && mapPoint.y-worldPoint.y > 0.001))
						 ) {
							iMap.erase(iterator);
						}
					}
				}
			}
			Unlock();
		}
		
	}
}
	

void CJ2B2Demo::updateMap(ISGridPose2D pose, vector<ISGridPoint> scans)
{
	for (vector<ISGridPoint>::iterator iterator = scans.begin(); iterator < scans.end(); iterator++) {
		ISGridPoint point = *iterator;
		ISGridPoint worldPoint = robotToWorld(pose, point);
		if (!setContainsPoint(iGridMap, worldPoint)) {
			iGridMap.push_back(worldPoint);
		}
	}
}
	

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
    iLaserPosition(),
    iFirstSLAMAttempt(true),
    iRobotPose(),
    iPreviousRobotPose(),
    iOdometryPose(),
    iPreviousOdometryPose(),
    iGridMap(),
    iMap(),
    iPreviousLaserData(),
	iSmoothAstarPath(),
	iAstarPath(),
    iNextWaypoint(),
    iPauseOn(true),
    iHasPlan(false),
    iIter(0),
    iBasePoint(),
    iPreviousDirection(DirectionForward),
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
	 
	int posSeq = -1;
	MaCI::Position::CPositionData pd;
	if (iInterface.iPositionOdometry) {
		iInterface.iPositionOdometry->GetPositionEvent(pd, &posSeq, 1000);
		const MaCI::Position::TPose2D *pose = pd.GetPose2D();
		iBasePoint.x = pose->y;
		iBasePoint.y = pose->x;
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
      
    
    // Draw the Map
    if (iMap.size() > 0) {
		
		MaCI::Position::CPositionData pd;
		if (iInterface.iPositionOdometry->CPositionClient::GetPositionEvent(pd, iLastLaserTimestamp.GetGimTime())) {
			const MaCI::Position::TPose2D *pose = pd.GetPose2D();
			
			SDL_Rect rect;
			rect.x = screen->w- 450;
			rect.y = 130 ;
			rect.w = 450 ;  // Set the width of this rectangle area
			rect.h = 450 ;  // Set the height of this rectangle area
			float center_x = rect.x+rect.w/2;
			float center_y = rect.y+rect.h/2;
			float m = 50;	//Multiplier for euclidean coordinates
			float m_a = 10; //Robot radius
		
			SDL_FillRect(screen , &rect , SDL_MapRGB(screen->format , 255 , 255 , 255 ) );
		  
			
			float robot_x = center_x+(pose->y-iBasePoint.x)*m;
			float robot_y = center_y+(pose->x-iBasePoint.y)*m;
			
			float laser_x = robot_x - iLaserPosition.x*sin(-pose->a)*m;
			float laser_y = robot_y + iLaserPosition.x*cos(-pose->a)*m;
			
			vector<TPoint> laserPoints;
			for(EACH_IN_i(iLastLaserDistanceArray)) {
				float laser_end_x = laser_x - i->distance*sin(-i->angle-pose->a)*m;
				float laser_end_y = laser_y + i->distance*cos(-i->angle-pose->a)*m;
				TPoint laser;
				laser.x = laser_end_x;
				laser.y = laser_end_y;
				laserPoints.push_back(laser);
				lineRGBA(screen, laser_x, laser_y, laser_end_x, laser_end_y, 255, 215, 215, 255);
			}
			
			float pointer_end_x = robot_x - m_a*sin(-pose->a);
			float pointer_end_y = robot_y + m_a*cos(-pose->a);
			filledCircleRGBA(screen, robot_x, robot_y, (int)10, 0, 255, 0, 255);
			lineRGBA(screen, robot_x, robot_y, pointer_end_x, pointer_end_y, 0, 0, 0, 255);

			filledCircleRGBA(screen, laser_x, laser_y, (int)2, 255, 0, 255, 255);
			
			//Lock();
			for (vector<TPoint>::iterator iterator = iMap.begin(); iterator < iMap.end(); iterator++) {
				TPoint point = *iterator;
				//filledCircleRGBA(screen, rect.x+rect.w/2-point.x*m, rect.y+rect.h/2-point.y*m, (int)1, 0, 0, 255, 255);
				filledCircleRGBA(screen, center_x+(point.x-iBasePoint.x)*m, center_y+(point.y-iBasePoint.y)*m, (int)1, 0, 0, 255, 255);
			}
			//Unlock(); 
			
			for (vector<TPoint>::iterator iterator = laserPoints.begin(); iterator < laserPoints.end(); iterator++) {
				TPoint laser = *iterator;
				filledCircleRGBA(screen, laser.x, laser.y, (int)2, 255, 0, 0, 255);
			}
			
			//Draw a star waypoints
			if (iAstarPath.size() > 0) {
				for (vector<node>::iterator iterator = iAstarPath.begin(); iterator < iAstarPath.end(); iterator++) {
					node astar = *iterator;
					circleRGBA(screen, center_x+(astar.x-iBasePoint.x)*m, center_y+(astar.y-iBasePoint.y)*m, (int)3, 255, 0, 255, 255);
				}
			}
			
			//Draw a navigation pointer
			float waypoint_x = center_x+(pose->y-iNextWaypoint.x)*m;
			float waypoint_y = center_y+(pose->x-iNextWaypoint.y)*m;
			lineRGBA(screen, robot_x, robot_y, waypoint_x, waypoint_y, 0, 0, 0, 255);
			
			//Draw odometry
			char mystr[255];
			sprintf(mystr, "Odometry: %f, %f, %f", pose->y, pose->x, pose->a);
			stringRGBA(screen, screen->w-450, 590,  mystr, 0, 255, 0, 150);
			sprintf(mystr, "Going to: %f, %f", iNextWaypoint.x,iNextWaypoint.y);
			stringRGBA(screen, screen->w-450, 600,  mystr, 0, 255, 0, 150);
		}
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
  int step = 0;
  int posSeq = -1;
  float K_alpha = 0.05;
  float proximityAngleLimit = M_PI/2.5;
  
  
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
		iBasePoint.x = pose->y;
		iBasePoint.y = pose->x;
	}
   
	iInterface.iBehaviourCtrl->SetStart();
	
	float x_next_stop_meters,Ax_next_stop_meters;
	float y_next_stop_meters,Ay_next_stop_meters;

	int iterations = 0;
	while(iDemoActive && iMotionThreadActive && (aIterations == -1 || iterations < aIterations) && iRobotState != RobotStateShutdown) {
		if (iInterface.iMotionCtrl) {
			if (iInterface.iPositionOdometry) {
				iInterface.iPositionOdometry->GetPositionEvent(pd, &posSeq, 1000);
				const TPose2D *pose = pd.GetPose2D();
				//iPose = pd.GetPose2D();
				dPrint(1, "ODO %f %f %f", pose->y, pose->x, pose->a);
					
				if (iRobotState == RobotStateMoveAway) {
					dPrint(1, "Moving away from the base");
					r_acc = 0.1;
					r_speed = -0.15;
					r_wspeed = 0;
					iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
					ownSleep_ms(20);
					if (fabs(iBasePoint.x-pose->y) > 0.2 || fabs(iBasePoint.y-pose->x) > 0.2) {
						iInterface.iMotionCtrl->SetStop();
						iRobotState = RobotStateShutdown;
						dPrintLCGreen(1, "Mission complete. Good bye!");
					}
				}
				else if (iRobotState == RobotStateOpenGripper) {
					dPrint(1, "Openning gripper");
					bool open = iInterface.iServoCtrl->SetPosition(0, KServoUserServo_0);
			        ownSleep_ms(200);	
			        if (open) {
						iRobotState = RobotStateMoveAway;
						continue;
					}
				}	
				else if (iRobotState == RobotStateCloseGripper) {
					dPrint(1, "Closing grippper");
					bool closed = iInterface.iServoCtrl->SetPosition(M_PI/3-M_PI/20, KServoUserServo_0);
			        ownSleep_ms(200);	
			        if (closed) {
						iNextWaypoint = iBasePoint;
						iRobotState = RobotStateGoHome;
						continue;
					}
				}
				else if (iRobotState == RobotStateAvoidObstacle) {
					
					dPrint(1, "Avoiding obstacle");
					
					r_acc = 0.1;
					r_speed = 0.0;
					r_wspeed = 0.0;
					
					//Read laser sensor;
					float distance = iSmallestDistanceToObject.distance;
					float angle = iSmallestDistanceToObject.angle;
					
					float proximityAlertLimit = 0.4;
					//Allow it to come closer on sides
					if (angle < proximityAngleLimit && angle > -proximityAngleLimit) {
						proximityAlertLimit = 0.3;
					}
					
					bool obstacle = distance <= proximityAlertLimit && distance >= 0;
					
					if (obstacle) {
						TurnDirection direction = iPreviousDirection != DirectionUnknown && iPreviousDirection != DirectionForward ? iPreviousDirection : angle < 0 ? DirectionRight : DirectionLeft;
						
						//Turn by random angle
						r_wspeed = angspeed;
						r_wspeed *= direction == DirectionLeft ? 1 : -1;
						dPrint(1,"Obstacle at distance %f angle %f. Ang spd %f", distance, angle, r_wspeed);
						iPreviousDirection = direction;
						iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
						ownSleep_ms(20);
					}
					else {
						iInterface.iMotionCtrl->SetStop();
						ownSleep_ms(20);
						iRobotState = iPreviousRobotState;
						continue;
					}
				}
				else if (iRobotState == RobotStateWander) {
					
					dPrint(1, "Wandering");
					r_acc = 0.1;
					r_speed = 0.0;
					r_wspeed = 0.0;
					
					//Read laser sensor;
					float distance = iSmallestDistanceToObject.distance;
					float angle = iSmallestDistanceToObject.angle;
					
					float proximityAlertLimit = 0.4;
					//Allow it to come closer on sides
					if (angle < proximityAngleLimit && angle > -proximityAngleLimit) {
						proximityAlertLimit = 0.3;
					}
					
					bool obstacle = distance <= proximityAlertLimit && distance >= 0;
					
					if (obstacle) {
						iInterface.iMotionCtrl->SetStop();
						ownSleep_ms(20);
						iPreviousRobotState = iRobotState;
						iRobotState = RobotStateAvoidObstacle;
						continue;
					}
					else {
						if (cnt++ >= 60) {
							cnt = 0;
							analyzeCamera();
							continue;
						}
						
						r_acc = 0.1;
						r_wspeed = 0.0;
						r_speed = 0.15;
						dPrint(1,"No Obstacle. Going forward (%f,%f,%f)", r_speed, r_wspeed, r_acc);
						iPreviousDirection = DirectionForward;
						iInterface.iMotionCtrl->SetSpeed(r_speed, r_wspeed, r_acc);
						ownSleep_ms(20);
					}
					
				} else if (iRobotState == RobotStateGoHome || RobotStateGoToStone) {
					
					dPrint(1, "Navigating to %f, %f", iNextWaypoint.x,iNextWaypoint.y);
					
				     ////Run A*
					if (!iHasPlan) {
					
						int *searchMap = new int[MAP_COLS*MAP_ROWS];
						TPoint bias = mapMatrixRepresentation(iMap, &searchMap);
						dPrint(1, "Map is biased by %f, %f", bias.x, bias.y);
						//dPrint(1,"Map:");
						//for (int i = 0; i < MAP_ROWS; i++) {
							//string line;
							//for (int j = 0; j < MAP_COLS; j++) {
								//line.append((const char *)(searchMap[i*MAP_COLS+j] == 1 ? "1" : "0"));
							//}
							//dPrint(1,"%s",line.c_str());
						//}
						
						//Bias the map because it contains negative values also
						//But AStar algorithm needs only non-negative values
						
						
						//Current robot position in grid coordinates
						int grid_x = round((pose->y-bias.x)/X_RES);
						int grid_y = round((pose->x-bias.y)/Y_RES);
						
						//Waypoint position in grid coordinates
						int p_x = round((iNextWaypoint.x-bias.x)/X_RES);
						int p_y = round((iNextWaypoint.y-bias.y)/Y_RES);
						
						dPrint(1, "On the map navigating from %d, %d to %d, %d", grid_x,grid_y,p_x,p_y);
					
						pathplan2 plan;
						iAstarPath = plan.get_graph(searchMap,MAP_COLS,MAP_ROWS,grid_x,grid_y,p_x,p_y);
						
						//Unbias nodes
						for (int i = 0; i < (int)iAstarPath.size(); i++) {
							node n = iAstarPath.at(i);
							n.x += bias.x;
							n.y += bias.y;
							iAstarPath.at(i) = n;
						}
										
						iSmoothAstarPath = smooth(iAstarPath,WEIGHT_DATA,WEIGHT_SMOOTH,A_TOLERANCE);
						
						iHasPlan = true;
						
						//Start with node 1, because node 0 is current position;
						step = 1;
					}
		
	                if (step < (int)iSmoothAstarPath.size()-1) {
						       
						dPrint(1,"Driving to a intermediate waypoint %d of %d", step, iSmoothAstarPath.size());
						node present = iAstarPath.at(step);
						Ax_next_stop_meters = present.x * X_RES;
						Ay_next_stop_meters = present.y * Y_RES;
						
						TPose2D next_stop = iSmoothAstarPath.at(step+1);
						x_next_stop_meters = next_stop.x * X_RES;
						y_next_stop_meters = next_stop.y * Y_RES;
						
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
                            step++; 
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
								
								dPrint(1,"Going forward");
								
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
								
								dPrint(1,"Turning");
									    
							    r_speed = 0;                    
								r_wspeed = MAGIC_CNST*alpha;
								if (r_wspeed >= 0.0 || r_wspeed == -0.0) {
									r_wspeed = MAX(MIN(r_wspeed, MAX_WSPEED), MIN_WSPEED);
								}
								else {
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
								dPrint(1,"Starting motion");
								iMotionState = MotionStateTurning;
							break;
							}
					}
					else {
						dPrintLCGreen(1,"Waypoint reached");
						iHasPlan = false;
						iMotionState = MotionStateIdle;
						if (iRobotState == RobotStateGoToStone) {
							iRobotState = RobotStateCloseGripper;
						}
						else if (iRobotState == RobotStateGoHome) {
							iRobotState = RobotStateOpenGripper;
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
		iInterface.iRangingLaser->GetDevicePosition (iLaserPosition);
		
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
