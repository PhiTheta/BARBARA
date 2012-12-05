#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include "slam.h"


bool gridPoseEqualsToGridPose(ISGridPose2D firstPose, ISGridPose2D secondPose)
{
	bool res = false;
	if (fabs(firstPose.x-secondPose.x) < 0.0001 && fabs(firstPose.y-secondPose.y) < 0.0001 && fabs(firstPose.angle-secondPose.angle) < 0.0001) {
		res = true;
	}
	return res;
}


bool pointEqualsToPoint(ISGridPoint point1, ISGridPoint point2)
{
	return point1.x == point2.x && point1.y == point2.y;
}

bool setContainsPoint(vector<ISGridPoint> set, ISGridPoint point)
{
	bool res = false;
	for (vector<ISGridPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
		ISGridPoint testPoint = *iterator;
		if (pointEqualsToPoint(testPoint, point)) {
			res = true;
			break;
		}
	}
	return res;	
}

bool setContainsPose(vector<ISGridPose2D> set, ISGridPose2D pose)
{
	bool res = false;
	for (vector<ISGridPose2D>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
		ISGridPose2D testPose = *iterator;
		if (gridPoseEqualsToGridPose(testPose, pose)) {
			res = true;
			break;
		}
	}
	return res;	
}

ISGridPoint euclideanLaserPoint(double distance, double angle, float laser_to_robot, float x_res, float y_res, int index)
{
	ISGridPoint point;
	
	//Laser origin coordinates
	double x = distance*sin(angle);
	double y = distance*cos(angle);
	
	//Robot origin coordinates
	y += laser_to_robot;
	
	point.x = round(x/x_res);
	point.y = round(y/y_res);
	point.index = index;
	
	return point;
}

vector<ISGridPose2D> generateGridPoses(ISGridPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations, float x_res, float y_res)
{
    vector<ISGridPose2D> poses;
    for (int i = 0; i < numRadiuses; i++) {
        if (i == 0) {
            for (int k = -(numOrientations-1)/2; k <= (numOrientations-1)/2; k++) {
                float angle = k == 0 ? 0 : maxAngleDeviation/k;
                ISGridPose2D pose = currentPose;
                pose.angle += angle;
                if (!setContainsPose(poses, pose)) {
	                poses.push_back(pose);
				}
            }
		}
		else {
			float r = maxRadius/i;
	        for (int j = 1; j <= numPositions; j++) {
	            float theta = 2*M_PI/j;
	            for (int k = -(numOrientations-1)/2; k <= (numOrientations-1)/2; k++) {
	                float angle = k == 0 ? 0 : maxAngleDeviation/k;
	                ISGridPose2D pose;
	                double x = r*cos(theta);
	                double y = r*sin(theta);
	                pose.x = round(x/x_res);
	                pose.y = round(y/y_res);
	                pose.x += currentPose.x;
	                pose.y += currentPose.y;
	                pose.angle = currentPose.angle+angle;
	                if (!setContainsPose(poses, pose)) {
		                poses.push_back(pose);
					}
	            }
	        }
		}
    }
    return poses;
}


vector<ISGridPoint> transformGridPoints(ISGridPose2D startPose, ISGridPose2D endPose, vector<ISGridPoint> points)
{
	if (gridPoseEqualsToGridPose(startPose, endPose)) {
		return points;
	}
	
    vector<ISGridPoint> transformedPoints;
    double theta = endPose.angle - startPose.angle;
    int tx = endPose.x - startPose.x;
    int ty = endPose.y - startPose.y;
    
	for (vector<ISGridPoint>::iterator iterator = points.begin(); iterator < points.end(); iterator++) {
        ISGridPoint point = *iterator;
        ISGridPoint newPoint;
        newPoint.index = point.index;
        
        //Rotation
        newPoint.x = round(point.x*cos(theta)-point.y*sin(theta)+tx);
        newPoint.y = round(point.x*sin(theta)+point.y*cos(theta)+ty);
        
        if (!setContainsPoint(transformedPoints, newPoint)) {
	        transformedPoints.push_back(newPoint);
		}
	}
    return transformedPoints;	
}

int sumGridDifferences(vector<ISGridPoint> firstSet, vector<ISGridPoint> secondSet, bool printdiff)
{
    int res = 0;
    
    for (unsigned int i = 0; i < firstSet.size(); i++) {
		for (unsigned int j = 0; j < secondSet.size(); j++) {
			ISGridPoint point1 = firstSet.at(i);
			ISGridPoint point2 = secondSet.at(j);
			if (point1.index == point2.index) {
				int diff = abs(point1.x-point2.x)+abs(point1.y-point2.y);
				res += diff;
				if (printdiff && i== firstSet.size()/2) cout << "DIFF " << i << ", " << diff << " (" << point1.x << "," << point1.y << "), (" << point2.x << "," << point2.y << ")" << endl;
				break;
			}
		}
	}
	return res;
}

ISGridPoint laserToWorld(double distance, double angle, ISGridPose2D currentPose, float laser_to_robot, float x_res, float y_res)
{
	//Laser origin coordinates
	float x = distance*cos(angle-currentPose.angle);
	float y = distance*sin(angle-currentPose.angle);
	
	//Robot origin coordinates
	x += laser_to_robot*cos(currentPose.angle);
	y += laser_to_robot*sin(currentPose.angle);
	
	//World coordinates
	ISGridPoint point;
	point.x = round(x/x_res);
	point.y = round(y/y_res);
	point.x += currentPose.x;
	point.y += currentPose.y;
	return point;
}

ISGridPoint robotToWorld(ISGridPose2D pose, ISGridPoint point)
{
	ISGridPoint newPoint;
	newPoint.x = round(point.x*cos(-pose.angle)-point.y*sin(-pose.angle)+pose.x);
	newPoint.y = round(point.x*sin(-pose.angle)+point.y*cos(-pose.angle)+pose.y);
	return newPoint;
}

int getPoseDifference(ISGridPose2D pose1, ISGridPose2D pose2)
{
	return abs(pose1.x-pose2.x)+abs(pose1.y-pose2.y)+abs((int)(10000*(pose1.angle-pose2.angle)));
}
