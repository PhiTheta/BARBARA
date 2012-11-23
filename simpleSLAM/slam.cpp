
#include <vector>
#include <math.h>
#include <iostream>
#include "slam.h"


ISPoint laserCartesian(double distance, double angle)
{
	ISPoint point;
	point.x = distance*cos(angle);
	point.y = -distance*sin(angle);
	return point;
}

ISPoint laserToWorld(double distance, double angle, ISPose2D currentPose)
{
	//Distance from the SICK mirror wheel center from back edge ~9.5cm
	//Distance from SICK back edge to the robot center ~2.7cm
	float laser_to_robot = 0.122;
	
	
	
	ISPoint point;
	
	//Laser origin coordinates
	point.x = distance*cos(angle+currentPose.angle);	
	point.y = -distance*sin(angle+currentPose.angle);
	
	//Robot origin coordinates
	point.x += laser_to_robot*cos(currentPose.angle);
	point.y += -laser_to_robot*sin(currentPose.angle);
	
	//World coordinates
	point.x += currentPose.x;
	point.y += currentPose.y;
	return point;
}	

float distanceToPoint(ISPoint point)
{
    return sqrt(pow(point.x, 2)+pow(point.y, 2));
}

vector<float> getDistances(vector<ISPoint> set)
{
    vector<float> res;
	for (vector<ISPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        ISPoint point = *iterator;
        float distance = distanceToPoint(point);
        res.push_back(distance);
	}
    return res;
}

ISPoint minValues(vector<ISPoint> set)
{
    ISPoint res;
    res.x = 0;
    res.y = 0;
	for (vector<ISPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        ISPoint point = *iterator;
        if (point.x < res.x) {
            res.x = point.x;
        }
        if (point.y < res.y) {
            res.y = point.y;
        }
	}
    return res;
}

vector<ISPoint> shiftPoints(vector<ISPoint> set, ISPoint offset)
{
    vector<ISPoint> res;
	for (vector<ISPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        ISPoint point = *iterator;
        point.x -= offset.x;
        point.y -= offset.y;
        res.push_back(point);
	}
    return res;
}

//Shift is the amount of how much it's needed to shift both sets to make all the values positive
//It should be calculated from min x and y from considered sets of points 
double getCorrelation(vector<ISPoint> firstSet, vector<ISPoint>secondSet, ISPoint shift)
{
    //Refer to http://paulbourke.net/miscellaneous/correlate/ for the explanation (especially what is delay)
    

    
    if (firstSet.size() != secondSet.size()) {
        cout << "Two sets should have the same size!" << endl;
        return 0;
    }
    
    vector<float> X = getDistances(shiftPoints(firstSet, shift));
    vector<float> Y = getDistances(shiftPoints(secondSet, shift));
    
    int N = X.size();
    
    int maxDelay = N/2;
    
    /* Calculate the means */
    double mx = 0, my = 0;
	for (int i = 0; i < N; i++) {
        mx += X.at(i);
        my += Y.at(i);
	}
    mx /= N;
    my /= N;
    
    
    /* Calculate the standart deviations */
    double sx = 0, sy = 0;
    for (int i = 0; i < N; i++) {
        sx += pow((X.at(i) - mx), 2);
        sy += pow((Y.at(i) - my), 2);
    }
    sx = sqrt(sx);
    sy = sqrt(sy);
    
    double r;
    
    /* Calculate the correlation series */
    for (int delay = -maxDelay; delay < maxDelay; delay++) {
        double sxy = 0;
        for (int i = 0; i < N; i++) {
            int j = i + delay;
                        
            if (j < 0 || j >= N) {
                //Filled with 0 beyound the set boundaries
                sxy += (X.at(i) - mx) * -my;
            }
            else {
                sxy += (X.at(i) - mx) * (Y.at(j) - my);
            }
        }
        r = sxy / (sx*sy);
    }
    
    return r;
}

//maxRadius how far the generated points can be from the current position
//numRadiuses how many circles should be between maxRadius and current position
//numPositions how many positions to generate in each circle
//maxAngleDeviation how much the orientation can differ
//numOrientations how many orientations to generate for each position
vector<ISPose2D> generatePoses(ISPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations)
{
    vector<ISPose2D> poses;
    for (int i = 1; i <= numRadiuses; i++) {
        float r = maxRadius/i;
        for (int j = 1; j <= numPositions; j++) {
            float theta = 2*M_PI/j;
            for (int k = -(numOrientations-1)/2; k <= (numOrientations-1)/2; k++) {
                float angle = k == 0 ? 0 : maxAngleDeviation/k;
                ISPose2D pose;
                pose.x = currentPose.x + r*cos(theta);
                pose.y = currentPose.y -r*sin(theta);
                pose.angle = currentPose.angle+angle;
                poses.push_back(pose);
            }
        }
    }
    return poses;
}

vector<ISPoint> transformPoints(ISPose2D startPose, ISPose2D endPose, vector<ISPoint> points)
{
    vector<ISPoint> transformedPoints;
    float theta = startPose.angle - endPose.angle;
    float tx = endPose.x - startPose.x;
    float ty = endPose.y - startPose.y;
    
	for (vector<ISPoint>::iterator iterator = points.begin(); iterator < points.end(); iterator++) {
        ISPoint point = *iterator;
        float x = point.x;
        float y = point.y;
        ISPoint newPoint;
        newPoint.x = x*cos(theta)-y*sin(theta)+tx;
        newPoint.y = x*sin(theta)+y*cos(theta)+ty;
        transformedPoints.push_back(newPoint);
	}
    return transformedPoints;
}
