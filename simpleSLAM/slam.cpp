#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include "slam.h"

vector<ISPoint> filterPoints(vector<ISPoint> points)
{
	float FILTER_THRESHOLD = 0.05;
    vector<ISPoint> res;
	for (unsigned int i = 1; i < points.size()-1; i++) {
		ISPoint previousPoint = points.at(i-1);
		ISPoint nextPoint = points.at(i+1);
		ISPoint currentPoint = points.at(i);
		double distance = distanceToPoint(currentPoint);
        double diff1 = distance-distanceToPoint(previousPoint);
        double diff2 = distance-distanceToPoint(nextPoint);
        if ((diff1 > FILTER_THRESHOLD && diff2 > FILTER_THRESHOLD) || (diff1 < -FILTER_THRESHOLD && diff2 < -FILTER_THRESHOLD)) {
			ISPoint correctedPoint;
			correctedPoint.x = (nextPoint.x-previousPoint.x)/2;
			correctedPoint.y = (nextPoint.y-previousPoint.y)/2;
			res.push_back(correctedPoint);
	 	}
	 	else {
			res.push_back(currentPoint);
		}
	}
    return res;
}

ISPoint laserCartesian(double distance, double angle, float laser_to_robot)
{
	ISPoint point;
	//Laser origin coordinates
	point.x = distance*cos(angle);
	point.y = distance*sin(angle);
	
	//Robot origin coordinates
	point.y += laser_to_robot;
	
	return point;
}

ISPoint laserToWorld(double distance, double angle, ISPose2D currentPose, float laser_to_robot)
{
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

double distanceToPoint(ISPoint point)
{
    return sqrt(pow(point.x, 2)+pow(point.y, 2));
}

vector<double> getDistances(vector<ISPoint> set)
{
    vector<double> res;
	for (vector<ISPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        ISPoint point = *iterator;
        double distance = distanceToPoint(point);
        res.push_back(distance);
	}
    return res;
}

double sumDifferences(vector<double> firstSet, vector<double> secondSet, bool printdiff)
{
    if (firstSet.size() != secondSet.size()) {
        cout << "Two sets should have the same size!" << endl;
        return 0;
    }
    
    double res = 0;
    
    for (unsigned int i = 0; i < firstSet.size(); i++) {
		 double diff = fabs(firstSet.at(i) - secondSet.at(i));// fabs(pow(firstSet.at(i),2) - pow(secondSet.at(i),2));
		 if (diff < 0.0001) diff = 0.0f;
		 res += diff;
		 //if (printdiff) cout << "DIFF " << i << ", " << diff << endl;
		 if (printdiff&& i== firstSet.size()/2) cout << "DIFF " << i << ", " << diff << firstSet.at(i) <<", " <<secondSet.at(i)<< endl;
		//res += fabs(firstSet.at(i) - secondSet.at(i));
	}
	return res;
}

double sumDifferences(vector<ISPoint> firstSet, vector<ISPoint> secondSet, bool printdiff)
{
    if (firstSet.size() != secondSet.size()) {
        cout << "Two sets should have the same size!" << endl;
        return 0;
    }
    
    double res = 0;
    
    for (unsigned int i = 0; i < firstSet.size(); i++) {
		ISPoint point1 = firstSet.at(i);
		ISPoint point2 = secondSet.at(i);
		double diff = fabs(point1.x-point2.x)+fabs(point1.y-point2.y);
		 //double diff = fabs(firstSet.at(i) - secondSet.at(i));// fabs(pow(firstSet.at(i),2) - pow(secondSet.at(i),2));
		 if (diff < 0.0001) diff = 0.0f;
		 res += diff;
		 //if (printdiff) cout << "DIFF " << i << ", " << diff << endl;
		 if (printdiff && i== firstSet.size()/2) cout << "DIFF " << i << ", " << diff << " (" << point1.x << "," << point1.y << "), (" << point2.x << "," << point2.y << ")" << endl;
		//res += fabs(firstSet.at(i) - secondSet.at(i));
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
    
    vector<double> X = getDistances(shiftPoints(firstSet, shift));
    vector<double> Y = getDistances(shiftPoints(secondSet, shift));
    
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
    for (int i = 0; i < numRadiuses; i++) {
        if (i == 0) {
            for (int k = -(numOrientations-1)/2; k <= (numOrientations-1)/2; k++) {
                float angle = k == 0 ? 0 : maxAngleDeviation/k;
                ISPose2D pose = currentPose;
                pose.angle += angle;
                poses.push_back(pose);
            }
		}
		else {
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
    }
    return poses;
}

vector<ISPoint> transformPoints(ISPose2D startPose, ISPose2D endPose, vector<ISPoint> points)
{
    vector<ISPoint> transformedPoints;
    double theta = endPose.angle - startPose.angle;
    double tx = endPose.x - startPose.x;
    double ty = endPose.y - startPose.y;
    
	for (vector<ISPoint>::iterator iterator = points.begin(); iterator < points.end(); iterator++) {
        ISPoint point = *iterator;
        double x = point.x;
        double y = point.y;
        ISPoint newPoint;
        newPoint.x = x*cos(theta)-y*sin(theta)+tx;
        newPoint.y = x*sin(theta)+y*cos(theta)+ty;
        transformedPoints.push_back(newPoint);
	}
    return transformedPoints;
}


double avg(vector<double> set)
{
	double res = 0.0f;
	unsigned int n = set.size();
	for (unsigned int i = 0; i < n; i++) {
		res += set.at(i);
	}
	res /= n;
	return res;
}

double crossCorrelation(vector<double> X, vector<double> Y, int timeLag)
{
	unsigned int n = X.size();
	
	//Mean values
	double mx = avg(X);
	double my = avg(Y);
	
	//Cross covariance and standart deviations
	double Cxy = 0.0f;
	double Sx = 0.0f;
	double Sy = 0.0f;
	for (unsigned int i = 0; i < n; i++) {
		double x = X.at(i); double y = Y.at(i);
        double xk = ((i-timeLag) >= 0 && (i-timeLag) < n ? X.at(i-timeLag) : 0);
        double yk = ((i+timeLag) < n && (i+timeLag) >= 0 ? Y.at(i+timeLag) : 0);
        if (timeLag >= 0) {
            Cxy += ((x-mx)*(yk-my));
        }
        else {
            Cxy += ((y-my)*(xk-mx));
        }
		Sx += pow((x-mx),2);
		Sy += pow((y-my),2);
	}
	Cxy /= n;
	Sx = sqrt(Sx/n);
	Sy = sqrt(Sy/n);
    
	
	//Cross correlation
	double r = Cxy/(Sx*Sy);
    
	return r;
}

double iterativeCrossCorrelation(vector<double> X, vector<double> Y)
{
    double res = 0.0f;
    for (int i = 0; i < (int)X.size()/2; i++) {
        res += crossCorrelation(X, Y, i);
    }
    return res;
}

bool poseEqualsToPose(ISPose2D firstPose, ISPose2D secondPose)
{
	bool res = false;
	if (fabs(firstPose.x-secondPose.x) < 0.0001 && fabs(firstPose.y-secondPose.y) < 0.0001 && fabs(firstPose.angle-secondPose.angle) < 0.0001) {
		res = true;
	}
	return res;
}


bool gridPoseEqualsToGridPose(ISGridPose2D firstPose, ISGridPose2D secondPose)
{
	bool res = false;
	if (firstPose.x == secondPose.x && firstPose.y == secondPose.y && fabs(firstPose.angle-secondPose.angle) < 0.0001) {
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

ISGridPoint euclideanLaserPoint(double distance, double angle, float laser_to_robot, float x_res, float y_res)
{
	ISGridPoint point;
	
	//Laser origin coordinates
	double x = distance*cos(angle);
	double y = distance*sin(angle);
	
	//Robot origin coordinates
	y += laser_to_robot;
	
	point.x = round(x/x_res/2);
	point.y = round(y/y_res/2);
	
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
	                double x = currentPose.x + r*cos(theta);
	                double y = currentPose.y + r*sin(theta);
	                pose.x = round(x/x_res/2);
	                pose.y = round(y/y_res/2);
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


vector<ISGridPoint> transformGridPoints(ISGridPose2D startPose, ISGridPose2D endPose, vector<ISGridPoint> points, float x_res, float y_res)
{
    vector<ISGridPoint> transformedPoints;
    double theta = endPose.angle - startPose.angle;
    int tx = endPose.x - startPose.x;
    int ty = endPose.y - startPose.y;
    
	for (vector<ISGridPoint>::iterator iterator = points.begin(); iterator < points.end(); iterator++) {
        ISGridPoint point = *iterator;
        ISGridPoint newPoint;
        
        //Rotation
        double x = point.x*cos(theta)-point.y*sin(theta);
        double y = point.x*sin(theta)+point.y*cos(theta); 
		newPoint.x = round(x/x_res/2);
		newPoint.y = round(y/y_res/2);
		
		//Translation
        newPoint.x += tx;
        newPoint.y += ty; 
        
        if (!setContainsPoint(transformedPoints, newPoint)) {
	        transformedPoints.push_back(newPoint);
		}
	}
    return transformedPoints;	
}

int sumGridDifferences(vector<ISGridPoint> firstSet, vector<ISGridPoint> secondSet, bool printdiff)
{
    if (firstSet.size() != secondSet.size()) {
        cout << "Two sets should have the same size!" << endl;
        return 0;
    }
    
    int res = 0;
    
    for (unsigned int i = 0; i < firstSet.size(); i++) {
		ISGridPoint point1 = firstSet.at(i);
		ISGridPoint point2 = secondSet.at(i);
		int diff = abs(point1.x-point2.x)+abs(point1.y-point2.y);
		res += diff;
		if (printdiff && i== firstSet.size()/2) cout << "DIFF " << i << ", " << diff << " (" << point1.x << "," << point1.y << "), (" << point2.x << "," << point2.y << ")" << endl;
	}
	return res;
}

ISGridPoint laserToWorld(double distance, double angle, ISGridPose2D currentPose, float laser_to_robot, float x_res, float y_res)
{
	//Laser origin coordinates
	double x = distance*cos(angle+currentPose.angle);
	double y = distance*sin(angle+currentPose.angle);
	
	//Robot origin coordinates
	x += laser_to_robot*cos(currentPose.angle);
	y += laser_to_robot*sin(currentPose.angle);
	
	//World coordinates
	ISGridPoint point;
	point.x = round(x/x_res/2);
	point.y = round(y/y_res/2);
	point.x += currentPose.x;
	point.y += currentPose.y;
	return point;
}
