#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>
#include "slam.h"

int main(int argc, char *argv[])
{
    TPose2D currentPose;
    currentPose.x = 3;
    currentPose.y = 2.7;
    currentPose.angle = 0;
    
    //Test generation of new poses
    vector<TPose2D> poses = generatePoses(currentPose, 0.5, 4, 8, 0.01, 5);
    /*
     cout << "Size: " << poses.size() << endl;
     
     for (vector<TPose2D>::iterator iterator = poses.begin(); iterator < poses.end(); iterator++) {
     TPose2D pose = *iterator;
     cout << "x: " << pose.x << "; y: " << pose.y << "; angle: " << pose.angle << endl;
     }
     */
    
    
    //Test transformation of points
    vector<TPoint> points;
    for (int i = 0; i < 10; i++) {
        TPoint point;
        point.x = i;
        point.y = i;
        points.push_back(point);
    }
    
    
    TPose2D newPose1;
    newPose1.x = 3;
    newPose1.y = 3;
    newPose1.angle = M_PI/6;
    
    TPose2D newPose2;
    newPose2.x = 3;
    newPose2.y = 3;
    newPose2.angle = 0;
    
    vector<TPoint> newPoints1 = transformPoints(currentPose, newPose1, points);
    vector<TPoint> newPoints2 = transformPoints(currentPose, newPose2, points);
    
    cout << endl;
    cout << "In 2nd column transformed points (270deg rotation, (1m; 0.3m) translation):" << endl;
    cout << "In 3rd column transformed points (0deg rotation, (1m; 0m) translation):" << endl;
    cout << "Assuming that x axis extends to the east (relative to the robot), y - to the north, and positive angle is CCW" << endl;
    cout << endl;
    cout << "     Initial points          |         T+R points           |     Slightly translated points " << endl;
    cout << "-----------------------------+------------------------------+--------------------------------" << endl;
    
	for (int i = 0; i < points.size(); i++) {
        TPoint point = points.at(i);
        TPoint newPoint1 = newPoints1.at(i);
        TPoint newPoint2 = newPoints2.at(i);
		cout << "x: " << setw(10) << setprecision(2) << point.x     << "; y: " << setw(10) << setprecision(2) << point.y     << " | "
        << "x: " << setw(10) << setprecision(2) << newPoint1.x << "; y: " << setw(10) << setprecision(2) << newPoint1.y << " | "
        << "x: " << setw(10) << setprecision(2) << newPoint2.x << "; y: " << setw(10) << setprecision(2) << newPoint2.y << endl;
	}
    cout << endl;
    
    float minX1 = minX(points);
    float minX2 = minX(newPoints1);
    float minX3 = minX(newPoints2);
    float minY1 = minY(points);
    float minY2 = minY(newPoints1);
    float minY3 = minY(newPoints2);
    
    TPoint offset;
    offset.x = min(min(minX1, minX2), minX3);
    offset.y = min(min(minY1, minY2), minY3);
    
    
    cout << "Offset: " << offset.x << ", " << offset.y << endl << endl;
    
    //Test correlation
    double correlation1 = getCorrelation(points, newPoints1, offset);
    double correlation2 = getCorrelation(points, newPoints2, offset);
    cout << "Correlation 1: " << correlation1 << endl;
    cout << "Correlation 2: " << correlation2 << endl;
    cout << endl;
    
    return 0;
}