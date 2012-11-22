#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>
#include "slam.h"

int main(int argc, char *argv[])
{
    ISPose2D currentPose;
    currentPose.x = 3;
    currentPose.y = 2.7;
    currentPose.angle = 0;
    
    //Test generation of new poses
    vector<ISPose2D> poses = generatePoses(currentPose, 0.5, 4, 8, 0.01, 5);
    /*
     cout << "Size: " << poses.size() << endl;
     
     for (vector<ISPose2D>::iterator iterator = poses.begin(); iterator < poses.end(); iterator++) {
     ISPose2D pose = *iterator;
     cout << "x: " << pose.x << "; y: " << pose.y << "; angle: " << pose.angle << endl;
     }
     */
    
    
    //Test transformation of points
    vector<ISPoint> points;
    for (int i = 0; i < 10; i++) {
        ISPoint point;
        point.x = i;
        point.y = i;
        points.push_back(point);
    }
    
    
    ISPose2D newPose1;
    newPose1.x = 3;
    newPose1.y = 3;
    newPose1.angle = M_PI/6;
    
    ISPose2D newPose2;
    newPose2.x = 3;
    newPose2.y = 3;
    newPose2.angle = 0;
    
    vector<ISPoint> newPoints1 = transformPoints(currentPose, newPose1, points);
    vector<ISPoint> newPoints2 = transformPoints(currentPose, newPose2, points);
    
    cout << endl;
    cout << "In 2nd column transformed points (270deg rotation, (1m; 0.3m) translation):" << endl;
    cout << "In 3rd column transformed points (0deg rotation, (1m; 0m) translation):" << endl;
    cout << "Assuming that x axis extends to the east (relative to the robot), y - to the north, and positive angle is CCW" << endl;
    cout << endl;
    cout << "     Initial points          |         T+R points           |     Slightly translated points " << endl;
    cout << "-----------------------------+------------------------------+--------------------------------" << endl;
    
	for (int i = 0; i < points.size(); i++) {
        ISPoint point = points.at(i);
        ISPoint newPoint1 = newPoints1.at(i);
        ISPoint newPoint2 = newPoints2.at(i);
		cout << "x: " << setw(10) << setprecision(2) << point.x     << "; y: " << setw(10) << setprecision(2) << point.y     << " | "
        << "x: " << setw(10) << setprecision(2) << newPoint1.x << "; y: " << setw(10) << setprecision(2) << newPoint1.y << " | "
        << "x: " << setw(10) << setprecision(2) << newPoint2.x << "; y: " << setw(10) << setprecision(2) << newPoint2.y << endl;
	}
    cout << endl;
    
    float min1 = minValues(points);
    float min2 = minValues(newPoints1);
    float min3 = minValues(newPoints2);
    
    ISPoint offset;
    offset.x = min(min(min1.x, min2.x), min3.x);
    offset.y = min(min(min1.y, min2.y), min3.y);
    
    
    cout << "Offset: " << offset.x << ", " << offset.y << endl << endl;
    
    //Test correlation
    double correlation1 = getCorrelation(points, newPoints1, offset);
    double correlation2 = getCorrelation(points, newPoints2, offset);
    cout << "Correlation 1: " << correlation1 << endl;
    cout << "Correlation 2: " << correlation2 << endl;
    cout << endl;
    
    return 0;
}
