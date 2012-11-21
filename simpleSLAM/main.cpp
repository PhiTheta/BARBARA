
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

struct TPose2D {
    float x, y, angle;
};

struct TPoint {
    float x, y;
};

//maxRadius how far the generated points can be from the current position
//numRadiuses how many circles should be between maxRadius and current position
//numPositions how many positions to generate in each circle
//maxAngleDeviation how much the orientation can differ
//numOrientations how many orientations to generate for each position
vector<TPose2D> generatePoses(TPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations) {
    vector<TPose2D> poses;
    for (int i = 1; i <= numRadiuses; i++) {
        float r = maxRadius/i;
        for (int j = 1; j <= numPositions; j++) {
            float theta = 2*M_PI/j;
            for (int k = -(numOrientations-1)/2; k <= (numOrientations-1)/2; k++) {
                float angle = k == 0 ? 0 : maxAngleDeviation/k;
                TPose2D pose;
                pose.x = currentPose.x + r*cos(theta);
                pose.y = currentPose.y -r*sin(theta);
                pose.angle = currentPose.angle+angle;
                poses.push_back(pose);
            }
        }
    }
    return poses;
}

vector<TPoint> transformPoints(TPose2D startPose, TPose2D endPose, vector<TPoint> points)
{
    vector<TPoint> transformedPoints;
    float theta = startPose.angle - endPose.angle;
    float tx = endPose.x - startPose.x;
    float ty = endPose.y - startPose.y;
    
	for (vector<TPoint>::iterator iterator = points.begin(); iterator < points.end(); iterator++) {
        TPoint point = *iterator;
        float x = point.x;
        float y = point.y;
        TPoint newPoint;
        newPoint.x = x*cos(theta)-y*sin(theta)+tx;
        newPoint.y = x*sin(theta)+y*cos(theta)+ty;
        transformedPoints.push_back(newPoint);
	}
    return transformedPoints;
}


int main(int argc, char *argv[])
{
    TPose2D currentPose;
    currentPose.x = 3;
    currentPose.y = 2.7;
    currentPose.angle = 0;
    
    vector<TPose2D> poses = generatePoses(currentPose, 0.5, 4, 8, 0.01, 5);
    
    cout << "Size: " << poses.size() << endl;
    
	for (vector<TPose2D>::iterator iterator = poses.begin(); iterator < poses.end(); iterator++) {
        TPose2D pose = *iterator;
		cout << "x: " << pose.x << "; y: " << pose.y << "; angle: " << pose.angle << endl;
	}

    vector<TPoint> points;
    for (int i = 0; i < 10; i++) {
        TPoint point;
        point.x = i;
        point.y = i;
        points.push_back(point);
    }
    
    cout << "Initial points:" << endl;
	for (vector<TPoint>::iterator iterator = points.begin(); iterator < points.end(); iterator++) {
        TPoint point = *iterator;
		cout << "x: " << point.x << "; y: " << point.y << endl;
	}
    
    cout << "Transformed points (270deg rotation, (1m; 0.3m) translation):" << endl;
    cout << "Assuming that x axis extends to the east (relative to the robot), y - to the north, and positive angle is CCW" << endl;
    TPose2D newPose;
    newPose.x = 4;
    newPose.y = 3;
    newPose.angle = 3*M_PI_2;
    vector<TPoint> newPoints = transformPoints(currentPose, newPose, points);
	for (vector<TPoint>::iterator iterator = newPoints.begin(); iterator < newPoints.end(); iterator++) {
        TPoint point = *iterator;
		cout << "x: " << point.x << "; y: " << point.y << endl;
	}
    
    
    return 0;
}
