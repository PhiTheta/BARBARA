
#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>

using namespace std;

struct TPose2D {
    float x, y, angle;
};

struct TPoint {
    float x, y;
};

float distanceToPoint(TPoint point)
{
    return sqrt(pow(point.x, 2)+pow(point.y, 2));
}

vector<float> getDistances(vector<TPoint> set)
{
    vector<float> res;
	for (vector<TPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        TPoint point = *iterator;
        float distance = distanceToPoint(point);
        res.push_back(distance);
	}
    return res;
}

float minX(vector<TPoint> set) {
    float res = 0;
	for (vector<TPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        TPoint point = *iterator;
        if (point.x < res) {
            res = point.x;
        }
	}
    return res;
}

float minY(vector<TPoint> set) {
    float res = 0;
	for (vector<TPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        TPoint point = *iterator;
        if (point.y < res) {
            res = point.y;
        }
	}
    return res;
}

vector<TPoint> shiftPoints(vector<TPoint> set, TPoint offset)
{
    vector<TPoint> res;
	for (vector<TPoint>::iterator iterator = set.begin(); iterator < set.end(); iterator++) {
        TPoint point = *iterator;
        point.x -= offset.x;
        point.y -= offset.y;
        res.push_back(point);
	}
    return res;
}

//Shift is the amount of how much it's needed to shift both sets to make all the values positive
//It should be calculated from min x and y from considered sets of points 
double getCorrelation(vector<TPoint> firstSet, vector<TPoint>secondSet, TPoint shift)
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
