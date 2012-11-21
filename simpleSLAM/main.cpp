
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

double getCorrelation(vector<TPoint> firstSet, vector<TPoint>secondSet)
{
    //Refer to http://paulbourke.net/miscellaneous/correlate/ for the explanation (especially what is delay)
    

    
    if (firstSet.size() != secondSet.size()) {
        cout << "Two sets should have the same size!" << endl;
        return 0;
    }
    
    int N = firstSet.size();
    
    //note that X and Y here correspond to the formula, and is not related to points x and y
    
    double r1, r2;
    
    int maxDelay = N/2;
    
    /* Calculate the means */
    double mx1 = 0, mx2 = 0, my1 = 0, my2 = 0;
	for (int i = 0; i < N; i++) {
        TPoint point1 = firstSet.at(i);
        TPoint point2 = secondSet.at(i);
        mx1 += point1.x;
        my1 += point2.x;
        mx2 += point1.y;
        my2 += point2.y;
	}
    mx1 /= N;
    mx2 /= N;
    my1 /= N;
    my2 /= N;
    
    
    /* Calculate the standart deviations */
    double sx1 = 0, sx2 = 0, sy1 = 0, sy2 = 0;
    for (int i = 0; i < N; i++) {
        TPoint point1 = firstSet.at(i);
        TPoint point2 = secondSet.at(i);
        sx1 += pow((point1.x - mx1), 2);
        sy1 += pow((point2.x - my1), 2);
        sx2 += pow((point1.y - mx2), 2);
        sy2 += pow((point2.y - my2), 2);
    }
    sx1 = sqrt(sx1);
    sx2 = sqrt(sx2);
    sy1 = sqrt(sy1);
    sy2 = sqrt(sy2);
    
    /* Calculate the correlation series */
    for (int delay = -maxDelay; delay < maxDelay; delay++) {
        double sxy1 = 0, sxy2 = 0;
        for (int i = 0; i < N; i++) {
            int j = i + delay;
            
            TPoint point1 = firstSet.at(i);
            
            if (j < 0 || j >= N) {
                //Filled with 0 beyound the set boundaries
                sxy1 += (point1.x - mx1) * -my1;
                sxy2 += (point1.y - mx2) * -my2;
            }
            else {
                TPoint point2 = secondSet.at(j);
                sxy1 += (point1.x - mx1) * (point2.x - my1);
                sxy2 += (point1.y - mx2) * (point2.y - my2);
            }
        }
        r1 = sxy1 / (sx1*sy1);
        r2 = sxy2 / (sx2*sy2);
    }
    
    //Take the mean of x correlation and y correlation as the final correlation
    double r = (fabs(r1)+fabs(r2))/2;
    
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
    newPose1.angle = 3*M_PI_2;
    
    TPose2D newPose2;
    newPose2.x = 3;
    newPose2.y = 2.7;
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
    
    //Test correlation
    double correlation1 = getCorrelation(points, newPoints1);
    double correlation2 = getCorrelation(points, newPoints2);
    cout << "Correlation 1: " << correlation1 << endl;
    cout << "Correlation 2: " << correlation2 << endl;
    cout << endl;
    
    return 0;
}
