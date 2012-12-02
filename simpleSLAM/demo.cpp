#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>
#include "slam.h"

int main(int argc, char *argv[])
{
    float x_res = 0.1;
    float y_res = 0.1;
    
    ISGridPose2D predictedPose;
    predictedPose.x = 0;
    predictedPose.y = 0;
    predictedPose.angle = 0.0f;
    
    //Test transformation of points
    vector<ISGridPoint> scans;
    for (int i = 0; i < 10; i++) {
        ISGridPoint point;
        point.x = -5;
        point.y = i;
        scans.push_back(point);
        point.x = 5;
        scans.push_back(point);
    }
    for (int i = -4; i <= 4; i++) {
        ISGridPoint point;
        point.x = i;
        point.y = 10;
        scans.push_back(point);
    }
    
    ISGridPose2D generatedPose = predictedPose;
    generatedPose.angle = M_PI_4;
    vector<ISGridPoint> transformedScans = transformGridPoints(predictedPose, generatedPose, scans, x_res, y_res);
    
    cout << "   Predicted   |   Transformed   " << endl;
    cout << "---------------+-----------------" << endl;
    for (unsigned int i = 0; i < scans.size(); i++) {
        ISGridPoint originalPoint = scans.at(i);
        cout << "   " << setw(3) << originalPoint.x << ", " << setw(3) << originalPoint.y << "    |   ";
        if (i < transformedScans.size()) {
            ISGridPoint transformedPoint = transformedScans.at(i);
            cout << setw(3) << transformedPoint.x << ", " << setw(3) << transformedPoint.y;
        }
        cout << endl;
    }
    
    cout << endl << endl;
    
    
    
    
    return 0;
}
