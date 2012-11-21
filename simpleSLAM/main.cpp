
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

struct TPose2D {
    float x, y, angle;
};


//maxRadius how far the generated points can be from the current position
//numRadiuses how many circles should be between maxRadius and current position
//numPositions how many positions to generate in each circle
//maxAngleDeviation how much the orientation can differ
//numOrientations how many orientations to generate for each position
std::vector<TPose2D> generatePoses(TPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations) {
    std::vector<TPose2D> poses;
    poses.begin();
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


int main(int argc, char *argv[])
{
    TPose2D currentPose;
    currentPose.x = 3;
    currentPose.y = 2.7;
    currentPose.angle = 0;
    
    std::vector<TPose2D> poses = generatePoses(currentPose, 0.5, 4, 8, 0.01, 5);
    
    std::cout << "Size: " << poses.size() << endl;
    
	for (std::vector<TPose2D>::iterator iterator = poses.begin(); iterator < poses.end(); iterator++) {
        TPose2D pose = *iterator;
		cout << "x: " << pose.x << "; y: " << pose.y << "; angle: " << pose.angle << endl;
	}

    
    
    return 0;
}
