
using namespace std;

struct ISGridPose2D {
	int x, y;
	double angle;
};

struct ISGridPoint {
	int x, y, index;
};

bool gridPoseEqualsToGridPose(ISGridPose2D firstPose, ISGridPose2D secondPose);
bool pointEqualsToPoint(ISGridPoint point1, ISGridPoint point2);
bool setContainsPoint(vector<ISGridPoint> set, ISGridPoint point);
bool setContainsPose(vector<ISGridPose2D> set, ISGridPose2D pose);
ISGridPoint euclideanLaserPoint(double distance, double angle, float laser_to_robot, float x_res, float y_res, int index);
vector<ISGridPose2D> generateGridPoses(ISGridPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations, float x_res, float y_res);
vector<ISGridPoint> transformGridPoints(ISGridPose2D startPose, ISGridPose2D endPose, vector<ISGridPoint> points);
int sumGridDifferences(vector<ISGridPoint> firstSet, vector<ISGridPoint> secondSet, bool printdiff);
ISGridPoint laserToWorld(double distance, double angle, ISGridPose2D currentPose, float laser_to_robot, float x_res, float y_res);
ISGridPoint robotToWorld(ISGridPose2D pose, ISGridPoint point);
int getPoseDifference(ISGridPose2D pose1, ISGridPose2D pose2);
