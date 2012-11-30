
using namespace std;

struct ISPose2D {
    double x, y, angle;
};

struct ISGridPose2D {
	int x, y;
	double angle;
};

struct ISPoint {
    double x, y;
};

struct ISGridPoint {
	int x, y;
};

ISPoint laserCartesian(double distance, double angle, float laser_to_robot);
ISPoint laserToWorld(double distance, double angle, ISPose2D currentPose, float laser_to_robot);
double distanceToPoint(ISPoint point);
vector<ISPoint> filterPoints(vector<ISPoint> points);
vector<double> getDistances(vector<ISPoint> set);
double sumDifferences(vector<double> firstSet, vector<double> secondSet, bool printdiff);
double sumDifferences(vector<ISPoint> firstSet, vector<ISPoint> secondSet, bool printdiff);
ISPoint minValues(vector<ISPoint> set);
vector<ISPoint> shiftPoints(vector<ISPoint> set, ISPoint offset);
double getCorrelation(vector<ISPoint> firstSet, vector<ISPoint>secondSet, ISPoint shift);
vector<ISPose2D> generatePoses(ISPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations);
vector<ISPoint> transformPoints(ISPose2D startPose, ISPose2D endPose, vector<ISPoint> points);
bool poseEqualsToPose(ISPose2D firstPose, ISPose2D secondPose);

double avg(vector<double> set);
double crossCorrelation(vector<double> X, vector<double> Y, int timeLag);
double iterativeCrossCorrelation(vector<double> X, vector<double> Y);


//GRID ORIENTED METHODS

bool gridPoseEqualsToGridPose(ISGridPose2D firstPose, ISGridPose2D secondPose);
bool pointEqualsToPoint(ISGridPoint point1, ISGridPoint point2);
bool setContainsPoint(vector<ISGridPoint> set, ISGridPoint point);
bool setContainsPose(vector<ISGridPose2D> set, ISGridPose2D pose);
ISGridPoint euclideanLaserPoint(double distance, double angle, float laser_to_robot, float x_res, float y_res);
vector<ISGridPose2D> generateGridPoses(ISGridPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations, float x_res, float y_res);
vector<ISGridPoint> transformGridPoints(ISGridPose2D startPose, ISGridPose2D endPose, vector<ISGridPoint> points, float x_res, float y_res);
int sumGridDifferences(vector<ISGridPoint> firstSet, vector<ISGridPoint> secondSet, bool printdiff);
ISGridPoint laserToWorld(double distance, double angle, ISGridPose2D currentPose, float laser_to_robot, float x_res, float y_res);
