
using namespace std;

struct ISPose2D {
    double x, y, angle;
};

struct ISPoint {
    double x, y;
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

double avg(vector<double> set);
double crossCorrelation(vector<double> X, vector<double> Y, int timeLag);
double iterativeCrossCorrelation(vector<double> X, vector<double> Y);

bool poseEqualsToPose(ISPose2D firstPose, ISPose2D secondPose);
