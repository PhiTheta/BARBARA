
using namespace std;

struct ISPose2D {
    float x, y, angle;
};

struct ISPoint {
    float x, y;
};

ISPoint laserCartesian(double distance, double angle);
ISPoint laserToWorld(double distance, double angle, ISPose2D currentPose);
float distanceToPoint(ISPoint point);
vector<float> getDistances(vector<ISPoint> set);
ISPoint minValues(vector<ISPoint> set);
vector<ISPoint> shiftPoints(vector<ISPoint> set, ISPoint offset);
double getCorrelation(vector<ISPoint> firstSet, vector<ISPoint>secondSet, ISPoint shift);
vector<ISPose2D> generatePoses(ISPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations);
vector<ISPoint> transformPoints(ISPose2D startPose, ISPose2D endPose, vector<ISPoint> points);
