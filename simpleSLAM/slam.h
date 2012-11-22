
using namespace std;

struct TPose2D {
    float x, y, angle;
};

struct TPoint {
    float x, y;
};


float distanceToPoint(TPoint point);
vector<float> getDistances(vector<TPoint> set);
float minX(vector<TPoint> set);
float minY(vector<TPoint> set);
vector<TPoint> shiftPoints(vector<TPoint> set, TPoint offset);
double getCorrelation(vector<TPoint> firstSet, vector<TPoint>secondSet, TPoint shift);
vector<TPose2D> generatePoses(TPose2D currentPose, float maxRadius, int numRadiuses, int numPositions, float maxAngleDeviation, int numOrientations);
vector<TPoint> transformPoints(TPose2D startPose, TPose2D endPose, vector<TPoint> points);