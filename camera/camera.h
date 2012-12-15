
using namespace std;

struct Camera_Obstacle_Alarm{

	bool Red_Target_Flag; 
	bool Blue_Target_Flag;
	int c_x;
	int c_y;

};

struct Camera_Distance{

	float distance;
	float angle;

};
Camera_Obstacle_Alarm Find_Object(unsigned char *image, int width, int height, int color_flag);
Camera_Distance Postion_Object(int X_center, int Y_center, int width, int height);

