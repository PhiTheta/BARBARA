#include "odoEKF.hpp"
#include <iostream>
using namespace std;

inline double randomize()
{
	int rnd = rand() % 100;
	rnd -= 50;
	double frand = rnd/100.0;
	return frand;
}

int main(int argc, char** argv)
{
	cout << "Running test" << endl;
	
	cout << "Creating control signals: ";
	double vc[100];
	for (int i = 0; i < 50; i++) vc[i] = 1;
	cout << "vc, ";
	
	double wc[100];
	for (int i = 51; i < 100; i++) wc[i] = 0.1;
	cout << "wc, ";
	
	double ac[100];
	for (int i = 0; i < 100; i++) ac[i] = 0.15;
	cout << "ac, ";
	
	double alphac[100];
	for (int i = 0; i < 100; i++) alphac[i] = 0.15;
	cout << "alphac" << endl;
	
	
	cout << "Creating real values: ";
	
	
	double xr[100];
	for (int i = 0; i < 100; i++) xr[i] = i;
	cout << "xr, ";
	
	double yr[100];
	for (int i = 0; i < 100; i++) yr[i] = sin(xr[i]/10);
	cout << "yr, ";
	
	double ar[100];
	for (int i = 0; i < 100; i++) ar[i] = 0.15;
	cout << "ar" << endl;
	
	
	cout << "Creating noisy measurement: ";
	
	
	double xm[100];
	for (int i = 0; i < 100; i++) xm[i] = xr[i]+randomize()*(i+1)*0.01;
	cout << "xm, ";
	
	double ym[100];
	for (int i = 0; i < 100; i++) ym[i] = yr[i]+randomize()*(i+1)*0.01;
	cout << "ym, ";
	
	double am[100];
	for (int i = 0; i < 100; i++) am[i] = ar[i]+randomize()*(i+1)*0.01;
	cout << "am" << endl;
	
		
	cout << "Creating EKF" << endl;
	
	odoEKF *obj = new odoEKF();
	obj->setDt(0.5);
	
	for (int i = 0; i < 100; i++) {
		obj->updateControl(vc[i], ac[i], wc[i], alphac[i]);
		obj->insertMeasurement(xm[i], ym[i], am[i]);
		double xe, ye, ae, ve, we;
		obj->readEstimates(&xe, &ye, &ae, &ve, &we);
		cout << "Real/Measurement/Estimate"
			 << "X: "   << xr[i] << "/" << xm[i] << "/" << xe
			 << "; Y: " << yr[i] << "/" << ym[i] << "/" << ye 
			 << "; A: " << ar[i] << "/" << am[i] << "/" << ae
			 << "; V: " << ve << "; W: " << we << endl;
			 
		usleep(1000*obj->dt()*10);
	}
	
	

	return 0;
}
