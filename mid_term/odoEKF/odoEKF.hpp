
#ifndef _ODOEKF_HPP_
#define _ODOEKF_HPP_


#include <armadillo>
using namespace arma;

class odoEKF {
private:
		fmat _Q;
		Mat<int> _H;
		fmat _PPri;
		int _iteration;
		float _dt;
		
		//Estimation
		double _xe;
		double _ye;
		double _ae;
		double _ve;
		double _we;
		
		//Control signals
		double _vc;
		double _ac;
		double _wc;
		double _alphac;
		
public:
	odoEKF();
	~odoEKF();
	float dt();
	void setDt(float dt);
	void insertMeasurement(double xm, double ym, double am);
	void updateControl(double vc, double ac, double wc, double alphac);
	void readEstimates(double *xe, double *ye, double *ae, double *ve, double *we);
};

#endif
