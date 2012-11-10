#include <iostream>
#include <iomanip>
#include "odoEKF.hpp"

using namespace std;

inline int sgn(double x) 
{ 
	return x == 0 ? 0 : x > 0 ? 1 : -1; 
}

odoEKF::odoEKF():_iteration(0), _dt(0.1), _vc(0), _wc(0), _ac(0), _alphac(0), _xe(0), _ye(0), _ae(0), _ve(0), _we(0)
{
	_Q = fmat(5,5);
	_Q.zeros();
	_Q.diag().fill(0.1);
	_Q(2,2) = _Q(3,3) = 0.01;
	_Q(0,0) = 1;
	cout << "Q matrix:" << endl;
	cout << _Q << endl;
	
	_H = Mat<int>(3,5);
	_H.zeros();
	_H.eye();
	cout << "H matrix:" << endl;
	cout << _H << endl;
	
	_PPri = fmat(5,5);
	_PPri.zeros();
	_PPri.diag().fill(0.01);
	cout << "PPri matrix:" << endl;
	cout << _PPri << endl;
}

odoEKF::~odoEKF()
{
}

float odoEKF::dt()
{
	return _dt;
}

void odoEKF::setDt(float dt)
{
	_dt = dt;
	cout << "dt set to " << setprecision(3) << dt << endl;
}

void odoEKF::updateControl(double vc, double ac, double wc, double alphac)
{
	_vc = vc;
	_ac = ac;
	_wc = wc;
	_alphac = alphac;
	//cout << "Updating control:" << endl;
	//cout << "v: " << setprecision(3) << vc << "; a: " << setprecision(3) << ac;
	//cout << "; w: " << setprecision(3) << wc << "; alpha: " << setprecision(3) << alphac << endl;
}

void odoEKF::insertMeasurement(double xm, double ym, double am)
{
	//Control signals calculation
	double u1 = sgn(_ve-_vc)*_ac;
	double u2 = sgn(_we-_wc)*_alphac;
	
	//State estimation
	fmat xPri = fmat(5,1);
	xPri(0,0) = _xe+_ve*_dt*cos(_ae);
	xPri(1,0) = _ye+_ve*_dt*sin(_ae);
	xPri(2,0) = _ae+_we*_dt;
	xPri(3,0) = _ve+u1*_dt;
	xPri(4,0) = _we+u2*_dt;
	
	//Measurement matrix
	fmat z = fmat(3,1);
	z(0,0) = xm;
	z(1,0) = ym;
	z(2,0) = am;
	
	//Innovation
	fmat v = z-_H*xPri;
	
	//Jacobian matrix
	fmat F = fmat(5,5);
	F.eye();
	F(0,2) = -_ve*_dt*sin(_ae);
	F(0,3) = _dt*cos(_ae);
	F(1,2) = _ve*_dt*cos(_ae);
	F(1,3) = _dt*sin(_ae);
	F(2,4) = _dt;
		
	//State prediction covariance
	_PPri = F*_PPri*F.t() + _Q;
	
	//Measurement covariance
	fmat R = fmat(3,3);
	R.zeros();
	R(0,0) = 0.1*pow(fabs(z(0,0)),2);
	for (int i = 1; i < 3; i++) {
		R(i,i) = 1.5*pow(fabs(z(i,0)),2);
	}
	
	//Residual covariance
	fmat S = _H*_PPri*_H.t() + R;
	
	//Filter gain
	fmat W = _PPri*_H.t()*S.i();
	
	//Update state estimation
	fmat xPos = xPri+W*v;
	_xe = xPos(0,0);
	_ye = xPos(1,0);
	_ae = xPos(2,0);
	_ve = xPos(3,0);
	_we = xPos(4,0);
	
	//Update state covariance
	fmat Eye = fmat(5,5);
	Eye.eye();
	_PPri = (Eye-W*_H)*_PPri;
	
	_iteration++;
}

void odoEKF::readEstimates(double *xe, double *ye, double *ae, double *ve, double *we)
{
	*xe = _xe;
	*ye = _ye;
	*ae = _ae;
	*ve = _ve;
	*we = _we;
}
