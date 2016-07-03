#include "obstacle.h"

using namespace std;

namespace Kalman {

obstacleEKF::obstacleEKF()
{
	setDim(3, 1, 3, 1, 3);
	T = 0.05;
	Distance = 0.15;
}

bool obstacleEKF::dataFeasibility(const Vector& z)
{
	rc = (z(1)*z(1)/2 - (z(2)*z(2)-Distance*Distance)/4 - (z(3)*z(3)-Distance*Distance)/4 + Distance*Distance/2) / 2 / (sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1));
	temp_A = 2*Distance*(z(1)/2+rc);
	temp_B = (z(3)*z(3)-Distance*Distance)/4 - (z(2)*z(2)-Distance*Distance)/4 + 2*rc*(sqrt(z(3)*z(3)-Distance*Distance)/2 - sqrt(z(2)*z(2)-Distance*Distance)/2);
	
	if (temp_A*temp_A-temp_B*temp_B < 0)
	{
		return false;	
	}
	else return true;
}

double obstacleEKF::sensorMeasure(const Vector& z)
{
	return z(1)/2*sqrt(temp_A*temp_A-temp_B*temp_B)/abs(temp_A);
}

void obstacleEKF::makeBaseA()
{
	A(1,1) = 1.0;
	A(1,2) = T;
	A(1,3) = 0.5*T*T;
	A(2,1) = 0.0;
	A(2,2) = 1.0;
	A(2,3) = T;
	A(3,1) = 0;
	A(3,2) = 0;
	A(3,3) = 1;
}

void obstacleEKF::makeBaseW()
{
	W(1,1) = 1.0;
	W(1,2) = 0.0;
	W(1,3) = 0.0;
	W(2,1) = 0.0;
	W(2,2) = 1.0;
	W(2,3) = 0.0;
	W(3,1) = 0.0;
	W(3,2) = 0.0;
	W(3,3) = 1.0;
}

// system error covariance
void obstacleEKF::makeBaseQ()
{
	Q(1,1) = 0.002*0.002;
	Q(1,2) = 0.0;
	Q(1,3) = 0.0;
	Q(2,1) = 0.0;
	Q(2,2) = 0.01*0.01;
	Q(2,3) = 0.0;
	Q(3,1) = 0.0;
	Q(3,2) = 0.0;
	Q(3,3) = 0.025*0.025;
}

void obstacleEKF::makeBaseH()
{
	H(1,1) = 1.0;
	H(1,2) = 0.0;
	H(1,3) = 0.0;
}

void obstacleEKF::makeV1(const Vector& z)
{
	rc = (z(1)*z(1)/2 - (z(2)*z(2)-Distance*Distance)/4 - (z(3)*z(3)-Distance*Distance)/4 + Distance*Distance/2) / 2 / (sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1));

	temp_A = 2*Distance*(z(1)/2+rc);

	temp_B = (z(3)*z(3)-Distance*Distance)/4 - (z(2)*z(2)-Distance*Distance)/4 + 2*rc*(sqrt(z(3)*z(3)-Distance*Distance)/2 - sqrt(z(2)*z(2)-Distance*Distance)/2);

	rcd_1 = (2*(sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1)) * z(1) + 2 * (z(1)*z(1)/2 - (z(2)*z(2)-Distance*Distance)/4 - (z(3)*z(3)-Distance*Distance)/4 + Distance*Distance/2)) / pow(2*(sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1)),2);

	rcd_2 = (2*(sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1)) * -z(2)/2 - 0.5*pow((z(2)*z(2)-Distance*Distance),-0.5) * (z(1)*z(1)/2 - (z(2)*z(2)-Distance*Distance)/4 - (z(3)*z(3)-Distance*Distance)/4 + Distance*Distance/2)) / pow(2*(sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1)),2);

	rcd_3 = (2*(sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1)) * -z(3)/2 - 0.5*pow((z(3)*z(3)-Distance*Distance),-0.5) * (z(1)*z(1)/2 - (z(2)*z(2)-Distance*Distance)/4 - (z(3)*z(3)-Distance*Distance)/4 + Distance*Distance/2)) / pow(2*(sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1)),2);

	V(1,1) = 0.5*sqrt(temp_A*temp_A-temp_B*temp_B)/temp_A + z(1)/2 * (abs(temp_A)*0.5*pow((temp_A*temp_A-temp_B*temp_B),-0.5) * (4*temp_A*Distance*(0.5+rcd_1) - 4*temp_B*rcd_1*(sqrt(z(3)*z(3)-Distance*Distance)/2-sqrt(z(2)*z(2)-Distance*Distance)/2)) - ((temp_A > 0)-(temp_A < 0))*sqrt(temp_A*temp_A-temp_B*temp_B)*2*Distance*(0.5+rcd_1)) / pow(temp_A,2);

	V(1,2) = -z(1)*temp_B/2 * pow((temp_A*temp_A-temp_B*temp_B),-0.5) * (-z(2)/2 + 2*rcd_2*(sqrt(z(3)*z(3)-Distance*Distance)/2-sqrt(z(2)*z(2)-Distance*Distance)/2) - rc*pow((z(2)*z(2)-Distance*Distance),-0.5)*z(2)) / abs(temp_A);

	V(1,3) = -z(1)*temp_B/2 * pow((temp_A*temp_A-temp_B*temp_B),-0.5) * (z(3)/2 + 2*rcd_2*(sqrt(z(3)*z(3)-Distance*Distance)/2-sqrt(z(2)*z(2)-Distance*Distance)/2) + rc*pow((z(3)*z(3)-Distance*Distance),-0.5)*z(3)) / abs(temp_A);
}

void obstacleEKF::makeV2()
{
	V(1,1) = 0;
	V(1,2) = 0;
	V(1,3) = 6;
}

// measurement error covariance
void obstacleEKF::makeBaseR()
{
	R(1,1) = 0.0225;
	R(1,2) = 0.0;
	R(1,3) = 0.0;
	R(2,1) = 0.0;
	R(2,2) = 0.0225;
	R(2,3) = 0.0;
	R(3,1) = 0.0;
	R(3,2) = 0.0;
	R(3,3) = 0.0225;
}

void obstacleEKF::makeProcess()
{
	x(1) = x(1) + T*x(2) + 0.5*T*T*x(3);
	x(2) = x(2) + T*x(3);
}

void obstacleEKF::makeMeasure()
{
	z(1) = x(1);
}
}
