//please refer to ekfilter.hpp to get a general idea how kalman filter works, before reading the following explanation of sensor fusion in iron car project.
//
//System Model:
//x_(k+1) = A*x_k + W*w_k
//where:
//x = [d is the system state,
//     v
//     s]
//
//Non-linear Ultrasonic Sensor Measurement Model:
//x_k(1) - z(1)/2 * sqrt((2*Distance*(z(1)/2+rc))²-((z(3)*z(3)-Distance*Distance)/4 - (z(2)*z(2)-Distance*Distance)/4 + 2*rc*(sqrt(z(3)*z(3)-Distance*Distance)/2 - sqrt(z(2)*z(2)-Distance*Distance)/2))²) / abs(2*Distance*(z(1)/2+rc)) = 0
//where: 
//rc = (z(1)*z(1)/2 - (z(2)*z(2)-Distance*Distance)/4 - (z(3)*z(3)-Distance*Distance)/4 + Distance*Distance/2) / 2 / (sqrt(z(2)*z(2)-Distance*Distance)/2 + sqrt(z(3)*z(3)-Distance*Distance)/2 - z(1))
// z = [m1 is the ultrasonic sensor measurement. 
//      m2
//      m3]
//
//since this model is non-linear, it's necessary that we linearize it to the form:
//z_k = H*x_k + V*v_k
//where:
//z_k is the new measurement we feed to Kalman Filter
//H = [0 1]
//V is one linearized term which is dependent on the measurement
//
//to better undestand the ultrasonic sensor measurement model, please refer to this paper:
//H. Peremans, K. Audenaert and J. M. Van Campenhout, ''A High-Resolution Sensor Based on Tri-aural Perception'', IEEE Transactions on Robtoics and Automation, Vol.9, No.1, February 1993.

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "ekfilter.hpp"

namespace Kalman {
class obstacleEKF : public Kalman::EKFilter<double,1>
{
public:
	//default constructor
	obstacleEKF();

	//member funciton used to check the feasibility of a certain 
	//ultrasonic measurements. Bases on the check, it is decided
 	//whether this measurement is used or not. Due to trangulization 
 	//relations, some measurements makes some terms under the square 
	//root negative values in the algorithms, should therefore be 
	//discarded. 
	bool dataFeasibility(const Vector& z);

	//used to calculate the distance to the object based on sensor
	//measurement z. 
	double sensorMeasure(const Vector& z);
protected:
	//refer to ekfilter.hpp for the details of these matrices.
	//these functions override the ones in the EKFilter class.

	//set A matrix of the system model
	void makeBaseA();

	//set W matrix of the system model
	void makeBaseW();

	//set Q matrix of the system model
	void makeBaseQ();

	//set H matrix of the measurement model
	void makeBaseH();

	//set V matrix of the ultrasonic sensor measurement model
	void makeV1(const Vector& z);

	//set V matrix of the depth map measurement model
	void makeV2();

	//set R matrix of the measurement model
	void makeBaseR();

	//set the system model
	void makeProcess();

	//set the measurement model
	void makeMeasure();

	//T is the time step of Kalman Filter
	//Distance is the distance between the receivers of the tri-aural 
	//ultrasonic sensor setup
	double T, Distance;
};

typedef obstacleEKF::Vector Vector;
typedef obstacleEKF::Matrix Matrix;
}

#endif
