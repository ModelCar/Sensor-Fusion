#include "obstacle.h"
#include <fstream>

using namespace std;
using namespace Kalman;

int main()
{
	//initialize Kalman Filter
	obstacleEKF filter;
	static const double temp_P[] = {0.25, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25};
	Matrix P(3,3,temp_P);
	Vector x(3);
	x(1) = 5; //position
	x(2) = -3; //velocity
	x(3) = -2; //acceleration
	filter.init(x,P);

	//open files for reading and writing
	selectKVectorContext(createKVectorContext(" ", "[ ", " ];", 4));
	selectKMatrixContext(createKMatrixContext(" ", " ;\n  ", "[ ", " ];", 4));

	ifstream dataInput;
	ofstream dataOutput;
	std::string tmpStr;

	dataInput.open("sensor_data.m",ifstream::in);
	dataOutput.open("sensor_fusion.csv", ofstream::out | ofstream::trunc);

	if (dataInput.fail())
	{
		cout << "Unable to open input file!" << endl;
		return 0;
	}

	cout.setf(ios::fixed, ios::floatfield); 	
	cout.precision(5);
	cout << endl << "...Loading data from file <sensor_data.m>." << endl;

	//initialize data holder
	const unsigned N = 500;
	Vector F(N);
	Matrix measure_ultrasonic(21,N);
	Vector measure_depthmap(N);
	Matrix trajectory(3,N);

	//read the control vector.
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>F;

	if (dataInput.fail())
	{
		cout<<"IO error!"<<endl;
		return 0;
	}

	//read the measurements of ultrasonic sensor.
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>measure_ultrasonic;

	if (dataInput.fail())
	{
		cout<<"IO error!"<<endl;
		return 0;
	}

	//read the measurements of depth map. 
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>measure_depthmap;

	if (dataInput.fail())
	{
		cout<<"IO error!"<<endl;
		return 0;
	}
	
	//read the actual trajectory. 
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>tmpStr;
	dataInput>>trajectory;

	if (dataInput.fail())
	{
		cout<<"IO error!"<<endl;
		return 0;
	}

	//initialize input variables
	Vector u_(1,1.0);
	Vector z_sensor(3);
	Vector z_depthmap(1);
	unsigned k;

	cout << endl << "...Running sensor fusion algorithm" << endl;
	cout << endl << "Ultrasonic sensor data     Depth map data             Fused object states(d,v,a)            Simulator ground truth(d,v,a)" << endl;

	//run sensor fusion
	for (unsigned i = 1; i <= 25; i++)
	{
		z_depthmap(1) = measure_depthmap(i);

		z_sensor(1) = measure_ultrasonic(1,i);
		z_sensor(2) = measure_ultrasonic(2,i);
		z_sensor(3) = measure_ultrasonic(3,i);

		//discard infeasible sensor measurement and use the next one
		k = 0;
		while (!filter.dataFeasibility(z_sensor))
		{
			k++;
			if (k == 7)
			{
				filter.step(u_, z_depthmap);
				cout << "ultrasonic measurements are not used for EKF update";	
				break;
			}
			z_sensor(1) = measure_ultrasonic(1+(3*k),i);
			z_sensor(2) = measure_ultrasonic(2+(3*k),i);
			z_sensor(3) = measure_ultrasonic(3+(3*k),i);		
		}

		if (k <= 6)
		{
			filter.step(u_, z_sensor, z_depthmap);
		}

		cout << "    [  " << filter.sensorMeasure(z_sensor) << "  ];      " << z_depthmap << "    " << filter.getX();
		cout << "   [  " << trajectory(1,i) << " " << trajectory(2,i) << " " << trajectory(3,i) << "  ];" << endl;
		//cout << filter.calculateP() << endl;
		dataOutput << filter.sensorMeasure(z_sensor) << z_depthmap << filter.getX() << trajectory(1,i)<<" "<<trajectory(2,i)<<" "<<trajectory(3,i) << endl;
	}
	cout << endl << "...Fused data is written to file <sensor_fusion.csv>." << endl << endl;
}
