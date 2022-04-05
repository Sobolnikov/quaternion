#include <iostream>
#include <eigen-3.4.0/Eigen/Dense>

using Eigen::VectorXf;
using Eigen::Quaternion;

using namespace std;

main()
{

	//Angular velocity from quaternions difference
	VectorXf euler(3);
	VectorXf omega(3);

  	float dt = 0.1;

	Quaternion<float> qt_1 (  1,  0, 0,    0);	
	Quaternion<float> qt   (  1,  0, 0.09, 0);
	euler = qt.toRotationMatrix().eulerAngles(2, 1, 0);
	cout << "qt = " << qt << endl << euler<< endl;

  	Quaternion<float> dqdt;
  	dqdt.coeffs() = (2/dt)*(qt.coeffs() - qt_1.coeffs());

  	Quaternion<float> omq = dqdt*qt.conjugate();
  	omega = omq.vec();

  	cout << "omega = " << endl <<omega << endl;

  	//Quaternion from angular velocity

  	Quaternion<float> q ( 1, 0, 0, 0);	
  	Quaternion<float> m1, m2;
  	Quaternion<float> l1, l2;

  	//First order method
  	m1.vec() = (dt/2)*omega; 
  	m1.w() = 1; 
  	//Second order method
  	m2.vec() = (dt/2)*omega; 
  	m2.w() = 1 - omega.norm()*omega.norm()*dt*dt/12;  	

  	l1 = q;
  	l2 = q;
  	cout << "m1 = " << m1 << "\t\t\t"
  	     << "m2 = " << m2 << endl;

  	for (int i = 0; i < 73; i++)  
  	{
  		l1 = l1*m1;
  		l1.normalize();
  		l2 = l2*m2;
  		l2.normalize();
  		cout << i << " l1 = " << l1 << "\t\t\t"
  		          << " l2 = " << l2 << endl;
  	}

  	euler = l1.toRotationMatrix().eulerAngles(2, 1, 0);
  	cout << euler << endl;
  	euler = l2.toRotationMatrix().eulerAngles(2, 1, 0);
  	cout << euler << endl;
}