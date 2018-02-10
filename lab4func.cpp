#include "lab4pkg/lab4.h"
#include "math.h"
/** 
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{

	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end,yaw; 
	double xgrip,ygrip,zgrip;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;
	double p,r,l;

	a1 = 0;
	d1 = 0.152;
	a2 = 0.244;
	d2 = 0.120;
	a3 = 0.213;
	d3 = -0.093;
	a4 = 0;
	d4 = 0.083;
	a5 = 0;
	d5 = 0.083;
	a6 = 0.0535;
	d6 = (0.082+0.056);

	yaw = yaw_WgripDegree*PI/180;

	xgrip = xWgrip + 0.15+0.0116;
	ygrip = yWgrip - 0.15+0.0116 ;
	zgrip = zWgrip - 0.013;

	xcen= xgrip - a6*cos(yaw);
	ycen= ygrip - a6*sin(yaw);
	zcen= zgrip;

	p = d2 + d3 + d4;
	
	theta1 = atan2(ycen,xcen) - asin(p/(sqrt(xcen*xcen + ycen*ycen)));  
	theta6 = (PI/2) - yaw + theta1; 
 
	x3end = -p*cos(yaw+theta6) - d5*sin(yaw+theta6) + xcen;
	y3end = -p*sin(yaw+theta6) + d5*cos(yaw+theta6) + ycen;
	z3end = zcen + d6;

	r = sqrt(x3end*x3end + y3end*y3end);
	l = sqrt(r*r + (z3end-d1)*(z3end-d1));

	theta2= acos(r/l) + acos((l*l + a2*a2 - a3*a3)/(2*l*a2)); 
	theta3= acos((l*l - a2*a2 - a3*a3)/(2*a2*a3)); 
	theta4= acos((l*l - a2*a2 + a3*a3)/(2*l*a3)) + asin(r/l); 
	theta5=-PI/2;  
	
	//Using Angle convention from Lab Manual
	theta2 = -theta2;
	theta3 = theta3;
	theta4 = -theta4;
	theta6 = theta6;


	// View values
	//use cout
	cout<<"theta1: "<< theta1<<endl;
	cout<<"theta2: "<< theta2<<endl;
	cout<<"theta3: "<< theta3<<endl;
	cout<<"theta4: "<< theta4<<endl;
	cout<<"theta5: "<< theta5<<endl;
	cout<<"theta6: "<< theta6<<endl;

	// check that your values are good BEFORE sending commands to UR3
	//lab_fk calculates the forward kinematics and convert it to std::vector<double>
	return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}
