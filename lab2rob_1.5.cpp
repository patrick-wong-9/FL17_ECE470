#include "lab2pkg/lab2.h" 
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */


//arrays defining Waypoints
double home[]={120*PI/180,-90*PI/180,90*PI/180,-90*PI/180,-90*PI/180,0};

//Joint Angles
	double P11[] = {121.6*PI/180, -60.31*PI/180, 131.18*PI/180, -160.75*PI/180, -90.07*PI/180, 329.90*PI/180-2*PI};
	double P12[] = {121.6*PI/180, -69.05*PI/180, 130.02*PI/180, -150.86*PI/180, -90.07*PI/180, 329.90*PI/180-2*PI};
	double P13[] = {121.6*PI/180, -76.74*PI/180, 127.57*PI/180, -140.71*PI/180, -90.07*PI/180, 329.90*PI/180-2*PI};
	
	double P21[] = {135.70*PI/180, -64.52*PI/180, 145.26*PI/180, -170.58*PI/180, -90.05*PI/180, 344.00*PI/180-2*PI};
	double P22[] = {136.42*PI/180, -76.05*PI/180, 142.64*PI/180, -156.44*PI/180, -90.04*PI/180, 344.70*PI/180-2*PI};
	double P23[] = {136.41*PI/180, -86.30*PI/180, 139.50*PI/180, -143.04*PI/180, -90.05*PI/180, 344.70*PI/180-2*PI};
	
	double P31[] = {158.91*PI/180, -65.16*PI/180, 146.50*PI/180, -171.15*PI/180, -89.99*PI/180, 1.66*PI/180};
	double P32[] = {158.90*PI/180, -76.99*PI/180, 145.07*PI/180, -157.89*PI/180, -89.99*PI/180, 1.65*PI/180};
	double P33[] = {158.90*PI/180, -87.90*PI/180, 141.83*PI/180, -143.74*PI/180, -90.00*PI/180, 1.64*PI/180};
// array to define final velocity of point to point moves.  For now slow down to zero once 
// each point is reached
double arrv[]={0,0,0,0,0,0};

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

std::vector<double> Q11 (P11,P11+sizeof(P11) / sizeof(P11[0]));
std::vector<double> Q12 (P12,P12+sizeof(P12) / sizeof(P12[0]));
std::vector<double> Q13 (P13,P13+sizeof(P13) / sizeof(P13[0]));
std::vector<double> Q21 (P21,P21+sizeof(P21) / sizeof(P21[0]));
std::vector<double> Q22 (P22,P22+sizeof(P22) / sizeof(P22[0]));
std::vector<double> Q23 (P23,P23+sizeof(P23) / sizeof(P23[0]));
std::vector<double> Q31 (P31,P31+sizeof(P31) / sizeof(P31[0]));
std::vector<double> Q32 (P32,P32+sizeof(P32) / sizeof(P32[0]));
std::vector<double> Q33 (P33,P33+sizeof(P33) / sizeof(P33[0]));

std::vector<double> v (arrv,arrv+sizeof(arrv) / sizeof(arrv[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q[3][3] = {
    {Q11, Q12, Q13},
    {Q21, Q22, Q23},
    {Q31, Q32, Q33}
};


// Global bool variables that are assigned in the callback associated when subscribed 
// to the "ur3/position" topic
bool isReady=1;
bool pending=0;
bool gripReady = 1;
bool changed = 0;
// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
						  // and is ready to be told to go to another point if desired.
	pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
//	ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}

void grip_callback(const ur_msgs::IOStates::ConstPtr &msg) {
  gripReady = msg->digital_in_states[0].state;
  changed = 1;
}

int suction_on(ros::ServiceClient& srv_SetIO, ur_msgs::SetIO& srv){
	int error = 0;
	ros::Rate loop_rate(SPIN_RATE);
	srv.request.fun = 1;
	srv.request.pin = 0;  //Digital Output 0
	srv.request.state = 1.0; //Set DO0 on
	if (srv_SetIO.call(srv)) {
		ROS_INFO("True: Switched Suction ON");
	} else {
		ROS_INFO("False");
	}
	ros::Duration(1).sleep();
	changed = 0;
	while (changed == 0){
	ros::spinOnce();
	loop_rate.sleep();
	ROS_INFO("Checking");
	}
	if (gripReady == 0){
	ROS_INFO("Couldn't pick the block");
	error = 1;
	}
	return error;
}


int move_arm(	ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
{
    int error = 0;
    ece470_ur3_driver::command driver_msg;

	driver_msg.destination=dest;  // Set desired position to move home 
	driver_msg.duration=duration;
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
									  // the subscriber will not receive this message.
	int spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}

	ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}	
    
    return error;
}

int move_block(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int start_loc,
                int start_height,
                int end_loc,
                int end_height)
{
    int error = 0;
    move_arm( pub_command , loop_rate, QH, 2.0);
    
	ROS_INFO("sending Goals");
	move_arm( pub_command , loop_rate, Q[start_loc][start_height], 2.0);

	error = suction_on(srv_SetIO, srv);
	if (error == 1)
	{
		return error;
	}
	
	move_arm( pub_command , loop_rate, QH, 2.0);

	move_arm( pub_command , loop_rate, Q[end_loc][end_height], 2.0);

	srv.request.fun = 1;
	srv.request.pin = 0; // Digital Output 0
	srv.request.state = 0.0; //Set DO0 off
	if (srv_SetIO.call(srv)) {
		ROS_INFO("True: Switched Suction OFF");
	} else {
		ROS_INFO("False");
	}
    return error;
}

int main(int argc, char **argv)
{

//initialization & variable definition
	ros::init(argc, argv, "lab2node");	//initialzation of ros required for each Node.
	ros::NodeHandle nh;				//handler for this node.
	
	//initialized publisher ur3/command, buffer size of 10.
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	
	ros::Subscriber grip = nh.subscribe("ur_driver/io_states", 1, grip_callback);

	ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
	ur_msgs::SetIO srv;

	ece470_ur3_driver::command driver_msg;

	std::string inputString;
	std::string input1;
	std::string input2;	
	/*std::vector<double> Q [3][3] = {
    		
    		{Q21, Q22, Q23},
    		{Q11, Q12, Q13},    		
    		{Q31, Q32, Q33}
			};
*/
	std::cout << "Enter initial tower position";
	std::getline(std::cin, input1);
	std::cout << "Enter final tower position";
	std::getline(std::cin, input2);

	if(input1=="1"){

		if(input2=="2"){
			//1-2
Q[0][0]=Q11;
Q[0][1]=Q12;
Q[0][2]=Q13;
Q[1][0]=Q21;
Q[1][1]=Q22;
Q[1][2]=Q23;
Q[2][0]=Q31;
Q[2][1]=Q32;
Q[2][2]=Q33;
		}else{
		//1-3
Q[0][0]=Q11;
Q[0][1]=Q12;
Q[0][2]=Q13;
Q[1][0]=Q31;
Q[1][1]=Q32;
Q[1][2]=Q33;
Q[2][0]=Q21;
Q[2][1]=Q22;
Q[2][2]=Q23;
		}
	}
	if(input1=="2"){

			if(input2=="1"){
			//2-1
Q[0][0]=Q21;
Q[0][1]=Q22;
Q[0][2]=Q23;
Q[1][0]=Q11;
Q[1][1]=Q12;
Q[1][2]=Q13;
Q[2][0]=Q31;
Q[2][1]=Q32;
Q[2][2]=Q33;
			}else{
			//2-3
Q[0][0]=Q31;
Q[0][1]=Q32;
Q[0][2]=Q33;
Q[1][0]=Q11;
Q[1][1]=Q12;
Q[1][2]=Q13;
Q[2][0]=Q21;
Q[2][1]=Q22;
Q[2][2]=Q23;
			}
		}
	if(input1=="3"){
	
		if(input2=="1"){
			//3-1
Q[0][0]=Q21;
Q[0][1]=Q22;
Q[0][2]=Q23;
Q[1][0]=Q31;
Q[1][1]=Q32;
Q[1][2]=Q33;
Q[2][0]=Q11;
Q[2][1]=Q12;
Q[2][2]=Q13;
			}else{
		//3-2
Q[0][0]=Q31;
Q[0][1]=Q32;
Q[0][2]=Q33;
Q[1][0]=Q21;
Q[1][1]=Q22;
Q[1][2]=Q23;
Q[2][0]=Q11;
Q[2][1]=Q12;
Q[2][2]=Q13;
			}
	}

	while(!ros::ok()){};	//check if ros is ready for operation
		
	ROS_INFO("sending Goals 0");

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
 	int error = 0;
 		error = move_block( pub_command,loop_rate,srv_SetIO,srv,0,2,1,0);
 		if (error == 1)
 		{
 			goto endcode;
 		}
 		error = move_block( pub_command,loop_rate,srv_SetIO,srv,0,1,2,0);
 		if (error == 1)
 		{
 			goto endcode;
 		}
 		error = move_block( pub_command,loop_rate,srv_SetIO,srv,1,0,2,1);
 		if (error == 1)
 		{
 			goto endcode;
 		}
 		error = move_block( pub_command,loop_rate,srv_SetIO,srv,0,0,1,0);
 		if (error == 1)
 		{
 			goto endcode;
 		}
 		error = move_block( pub_command,loop_rate,srv_SetIO,srv,2,1,0,0);
 		if (error == 1)
 		{
 			goto endcode;
 		}
 		error = move_block( pub_command,loop_rate,srv_SetIO,srv,2,0,1,1);
 		if (error == 1)
 		{
 			goto endcode;
 		}
 		error = move_block( pub_command,loop_rate,srv_SetIO,srv,0,0,1,2);
 		if (error == 1)
 		{
 			goto endcode;
 		}

 	endcode:
		srv.request.fun = 1;
		srv.request.pin = 0;  //Digital Input 0
		srv.request.state = 0.0; //Set DO0 off ;

	return 0;
}