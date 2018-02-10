#include "lab56pkg/lab56.h"

extern ImageConverter* ic_ptr; //global pointer from the lab56.cpp

#define SPIN_RATE 20  /* Hz */

bool isReady=1;
bool pending=0;

float SuctionValue = 0.0;

bool leftclickdone = 1;
bool rightclickdone = 1;

/*****************************************************
* Functions in class:
* **************************************************/

//constructor(don't modify)
ImageConverter::ImageConverter():it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera_node/image_raw", 1,
    	&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);
    pub_command=nh_.advertise<ece470_ur3_driver::command>("ur3/command",10);
    sub_position=nh_.subscribe("ur3/position",1,&ImageConverter::position_callback,this);

	sub_io_states=nh_.subscribe("ur_driver/io_states",1,&ImageConverter::suction_callback,this);

	srv_SetIO = nh_.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");


    driver_msg.destination=lab_invk(-.3,-.3,0.2,-90);

	//publish the point to the robot
    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
	driver_msg.duration = 3.0;
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
										  // the subscriber will not receive this message.
	spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO_STREAM("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}
	ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM("Ready for new point");

}

//destructor(don't modify)
ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady;
	pending=msg->pending;
}

void ImageConverter::suction_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
	SuctionValue = msg->analog_in_states[0].state;
}


//subscriber callback function, will be called when there is a new image read by camera
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // create an gray scale version of image
    Mat gray_image;
	cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );
    // convert to black and white img, then associate objects:

// FUNCTION you will be completing
    Mat bw_image = thresholdImage(gray_image); // bw image from own function

// FUNCTION you will be completing
    Mat associate_image = associateObjects(bw_image); // find associated objects

    // Update GUI Window
    imshow("Image window", cv_ptr->image);
    imshow("gray_scale", gray_image);
    imshow("black and white", bw_image);
    imshow("associate objects", associate_image);
    waitKey(3);
    // Output some video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

double avg(int* array,int start_idx, int end_idx)
{
    double temp = 0.0;
    for(int i=start_idx;i<=end_idx;i++)
    {
        temp += array[i];
    }
    return temp/(end_idx-start_idx+1);
}

/*****************************************************
	 * Function for Lab 5
* **************************************************/
// Take a grayscale image as input and return an thresholded image.
// You will implement your algorithm for calculating threshold here.
Mat ImageConverter::thresholdImage(Mat gray_img)
{
		int   totalpixels;
		Mat bw_img  = gray_img.clone(); // copy input image to a new image
		totalpixels	  = gray_img.rows*gray_img.cols;			// total number of pixels in image
		uchar graylevel; // use this variable to read the value of a pixel
		int zt=0; // threshold grayscale value

		int H[256];
        for(int i=0;i<256;i++)
        {
            H[i]=0;
        }

        for(int i=0;i<totalpixels;i++)
        {
            H[bw_img.data[i]]++;
        }

		zt = 0;  // you will be finding this automatically

        double p[256];
        double mu = 0.0;
        for(int i=0;i<256;i++)
        {
            p[i]=(double)H[i]/(double)totalpixels;
            mu += i*p[i];
        }
        double q0_0=p[0];
        double q1_0=1.0-q0_0;

        double mu0_0=0;
        double mu1_0=0.0;
        mu1_0 = (mu-mu0_0*q0_0)/q1_0;

        double sigma_b_0 = q0_0*(1.0-q0_0)*(mu0_0-mu1_0)*(mu0_0-mu1_0);
        double sigma_b_max = sigma_b_0;
        for(int i=1;i<256;i++)
        {
            double q0=p[i]+q0_0;
            if (q0 == 0) q0 = 1.0/(double)totalpixels;
            if (q0 == 1) q0 = (double)(totalpixels-1)/(double)totalpixels;

            double mu0=i*p[i]/q0+q0_0*mu0_0/q0;
            double mu1=(mu-q0*mu0)/(1-q0);
            sigma_b_0=q0*(1.0-q0)*(mu0-mu1)*(mu0-mu1);
            if(sigma_b_0>sigma_b_max)
            {
                sigma_b_max=sigma_b_0;
                zt=i;
            }
            mu0_0=mu0;
            mu1_0=mu1;
            q0_0=q0;
        }
        // zt=90;
        //std::cout<<zt<<std::endl;
		// threshold the image
		for(int i=0; i<totalpixels; i++)
		{
			graylevel = gray_img.data[i];
			if(graylevel>zt) bw_img.data[i]= 255; // set rgb to 255 (white)
			else             bw_img.data[i]= 0; // set rgb to 0   (black)
		}
    //cout<<"threshold: "<<zt<<endl;
	return bw_img;
}
/*****************************************************
	 * Function for Lab 5
* **************************************************/
// Take an black and white image and find the object it it, returns an associated image with different color for each image
// You will implement your algorithm for rastering here

Mat ImageConverter::associateObjects(Mat bw_img)
{
	//initiallize the variables you will use
	int height,width; // number of rows and colums of image
	int red, green, blue; //used to assign color of each objects
	uchar pixel; //used to read pixel value of input image
	height = bw_img.rows;
	width = bw_img.cols;
	int num = 0;
	// initialize an array of labels, assigning a label number to each pixel in the image
	// this create a 2 dimensional array pixellabel[row][col]
	int ** pixellabel = new int*[height];
	for (int i=0;i<height;i++) {
		pixellabel[i] = new int[width];
	}

	// creating a demo image of colored lines
    int label[3000];
    int *equiv[3000];
    int label_freq[3000];
    int true_label[3000];

    for (int i = 0; i < 3000; i++) {
        equiv[i] = &label[i];
        label_freq[i]=0;
        true_label[i]=-1;
    }

    // cout<<"width: "<<width<<" height: "<<height<<endl;
    // cout<<"initialization"<<endl;
	for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
			if (bw_img.data[row*width+col] == 0) pixellabel[row][col] = 0;
            else pixellabel[row][col] = -1;
            // cout<<pixellabel[row][col];
		}
        // cout<<endl;
	}
    // cout<<"first scan"<<endl;
    int labelnum = 1;
    for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
            // cout<<row<<','<<col<<','<<labelnum<<endl;
            int left = -1;
            int above = -1;
            int pixel = pixellabel[row][col];
            if (col != 0) left = pixellabel[row][col-1];
            if (row != 0) above = pixellabel[row-1][col];

            if (pixel != -1) {
                if (left == -1 && above == -1) {
                    pixellabel[row][col] = labelnum;
                    label[labelnum] = labelnum;
                    labelnum++;
                }

                if (left != -1 and above == -1) {
                    pixellabel[row][col] = left;
                }

                if (left == -1 and above != -1) {
                    pixellabel[row][col] = above;
                }

                if (left != -1 && above != -1) {
                    int smallerbaselabel = std::min(*equiv[left],*equiv[above]);
                    int min,max;
                    if(smallerbaselabel==*equiv[left])
                    {
                        min=left;
                        max=above;
                    }
                    else
                    {
                        min=above;
                        max=left;
                    }
                    pixellabel[row][col]=smallerbaselabel;
                    *equiv[max]=*equiv[min];
                    equiv[max]=equiv[min];
                }
            }
		}
	}

    // cout<<"second scan"<<endl;
    for(int row=0;row<height;row++)
    {
        for(int col=0;col<width;col++)
        {
            int pixel=pixellabel[row][col];
            if(pixel!=-1)
            {
                pixellabel[row][col]=*equiv[pixel];
                label_freq[*equiv[pixel]]++;
            }
            // cout<<pixellabel[row][col];
        }
        // cout<<endl;
    }

    labelnum=0;
    for(int row=0;row<height;row++)
    {
        for(int col=0;col<width;col++)
        {
            if(pixellabel[row][col]>0)
            {
                if(label_freq[pixellabel[row][col]]<500||label_freq[pixellabel[row][col]]>1000)
                {
                    pixellabel[row][col]=-1;
                }
                else
                {
                    if(true_label[pixellabel[row][col]] == -1)
                    {
                        true_label[pixellabel[row][col]]=labelnum;
                        labelnum++;
                    }
                }
            }
        }
    }

    for(int row=0;row<height;row++)
    {
        for(int col=0;col<width;col++)
        {
            if(pixellabel[row][col]>0)
            {
                pixellabel[row][col]=true_label[pixellabel[row][col]];
            }
        }
    }

	// assign UNIQUE color to each object
	Mat associate_img = Mat::zeros( bw_img.size(), CV_8UC3 ); // function will return this image
	Vec3b color;
	for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
			switch (  pixellabel[row][col] )
			{

				case 0:
					red    = 255; // you can change color of each objects here
					green = 255;
					blue   = 255;
					break;
				case 1:
					red    = 255; // you can change color of each objects here
					green  = 0;
					blue   = 0;
					break;
				case 2:
					red    = 0;
					green  = 255;
					blue   = 0;
					break;
				case 3:
					red    = 0;
					green  = 0;
					blue   = 255;
					break;
				case 4:
					red    = 255;
					green  = 255;
					blue   = 0;
					break;
				case 5:
					red    = 255;
					green  = 0;
					blue   = 255;
					break;
				case 6:
					red    = 0;
					green  = 255;
					blue   = 255;
					break;
                case 7:
                    red    = 128;
                    green  = 128;
                    blue   = 0;
                    break;
                case 8:
                    red    = 128;
                    green  = 0;
                    blue   = 128;
                    break;
                case 9:
                    red    = 0;
                    green  = 128;
                    blue   = 128;
                 	break;
				default:
					red    = 0;
					green = 0;
					blue   = 0;
					break;
			}

			color[0] = blue;
			color[1] = green;
			color[2] = red;
			associate_img.at<Vec3b>(Point(col,row)) = color;
		}
	}
    //cout<<"number of objects: "<<labelnum<<endl;
    //cout<<"number of objects: "<<label_freq[0]<<endl;

    int centMat[10][3] = {};
    for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
			if(pixellabel[row][col]!=-1){
				centMat[pixellabel[row][col]][0] += row;///label_freq[pixellabel[row][col]];
				centMat[pixellabel[row][col]][1] += col;///label_freq[pixellabel[row][col]];
				centMat[pixellabel[row][col]][2]++;
			}
		}
	}

	
	for(int i=0;i<labelnum;i++){
			centMat[i][0] = (int)(centMat[i][0]/centMat[i][2]);
			centMat[i][1] = (int)(centMat[i][1]/centMat[i][2]);

			color[0] = 0;
			color[1] = 128;
			color[2] = 128;

			float xc = (centMat[i][0]-320)/0.71;
			float yc = (centMat[i][1]-240)/0.71;

			float xw = -(xc+96.69)/1000;
			float yw = -(yc-175.83)/1000;

			cout<<"Centroid of Object "<<i+1<<endl;
			cout<<"Centroid X: "<<xw<<endl;
			cout<<"Centroid Y: "<<yw<<endl;

			for(int x = centMat[i][0]-5; x < centMat[i][0]+5; x++){
				int y = centMat[i][1];
				if(x<0||y<0||x>height-1||y>width-1){}
				else{associate_img.at<Vec3b>(Point(y,x)) = color;}
			}

			for(int y = centMat[i][1]-5; y < centMat[i][1]+ 5; y++){
				int x = centMat[i][0];
				if(x<0||y<0||x>height-1||y>width-1){}
				else{associate_img.at<Vec3b>(Point(y,x)) = color;}
			}



	}
	//CALIBRATION
	// float Or = height/2;
	// float Oc = width/2;
	// cout<<"Or: "<<Or<<endl;
	// cout<<"Oc: "<<Oc<<endl;

	//Beta Calculation
	// float beta = sqrt(pow(centMat[0][0]-centMat[1][0],2)+pow(centMat[0][1]-centMat[1][1],2))/180; 
	// cout<<"beta: "<<beta<<endl;

	float beta = 0.71;

	//Theta
	//float theta = atan2((centMat[0][0]-centMat[1][0]),(centMat[0][1]-centMat[1][1]));
	//cout<<"theta: "<<theta<<endl;

	float theta = 0;

	//Tx Ty
	// float Tx = ((centMat[0][0]-centMat[1][0])/beta) - (60);
	// float Ty = ((centMat[0][1]-centMat[1][1])/beta) - (150);
	// cout<<"Tx: "<<Tx<<endl;
	// cout<<"Ty: "<<Ty<<endl;

	float Tx = -136.05;
	float Ty = 4;

	return associate_img;
}


/*****************************************************
	*Function for Lab 6
 * **************************************************/
 //This is a call back function of mouse click, it will be called when there's a click on the video window.
 //You will write your coordinate transformation in onClick function.
 //By calling onClick, you can use the variables calculated in the class function directly and use publisher
 //initialized in constructor to control the robot.
 //lab4 and lab3 functions can be used since it is included in the "lab4.h"
void onMouse(int event, int x, int y, int flags, void* userdata)
{
		ic_ptr->onClick(event,x,y,flags,userdata);
}
void ImageConverter::onClick(int event,int x, int y, int flags, void* userdata)
{
	// For use with Lab 6
	// If the robot is holding a block, place it at the designated row and column.
	if  ( event == EVENT_LBUTTONDOWN ) //if left click, do nothing other than printing the clicked point
	{
		if (leftclickdone == 1) {
			leftclickdone = 0;  // code started
			//ROS_INFO_STREAM("left click:  (" << x << ", " << y << ")");  //the point you clicked

			float xc = (y-320)/0.71;
			float yc = (x-240)/0.71;

			float xw = -(xc+96.69)/1000;
			float yw = -(yc-175.83)/1000;
			float zw = 23.0/1000;

			driver_msg.destination=lab_invk(xw,yw,zw+0.2,10);

			//publish the point to the robot
		    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
			int spincount = 0;
			driver_msg.duration = 3.0;
			pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
			spincount = 0;
			while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
				ros::spinOnce();  // Allow other ROS functionallity to run
				loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
				if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
					pub_command.publish(driver_msg);
					ROS_INFO_STREAM("Just Published again driver_msg");
					spincount = 0;
				}
				spincount++;  // keep track of loop count
			}
			ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

			while(!isReady)
			{
				ros::spinOnce();
				loop_rate.sleep();
			}


			driver_msg.destination=lab_invk(xw,yw,zw,10);

			driver_msg.duration = 3.0;
			pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
			spincount = 0;
			while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
				ros::spinOnce();  // Allow other ROS functionallity to run
				loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
				if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
					pub_command.publish(driver_msg);
					ROS_INFO_STREAM("Just Published again driver_msg");
					spincount = 0;
				}
				spincount++;  // keep track of loop count
			}
			ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

			while(!isReady)
			{
				ros::spinOnce();
				loop_rate.sleep();
			}

			//Suction ON
			srv.request.fun = 1;
			srv.request.pin = 0;  //Digital Output 0
			srv.request.state = 1.0; //Set DO0 on
			if (srv_SetIO.call(srv)) {
				ROS_INFO("True: Switched Suction ON");
			} else {
				ROS_INFO("False");
			}
			ros::Duration(2).sleep();

			driver_msg.destination=lab_invk(xw,yw,zw+0.2,10);
		    
			driver_msg.duration = 3.0;
			pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
			spincount = 0;
			while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
				ros::spinOnce();  // Allow other ROS functionallity to run
				loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
				if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
					pub_command.publish(driver_msg);
					ROS_INFO_STREAM("Just Published again driver_msg");
					spincount = 0;
				}
				spincount++;  // keep track of loop count
			}
			ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

			while(!isReady)
			{
				ros::spinOnce();
				loop_rate.sleep();
			}

			driver_msg.destination=lab_invk(-.3,-.3,0.2,-90);

			driver_msg.duration = 3.0;
			pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
			spincount = 0;
			while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
				ros::spinOnce();  // Allow other ROS functionallity to run
				loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
				if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
					pub_command.publish(driver_msg);
					ROS_INFO_STREAM("Just Published again driver_msg");
					spincount = 0;
				}
				spincount++;  // keep track of loop count
			}
			ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

			while(!isReady)
			{
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO_STREAM("Ready for new point");

			leftclickdone = 1; // code finished
		} else {
			ROS_INFO_STREAM("Previous Left Click not finshed, IGNORING this Click");
		}
	}
	else if  ( event == EVENT_RBUTTONDOWN )//if right click, find nearest centroid,
	{
		if (rightclickdone == 1) {  // if previous right click not finished ignore
			rightclickdone = 0;  // starting code
			ROS_INFO_STREAM("right click:  (" << x << ", " << y << ")");  //the point you clicked
			float xc = (y-320)/0.71;
			float yc = (x-240)/0.71;

			float xw = -(xc+96.69)/1000;
			float yw = -(yc-175.83)/1000;
			float zw = 23.0/1000;

			driver_msg.destination=lab_invk(xw,yw,zw+0.2,10);

			//publish the point to the robot
		    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
			int spincount = 0;
			driver_msg.duration = 3.0;
			pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
			spincount = 0;
			while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
				ros::spinOnce();  // Allow other ROS functionallity to run
				loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
				if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
					pub_command.publish(driver_msg);
					ROS_INFO_STREAM("Just Published again driver_msg");
					spincount = 0;
				}
				spincount++;  // keep track of loop count
			}
			ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

			while(!isReady)
			{
				ros::spinOnce();
				loop_rate.sleep();
			}


			driver_msg.destination=lab_invk(xw,yw,zw,10);

			driver_msg.duration = 3.0;
			pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
			spincount = 0;
			while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
				ros::spinOnce();  // Allow other ROS functionallity to run
				loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
				if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
					pub_command.publish(driver_msg);
					ROS_INFO_STREAM("Just Published again driver_msg");
					spincount = 0;
				}
				spincount++;  // keep track of loop count
			}
			ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.

			while(!isReady)
			{
				ros::spinOnce();
				loop_rate.sleep();
			}

			//Suction OFF
			srv.request.fun = 1;
			srv.request.pin = 0;  //Digital Output 0
			srv.request.state = 0.0; //Set DO0 on
			if (srv_SetIO.call(srv)) {
				ROS_INFO("True: Switched Suction ON");
			} else {
				ROS_INFO("False");
			}
			ros::Duration(2).sleep();

			rightclickdone = 1; // code finished
		} else {
			ROS_INFO_STREAM("Previous Right Click not finshed, IGNORING this Click");
		}
	}
}
