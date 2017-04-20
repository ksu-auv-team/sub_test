/******************************
 * Message guide:
 * float = hold current altitude (5 sec)
 * f = forward (5 sec)
 * rl = rotate_left (5sec)
 * rr = rotate_right (5 sec)
 * kill = disable (until ctl+c)
 * *******************************/

/* copy these commands into the ground station terminal:

echo "alias float='rostopic pub -1 /manual_override/msg std_msgs/Int8 5'" >> ~/.bash_aliases #hold pressure (altitude)
echo "alias f='rostopic pub -1 /manual_override/msg std_msgs/Int8 2'" >> ~/.bash_aliases #forward
echo "alias rl='rostopic pub -1 /manual_override/msg std_msgs/Int8 3'" >> ~/.bash_aliases #rotate left
echo "alias rr='rostopic pub -1 /manual_override/msg std_msgs/Int8 4'" >> ~/.bash_aliases #rotate right
echo "alias kill='rostopic pub -r /manual_override/msg std_msgs/Int8 1'" >> ~/.bash_aliases #disable
echo "alias f2='rostopic pub -1 /manual_override/msg std_msgs/Int8 6'" >> ~/.bash_aliases #2ft
echo "alias f3='rostopic pub -1 /manual_override/msg std_msgs/Int8 7'" >> ~/.bash_aliases #3ft
echo "alias f4='rostopic pub -1 /manual_override/msg std_msgs/Int8 8'" >> ~/.bash_aliases #4ft
echo "alias f5='rostopic pub -1 /manual_override/msg std_msgs/Int8 9'" >> ~/.bash_aliases #5ft
echo "alias f6='rostopic pub -1 /manual_override/msg std_msgs/Int8 10'" >> ~/.bash_aliases #6ft
source ~/.bashrc

****************************************************************************************/

/* these commands are not ready yet:
echo "alias ai='rostopic pub -1 /manual_override/msg std_msgs/Int8 0'" >> ~/.bash_aliases
echo "alias full_speed_ahead='rostopic pub -1 /manual_override/msg std_msgs/Int8 69'" >> ~/.bash_aliases
*/
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv/cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/FluidPressure.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <vector>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>
//~ #include <chrono>

//#include <mavros_msgs/cmd/armimg>

#define ROLL_CHAN 	0
#define PITCH_CHAN 	1
#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define MODES_CHAN 	4	

#define HIGH_PWM	2000
#define MID_PWM 	1500
#define LOW_PWM 	1000

using namespace std;
using namespace cv;

bool debug = true;
bool showVid = true;
bool blob_detected = false;

Scalar gateMinRange = Scalar(100,140,150); //hotel room 7-29
Scalar gateMaxRange = Scalar(150,190,200); //hotel room 7-29
Scalar bouyMinGreen = Scalar(209,239,3);
Scalar bouyMaxGreen = Scalar(151,168,0);
Scalar bouyMinRed = Scalar(148,112,38);
Scalar bouyMaxRed = Scalar(212,164,192);
Scalar bouyMinYellow = Scalar(115,184,33);
Scalar bouyMaxYellow =	Scalar(165,255,169);
//defining number of seconds for wait time
//unsigned int usecs = 10000;

namespace enc = sensor_msgs::image_encodings;
ros::Publisher pub_mavros;
////////////////////////////////////////////////
bool kill = false;//this is not safe, return it to true when we have arduino coms
///////////////////////////////////////////////
bool manual_control = false;

enum MODE 
{
	DISABLE,
	FLOAT,
	FORWARD,
	TO_GATE,
	TO_RED,
	TO_GREEN,
	ROTATE_R,
	ROTATE_L
};

enum TARGET
{
	GATE,
	RED,
	GREEN
};

TARGET current_target = GATE;
MODE current_mode = FLOAT;
MODE manual_mode = FLOAT;

static const char WINDOW[] = "Image Unprocessed";
static const char WINDOW2[] = "Image Processed";


double surface_pressure = 101325; //0ft
double target_pressure=surface_pressure; // 0 ft
bool pressure_collected = false;
vector<double> pressure_data;
double max_pressure = surface_pressure+6*2988.98; //6 ft


Point2f direction_error;
Point2f prev_direction_error;
double altitude_error_ratio = 0.0d;
int yaw_gain = 200;

double start_time;

double last_manual_msg_time;


void morphOps(Mat& src)
{
	Mat _erode= getStructuringElement(MORPH_RECT, Size(3,3));
	Mat _dilate= getStructuringElement(MORPH_RECT, Size(8,8));
	
	erode(src, src, _erode);
	erode(src, src, _erode);
	
	dilate(src, src, _dilate);
	dilate(src, src, _dilate);
}

void blob_track_CB (Mat src)
{
	morphOps(src);//make the image less noisy
	
	int tooSmallBlobs = 0;
	int tooLargeBlobs = 0;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> centers_of_blobs;
	
	//find blobs (shapes) from the color filtered image
	findContours(src.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	//go through the blobs if there is more than 0 blobs and less than 200 blobs
	if (hierarchy.size() > 0 && hierarchy.size() < 50)
	{
		//find the center of the blob and save it to centers
		for (int i = 0; i >= 0 ; i = hierarchy[i][0])
		{
			Moments moment = moments((cv::Mat) contours[i]);
			// moment.m00 is the area
			if ( (moment.m00 > 100) && (moment.m00 < 30000))//if area is > 200 pixels^2 and < 30000 pixels^2
			{				
				blob_detected = true;//we have detected a large blob
				
				//find the midpoint:
				Point p(moment.m10/moment.m00,
						moment.m01/moment.m00);
						
				centers_of_blobs.push_back(p); //save midpoint to centers vector
			}
			else
			{
				if (moment.m00 < 400)
				{
					tooSmallBlobs ++;
				}
				if (moment.m00 > 30000)
				{
					tooLargeBlobs ++;
				}
				blob_detected = false;//we did not detect any large blobs
			}
			if (debug)
			{
				if(tooSmallBlobs > 0)
					ROS_INFO("%i too small blobs detected", tooSmallBlobs);
				if(tooLargeBlobs > 0)
					ROS_INFO("%i too large blobs detected", tooLargeBlobs);
			}
		}
		
	}
	else 
	{
		if (debug)
		{
			if (hierarchy.size() > 0)
				ROS_WARN("Too many blobs detected, bad filter");
			if (hierarchy.size() == 0)
				ROS_WARN("No blobs detected, bad filter");
		}
		blob_detected = false; //we did not detect any blobs at all
	}
	
	//ignore blob centers that are high in the image
	for(int i = 0; i < centers_of_blobs.size(); i++)
	{
		if ( centers_of_blobs[i].y < 128 )
			centers_of_blobs.erase(centers_of_blobs.begin() + i);
	}
	
	if (centers_of_blobs.size() > 2)//if two valid midpoints recorded (pretty sure its the gate)
	{
		yaw_gain = 200;
	}
	else
	{
		yaw_gain = 50;
	}
	
	Point average;
	//find the average point of all the centers:
	if (centers_of_blobs.size() > 0)
	{
		for (int i = 0; i < centers_of_blobs.size(); i++)
		{
			average.x += centers_of_blobs[i].x;
			average.y += centers_of_blobs[i].y;
		}
		average.x /= centers_of_blobs.size();
		average.y /= centers_of_blobs.size();
	}
	
	prev_direction_error = direction_error; //save previous 
	direction_error = average;
	
	if (debug && showVid)
	{
		int alpha = 255;
		cv::line(src, direction_error, Point(src.size().width/2, src.size().height/2), Scalar(alpha, alpha, alpha), 3);
		
		circle(src, average, 3, Scalar(alpha, alpha, alpha), 5);
		//~ cout << "count: " << endl;
		
		
		imshow(WINDOW2, src);
	}
	
	direction_error.x -= src.size().width/2;
	direction_error.y -= src.size().height/2;
	
	if ( ((direction_error.x > 320) && (direction_error.y > 230) ) || 
		 ((direction_error.x < -320) && (direction_error.y < -230)) )
	{
		direction_error.x = 0;
		direction_error.y = 0;
	}
	
	Point2f delta_error;
	delta_error.x = 
	
	waitKey(30);
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{		
	//cout << "cam callback" << endl;
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	//~ imshow(WINDOW, cv_ptr->image);
    
    cv::Mat current_frame = cv_ptr->image;
    
    cvtColor(current_frame, current_frame, CV_BGR2Lab);
	
	//current_target= GATE;//change it to test different colors
	
	if (current_target == GATE)
		inRange(current_frame, gateMinRange, gateMaxRange, current_frame);
	else if (current_target == RED)
		inRange(current_frame, bouyMinRed, bouyMaxRed, current_frame);
	else
		inRange(current_frame, bouyMinGreen, bouyMaxGreen, current_frame);
	if (debug && showVid)
			imshow(WINDOW, current_frame);
	blob_track_CB(current_frame);
	
	/*switch(current_target)
	{
		case GATE:
			//inRng(input_image (LUV color mins), (LUV color max), output)
			//inRange(current_frame, (117, 172, 151), (167, 222, 201), current_frame); ///ksu lab
			inRange(current_frame, gateMinRange, gateMaxRange, current_frame); //hotel room
			if (debug && showVid)
				imshow(WINDOW,current_frame);
			blob_track_CB(current_frame);
			break;
			
		case RED:
			//inRange(current_frame, minRange, maxRange, current_frame);
			//blob_track_CB(current_frame);
			inRange(current_frame, bouyMinRed, bouyMaxRed, current_frame);
			if (debug && showVid)
				imshow(WINDOW,current_frame);
			blob_track_CB(current_frame);
			break;
			
		case GREEN:
			inRange(current_frame, bouyMinGreen, bouyMaxGreen, current_frame);
			if (debug && showVid)
				imshow(WINDOW,current_frame);
			blob_track_CB(current_frame);
			break;
	}*/
	
	waitKey(30);

}

void pressure_callback(const sensor_msgs::FluidPressure& msg)
{

	//These are in Pa
	//1 ATM = surface_pressure Pa
	//collect data for first three seconds;
	double current_time = ros::Time::now().toSec();
	if ( (current_time - start_time) < 5.0d )
	{
		pressure_data.push_back(msg.fluid_pressure);
		ROS_INFO("Finished collecting atmospheric pressure");
	}
	
	//average the data then set pressure_detected so it never averages again
	else if ( !pressure_collected && ((current_time - start_time) >= 5.0d) )
	{
		int samplesize = pressure_data.size();
		if (samplesize == 0)
			 ROS_WARN("NO Presssure Data Collected During First 3 Secs, defaulting to 1ATM");
		else
		{
			surface_pressure = 0;
			for (int i = 0; i < samplesize; i++)
				surface_pressure += pressure_data[i]/samplesize;
		}
		{
		target_pressure>surface_pressure+2*2988.98;//2ft
		pressure_collected = true;}
	}
	
	//use target pressure and current_pressure to calculate a proportional error ratio
	else
	{
		double current_pressure = msg.fluid_pressure;
		altitude_error_ratio = (target_pressure - current_pressure) / (max_pressure - surface_pressure);
		if (altitude_error_ratio < -1)
			altitude_error_ratio = -1;
		else if (altitude_error_ratio > 1)
			altitude_error_ratio = 1;
	}
	
}

//arduino data callback
void status_callback(const std_msgs::Bool &msg)
{
	if (msg.data == false)
	{
		kill = true;
	}
	else
	{
		kill == false;
	}
}

void manual_override_callback(const std_msgs::Int8 &msg)
{
	switch (msg.data)
	{
		case 0:
			manual_control = false;
			kill = true;
			manual_mode = DISABLE;
			break;
		case 1:
			manual_control = false;
			kill = false;
			manual_mode = DISABLE;
			break;
		case 2:
			manual_control = true;
			manual_mode = FORWARD;
			target_pressure=surface_pressure+2*2988.98;
			break;
		case 3:
			manual_control = true;
			manual_mode = ROTATE_L;
			target_pressure=surface_pressure+2*2988.98;
			break;
		case 4:
			manual_control = true;
			manual_mode = ROTATE_R;
			target_pressure=surface_pressure+2*2988.98;
			break;
		case 5:
			manual_control = true;
			manual_mode = FLOAT;
			target_pressure=surface_pressure+2*2988.98;
			break;
		case 6: //2 ft
			manual_control = true; 
			manual_mode = FLOAT;
			target_pressure=surface_pressure+2*2988.98;
			break;
		case 7: //3ft
			manual_control = true;
			manual_mode = FLOAT;
			target_pressure=surface_pressure+3*2988.98;
			break;
		case 8: //4ft
			manual_control = true;
			manual_mode = FLOAT;
			target_pressure=surface_pressure+4*2988.98;
			break;
		case 9: //5ft
			manual_control = true;
			manual_mode = FLOAT;
			target_pressure=surface_pressure+5*2988.98;
			break;
		case 10: //6ft
			manual_control = true;
			manual_mode = FLOAT;
			target_pressure=surface_pressure+6*2988.98;
			break;
	}
	last_manual_msg_time = ros::Time::now().toSec();
}


MODE decide_current_mode()
{
	double current_time = ros::Time::now().toSec();
	double time_elapsed = current_time - start_time;
	if (manual_control &&  (current_time - last_manual_msg_time) < 5.0d)
	{
		return manual_mode;
	}
	if (kill)
	{
		if (debug)
			ROS_WARN("Kill Switch Activated");
		return DISABLE;
	}
	if (blob_detected)
	{
		return  TO_GATE;
	}
	if (time_elapsed <= 1800.0d)
	{
		target_pressure=surface_pressure+3*2988.98;//3ft
		return FORWARD;
	}
	if (debug)
		ROS_INFO("AI auto-timeout after %f seconds", time_elapsed);
	return DISABLE;
}

int main (int argc, char **argv)
{
	usleep(10000);
	 
	if (debug && showVid)
	{
		namedWindow(WINDOW);
		namedWindow(WINDOW2);
	}
	ros::init(argc, argv, "sub_ai");
	ros::NodeHandle n;
	start_time = ros::Time::now().toSec(); 	
	//~ cout << "waiting 10 seconds" << endl;
	//~ ~ ros::Duration(10).sleep();
	//~ mavros_msgs::CommandBool ;
	image_transport::ImageTransport it(n);

	/*****************************************************************************/
	///This is where you change color range
	/****************************************************************************/

	//gateMinRange = Scalar(65,113,138); //hotel room 7-25
	//gateMaxRange = Scalar(117,163,188); //hotel room 7-25
	/*gateMinRange2 = Scalar(77,110,133); //hotel room 7-26
	gateMaxRange2 = Scalar(127,160,183); //hotel room 7-26
	gateMinRange3 = Scalar(77,110,133); //hotel room 7-26
	gateMaxRange3 = Scalar(127,160,183); //hotel room 7-26*/
	gateMinRange = Scalar(100,140,150); //hotel room 7-29
	gateMaxRange = Scalar(150,190,200); //hotel room 7-29
	bouyMinGreen = Scalar(209,239,3);
	bouyMaxGreen = Scalar(151,168,0);
	bouyMinRed = Scalar(148,112,38);
	bouyMaxRed = Scalar(212,164,192);
	bouyMinYellow = Scalar(115,184,33);
	bouyMaxYellow =	Scalar(165,255,169);
	
	image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	//ros::Subscriber status_sub = n.subscribe("mission_status", 1, status_callback); //subscribe to arduino topic
	ros::Subscriber pressure_sub = n.subscribe("/mavros/imu/atm_pressure", 1, pressure_callback); //subscribe to arduino topic
	ros::Subscriber manual_sub = n.subscribe("/manual_override/msg", 1, manual_override_callback);
	pub_mavros = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
	 
	 
	mavros_msgs::OverrideRCIn MAV_MSG;
	
	ros::Rate RC_COMM_RATE(45);
	
	long msg_iterator = 0;
	int msg_freq = 49;
	
	//ros::Duration(10).sleep();	
		//~ ros::ServiceClient armimg_client = n.ServiceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	//~ 
	//~ mavros_msgs::CommandBool arm_cmd;
	//~ arm_cmd.request.value=true;
	//~ 
	//~ ( arming_client.call(arm_cmd) && arm.cmd.response.success){
		//~ ROS_INFO("Armed");

	//~ ros::ServiceClient arming_cl = n.ServiceClient<mavros::CommandBool>("/mavros/cmd/armimg");
	//~ mavros::CommandBool srv;
	//~ srv.request.value = true;
	//~ if(arming_cl.call(srv)){
		//~ ROS_INFO("Armed", srv.response.success):
	//~ {else}
		//~ ROS_ERROR("Failed Arming");
	//~ }
	
		
	//ros::Duration(180).sleep();
	
		target_pressure=surface_pressure+2*2988.98;	
		
	while(ros::ok())
	{
		switch(decide_current_mode())
		{
			case FLOAT:
				if (debug)
				{
					ROS_INFO("Float mode");	
				}
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = MID_PWM - altitude_error_ratio*200;
				MAV_MSG.channels[YAW_CHAN] = MID_PWM;
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				if (debug && msg_iterator%msg_freq == 0)
				{
					if (MAV_MSG.channels[THROT_CHAN] < MID_PWM)
						ROS_INFO("Down: %i", MAV_MSG.channels[THROT_CHAN] );
					if (MAV_MSG.channels[THROT_CHAN] > MID_PWM)
						ROS_INFO("Up: %i", MAV_MSG.channels[THROT_CHAN] );
				}
				break;
				
			case ROTATE_R:
				if (debug && msg_iterator%msg_freq == 0)
					ROS_INFO("Rotate right mode");
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = MID_PWM - altitude_error_ratio*200;
				MAV_MSG.channels[YAW_CHAN] = HIGH_PWM;//right rotation
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				if (debug && msg_iterator%msg_freq == 0)
				{
					if (MAV_MSG.channels[THROT_CHAN] < MID_PWM)
						ROS_INFO("Down: %i", MAV_MSG.channels[THROT_CHAN] );
					if (MAV_MSG.channels[THROT_CHAN] > MID_PWM)
						ROS_INFO("Up: %i", MAV_MSG.channels[THROT_CHAN] );
				}
				break;
				
			case ROTATE_L:
				if (debug && msg_iterator%msg_freq == 0)
					ROS_INFO("Rotate left mode");
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = MID_PWM - altitude_error_ratio*200;
				MAV_MSG.channels[YAW_CHAN] = LOW_PWM;//left rotation
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;				
				if (debug && msg_iterator%msg_freq == 0)
				{
					if (MAV_MSG.channels[THROT_CHAN] < MID_PWM)
						ROS_INFO("Down: %i", MAV_MSG.channels[THROT_CHAN] );
					if (MAV_MSG.channels[THROT_CHAN] > MID_PWM)
						ROS_INFO("Up: %i", MAV_MSG.channels[THROT_CHAN] );
				}
				break;
				
			case TO_GATE:
				if (debug && msg_iterator%msg_freq == 0)
					ROS_INFO("To Gate mode");
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = HIGH_PWM-altitude_error_ratio*200;/*direction_error.y/240.0f*200*/;//change the "*x" for faster/slower correction (range is 0-500, 0 doing nothing)
				MAV_MSG.channels[YAW_CHAN] = MID_PWM + direction_error.x/320.0f*yaw_gain;//change the "*x" number for faster/slower correction (range is 0-500, 0 doing nothing)
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				if (debug && msg_iterator%msg_freq == 0)
				{
					if (MAV_MSG.channels[THROT_CHAN] < MID_PWM)
						ROS_INFO("Down: %i", MAV_MSG.channels[THROT_CHAN] );
					if (MAV_MSG.channels[THROT_CHAN] > MID_PWM)
						ROS_INFO("Up: %i", MAV_MSG.channels[THROT_CHAN] );
				}
				if (debug && blob_detected)
				{
					if (MAV_MSG.channels[YAW_CHAN] > MID_PWM)
						ROS_INFO("Right: %i", MAV_MSG.channels[YAW_CHAN] );
					if (MAV_MSG.channels[YAW_CHAN] < MID_PWM)
						ROS_INFO("Left: %i", MAV_MSG.channels[YAW_CHAN] );
				}
				break;
				
			case DISABLE:
				if (debug && msg_iterator%msg_freq == 0)
					ROS_INFO("Sub disabled");
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = LOW_PWM;
				MAV_MSG.channels[YAW_CHAN] = MID_PWM;
				MAV_MSG.channels[MODES_CHAN] = LOW_PWM;
				if (debug && msg_iterator%msg_freq == 0)
				{
					if (MAV_MSG.channels[THROT_CHAN] < MID_PWM)
						ROS_INFO("Down: %i", MAV_MSG.channels[THROT_CHAN] );
					if (MAV_MSG.channels[THROT_CHAN] > MID_PWM)
						ROS_INFO("Up: %i", MAV_MSG.channels[THROT_CHAN] );
				}
				break;
			
			case FORWARD:
				if (debug && msg_iterator%msg_freq == 0)
					ROS_INFO("Sub forward");
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = HIGH_PWM-altitude_error_ratio*200;
				MAV_MSG.channels[YAW_CHAN] = MID_PWM;
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				if (debug && msg_iterator%msg_freq == 0)
				{
					if (MAV_MSG.channels[THROT_CHAN] < MID_PWM)
						ROS_INFO("Down: %i", MAV_MSG.channels[THROT_CHAN] );
					if (MAV_MSG.channels[THROT_CHAN] > MID_PWM)
						ROS_INFO("Up: %i", MAV_MSG.channels[THROT_CHAN] );
				}
				break;
		}
		pub_mavros.publish(MAV_MSG);
		msg_iterator ++;
		ros::spinOnce();
		RC_COMM_RATE.sleep();
	}
	return 0;
}
