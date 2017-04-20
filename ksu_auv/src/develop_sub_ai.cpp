#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv/cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <iostream>
#include <vector>
/*#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>*/
#include <mavros_msgs/State.h>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;
ros::Publisher pub_mavros;
vector<int> minRange;
vector<int> maxRange;

/*Always show state of Mavros*/
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg
}


static const char WINDOW[] = "Image Unprocessed";
static const char WINDOW2[] = "Image Processed";
static const char WINDOW_RANGE[] = "Color Range";

Point direction_error;

/*Generate User Windows for user input - define specific color
 * 
 * GENERATE LIST OF COLORS TO AUTODEFINE FOR EACH SECTION OF THE COURSE
 * 
 * Start gate: BLAZE ORANGE, General center has been defined to go through the start gate
 * 
 * PATH MARKERS: BLAZE ORANGE, Use To Orient direction for Sub to go to next objectives
 * 
 * Bouys: RED, then GREEN. avoid YELLOW.
 * 	Points are providesd for dragging YELLOW down
 * 
 * Idenify NAVIGATE CHANNEL: COLOR UNKNOWN
 * Must idenify the colored PVC pipes, do not worry about style, just navigate along the path.
 * 
 * Skip Weigh anchor
 * 
 * Skip Torpedos
 * 
 * **Octagon, most likely skip
 * */
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
    
   // imshow(WINDOW, cv_ptr->image);
    
    cv::Mat current_frame = cv_ptr->image;
    
    cvtColor(current_frame, current_frame, CV_BGR2Luv);
    
    // set click
    //                                                                                                                                                                                                                                                                                                                                                                                                                                                                             update_process();
	
    //processing
    inRange(current_frame, minRange, maxRange, current_frame);
    
    imshow(WINDOW2, current_frame);
    
    //Convert to publish
    cv_bridge::CvImage out_msg;
	out_msg.header   = msg->header; // Same timestamp and tf frame as input image
	out_msg.encoding = enc::MONO8; // sensor_msgs::image_encodings
	out_msg.image    = current_frame; // Your cv::Mat
    
    
	image_pub.publish(out_msg.toImageMsg());
	//cout << "Published" << endl;
	if (debug)
	{
		cout << "minRange:" << minRange[0] << ", "<< minRange[1] << ", "<< minRange[2] << endl;
		cout << "maxRange:" << maxRange[0] << ", "<< maxRange[1] << ", "<< maxRange[2] << endl;
	}

	//reset current_frame
	current_frame = cv_ptr->image;
	
	waitKey(10);//*/
}

void morphOps(Mat& src)
{
	Mat _erode= getStructuringElement(MORPH_RECT, Size(3,3));
	Mat _dilate= getStructuringElement(MORPH_RECT, Size(8,8));
	
	erode(src, src, _erode);
	erode(src, src, _erode);
	
	dilate(src, src, _dilate);
	dilate(src, src, _dilate);
}

void gate_track_CB (const sensor_msgs::ImageConstPtr& msg)
{
	Mat src;
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	src = cv_ptr->image;
	
	morphOps(src);
	
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> center_blobs;
	
	findContours(src.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	if (hierarchy.size() < 100 && hierarchy.size() > 0)
	{
		for (int i = 0; i >= 0 ; i = hierarchy[i][0])
		{
			Moments moment = moments((cv::Mat) contours[i]);
			// moment.m00 is the area
			if (moment.m00 > 1600)
			{
				Point p(moment.m10/moment.m00,
						moment.m01/moment.m00);
				center_blobs.push_back(p);
			}
			
		}
		
	}
	
	Point average;
	
	if (center_blobs.size() > 0)
	{
		for (int i = 0; i < center_blobs.size(); i++)
		{
			average.x += center_blobs[i].x;
			average.y += center_blobs[i].y;
		}
		average.x /= center_blobs.size();
		average.y /= center_blobs.size();
	}
	
	direction_error = average;
	
	int alpha = 255;
	cv::line(src, direction_error, Point(src.size().width/2, src.size().height/2), Scalar(alpha, alpha, alpha), 3);
	
	circle(src, average, 3, Scalar(alpha, alpha, alpha), 5);
	//~ cout << "count: " << _count << endl;
	imshow("debug", src);
	
	direction_error.x -= src.size().width/2;
	direction_error.y -= src.size().height/2;
	
	waitKey(10);
}

int main (int argc, char **argv)
{
	/// Change default min and max here
	minRange.push_back(117);
	minRange.push_back(172);
	minRange.push_back(151);

	maxRange.push_back(167);
	maxRange.push_back(222);
	maxRange.push_back(201);
	
	namedWindow("debug");
	
	ros::init(argc, argv, "process_image");
	
	ros::NodeHandle n;
	
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("/color_filtered", 1, gate_track_CB);
	pub_mavros = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
	
	ros::spin();
	return 0;
}

/*int main(int argc, char **argv) *MAVROS OFF BOARDMODE DO NOT USE, USES SETPOINTS. WE DO NOT KNOW GLOBAL POSITION, ONLY RELATIVE 
{
	ros:init(argc, argv, "offb_node");
	ros:NodeHandle nh;
	
	ros:Subscribed state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_cb);
	ros:Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", );
	ros::ServiceClient armimg_client = nh.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/armimg");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");
		
//the setpoint publishing rate must be faster than 2Hz or the pixhawk will enter out of Offboard mode
ros::Rate rate(20.0);

//wait for FCU connection, this is a loop that will not allow any movement until a connection to the Pixhawk is established. it is exited once a heartbeat message is recieved
while(ros::ok() && current_state.connected){
	ros:spinOnce();
	rate.sleep();
}
//Boolean Setpoints, without publishing setpoints Mavros will not enter Offboard board
geometry_msgs::PoseStampled pose;
pose.pose.position.x =
pose.pose.position.y =
pose.pose.position.z =
rate.sleep();
}
*/

/*Use Offboard mode to directly control the motors
 * */
int 
