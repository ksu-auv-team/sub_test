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
#include <iostream>
#include <vector>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <math.h>

#define ROLL_CHAN 0
#define PITCH_CHAN 1
#define THROT_CHAN 2
#define YAW_CHAN 3
#define MODES_CHAN 4	

#define HIGH_PWM 2000
#define MID_PWM 1500
#define LOW_PWM 1000

ros::Publisher pub_mavros;

bool manual_control = true;

mavros_msgs::OverrideRCIn MAV_MSG;

void manual_override_callback(const std_msgs::Int8 &msg)
{
	switch (msg.data)
	{
			case 1: //stop
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = MID_PWM;
				MAV_MSG.channels[YAW_CHAN] = MID_PWM;
				MAV_MSG.channels[MODES_CHAN] = MID_PWM;
				break;		
			case 2: //roll:
				MAV_MSG.channels[ROLL_CHAN] = HIGH_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = MID_PWM;
				MAV_MSG.channels[YAW_CHAN] = MID_PWM;
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				break;
			case 3: //pitch:
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = HIGH_PWM;
				MAV_MSG.channels[THROT_CHAN] = MID_PWM;
				MAV_MSG.channels[YAW_CHAN] = MID_PWM;
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				break;
			case 4: //yaw:
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = HIGH_PWM;
				MAV_MSG.channels[YAW_CHAN] = MID_PWM;
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				break;
			case 5: //throttle:
				MAV_MSG.channels[ROLL_CHAN] = MID_PWM;
				MAV_MSG.channels[PITCH_CHAN] = MID_PWM;
				MAV_MSG.channels[THROT_CHAN] = MID_PWM;
				MAV_MSG.channels[YAW_CHAN] = HIGH_PWM;
				MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
				break;
			}
					pub_mavros.publish(MAV_MSG);
		ros::spinOnce();
		return 0;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "channel_test");
	
	ros::NodeHandle n;
	
	ros::Subscriber manual_sub = n.subscribe("/manual_override/msg", 1, manual_override_callback);
	pub_mavros = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

		switch(manual_override_callback())
		
		pub_mavros.publish(MAV_MSG);
		ros::spinOnce();
		RC_COMM_RATE.sleep();
	}

}
