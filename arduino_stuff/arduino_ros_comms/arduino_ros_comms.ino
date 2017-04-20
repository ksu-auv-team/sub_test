/*
 * rosserial Publisher
 * This is an arduino sketch 
 * to use it, flash it to arduino through IDE
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

std_msgs::Bool engage_bool; //boolean for engage/disengage
ros::Publisher status_("mission_status", &engage_bool); //publisher for the boolean

//set up arduino:
//int sensorPin = A0;
int sensorPin = 7;

int ledPin = 13;
int sensorValue = 0;


void setup()
{
  //initialize the ros node:
  nh.initNode();
  nh.advertise(status_);
  
  //serial comms for debugging
  //Serial.begin(9600);
  
  //declare sensorpin as input:
  pinMode(sensorPin, INPUT);
  //declare ledpin as output:
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  //if an analog pin:
   // sensorValue = analogRead(sensorPin);
  sensorValue = digitalRead(sensorPin);
  
  //serial print for debugging:
  //Serial.print(sensorValue, DEC);
  //Serial.print("\n");
  if (sensorValue == 1)
  {
    engage_bool.data = true;
    digitalWrite(ledPin, HIGH);
  }
  else if (sensorValue == 0)
  {
    engage_bool.data = false;
    digitalWrite(ledPin, LOW);
  }
  status_.publish( &engage_bool );
  nh.spinOnce();
  delay(500);
}
