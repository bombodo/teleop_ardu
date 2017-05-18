#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <Arduino.h>

ros::NodeHandle nh;

geometry_msgs::Twist twst_msg;
ros::Publisher chatter("twist", &twst_msg);

void messageMV(const geometry_msgs::Twist& move_msg)
{
  double forward = move_msg.linear.x; //important to note that float and double may be same precision
  double turn = move_msg.angular.z; //arduino hardware (UNO and ATMEGA based)
  twst_msg.linear.x = forward;
  twst_msg.angular.z = turn;
}

ros::Subscriber<geometry_msgs::Twist> input("turtle1/cmd_vel", &messageMV); 

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.subscribe(input);
  nh.advertise(chatter);
}

void loop()
{
  chatter.publish( &twst_msg );
  nh.spinOnce();
  delay(1);
}
