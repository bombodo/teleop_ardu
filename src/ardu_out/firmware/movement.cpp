#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <Arduino.h>

ros::NodeHandle nh;

geometry_msgs::Twist twst_msg;
ros::Publisher chatter("twist", &twst_msg);
int frontL = 3;
int frontR = 9;
int rearL = 10;
int rearR = 11;
double left_percent;
double right_percent;
double left_speed;
double right_speed;

void messageMV(const geometry_msgs::Twist& move_msg)
{
  double forward = move_msg.linear.x; //important to note that float and double may be same precision
  double turn = move_msg.angular.z; //arduino hardware (UNO and ATMEGA based)
  twst_msg.linear.x = forward;
  twst_msg.angular.z = turn;
  //angular determines percent devoted to each side
  left_percent = (turn>0)?(0.5*(1 + turn/10)):(0.5*(1 + turn/10));
  right_percent = 1-left_percent;
  left_speed = (int)(left_percent*forward*255/10);
  right_speed = (int)(right_percent*forward*255/10);
  analogWrite(frontL,left_speed);
  analogWrite(frontR,right_speed);
  analogWrite(rearL,left_speed);
  analogWrite(rearR,right_speed);
}

ros::Subscriber<geometry_msgs::Twist> input("turtle1/cmd_vel", &messageMV);

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
  delay(5);
}
