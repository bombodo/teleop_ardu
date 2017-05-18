#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

class TeleopTurtle
{
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle nh_;

    int forward_, reverse_, angular_;
    int lin_boost_,ang_boost_;
    int brake_;
    double l_scale_, a_scale_;

    int prev_lboost_;
    int prev_aboost_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

public:
    TeleopTurtle();
};

TeleopTurtle::TeleopTurtle():
    forward_(4),reverse_(5),angular_(0),lin_boost_(3),ang_boost_(2),brake_(7), //set defaults in case user specified unable to be polled
    l_scale_(1),a_scale_(1)
{
    nh_.param("axis_forward",forward_,forward_);
    nh_.param("axis_reverse",reverse_,reverse_);
    nh_.param("axis_angular",angular_,angular_);
    nh_.param("axis_lboost",lin_boost_,lin_boost_);
    nh_.param("axis_aboost",ang_boost_,ang_boost_);
    nh_.param("button_brake",brake_,brake_);

    prev_lboost_ = 0;
    prev_aboost_ = 0;

    l_scale_ = 1;
    a_scale_ = 1;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
    //not sure why keeping a button/axis at constant value results in a stop
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",50,&TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

    //to prevent multiple boost change triggers when receiving joy events
    double boost = joy->axes[lin_boost_]; //dummy var
    if(boost!=0)
    {
        //one nasty long line of cocaine
        l_scale_ = ((boost>0 && boost+l_scale_>10)||(boost<0 && boost+l_scale_<1))?(l_scale_):(l_scale_ + boost); //keep it from going over 10 or under 1
        //IDEA: if using button, use modulo to cycle through boosts
    } //else l_scale unchanged
    prev_lboost_ = boost;

    boost = -1*joy->axes[ang_boost_];
    if(boost!=0)
    {
        a_scale_ = ((boost>0 && boost+a_scale_>10)||(boost<0 && boost+a_scale_<1))?(a_scale_):(a_scale_ + boost); //keep it from going over 10 or under 1
    } //else a_scale unchanged;
    prev_aboost_ = boost;

    double forward_vel = 0.5*(1 - joy->axes[forward_]); //0.5 to account for mapping
    //less speed in reverse
    double reverse_vel = 0.75*0.5*(1 - joy->axes[reverse_]);
    double speed = l_scale_*(forward_vel-reverse_vel);
    //apply breaks if needed
    twist.linear.x = (joy->buttons[brake_]==0)?(speed):(0);
    twist.angular.z = a_scale_*joy->axes[angular_];
    vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"teleop_turtle");
    TeleopTurtle teleop_turtle;

    ros::spin();
}
