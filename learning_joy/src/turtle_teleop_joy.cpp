#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  double l_tur_scale,a_tur_scale;
  ros::Publisher twist_pub_;
  ros::Subscriber joy_sub_;
  int enable_key,turbo_key;

  // use for cmd_vel_step
  // parameter
  double a_step = 0.01,l_step = 0.01;
  bool use_step = false;

  // memory
  double c_a_cmd_vel=0.0,c_l_cmd_vel = 0.0;

  
};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  // read param
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("turbo_scale_angular",a_tur_scale,a_tur_scale);
  nh_.param("turbo_scale_linear",l_tur_scale,l_tur_scale);
  nh_.param("enable_key",enable_key,enable_key);
  nh_.param("turbo_key",turbo_key,turbo_key);

  // make smooth cmd_vel # anakin
  nh_.param("a_step",a_step,a_step);
  nh_.param("l_step",l_step,l_step);
  nh_.param("use_step",use_step,use_step);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // check enable
  if(joy->buttons[enable_key]){
    geometry_msgs::Twist twist;

    double a_scale = a_scale_;
    double l_scale = l_scale_;

    if(joy->buttons[turbo_key]){
      a_scale = a_tur_scale;
      l_scale = l_tur_scale;
    }

    twist.angular.z = a_scale*joy->axes[angular_];
    twist.linear.x = l_scale*joy->axes[linear_];

    if(use_step){

      std::printf("hi");

      // angular
      if(twist.angular.z > c_a_cmd_vel)
        c_a_cmd_vel = std::min(twist.angular.z,c_a_cmd_vel + a_step);
      else
        c_a_cmd_vel = std::max(twist.angular.z,c_a_cmd_vel - a_step);

      // linear
      if(twist.linear.x > c_l_cmd_vel)
        c_l_cmd_vel = std::min(twist.linear.x,c_l_cmd_vel + l_step);
      else
        c_l_cmd_vel = std::max(twist.linear.x,c_l_cmd_vel - l_step);

      twist.angular.z = c_a_cmd_vel;
      twist.linear.x = c_l_cmd_vel;
    }

    twist_pub_.publish(twist);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
