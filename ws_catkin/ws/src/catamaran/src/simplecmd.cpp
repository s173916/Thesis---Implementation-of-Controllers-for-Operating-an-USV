#include "ros/ros.h"
#include "std_msgs/String.h"
# include "std_msgs/Int16.h"
# include "std_msgs/Float32.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  int cnt = 0;
  std_msgs::Float32 cmd_left_angle, cmd_right_angle;
  std_msgs::Int16 cmd_left_thrust, cmd_right_thrust;
  
  ros::init(argc, argv, "simplecmd");

  ros::NodeHandle n;

  ros::Publisher out_angle_left = n.advertise<std_msgs::Float32>("/workshop_setup/pod_steer/left_steer", 1000);
  ros::Publisher out_angle_right = n.advertise<std_msgs::Float32>("/workshop_setup/pod_steer/right_steer", 1000);
  
  ros::Publisher out_thrust_left = n.advertise<std_msgs::Int16>("/workshop_setup/pods/left", 1000);
  ros::Publisher out_thrust_right = n.advertise<std_msgs::Int16>("/workshop_setup/pods/right", 1000);


  ros::Rate loop_rate(10);

  while(ros::ok())
  {
      cmd_left_angle.data = 75.0;
      cmd_right_angle.data = -75.0;
      out_angle_left.publish(cmd_left_angle);
      out_angle_right.publish(cmd_right_angle);
      
      cmd_left_thrust.data = 200;
      cmd_right_thrust.data = 200;
      out_thrust_left.publish(cmd_left_thrust);
      out_thrust_right.publish(cmd_right_thrust);
    
    /* Toggle every 10 sec */
    /*
    if(cnt == 100)
    {
      cmd_left_angle.data = 75.0;
      cmd_right_angle.data = -75.0;
      out_angle_left.publish(cmd_left_angle);
      out_angle_right.publish(cmd_right_angle);
      
      cmd_left_thrust.data = 200;
      cmd_right_thrust.data = 200;
      out_thrust_left.publish(cmd_left_thrust);
      out_thrust_right.publish(cmd_right_thrust);
    
    }
    
    if(cnt == 200)
    {
      cmd_left_angle.data = -75.0;
      cmd_right_angle.data = 75.0;
      out_angle_left.publish(cmd_left_angle);
      out_angle_right.publish(cmd_right_angle);
      
      cmd_left_thrust.data = -200;
      cmd_right_thrust.data = -200;
      out_thrust_left.publish(cmd_left_thrust);
      out_thrust_right.publish(cmd_right_thrust);
      
      
      
      cnt = 0;
    }
    
    cnt++;
    */
    
    ros::spinOnce();
    loop_rate.sleep();
  }


  return(0);
}
