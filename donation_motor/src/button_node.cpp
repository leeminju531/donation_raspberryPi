#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <std_msgs/Byte.h>
#define BUTTON_READER 25

int main(int argc, char** argv){

  ros::init(argc, argv, "button_node");
  ros::NodeHandle nh;
  ros::Publisher buttonPub = nh.advertise<std_msgs::Byte>("/button_node/button",10);
  int pinum=pigpio_start(NULL, NULL);
  std_msgs::Byte pushed;
  if(pinum<0)
  {
    ROS_INFO("Button Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }
  
  set_mode(pinum, BUTTON_READER, PI_INPUT);
  set_pull_up_down(pinum, BUTTON_READER, PI_PUD_UP);
  
  ros::Rate loop_rate(15);


  while(ros::ok()){
    
    if(gpio_read(pinum, BUTTON_READER) == false){
        pushed.data = true;
        buttonPub.publish(pushed);
        printf("ddd\n");
    }else{
        pushed.data = false;
        buttonPub.publish(pushed);
        printf("xxx\n");
    }
  }
    return 0;
}

