#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
//#include <std_msgs/Byte.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.14159265
#define ACTIVITY_AREA 3 // temporary 


#define MAX_ANGULAR 1
#define MAX_LINEAR 0.3


double obstacle_distance = 5;
double obstacle_theta = 0;


enum _case{
    Basic = 0,
    Stop ,
    Tracking,
    Close_Distance,
    Lost_Detection,
    OutOfActionArea,
    Donation_End,
    Back_Origin,
};

#define OBSTACLEDETECTION 0.7


// msg.x mean distance 
// msg.x unit : m
// this subscriebed only by lidar
void poseMsgCallback(const geometry_msgs::Pose2D msg){
    printf("lidar sub <tf listener>\n");
    obstacle_distance = msg.x;
    obstacle_theta = msg.theta;
    
    if(obstacle_distance == 0)  obstacle_distance=5; // obstacle_distance = 0 mean detect nothing
}

float rad2deg(float radian)
{
    return radian*180/PI;
}
float deg2rad(float degree)
{
    return degree*PI/180;
}

float x = 0;
float y = 0;
float theta = 0;

int main(int argc, char** argv){
	ros::init(argc,argv,"donation_tf_listener");
	ros::NodeHandle node;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
//	ros::Publisher flag_pub = node.advertise<std_msgs::Byte>("/donationTF/flag",10);
	ros::Publisher pose_pub = node.advertise<geometry_msgs::Pose2D>("/donationTF/pose",10);
    ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("/donationTF/cmd",10);
    ros::Subscriber pose_sub = node.subscribe("/motor_node/ydlidar/pose",10,poseMsgCallback);

//std_msgs::Byte flag;
	geometry_msgs::Pose2D pose;
    geometry_msgs::Twist vel;

	ros::Rate rate(20.0);
	while(node.ok()){
		geometry_msgs::TransformStamped transformStamped;
		try{
			transformStamped = tfBuffer.lookupTransform("base","world",ros::Time(0)); // the real work is done
			//ros::Time(0) will just get us the latest available transform in the buffer
		}
		catch(tf2::TransformException &ex){
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();  // mean sec
			continue; // -> ★★★★★★ in case by doing 'continue', process restart 'while')
		}
		x = transformStamped.transform.translation.x;
		y = transformStamped.transform.translation.y;
        //theta = transformStamped.transform.rotation.z;
		if(x == 0){
			if( y > 0)	theta = deg2rad(90);
			else if( y<0 ) theta = deg2rad(-90);
			else theta = 0; // case x=y=0
		}
		theta = atan2(y,x);

		pose.x = sqrt(x*x+y*y);
		pose.theta = theta;
		pose_pub.publish(pose);

    //    printf("distance : %f || theta = %f(deg) \n",pose.x,rad2deg(theta));
	//	printf("origin at base || x : %.3f(m) || y : %.3f(m) || theta =: %.3f(deg)\n",x,y,rad2deg(theta)); 


        /* based on lidar , pose position
           publish linearVel, angularVel
        */
        
		
     //   
        
        


       if( obstacle_distance < OBSTACLEDETECTION && obstacle_theta < deg2rad(20) && 
obstacle_theta > deg2rad(-20) ){
            //if (obstacle_theta >= 0)    vel.angular.z = -MAX_ANGULAR;
            //else    vel.angular.z = +MAX_ANGULAR;
            vel.angular.z = 0;
            vel.linear.x = 0;
            cmd_pub.publish(vel);
            continue;
            
        }


        double x = pose.theta;
        double accel_limit_angle = deg2rad(30); // temporary

        if(pose.theta > accel_limit_angle)  vel.angular.z = MAX_ANGULAR;
        else if (pose.theta < -accel_limit_angle) vel.angular.z = -MAX_ANGULAR;
        else vel.angular.z = x*( (x+accel_limit_angle)*(x-accel_limit_angle) + MAX_ANGULAR/accel_limit_angle);

        double accel_limit_distance = 0.1;
        if(pose.x >= accel_limit_distance)  vel.linear.x = MAX_LINEAR;
        else vel.linear.x = MAX_LINEAR/accel_limit_distance * pose.x;
       
        cmd_pub.publish(vel);

	}


	return 0;
}
