// this code 
// broadcast "world_Frame " -> "base_Frame" based on cmd_vel

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>



double x = 0;
double y = 0;
double th = 0;



void velCallback(const geometry_msgs::Twist msg){
	static tf2_ros::TransformBroadcaster br;

	// msg will be subscribed every ControlCycle(10hz)	
	double delta_distance = msg.linear.x / 10;
	double delta_angle = msg.angular.z / 10;

	double delta_x = delta_distance * cos(th);
	double delta_y = delta_distance * sin(th);

	x += delta_x;
	y += delta_y;
	th += delta_angle;


	geometry_msgs::TransformStamped transformStamped; 

	transformStamped.header.stamp = ros::Time::now();//we need to give the transform being published a timestamp, in this case current time
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "base";
	transformStamped.transform.translation.x = x;
	transformStamped.transform.translation.y = y;
	transformStamped.transform.translation.z = 0; 
	
	tf2::Quaternion q;
	q.setRPY(0,0,th);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped); // this is where real work is done

	printf("delta_x = %.3f || delta_y = %.3f || delta_th = %.3f\n",delta_x,delta_y,delta_angle);
	printf("x = %.3f || y = %.3f || theta = %.3f\n",x,y,th);
	printf("delta_distance : %f || delta_angle = %f\n",delta_distance,delta_angle);






}



int main(int argc, char** argv){
	ros::init(argc,argv,"donation_tf_broadcaster");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("/motor_node/cmd_vel",10,velCallback);
	
	ros::spin();
	return 0;
}


