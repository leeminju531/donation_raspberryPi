
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Pose2D.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(X) ((X)*M_PI/180.)
#define DEGREE 0
#define DISTANCE 1

#define HARDWARE_ANGLE 60 // DEGREE



int cnt = 0;
float degree_distance[400][2]{};
int cant_find= false;
float x_axis_closeDegree,x_axis_closeDistance;

float min_distance = 20.0;
int index1;

geometry_msgs::Pose2D pose;
ros::Publisher pose_pub ;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	    if(degree > -80 + HARDWARE_ANGLE && degree< 80 + HARDWARE_ANGLE){
            printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
            degree -= HARDWARE_ANGLE;
            degree_distance[cnt][DEGREE] = degree;
            degree_distance[cnt][DISTANCE] = scan->ranges[i];
            cnt++;    
        }             
    }
    int start = cnt/2; // start at degree:0
    for(int i = 0 ; i <start ; i++){
        

        if(degree_distance[start+i][DISTANCE] > 0.1 && degree_distance[start+i][DISTANCE]<1.7){
          
            x_axis_closeDegree = degree_distance[start+i][DEGREE];
            x_axis_closeDistance = degree_distance[start+i][DISTANCE];
            break;
        }else if(degree_distance[start-i][DISTANCE] > 0.1 && degree_distance[start-i][DISTANCE]<1.7){
            
            x_axis_closeDegree = degree_distance[start-i][DEGREE];
            x_axis_closeDistance = degree_distance[start-i][DISTANCE];
            break;
        }
        if( i == start-1){
            cant_find = true;
            x_axis_closeDegree = 0;
            x_axis_closeDistance = 0;
            break;
        }
    }
    
//    for (int i=0; i<cnt; i++){
//        if (degree_distance[i][DISTANCE] <= 0.05) continue;
        
//        if(degree_distance[i][DISTANCE] < min_distance){
//            min_distance = degree_distance[i][DISTANCE];
 //           index1 = i;
 //       }
 //   }


 //   x_axis_closeDistance = min_distance;
 //   x_axis_closeDegree = degree_distance[index1][DEGREE];


    if(cant_find == true) {
        cant_find = false;
        printf("can't find close points \n");
        x_axis_closeDistance = 0;
        x_axis_closeDegree = 0;
    }

    printf("X axis close distance : %f  || close degree %f(deg) \n",x_axis_closeDistance ,x_axis_closeDegree);
    pose.x = x_axis_closeDistance;
    pose.theta = DEG2RAD(x_axis_closeDegree);
    pose_pub.publish(pose);
    

    cnt = 0;
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ydlidar_publisher");
    ros::NodeHandle n;
    pose_pub = n.advertise<geometry_msgs::Pose2D>("/motor_node/ydlidar/pose",10);

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
    ros::spin();
    
    return 0;
}
