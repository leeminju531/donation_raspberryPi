#include <ros/ros.h>
#include "motor.cpp"
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>
#include <pthread.h>


enum _case{
    Basic = 0,
    Stop ,
    Tracking,
    INTERACTION,
    Close_Distance,
    Lost_Detection,
    Back_Origin
};

enum _monitor{
    VoiceCall=1,
    StartDisplay
};


#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(X) ((X)*M_PI/180.)

#define FIRST_DETECTION_DISTANCE_AREA 1.5

#define MAX_LINEAR_VEL 0.5 //(m/s)
#define MAX_ANGULAR_VEL 2

#define CLOSE_DISTANCE 0.9 // m

#define DELTA_LINEAR_VEL 0.05
#define DELTA_ANGULAR_VEL 0.2

char motor_Action_Flag = Basic;

double linearV_X2Rpm[2]{};
double angularV_Z2Rpm[2]{};

void pos_pid();
void linearV2rpm(double velocity);
void angularV2rpm(double Angular_velocity);

double linearVel=0;
double angularVel=0;

double real_linerVel_X=0;
double real_angularVel_Z=0;

void update_real_vel();

int interaction_done=false;
int push_Button=false;
int detecPerson = false;

double world_distance_at_robotFrame;
double world_theta_at_robotFrame;

//  vel unit is m/sec
double RPM2Vel(double rpm){
    //RPM : cm/minute -> linearV = m/sec
    double vel = 2*PI*Wheel_radius*rpm/60/100;    
    return vel;
}

double encoder2Vel(double encoder){

    
}

// rpm unit : 2*pi/60 (rad/min)
double vel2RPM(double vel){
    double rpm = vel/(2*PI*Wheel_radius)*60*100;
    return rpm;
}


//linearV_X : m/s
//angularV_Z : rad/s
void update_targetRPM(double linearV_X,double angularV_Z){
    double LEFT_TARGET_RPM,RIGHT_TARGET_RPM;    
    //update linearV_X2Rpm[2], angularV_Z2Rpm[2]
    linearV2rpm(linearV_X); 
    angularV2rpm(angularV_Z);
    
    LEFT_TARGET_RPM = linearV_X2Rpm[LEFT_MOTOR] + angularV_Z2Rpm[LEFT_MOTOR];
    RIGHT_TARGET_RPM = linearV_X2Rpm[RIGHT_MOTOR] + angularV_Z2Rpm[RIGHT_MOTOR];

    if(LEFT_TARGET_RPM >= 0) MotorDir[LEFT_MOTOR] = true;
    else MotorDir[LEFT_MOTOR] = false;

    if(RIGHT_TARGET_RPM >= 0) MotorDir[RIGHT_MOTOR] = true;
    else MotorDir[RIGHT_MOTOR] = false;
    
    TargetRPM[LEFT_MOTOR] = abs(LEFT_TARGET_RPM);
    TargetRPM[RIGHT_MOTOR] = abs(RIGHT_TARGET_RPM);

}

//input unit : m/s
// update linearV_X2Rpm[2] 
void linearV2rpm(double velocity){
    
    linearVel = velocity ; //to print linearVel
    if(linearVel > MAX_LINEAR_VEL) linearVel = MAX_LINEAR_VEL; // limit : linearVelocity : 0.5m/s
    
    linearV_X2Rpm[LEFT_MOTOR] = linearVel/(2*PI*Wheel_radius)*60*100;
    linearV_X2Rpm[RIGHT_MOTOR] = linearV_X2Rpm[LEFT_MOTOR];
}

// input unit : rad/s
//update angularV_Z2Rpm[2]
void angularV2rpm(double Angular_velocity){
    angularVel = Angular_velocity;
    if (angularVel > MAX_ANGULAR_VEL) angularVel = MAX_ANGULAR_VEL; // limit : angularVelocity : 1rad/s
    else if(angularVel < -MAX_ANGULAR_VEL) angularVel= -MAX_ANGULAR_VEL;
   
    angularV_Z2Rpm[LEFT_MOTOR]  = -angularVel * (Robot_radius / Wheel_radius) * 60/2/PI;
    angularV_Z2Rpm[RIGHT_MOTOR] = +angularVel * (Robot_radius / Wheel_radius) * 60/2/PI;
    
}


void flagMsgCallback(const std_msgs::Byte msg){
    motor_Action_Flag = msg.data;
}

float distance = 0; // unit : m
float theta = 0; // unit : rad


// msg.x mean distance 
// msg.x unit : m
// this subscriebed only by lidar
void poseMsgCallback(const geometry_msgs::Pose2D msg){

  //  if(motor_Action_Flag != Tracking ) return;
    distance = msg.x;
    theta = msg.theta;
}


//note : 
//robot front : x axis      robot upward : z axis
void pos_pid(){
    float linearV_X,angularV_Z;
    //double p_vel_gain = 0.5; // temporary

    err_yaw = theta;
    err_yaw_sum += err_yaw;
    
    double p_yaw_val = (err_yaw)*p_yaw_gain;
    double i_yaw_val = err_yaw_sum*i_yaw_gain;
    double d_yaw_val = (err_yaw-err_yaw_prev)*d_yaw_gain;
    
    angularV_Z = p_yaw_val + i_yaw_val + d_yaw_val; //pd control about position angle
   

    if(theta < 0.088 && theta >-0.088) angularV_Z = 0; //0.088rad = 5 degree

    if(distance < 0.8)  {
        linearV_X = 0;
        if(theta < 0.088 && theta >-0.088) angularV_Z = 0; //0.088rad = 5 degree

    }else if(distance <= 1) linearV_X = MAX_LINEAR_VEL*( sqrt(1-(distance-1)/0.2) )*( sqrt(1+(distance-1)/0.2) );   //linearV_X = 5.56*(distance-0.7)*(distance-0.7);
    else if( distance > 1) linearV_X = MAX_LINEAR_VEL;
    
  //  linearV_X = 0;
   
    update_targetRPM(linearV_X,angularV_Z);
    err_yaw_prev = theta;
}

void pos_pid(float distance, float theta){
    float linearV_X,angularV_Z;
    //double p_vel_gain = 0.5; // temporary

    err_yaw = theta;
    err_yaw_sum += err_yaw;
    
    double p_yaw_val = (err_yaw)*p_yaw_gain;
    double i_yaw_val = err_yaw_sum*i_yaw_gain;
    double d_yaw_val = (err_yaw-err_yaw_prev)*d_yaw_gain;
    
    angularV_Z = p_yaw_val + i_yaw_val + d_yaw_val; //pd control about position angle
   

    if(theta < 0.088 && theta >-0.088) angularV_Z = 0; //0.088rad = 5 degree

    if(distance < 0.6)  {
        linearV_X = 0;
        if(linearV_X < 0 )  linearV_X = 0;

        if(theta < DEG2RAD(5) && theta >DEG2RAD(-5)) angularV_Z = 0; //0.088rad = 5 degree
    
    }else if(distance <= 1) linearV_X = MAX_LINEAR_VEL*( sqrt(1-(distance-1)/0.4) )*( sqrt(1+(distance-1)/0.4) );   //linearV_X = 5.56*(distance-0.7)*(distance-0.7);
    else if( distance > 1) linearV_X = MAX_LINEAR_VEL;
    
  //  linearV_X = 0;
   
    update_targetRPM(linearV_X,angularV_Z);
    err_yaw_prev = theta;
}





int a = 0;
int b = 0;
int output[2]{};
//to maintain TargetRPM[]


int pre_output[2]{};
//pid control about error 
void pidControl(){
    double p_val[2]{};
    double i_val[2]{};
    double d_val[2]{};

    
    int local_pwm[2]{};
    updateCurlRpm();
    update_real_vel();
// -------------------------------- LEFT MOTOR PID -----------------------
    err[LEFT_MOTOR] = TargetRPM[LEFT_MOTOR] - CurlRPM[LEFT_MOTOR];
//    if(abs(err[LEFT_MOTOR]) > 200)  {
//        printf("error occured\n");
//        printf("err[LEFT_MOTOR] = %f\n",err[LEFT_MOTOR]);
//        printf("TargetRPM[LEFT_MOTOR] = %f || CurRPM[LEFT_MOTOR] =%f \n", TargetRPM[LEFT_MOTOR],CurlRPM[LEFT_MOTOR]);
//        printf("LinearVel : %f || angularVel : %f \n",linearVel,angularVel);
//       printf("linearV_X2Rpm[LEFT] = %f || linearV_X2Rpm[RIGHT] = %f\n",linearV_X2Rpm[LEFT_MOTOR],linearV_X2Rpm[RIGHT_MOTOR]);
//        printf("angularV_Z2Rpm[LEFT] = %f || angularV_Z2Rpm[RIGHT] = %f\n",angularV_Z2Rpm[LEFT_MOTOR],angularV_Z2Rpm[RIGHT_MOTOR]);
//        printf("theta = %f || distance = %f \n",theta,distance);
//        Motor_Controller(1, true, 0);
//        Motor_Controller(2, true, 0);
//        exit(1);
//    }
    err_sum[LEFT_MOTOR] += err[LEFT_MOTOR];
    
    p_val[LEFT_MOTOR] = P[LEFT_MOTOR] * err[LEFT_MOTOR];
    i_val[LEFT_MOTOR] = I[LEFT_MOTOR] * err_sum[LEFT_MOTOR];
    d_val[LEFT_MOTOR] = D[LEFT_MOTOR] * ( err[LEFT_MOTOR] - err_prev[LEFT_MOTOR] );

    err_prev[LEFT_MOTOR] = err[LEFT_MOTOR];

    output[LEFT_MOTOR] = (int) (pre_output[LEFT_MOTOR] + p_val[LEFT_MOTOR] + i_val[LEFT_MOTOR] + d_val[LEFT_MOTOR] );

    local_pwm[LEFT_MOTOR] = Limit_Function(output[LEFT_MOTOR]);
    a = local_pwm[LEFT_MOTOR];
// -------------------------------- RIGHT MOTOR PID -----------------------

    err[RIGHT_MOTOR] = TargetRPM[RIGHT_MOTOR] - CurlRPM[RIGHT_MOTOR];
    err_sum[RIGHT_MOTOR] += err[RIGHT_MOTOR];

    p_val[RIGHT_MOTOR] = P[RIGHT_MOTOR] * err[RIGHT_MOTOR];
    i_val[RIGHT_MOTOR] = I[RIGHT_MOTOR] * err_sum[RIGHT_MOTOR];
    d_val[RIGHT_MOTOR] = D[RIGHT_MOTOR] * ( err[RIGHT_MOTOR] - err_prev[RIGHT_MOTOR] );

    err_prev[RIGHT_MOTOR] = err[RIGHT_MOTOR];

    output[RIGHT_MOTOR] = (int) (pre_output[RIGHT_MOTOR]+ p_val[RIGHT_MOTOR] + i_val[RIGHT_MOTOR] + d_val[RIGHT_MOTOR] );

    local_pwm[RIGHT_MOTOR] = Limit_Function(output[RIGHT_MOTOR] );
    b = local_pwm[RIGHT_MOTOR];
// -------------------------------------------------------------------------
    
    Motor_Controller(LEFT_MOTOR, MotorDir[LEFT_MOTOR] , local_pwm[LEFT_MOTOR] );
    Motor_Controller(RIGHT_MOTOR, MotorDir[RIGHT_MOTOR] , local_pwm[RIGHT_MOTOR]);
    

    pre_output[LEFT_MOTOR] = output[LEFT_MOTOR];
    pre_output[RIGHT_MOTOR] = output[RIGHT_MOTOR];
}

void update_real_vel(){

    double leftMotorVel = RPM2Vel(CurlRPM[LEFT_MOTOR]);
    double rightMotorVel = RPM2Vel(CurlRPM[RIGHT_MOTOR]);
    
    if(MotorDir[LEFT_MOTOR] == false) leftMotorVel = -leftMotorVel;

    if(MotorDir[RIGHT_MOTOR] == false)  rightMotorVel = -rightMotorVel;


    real_linerVel_X = ( leftMotorVel + rightMotorVel )/2;
    real_angularVel_Z = ( rightMotorVel - leftMotorVel )/2/Robot_radius*100;

    
}






void Motor_View()
{
    
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("Current RPM1 : %10.3f    || Current RPM2 : %10.3f\n", CurlRPM[LEFT_MOTOR] , CurlRPM[RIGHT_MOTOR]);
    printf("TargetRPM1 :%10.3f     ||  TargetRPM2 :%10.3f\n", TargetRPM[LEFT_MOTOR], TargetRPM[RIGHT_MOTOR]);
    printf("motor1_p : %f || motor2_P : %f \n",P[LEFT_MOTOR],P[RIGHT_MOTOR]);
    printf("motor1_I : %f || motor2_I : %f\n",I[LEFT_MOTOR],I[RIGHT_MOTOR]);
    printf("motor1_D : %f || motor2_D : %f \n",D[LEFT_MOTOR],D[RIGHT_MOTOR]);
    printf("motor1 pwm :%10.0d  || motor2 pwm :%10.0d\n",a,b);
    
    
    printf(" robot linearVelocity : %f(m/s) || angularVel : %f(rad/s)\n",linearVel,angularVel);
    printf(" sub distance : %f(m) || angle : %f (rad)\n",distance,theta);

    printf("motor1 dir : %d || motor2 dir: %d \n ",MotorDir[LEFT_MOTOR],MotorDir[RIGHT_MOTOR]);
    printf("target_linearVel : %.3f || target_angularVel : %.3f\n",linearVel,angularVel);
    printf("real_linearVel : %.3f || real_angularVel : %.3f\n",real_linerVel_X,real_angularVel_Z);

   //printf("distance with world : %f || theta with world %f\n",world_distance_at_robotFrame,world_theta_at_robotFrame);
    printf("flag : %d\n",motor_Action_Flag);
	printf("\n");
}


void tfCallback(const geometry_msgs::Pose2D msg){
    world_distance_at_robotFrame = msg.x; // unit : m
    world_theta_at_robotFrame = msg.theta; // unit : rad

}

void tfVelCallback(const geometry_msgs::Twist msg){

    linearVel = msg.linear.x;
    angularVel = msg.angular.z;
}


void InteractionCallback(const std_msgs::String msg){

    if(motor_Action_Flag != INTERACTION)    return;
    
    if(msg.data =="0")   interaction_done=false;
    else if (msg.data == "1") interaction_done = true;
//    interaction_done = stoi(msg.data);

}

void buttonCallback(const std_msgs::Byte msg){
    
   // if(motor_Action_Flag != Tracking || motor_Action_Flag != INTERACTION)    return;
    //printf("msg.data : %d \n",msg.data);
    push_Button = msg.data;
}

void detectCallback(const std_msgs::Byte msg){
    if(motor_Action_Flag != Basic)    return;

    detecPerson = msg.data;
   
}


void* thread_LeftEncoder(void *message){
   printf("start thread func\n");
    while(1){
        callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
        callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
        callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
        callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
        printf("in Thread func\n");
    }
    printf("end of Letf Encoder Thread !! \n");
}

void* thread_RightEncoder(void *message){
    while(1){
        
    }
    printf("end of Right Encoder Thread !! \n");

}


int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
 // void* result;
  printf("111");
  //pthread_t *ptr;
  //ptr = start_thread(thread_LeftEncoder,NULL);
  printf("222");

  //if( ptr == NULL )  {
  //  printf("fail Left Encoder thread\n");
  //  exit(1);
  //}

 // ptr_Right = start_thread(thread_RightEncoder,&result);
  //if( ptr_Right == NULL )  {
  //  printf("fail Right Encoder thread\n");
  //  exit(1);
 // }
  
  ros::Rate loop_rate(Control_cycle);
  //ros::Subscriber pose_sub = nh.subscribe("/motor_node/depth/pose",1,poseMsgCallback);
  ros::Subscriber pose_sub = nh.subscribe("/motor_node/ydlidar/pose",10,poseMsgCallback);
  ros::Subscriber flag_sub = nh.subscribe("/motor_node/flag",10,flagMsgCallback);
  ros::Subscriber poseSubByTf = nh.subscribe("/donationTF/pose",10,tfCallback);
  ros::Subscriber cmdSubByTf = nh.subscribe("/donationTF/cmd",10,tfVelCallback);
  MotorDir[LEFT_MOTOR] = true;
  MotorDir[RIGHT_MOTOR] = true;
  motor_Action_Flag = Basic;
  
  
  geometry_msgs::Twist odom;
  ros::Publisher odomPub = nh.advertise <geometry_msgs::Twist>("/motor_node/cmd_vel",10);

  std_msgs::String monitor_control;
  ros::Subscriber InteractionDone = nh.subscribe("/pyqt/interaction_Done",10,InteractionCallback);
  ros::Publisher MonitorControl  = nh.advertise<std_msgs::String>("/motor_node/monitorControl",10);
  ros::Subscriber PushButton = nh.subscribe("/button_node/button",10,buttonCallback);

  ros::Subscriber DetectPerson = nh.subscribe("/cam/detect",10,detectCallback);
  int detectCount = 0; 
  int trackingCount = 0;
  double pre_theta;
  double pre_distance;
  while(ros::ok())
  {
    switch(motor_Action_Flag){
        case Basic:
         //   ROS_INFO("BASIC STATE");
          //  ROS_INFO("PointTurn untill Person Detection");
            
         //   
            pos_pid(0,theta);
      //      update_targetRPM(0.4,0);   
    //        printf("distance : %f || theta = %f \n",distance, RAD2DEG(theta));
     //       printf("angularVel : %f\n",angularVel);
           // TargetRPM[LEFT_MOTOR] = 50;
           // TargetRPM[RIGHT_MOTOR] = 50;
            if(distance >0 && distance < FIRST_DETECTION_DISTANCE_AREA)
                if(theta > DEG2RAD(-5) && theta < DEG2RAD(5))   detectCount++;
                else    detectCount = 0;

            if(detectCount >= 30){ // 3sec
                 detectCount = 0;
                 motor_Action_Flag = Tracking;
            }          
            break;
       
        case Tracking :
            trackingCount++;
            ROS_INFO("Person Tracking");
            
            pos_pid();
   
            if(world_distance_at_robotFrame > 4){
                motor_Action_Flag=Back_Origin;
                trackingCount = 0;
            }
            if (trackingCount > 1/Control_cycle*10*60*5  ){ // person no response for 5 min ->back origin
                motor_Action_Flag = Back_Origin;
                trackingCount = 0;
            }
            if (distance > 0 && distance < CLOSE_DISTANCE){
                monitor_control.data = "1";
                MonitorControl.publish(monitor_control);
                trackingCount = 0;
                printf("voice call\n");
            }

           


            if(push_Button == true){
                push_Button = false;
                monitor_control.data = "2";
                MonitorControl.publish(monitor_control);
                motor_Action_Flag = INTERACTION;
                printf("start Interaction\n");
            }
            break;

        case INTERACTION :
            ROS_INFO("INTERACTION....");
            update_targetRPM(0,0);
        
            //temporary code        
            detectCount++;
            if(push_Button == true && detectCount > 20){
                detectCount = 0;
                push_Button = false;
                motor_Action_Flag = Back_Origin;
            }
            //end 
        
            if (interaction_done == true){
                interaction_done = false;
                
                motor_Action_Flag = Back_Origin;
            }

            break;  

        case Back_Origin:
            

            double OBSTACLEDETECTION = 1.2;
            if( distance >0 && distance < OBSTACLEDETECTION && theta < DEG2RAD(20) && 
theta > DEG2RAD(-20) ){
            linearVel = 0;
            angularVel = 0;
        }

            update_targetRPM(linearVel,angularVel);
            
            if(world_distance_at_robotFrame <0.1)//ERROR_DISTANCE)
                motor_Action_Flag = Basic;
            

            break;
            
            
    }
    pidControl();
    Motor_View();

    
    odom.linear.x = real_linerVel_X;
    odom.angular.z = real_angularVel_Z;
  
    odomPub.publish(odom);

    
    ros::spinOnce();
    loop_rate.sleep();
   
  }
  
  Motor_Controller(LEFT_MOTOR, true, 0);
  Motor_Controller(RIGHT_MOTOR, true, 0);
  //stop_thread(ptr);
 // stop_thread(ptr_Right);
  return 0;
}
