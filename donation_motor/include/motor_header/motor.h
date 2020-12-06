#ifndef MOTOR_H
#define MOTOR_H
#include <pigpiod_if2.h>

#define motor1_DIR 19
#define motor1_PWM 26
#define motor1_ENA 23
#define motor1_ENB 24

#define motor2_DIR 6
#define motor2_PWM 13
#define motor2_ENA 27
#define motor2_ENB 17

#define PI 3.141592

# define CLOSEDISTANCE 50 // robot is stop when between robot and detected person is 50 cm
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

//Text_Input
void Text_Input(void);
int PWM_range;
int PWM_frequency;
int PWM_limit;
double Control_cycle;
int Acceleration_ratio;
double Wheel_radius;
double Robot_radius;
int Encoder_resolution;
double Wheel_round;
double Robot_round;

double P[2] {};
double I[2]{};
double D[2]{};

double err[2]{};
double err_sum[2]{};
double err_prev [2]{};

double p_yaw_gain;
double i_yaw_gain;
double d_yaw_gain;
double err_yaw=0;
double err_yaw_sum=0;
double err_yaw_prev=0;

volatile double TargetRPM[2]{};
volatile double CurlRPM[2] {};
bool MotorDir[2]{};

//Motor_Setup
int Motor_Setup(void);
int pinum;
int current_PWM1;
int current_PWM2;
bool current_Direction1;
bool current_Direction2;
int acceleration;

//Interrupt_Setting
void Interrupt_Setiing(void);
volatile unsigned int EncoderCounter1;
volatile unsigned int EncoderCounter2;
volatile int EncoderCounter1A;
volatile int EncoderCounter1B;
volatile int EncoderCounter2A;
volatile int EncoderCounter2B;
volatile int EncoderSpeedCounter1=0;
volatile int EncoderSpeedCounter2=0;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int Motor1_Encoder_Sum();
int Motor2_Encoder_Sum();
void Init_Encoder(void);
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void);
void updateCurlRpm(void);
//Motor_Controller
void Motor_Controller(int motor_num, bool direction, int pwm);
void Accel_Controller(int motor_num, bool direction, int desired_pwm);



//Utiliy
int Limit_Function(int pwm);
double RPM_Value1;
double RPM_Value2;
void RPM_Calculator();
void Motor_View();

#endif // MOTOR_H
