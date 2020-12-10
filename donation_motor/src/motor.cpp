/*
 * motor.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <motor_header/motor.h>
#include <fstream>

pthread_t* thread_num1;

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/donation_raspberrypi4/donation_motor/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Acceleration_ratio = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
      case 8: P[LEFT_MOTOR] = atof(line.substr(found+2).c_str()); break;
      case 9: I[LEFT_MOTOR] = atof(line.substr(found+2).c_str()); break;    
      case 10: D[LEFT_MOTOR] = atof(line.substr(found+2).c_str()); break; 
      case 11: P[RIGHT_MOTOR] = atof(line.substr(found+2).c_str()); break; 
      case 12: I[RIGHT_MOTOR] = atof(line.substr(found+2).c_str()); break; 
      case 13: D[RIGHT_MOTOR] = atof(line.substr(found+2).c_str()); break;   
      case 14: p_yaw_gain = atof(line.substr(found+2).c_str()); break;  
      case 15: i_yaw_gain = atof(line.substr(found+2).c_str()); break;  
      case 16: d_yaw_gain = atof(line.substr(found+2).c_str()); break; 
          
      }
      i +=1;
  }
  inFile.close();
}



int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);
  
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_HIGH);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  MotorDir[LEFT_MOTOR] = true;
  MotorDir[RIGHT_MOTOR] = true;

  acceleration = PWM_limit/(Acceleration_ratio);

  ROS_INFO("Setup Fin");
  return 0;
}

void Interrupt_Setting(void)
{  // thread_num1 = start_thread(Interrupt1A, &dsa);
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    
}
int lastEncoded=0;
//gpioThreadFunc_t asd



void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
 
  //if(gpio_read(pinum,motor1_ENA)==true && gpio_read(pinum,motor1_ENB) ==false) EncoderCounter1A ++;
 // else if(gpio_read(pinum,motor1_ENA)==false & gpio_read(pinum,motor1_ENB) == true) EncoderCounter1A ++;
 // else if(gpio_read(pinum,motor1_ENA)==true && gpio_read(pinum,motor1_ENB) ==true) EncoderCounter1A-- ;
 // else if (gpio_read(pinum,motor1_ENA)==false && gpio_read(pinum,motor1_ENB) ==false) EncoderCounter1A-- ;
 //printf("A : %d || B : %d \n",gpio_read(pinum,motor1_ENA),gpio_read(pinum,motor1_ENB));
 //printf("   encoder A : %d \n",EncoderCounter1A);

  if(gpio_read(pinum, motor1_DIR) == true)  EncoderCounter1A ++;
  else EncoderCounter1A --;
  
  EncoderSpeedCounter1 ++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{

//if(gpio_read(pinum,motor1_ENA)==true && gpio_read(pinum,motor1_ENB) ==false) EncoderCounter1B --;
// else if(gpio_read(pinum,motor1_ENA)==false && gpio_read(pinum,motor1_ENB) == true) EncoderCounter1B --;
// else if(gpio_read(pinum,motor1_ENA)==true && gpio_read(pinum,motor1_ENB) ==true) EncoderCounter1B++ ;
// else if (gpio_read(pinum,motor1_ENA)==false && gpio_read(pinum,motor1_ENB) ==false) EncoderCounter1B++ ;
 if(gpio_read(pinum,motor1_DIR) == true) EncoderCounter1B ++;
 else EncoderCounter1B --;
 
  EncoderSpeedCounter1 ++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    
//int MSB = gpio_read(pinum,motor2_ENA);
//int LSB = gpio_read(pinum,motor2_ENB);

 //int encoded = (MSB<<1) | LSB;
 //int sum = (lastEncoded<<2) | encoded;

 //if(sum == 0b1000 || sum ==0b0001 || sum == 0b0111 || sum == 0b1110) EncoderCounter2A ++;
 //else if(sum == 0b0010 || sum ==0b1011 || sum == 0b1101 || sum == 0b0100) EncoderCounter2A --;

 //if(sum == 0b1000) EncoderCounter2A ++;
 
 //lastEncoded = encoded;

 //if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2A ++;
 //else if(gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) == true) EncoderCounter2A ++;
 //else if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==true) EncoderCounter2A-- ;
 //else if (gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2A-- ;
 //printf("A : %d || B : %d \n",gpio_read(pinum,motor2_ENA),gpio_read(pinum,motor2_ENB));
 //printf("   encoder A : %d \n",EncoderCounter2A);

 if(gpio_read(pinum, motor2_DIR) == false)EncoderCounter2A ++;
  else EncoderCounter2A --;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
 //if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2B --;
//else if(gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) == true) EncoderCounter2B --;
//else if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==true) EncoderCounter2B++ ;
//else if (gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2B++ ;
 

 if(gpio_read(pinum, motor2_DIR) == false)EncoderCounter2B ++;
 else EncoderCounter2B --;
  EncoderSpeedCounter2 ++;
}


int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  return EncoderCounter2;
}


void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();

  Wheel_round = 2*PI*Wheel_radius;
  Robot_round = 2*PI*Robot_radius;

  

  ROS_INFO("PWM_range %d", PWM_range);
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");  
}








void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if(motor_num == LEFT_MOTOR)
  {
    if(direction == true)
    {
      gpio_write(pinum, motor1_DIR,  PI_HIGH );
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      MotorDir[LEFT_MOTOR] = true;
      current_Direction1 = true; // for control accelation
    }
    else if(direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      MotorDir[LEFT_MOTOR] = false;
      current_Direction1 = false;
    }
  }
  
  else if(motor_num == RIGHT_MOTOR)
  {
   if(direction == true)
   {
     gpio_write(pinum, motor2_DIR, PI_LOW );
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = true;
     MotorDir[RIGHT_MOTOR] = true;
   }
   else if(direction == false)
   {
     gpio_write(pinum, motor2_DIR, PI_HIGH );
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = false;
     MotorDir[RIGHT_MOTOR] = false;
   }
  }
}


void Accel_Controller(int motor_num, bool direction, int desired_pwm)
{
  bool local_current_direction;
  int local_PWM;
  int local_current_PWM;

  if(motor_num == 1) // LEFT MOTOR
  {
    local_current_direction = current_Direction1;
    local_current_PWM = current_PWM1;
  }
  else if(motor_num == 2) // RIGHT MOTOR
  {
    local_current_direction = current_Direction2;
    local_current_PWM = current_PWM2;
  }
//--------------------------------------
  if(direction == local_current_direction)
  {
    if(desired_pwm > local_current_PWM)
    {
      local_PWM = local_current_PWM + acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else if(desired_pwm < local_current_PWM)
    {
      local_PWM = local_current_PWM - acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
  else
  {
	  if(desired_pwm >= 0)
	  {
      local_PWM = local_current_PWM - acceleration;
      if(local_PWM <= 0)
      {
        local_PWM = 0;
        Motor_Controller(motor_num, direction, local_PWM);
      }
      else Motor_Controller(motor_num, local_current_direction, local_PWM);
	  }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
}



int Limit_Function(int pwm)
{
  int output;
  if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)
  {
	output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output; 
}

//RPM_Calculator 지우기 ! 
void updateCurlRpm(){
  CurlRPM[LEFT_MOTOR] = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter1 = 0;
  CurlRPM[RIGHT_MOTOR] = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter2 = 0;
}


