#include <ros/ros.h>
#include <pigpiod_if2.h>

#define motor1_DIR 19
#define motor1_PWM 26
#define motor1_ENA 23
#define motor1_ENB 24

#define motor2_DIR 6
#define motor2_PWM 13
#define motor2_ENA 27
#define motor2_ENB 17
int PWM_range = 512;
int PWM_frequency = 20000;
int pinum;
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

void Interrupt_Setting(void)
{  // thread_num1 = start_thread(Interrupt1A, &dsa);
   
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    
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

   Interrupt_Setting();

  ROS_INFO("Setup Fin");
  return 0;
}
int EncoderCounter2A=0;
int EncoderCounter2B = 0;

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    
//int MSB = gpio_read(pinum,motor2_ENA);
//int LSB = gpio_read(pinum,motor2_ENB);

 //int encoded = (MSB<<1) | LSB;
 //int sum = (lastEncoded<<2) | encoded;

 //if(sum == 0b1000 || sum ==0b0001 || sum == 0b0111 || sum == 0b1110) EncoderCounter2A ++;
 //else if(sum == 0b0010 || sum ==0b1011 || sum == 0b1101 || sum == 0b0100) EncoderCounter2A --;

 //if(sum == 0b1000) EncoderCounter2A ++;
 
// lastEncoded = encoded;

 if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2A ++;
 else if(gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) == true) EncoderCounter2A ++;
 else if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==true) EncoderCounter2A-- ;
 else if (gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2A-- ;

}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
 if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2B --;
else if(gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) == true) EncoderCounter2B --;
else if(gpio_read(pinum,motor2_ENA)==true && gpio_read(pinum,motor2_ENB) ==true) EncoderCounter2B++ ;
else if (gpio_read(pinum,motor2_ENA)==false && gpio_read(pinum,motor2_ENB) ==false) EncoderCounter2B++ ;
 


}
int main(int argc, char** argv){
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  Motor_Setup();
 set_PWM_dutycycle(pinum, motor1_PWM, 80);
  set_PWM_dutycycle(pinum, motor1_PWM, 80);
 while(ros::ok()){
     printf("EncoderCounter2A : %d || EncoderCounter2B : %d\n",EncoderCounter2A,EncoderCounter2B);
    }
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

}
