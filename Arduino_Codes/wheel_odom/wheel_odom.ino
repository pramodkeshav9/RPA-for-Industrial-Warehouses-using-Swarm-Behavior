#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#define EN_BL 45
#define IN1_BL 8
#define IN2_BL 9
#define EN_BR 46
#define IN1_BR 10
#define IN2_BR 11
#define EN_FL 12
#define IN1_FL 5
#define IN2_FL 4
#define EN_FR 13
#define IN1_FR 7
#define IN2_FR 6
#define ENCODER_FR 3 
#define ENCODER_FL 2
#define BUFFER_TIME 1000
#define ENCODER_BRA 19       
#define ENCODER_BLA 18
#define ENCODER_BRB 20
#define ENCODER_BLB 21
#define PULSES_PER_REV 280
#define pi 3.14

volatile long rotary_encoder_right=0;
volatile long rotary_encoder_left=0;

float left_rev_count = 0;
float right_rev_count = 0;
float l_dist_val=0.0,r_dist_val=0.0;
volatile boolean fired_left
,up_left,fired_right,up_right;
unsigned long current_time = 0,previous_time = 0;
//x,y wheel odom positions

//Robot Parameters
float wheel_rad = 0.030;
float w_fr;
float w_fl;
float w_bl;
float w_br;
float wheel_sep = 0.175;

//const float dist_per_rev = (2 * pi * wheel_rad) * PULSES_PER_REV;
const float dist_per_rev = (2 * pi * wheel_rad);

// ---------------ALL ROS MSGS----------------- 
std_msgs::Float32 l_dist_msg, r_dist_msg;   //gives use left wheel and right wheel distances pointers 

//ROS Odom nodehandle and transform 
ros::NodeHandle n;

// ROS PUBLISHERS
ros::Publisher l_dist("l_dist", &l_dist_msg);
ros::Publisher r_dist("r_dist", &r_dist_msg);

float speed_ang=0, speed_lin=0;

void cmd_vel_callback( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_fr = (speed_lin/wheel_rad)+ ((speed_ang*wheel_sep)/(2.0*wheel_rad));  // speed_ang*2.91 -2.91
  w_fl = (speed_lin/wheel_rad)- ((speed_ang*wheel_sep)/(2.0*wheel_rad)); // 
  w_br = (speed_lin/wheel_rad)+ ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_bl = (speed_lin/wheel_rad)- ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_vel_callback );
// Motor init 
// ********************** FUNCTION DECLARATIONS *********************
//void update_right_encoder(void);
//void update_left_encoder(void);

void to_odom_publisher(void);
float update_left(void);
float update_right(void);

void isr_left(void);
void isr_right(void);

void MotorFL(int pwm);
void MotorBL(int pwm);
void MotorFR(int pwm);
void MotorBR(int pwm);

void pinmode_setup()
{
    pinMode(ENCODER_BRA, INPUT_PULLUP);
 pinMode(ENCODER_BLA, INPUT_PULLUP);
 pinMode(ENCODER_BRB, INPUT_PULLUP);
 pinMode(ENCODER_BLB, INPUT_PULLUP);
}

void pin_init()
{
 pinMode(EN_FL, OUTPUT);
 pinMode(EN_FR, OUTPUT);
 pinMode(EN_BL, OUTPUT);
 pinMode(EN_BR, OUTPUT);
 pinMode(IN1_FL, OUTPUT);
 pinMode(IN2_FL, OUTPUT);
 pinMode(IN1_FR, OUTPUT);
 pinMode(IN2_FR, OUTPUT);
 pinMode(IN1_BL, OUTPUT);
 pinMode(IN2_BL, OUTPUT);
 pinMode(IN1_BR, OUTPUT);
 pinMode(IN2_BR, OUTPUT);
 digitalWrite(EN_FL, LOW);
 digitalWrite(EN_FR, LOW);
 digitalWrite(EN_BL, LOW);
 digitalWrite(EN_BR, LOW);
 digitalWrite(IN1_FL, LOW);
 digitalWrite(IN2_FL, LOW);
 digitalWrite(IN1_FR, LOW);
 digitalWrite(IN2_FR, LOW);
 digitalWrite(IN1_BL, LOW);
 digitalWrite(IN2_BL, LOW);
 digitalWrite(IN1_BR, LOW);
 digitalWrite(IN2_BR, LOW);
}

void setup() {

    pinmode_setup();
    EncoderInit();

    pin_init();

    n.initNode();
    n.advertise(l_dist);
    n.advertise(r_dist);
    n.subscribe(sub_cmd_vel);

    current_time=millis();
    previous_time = current_time;
}

void loop() {
  
  current_time = millis();
  MotorFL(w_fl*50);
  MotorFR(w_fr*50);
  MotorBR(w_br*50);
  MotorBL(w_bl*50);

  if (current_time - previous_time >= BUFFER_TIME)
  {
    to_odom_publisher();
    previous_time = millis();
    rotary_encoder_left = 0;
    rotary_encoder_right = 0;
  }

  //rotary_encoder_left = 0;
  //rotary_encoder_right = 0;
 
  n.spinOnce();
}

void EncoderInit()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_BLA), isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BRA), isr_right, CHANGE);
  
}

//void update_right_encoder()
//{
//  rotary_encoder_right++;
//}
//
//
//void update_left_encoder()
//{
//  rotary_encoder_left++;
//}

float update_left()
{
  left_rev_count = (float)rotary_encoder_left/PULSES_PER_REV;
  l_dist_val = (float)left_rev_count * dist_per_rev;
  return l_dist_val/2.0;
}

float update_right()
{
  right_rev_count = (float)rotary_encoder_right/PULSES_PER_REV;
  r_dist_val = (float)right_rev_count * dist_per_rev;
  return r_dist_val/2.0;
}

void to_odom_publisher(void)
{
  l_dist_msg.data = update_left();
  r_dist_msg.data = update_right();

  l_dist.publish(&l_dist_msg);
  r_dist.publish(&r_dist_msg);
  return;
}

void MotorFL(int Pulse_Width1){
  if (Pulse_Width1 > 0){
   analogWrite(EN_FL, Pulse_Width1);
   digitalWrite(IN1_FL, HIGH);
   digitalWrite(IN2_FL, LOW);
  }
  if (Pulse_Width1 < 0){
    Pulse_Width1=abs(Pulse_Width1);
    analogWrite(EN_FL, Pulse_Width1);
    digitalWrite(IN1_FL, LOW);
    digitalWrite(IN2_FL, HIGH);
    }
  if (Pulse_Width1 == 0){
    analogWrite(EN_FL, Pulse_Width1);
    digitalWrite(IN1_FL, LOW);
    digitalWrite(IN2_FL, LOW);
  }
}
void MotorFR(int Pulse_Width2){
if (Pulse_Width2 > 0){
    analogWrite(EN_FR, Pulse_Width2);
    digitalWrite(IN1_FR, HIGH);
    digitalWrite(IN2_FR, LOW);
}
if (Pulse_Width2 < 0){
    Pulse_Width2=abs(Pulse_Width2);
    analogWrite(EN_FR, Pulse_Width2);
    digitalWrite(IN1_FR, LOW);
    digitalWrite(IN2_FR, HIGH);
}
if (Pulse_Width2 == 0){
    analogWrite(EN_FR, Pulse_Width2);
    digitalWrite(IN1_FR, LOW);
    digitalWrite(IN2_FR, LOW);
}
}
void MotorBL(int Pulse_Width3){
if (Pulse_Width3 > 0){
    analogWrite(EN_BL, Pulse_Width3);
    digitalWrite(IN1_BL, HIGH);
    digitalWrite(IN2_BL, LOW);
}
if (Pulse_Width3 < 0){
    Pulse_Width3=abs(Pulse_Width3);
    analogWrite(EN_BL, Pulse_Width3);
    digitalWrite(IN1_BL, LOW);
    digitalWrite(IN2_BL, HIGH);
}
if (Pulse_Width3 == 0){
    analogWrite(EN_BL, Pulse_Width3);
    digitalWrite(IN1_BL, LOW);
    digitalWrite(IN2_BL, LOW);
}
}
void MotorBR(int Pulse_Width4){
    if (Pulse_Width4 > 0){
    analogWrite(EN_BR, Pulse_Width4);
    digitalWrite(IN1_BR, HIGH);
    digitalWrite(IN2_BR, LOW);
    }
    if (Pulse_Width4 < 0){
    Pulse_Width4=abs(Pulse_Width4);
    analogWrite(EN_BR, Pulse_Width4);
    digitalWrite(IN1_BR, LOW);
    digitalWrite(IN2_BR, HIGH);
    }
    if (Pulse_Width4 == 0){
    analogWrite(EN_BR, Pulse_Width4);
    digitalWrite(IN1_BR, LOW);
    digitalWrite(IN2_BR, LOW);
    }
}


void isr_left(void)
{
  if (digitalRead(ENCODER_BLA))
  {
    up_left = digitalRead(ENCODER_BLB);
  }
  else
  {
    up_left = !digitalRead(ENCODER_BLB);
  }
  fired_left= true;
  
  if (fired_left)
  {
    if (up_left)
    {
      rotary_encoder_left--;
    }
    else
    {
      rotary_encoder_left++;
    }
  }
}

void isr_right(void)
{
  if (digitalRead(ENCODER_BRA))
  {
    up_right = digitalRead(ENCODER_BRB);
  }
  else
  {
    up_right = !digitalRead(ENCODER_BRB);
  }
  fired_right= true;

  if (fired_right)
  {
    if (up_right)
    {
      rotary_encoder_right++;
    }
    else
    {
      rotary_encoder_right--;
    }
  }
}
