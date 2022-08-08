#include <PID_v1.h>
#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>


ros::NodeHandle nh;

/*
 * Encoder 400 ppr
 * Trang    Kenh A
 * Xanh la  Kenh B
 * Do       VCC (5-24V)
 * Den      GND (0V)
 */

/*
 * Dong co 24V 60W 
 * toc do quay toi da 110 rpm
 */

/*
 * Duong kinh banh xe: d = 145 mm = 0.145 m
 * Ban kinh banh xe: r = 72.5 mm = 0.0725 m
 * => 131 rpm = 1 m/s
 * Robot di thang voi toc do  50 rpm = 0.37960911229791666 m/s ~ 0.3796 m/s ~ 0.38 m/s ~ 0.4 m/s
 * Robot di lui voi toc do    25 rpm = 0.18980455615438332 m/s ~ 0.1898 m/s ~ 0.19 m/s ~ 0.2 m/s
 * 
 * Khoang cach 2 banh xe: 550 mm = 0.55 m
 * Chu vi hinh tron la 0.55*pi = 1.72787595947 m, de xoay 180 do (1 pi radian) moi banh xe can di chuyen 0.86393797973 m
 * De xoay 1 radian, moi banh xe can di chuyen 0.86393797973/pi = 0.275 m/s (=> 1 rad/s = 0.275 m/s)
 */

//Dong co phai Right Motor (M1)
#define R_EN_Right  4
#define L_EN_Right  5
#define R_PWM_Right 6 
#define L_PWM_Right 7

//Dong co trai Left Motor (M2)
#define R_EN_Left   8
#define L_EN_Left   9
#define R_PWM_Left 10  
#define L_PWM_Left 11

//Encoder phai
#define ENA_Right   2
#define ENB_Right   3
volatile float encoderRightPos = 0;
float encoderRightLast = 0;
float encoderRightError = 0;

//Encoder trai
#define ENA_Left   19
#define ENB_Left   18
volatile float encoderLeftPos = 0;
float encoderLeftLast = 0;
float encoderLeftError = 0;

//PID controller motor Right
double Kp_Right = 1;//1.515;
double Ki_Right = 0.000000000000000000000000000000000000000000001;
double Kd_Right = 0.000000000000000000000000000000000000000000001;//0.009;
double input_Right = 0, setpoint_Right = 0, output_Right = 0;
PID PID_Right(&input_Right, &output_Right, &setpoint_Right, Kp_Right, Ki_Right, Kd_Right, DIRECT);

//PID controller motor Left 
double Kp_Left = 1;//1.526;
double Ki_Left = 0.000000000000000000000000000000000000000000001;
double Kd_Left = 0.000000000000000000000000000000000000000000001;//0.009;
double input_Left = 0, setpoint_Left = 0, output_Left = 0;
PID PID_Left(&input_Left, &output_Left, &setpoint_Left, Kp_Left, Ki_Left, Kd_Left, DIRECT);

//Thoi gian lay mau
double LOOPTIME = 100; //don vi ms
double TsPID = 100; //don vi ms
double Ts = 0.01; //thoi gian lay mau 10ms (0.01s)
unsigned long currentMillis = 0;
unsigned long prevMillis = 0;

//Dieu khien dong co
int pwmOutRight = 0;
int pwmOutLeft = 0;

double vel_linear_x = 0; //m/s (limit from 0 m/s to 0.4 m/s)
double vel_angular_z = 0; //rad/s (1 rad/s = 0.275 m/s)
double vel_Right = 0; // vel_wheel_1 = vel_linear_x - (vel_angular_z*0.275)
double vel_Left = 0; // vel_wheel_2 = vel_linear_x + (vel_angular_z*0.275)
double rpm_Right = 0;
double rpm_Left = 0;

void velCallback( const geometry_msgs::Twist& vel)
{
  vel_linear_x = vel.linear.x;
  vel_angular_z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);
std_msgs::Int16 right_wheel_msg;
ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);
std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);

geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);

void setup() {
  //Encoder pinMode
  pinMode(ENA_Right, INPUT_PULLUP);
  pinMode(ENB_Right, INPUT_PULLUP);
  pinMode(ENA_Left, INPUT_PULLUP);
  pinMode(ENB_Left, INPUT_PULLUP);

  //Motor pinMode
  pinMode(R_EN_Right, OUTPUT);
  pinMode(L_EN_Right, OUTPUT);
  pinMode(R_PWM_Right, OUTPUT);
  pinMode(L_PWM_Right, OUTPUT);
  pinMode(R_EN_Left, OUTPUT);
  pinMode(L_EN_Left, OUTPUT);
  pinMode(R_PWM_Left, OUTPUT);
  pinMode(L_PWM_Left, OUTPUT);

  //Khoi tao ngat
  /*
   * INT0 INT1 INT2 INT3 INT4 INT5
   *  21   20   19   18    2    3
   *-------------hoặc-------------
   *   2    3   21   20   19   18
   */
  attachInterrupt(0, read_encoder_Right, FALLING);
  attachInterrupt(4, read_encoder_Left, FALLING);
  
  //Setup PID 
  PID_Right.SetMode(AUTOMATIC);
  PID_Right.SetSampleTime(TsPID); //10ms 
  PID_Right.SetOutputLimits(-255, 255);
  PID_Left.SetMode(AUTOMATIC);
  PID_Left.SetSampleTime(TsPID); //10ms 
  PID_Left.SetOutputLimits(-255, 255);

  nh.initNode();
  nh.getHardware()->setBaud(57600); 
  nh.subscribe(sub);
  nh.advertise(right_wheel_pub);
  nh.advertise(left_wheel_pub);
  
  nh.advertise(speed_pub); 
}

void loop() {  
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME) //don vi ms
  {
    prevMillis = currentMillis;
    
    //Encoder 400 ppr
    //Tinh van toc M1 va M2 (toc do toi da 110 rpm)
    
    //rpm_M1 = (double(encoder1Pos)/400.0)*100.0*60.0;
    encoderRightError = encoderRightPos - encoderRightLast;
    rpm_Right = ((encoderRightError/400.0)*60.0)/(LOOPTIME*0.001);
    encoderRightLast = encoderRightPos;

    //rpm_M2 = (double(encoder2Pos)/400.0)*100.0*60.0;
    encoderLeftError = encoderLeftPos - encoderLeftLast;
    rpm_Left = ((encoderLeftError/400.0)*60.0)/(LOOPTIME*0.001);
    encoderLeftLast = encoderLeftPos;
     
    vel_Right = vel_linear_x + (vel_angular_z*0.25);
    vel_Left = vel_linear_x - (vel_angular_z*0.25);
    vel_Right = constrain(vel_Right, -0.4, 0.4);
    vel_Left = constrain(vel_Left, -0.4, 0.4);
    
    input_Right = rpm_Right;
    setpoint_Right = vel_Right*131; //convert to rpm
    PID_Right.Compute();
    pwmOutRight = int(output_Right);
        
    input_Left = rpm_Left;
    setpoint_Left = vel_Left*131; //convert to rpm
    PID_Left.Compute();
    pwmOutLeft = int(output_Left);
    
    //Dieu khien dong co
    if (setpoint_Right == 0 && setpoint_Left == 0)
    {
      driveMotor(0, 0);
    }
    else
    {
      driveMotor(pwmOutRight, pwmOutLeft);
    }
    publishPos(LOOPTIME);
    
    publishSpeed(LOOPTIME);
  }
  nh.spinOnce();
}

void driveMotor(int pwmOutRight, int pwmOutLeft)
{
  //Dieu khien M1
  if (pwmOutRight > 0)
  {
    digitalWrite(R_EN_Right, HIGH);
    digitalWrite(L_EN_Right, HIGH);
    analogWrite(R_PWM_Right, 0);
    analogWrite(L_PWM_Right, pwmOutRight);
  }
  else if (pwmOutRight < 0)
  {
    digitalWrite(R_EN_Right, HIGH);
    digitalWrite(L_EN_Right, HIGH);
    analogWrite(R_PWM_Right, abs(pwmOutRight));
    analogWrite(L_PWM_Right, 0);
  }
  else if (pwmOutRight == 0)
  {
    digitalWrite(R_EN_Right, LOW);
    digitalWrite(L_EN_Right, LOW);
    analogWrite(R_PWM_Right, 0);
    analogWrite(L_PWM_Right, 0);
  } 

  //Dieu khien M2
  if (pwmOutLeft > 0)
  {
    digitalWrite(R_EN_Left, HIGH);
    digitalWrite(L_EN_Left, HIGH);
    analogWrite(R_PWM_Left, pwmOutLeft);
    analogWrite(L_PWM_Left, 0);
  }
  else if (pwmOutLeft < 0)
  {
    digitalWrite(R_EN_Left, HIGH);
    digitalWrite(L_EN_Left, HIGH);
    analogWrite(R_PWM_Left, 0);
    analogWrite(L_PWM_Left, abs(pwmOutLeft));  
  }
  else if (pwmOutLeft == 0)
  {
    digitalWrite(R_EN_Left, LOW);
    digitalWrite(L_EN_Left, LOW);
    analogWrite(R_PWM_Left, 0);
    analogWrite(L_PWM_Left, 0);  
  }
}

void read_encoder_Right()
{
  if (digitalRead(ENB_Right) == HIGH) encoderRightPos--; //encoder1Pos++; //do encoder lap nguoc 
  else                                encoderRightPos++; //encoder1Pos--;
}

void read_encoder_Left()
{
  if (digitalRead(ENB_Left) == HIGH)  encoderLeftPos++; 
  else                                encoderLeftPos--;
}

void publishPos(double LOOPTIME)
{
  right_wheel_msg.data = encoderRightPos;
  left_wheel_msg.data = encoderLeftPos;
  right_wheel_pub.publish(&right_wheel_msg);
  left_wheel_pub.publish(&left_wheel_msg);
}






void publishSpeed(double LOOPTIME) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = rpm_Left;    //left wheel speed (in m/s)
  speed_msg.vector.y = rpm_Right;   //right wheel speed (in m/s)
  speed_msg.vector.z = LOOPTIME/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
//  nh.spinOnce();
//  nh.loginfo("Publishing odometry");
}
