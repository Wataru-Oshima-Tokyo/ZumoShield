#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Wire.h>
#include <ZumoShield.h>


long timer=0;              // Elapsed time since program started (milli second)
int16_t vright = 0;            // Left Morter velocity (speed of motor)
int16_t vleft = 0;  // Right Morter velocity (speed of motor)
int16_t previousNum = 0;
int16_t basespeed = 200;       // Base speed of Morter (Effective Range: 1 - 150)
int16_t positionLeft  = 0; // For encoder verification
int16_t positionRight = 0; // For encoder verification
int16_t newLeft = 0;       // Value of Encorder
int16_t newRight = 0;      // Value of Encorder
float ForwardSpeed = 0.0f;
float AngularSpeed = 0.0f;
std_msgs::String str_msg;  // Sensor value to be published
geometry_msgs::Twist cmd_vel; //cmd_vel value


LSM303 compass;            // Magnetometer
L3G gyro;                  // Gyrometer
ZumoMotors motors;     // Morter
ZumoShieldEncoders encoders; // Encoder
ros::NodeHandle nh;        // NodeHandler of ROS

void control_Callback(const geometry_msgs::Twist& cmd_vel_sub)
{
   
   ForwardSpeed = cmd_vel_sub.linear.x*basespeed;
   AngularSpeed = cmd_vel_sub.angular.z*basespeed;

    if (AngularSpeed == 0) 
      {
        
         motors.setSpeeds(ForwardSpeed, ForwardSpeed*0.7);
         delay(500);
      } else if (AngularSpeed < 0){
        //turn right
         AngularSpeed *=-1;
         motors.setSpeeds(AngularSpeed, 0);
         delay(500);
      } else{
         //turn left
        motors.setSpeeds(0, AngularSpeed); //to adjust the speed
        delay(500);
      }
     motors.setSpeeds(0, 0);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", control_Callback);
ros::Publisher chatter("/sensorval", &str_msg);

void setup()
{

  cli();
  /*
  * DEBUGにシリアルモニタを使用
  */
  Wire.begin();

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  nh.initNode();           // Init ROS Node
  nh.advertise(chatter);// Init ROS Publisher
  nh.subscribe(sub);       // Init ROS Subscriber
  compass.init();          // Init magnetometer
  compass.enableDefault();

  gyro.init();             // Init gyrometer
  gyro.enableDefault();
  Serial.begin(9600);
  sei();
}

void loop()
{
  compass.read();   // Read magnetometer
  gyro.read();      // Read gyrometer
  timer = millis();
  newLeft = encoders.getCountsAndResetLeft();
  newRight = encoders.getCountsAndResetRight();
  if (!(encoders.checkErrorLeft()) && !(encoders.checkErrorRight())) {
     positionLeft = newLeft;
     positionRight = newRight;    
   }
  String s = "";

  s += timer;          // [0]  Elapsed time since program started (milli second)
  s += ',';
  s += compass.a.x;    // [1]  Accelerometer.x
  s += ',';
  s += compass.a.y;    // [2]  Accelerometer.y
  s += ',';
  s += compass.a.z;    // [3]  Accelerometer.z
  s += ',';
  s += compass.m.x;    // [4]  Magnetometer.x
  s += ',';
  s += compass.m.y;    // [5]  Magnetometer.y
  s += ',';
  s += compass.m.z;    // [6]  Magnetometer.z
  s += ',';
  s += vleft;          // [7]  Left Morter velocity (speed of motor)
  s += ',';
  s += vright;         // [8]  Right Morter velocity (speed of motor)
  s += ',';
  s += positionLeft; //r_cnt;   // [9]  Left Morter odometry (Rotation angle of motor)
  s += ',';
  s += positionRight  ;//;  // [10] Right Morter odometry (Rotation angle of motor)
  s += ',';   
  s += gyro.g.x;       // [11] Gyrometer.x
  s += ',';
  s += gyro.g.y;       // [12] Gyrometer.y
  s += ',';
  s += gyro.g.z;       // [13] Gyrometer.z

  Serial.println(s);  // Debug Print
  
  str_msg.data = s.c_str();
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(10);
}
