#include <SPI.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

//Pins
const byte slave_select = 10;
const byte data_pin = 11;
const byte clock_pin = 13;

void callback( const geometry_msgs::Twist& twist_msg)
{
  /*if (twist_msg.linear.x > 0.5)
     digitalWrite(8, HIGH);
  else 
     digitalWrite(8, LOW);

  if (twist_msg.linear.x < -0.5)
     digitalWrite(7, HIGH);
  else 
     digitalWrite(7, LOW);
     
  if (twist_msg.linear.y > 0.5)
     digitalWrite(6, HIGH);
  else 
     digitalWrite(6, LOW);

  if (twist_msg.linear.y < -0.5)
     digitalWrite(5, HIGH);
  else 
     digitalWrite(5, LOW);
     
  if (twist_msg.angular.z > 0.5)
     digitalWrite(4, HIGH);
  else 
     digitalWrite(4, LOW);

  if (twist_msg.angular.z < -0.5)
     digitalWrite(3, HIGH);
  else 
     digitalWrite(3, LOW); */

     digitalWrite(slave_select, LOW);
     
     SPI.transfer(twist_msg.linear.x + 1);
     //Serial.println("Sent Vx");
     //SPI.transfer(twist_msg.linear.y);
     //Serial.println("Sent Vy");
     //SPI.transfer(twist_msg.angular.z);
     //Serial.println("Sent Wz");
     
     digitalWrite(slave_select, HIGH);
}

ros::Subscriber<geometry_msgs::Twist> subb("/mallard/cmd_vel", &callback );

void setup() 
{
  /* pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW); */
  nh.initNode();
  nh.subscribe(subb);
  
  //Serial.begin(57600);
  
  pinMode (slave_select, OUTPUT);
  digitalWrite(slave_select, HIGH);  //Set slave to idle

  pinMode(data_pin, OUTPUT);
  
  pinMode(clock_pin, OUTPUT);
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  //SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
}

void loop() 
{   
  //Serial.print("caca");
  nh.spinOnce();
  delay(1);
}

