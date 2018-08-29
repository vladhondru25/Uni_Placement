#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

void callback( const geometry_msgs::Twist& twist_msg)
{
  if (twist_msg.linear.x > 0.5)
     digitalWrite(13, HIGH);
  else 
     digitalWrite(13, LOW);

  if (twist_msg.linear.x < -0.5)
     digitalWrite(12, HIGH);
  else 
     digitalWrite(12, LOW);
     
  if (twist_msg.linear.y > 0.5)
     digitalWrite(11, HIGH);
  else 
     digitalWrite(11, LOW);

  if (twist_msg.linear.y < -0.5)
     digitalWrite(10, HIGH);
  else 
     digitalWrite(10, LOW);
     
  if (twist_msg.angular.z > 0.5)
     digitalWrite(9, HIGH);
  else 
     digitalWrite(9, LOW);

  if (twist_msg.angular.z < -0.5)
     digitalWrite(8, HIGH);
  else 
     digitalWrite(8, LOW);
}

ros::Subscriber<geometry_msgs::Twist> subb("/mallard/cmd_vel", &callback );

void setup()
{ 
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  nh.initNode();
  nh.subscribe(subb);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
