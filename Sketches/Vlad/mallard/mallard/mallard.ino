#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <SPI.h>

ros::NodeHandle  nh;

//Pins
const byte slaveAPin=10;
const byte slaveBPin=9;
const byte slaveCPin=8;
const byte slaveDPin=7;

//Parameters
unsigned short Vel[4];
byte mode=0;
unsigned short Data[4];
double WheelVelocity[4];
double Vx=0;
double Vy=0;
double Wz=0;

void callback( const geometry_msgs::Twist& twist_msg)
{

  Vx = twist_msg.linear.x / 4;
  Vy = twist_msg.linear.y / 4;
  Wz = twist_msg.angular.z / 4;
  
  /*if (twist_msg.linear.x > 0.5)
     digitalWrite(13, HIGH);
  else 
     digitalWrite(13, LOW);

  if (twist_msg.linear.x < -0.5)
     digitalWrite(14, HIGH);
  else 
     digitalWrite(14, LOW);
     
  if (twist_msg.linear.y > 0.5)
     digitalWrite(15, HIGH);
  else 
     digitalWrite(15, LOW);

  if (twist_msg.linear.y < -0.5)
     digitalWrite(16, HIGH);
  else 
     digitalWrite(16, LOW);
     
  if (twist_msg.angular.z > 0.5)
     digitalWrite(17, HIGH);
  else 
     digitalWrite(17, LOW);

  if (twist_msg.angular.z < -0.5)
     digitalWrite(18, HIGH);
  else 
     digitalWrite(18, LOW);  */
}

ros::Subscriber<geometry_msgs::Twist> subb("/mallard/cmd_vel", &callback );

void setup() {
  //Serial.begin(57600); //Begin serial data transfer
  //Init pins
  pinMode (slaveAPin, OUTPUT);
  pinMode (slaveBPin, OUTPUT);
  pinMode (slaveCPin, OUTPUT);
  pinMode (slaveDPin, OUTPUT);
  //Set all slave pins low
  digitalWrite(slaveAPin, HIGH);
  digitalWrite(slaveBPin, HIGH);
  digitalWrite(slaveCPin, HIGH);
  digitalWrite(slaveDPin, HIGH);
  //SPI setup
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();

  nh.initNode();
  nh.subscribe(subb);
}

void loop() {
  //Enters a loop, calling message callbacks as fast as possible
  nh.spinOnce();
  
  //Serial Data is input
  
  
  //Velocity Control Mode
  if(mode==0){
    //Calculate Motor Velocities that are Necessary in Order to Create the Velocities in the Body frame
    WheelVelocity[0]=(Vx-Vy+Wz*0.3)/0.05;
    WheelVelocity[1]=-((Vx+Vy-Wz*0.3)/0.05);
    WheelVelocity[2]=(Vx+Vy+Wz*0.3)/0.05;
    WheelVelocity[3]=-((Vx-Vy-Wz*0.3)/0.05);

    //Increase the Wheel Velocities to make them positive and 
    Vel[0]=int((WheelVelocity[0]+15)*100);
    Vel[1]=int((WheelVelocity[1]+15)*100);
    Vel[2]=int((WheelVelocity[2]+15)*100);
    Vel[3]=int((WheelVelocity[3]+15)*100);
  }
  //Force Control Mode
  else{
    //Calculate Motor Forces that are Necessary in Order to Create the Forces in the Body Frame
    WheelVelocity[0]=Vx*0.353553391-Vy*0.353553391+Wz*1.178511302;
    WheelVelocity[1]=-(Vx*0.353553391+Vy*0.353553391-Wz*1.178511302);
    WheelVelocity[2]=Vx*0.353553391+Vy*0.353553391+Wz*1.178511302;
    WheelVelocity[3]=-(Vx*0.353553391-Vy*0.353553391-Wz*1.178511302);

    //Increase the Wheel Forces to Make them Positive so they
    Vel[0]=int(WheelVelocity[0]+255);
    Vel[1]=int(WheelVelocity[1]+255);
    Vel[2]=int(WheelVelocity[2]+255);
    Vel[3]=int(WheelVelocity[3]+255);
  }
  //Write the wheel velocity data to the values sent to each wheel's slave microcontroller
  Data[0]=(Vel[0]&127|(Vel[0]&8064)<<1)|((mode<<14)|32768);
  Data[1]=(Vel[1]&127|(Vel[1]&8064)<<1)|((mode<<14)|32768);
  Data[2]=(Vel[2]&127|(Vel[2]&8064)<<1)|((mode<<14)|32768);
  Data[3]=(Vel[3]&127|(Vel[3]&8064)<<1)|((mode<<14)|32768);

  
  
  //Send SPI data
  //Send Data to A
  
  digitalWrite(slaveAPin, LOW);
  SPI.transfer16(Data[0]);
  digitalWrite(slaveAPin,HIGH);
  //Send Data to B
  digitalWrite (slaveBPin, LOW);
  SPI.transfer16(Data[1]);
  digitalWrite(slaveBPin,HIGH);
  //Send Data to C
  digitalWrite (slaveCPin, LOW);
  SPI.transfer16(Data[2]);
  digitalWrite(slaveCPin,HIGH);
  //Send Data to D
  digitalWrite (slaveDPin, LOW);
  SPI.transfer16(Data[3]);
  digitalWrite(slaveDPin,HIGH);
  
}
