#include <SPI.h>

//Pins
const byte slaveAPin=10;
const byte slaveBPin=9;
const byte slaveCPin=8;
const byte slaveDPin=7;

//Parameters
unsigned short Vel[4];
byte mode=0;
unsigned short Data[4];
int flag;
bool flag2;
double WheelVelocity[4];
double Vx=0;
double Vy=0;
double Wz=0;



void setup() {
  Serial.begin(9600); //Begin serial data transfer
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
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.setClockDivider(SPI_CLOCK_DIV128);
}

void loop() {
  //Serial Data is input
  if (Serial.available() > 0) {
    char Inp=Serial.read();
    flag2=0;
    if(Inp=='v'){
      //Set Mode To Velocity Mode
      mode=0;
      Vx=0;
      Vy=0;
      Wz=0;
    }
    else if(Inp=='f'){
      //Set Mode To Force Mode
      mode=1;
      Vx=0;
      Vy=0;
      Wz=0;
    }
    else if(Inp=='w'){
      //Move Forward
      if(mode==0){
        Vx+=0.1;
      }
      else{
        Vx+=100;
      }
    }
    else if(Inp=='a'){
      //Move Left
      if(mode==0){
        Vy+=0.1;
      }
      else{
        Vy+=100;
      }
    }
    else if(Inp=='s'){
      //Move BackWards
      if(mode==0){
        Vx-=0.1;
      }
      else{
        Vx-=100;
      }
    }
    else if(Inp=='d'){
      //Move Right
      if(mode==0){
        Vy-=0.1;
      }
      else{
        Vy-=100;
      }
    }
    else if(Inp=='q'){
      //Rotate Anticlockwise
      if(mode==0){
        Wz-=0.1;
      }
      else{
        Wz-=100;
      }
    }
    else if(Inp=='e'){
      //Rotate Clockwise
      if(mode==0){
        Wz+=0.1;
      }
      else{
        Wz+=100;
      }
    }
    //Print out Vx, Vy and Wz
    Serial.print("Vx is ");Serial.print(Vx,6);Serial.print("  Vy is ");Serial.print(Vy,6);Serial.print("  Wz is ");Serial.print(Wz,6);Serial.println("\n");
  }
  
  //Velocity Control Mode
  if(mode==0){
    //Calculate Motor Velocities that are Necessary in Order to Create the Velocities in the Body frame
    WheelVelocity[0]=(Vx-Vy+Wz*0.424264068)/0.05;
    WheelVelocity[1]=-((Vx+Vy-Wz*0.424264068)/0.05);
    WheelVelocity[2]=(Vx+Vy+Wz*0.424264068)/0.05;
    WheelVelocity[3]=-((Vx-Vy-Wz*0.424264068)/0.05);

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

  if(flag2==0){
    //Write the wheel velocity data to the values sent to each wheel's slave microcontroller
    Data[0]=(Vel[0]&127|(Vel[0]&8064)<<1)|((mode<<14)|32768);
    Data[1]=(Vel[1]&127|(Vel[1]&8064)<<1)|((mode<<14)|32768);
    Data[2]=(Vel[2]&127|(Vel[2]&8064)<<1)|((mode<<14)|32768);
    Data[3]=(Vel[3]&127|(Vel[3]&8064)<<1)|((mode<<14)|32768);
    flag2=1;
  }
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

