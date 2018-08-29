#include <SPI.h>

//Pins
const byte slave_select = 10;
const byte data_pin = 11;
const byte clock_pin = 13;

void setup() 
{
  Serial.begin(57600);
  
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
  if (Serial.available())
     { uint8_t c = Serial.read();
       digitalWrite(slave_select, LOW);
       if (c == '1')
          {SPI.transfer(0x01);
           Serial.println("Turn on LED");}
       else if (c == '0')
          {SPI.transfer(0x00);
           Serial.println("Turn off LED");}
       else 
          Serial.println("Wrong entry. Please press 1 or 0");
       digitalWrite(slave_select, HIGH);
     }
}

