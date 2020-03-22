/*
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
 */
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

#include "HX711.h"

#define INTERRUPT_PIN 0   // Arduino MEGA/UNO/NANO digital pin 2

volatile int interruptCount;

float rpm = 0;
double force;
double Torque;

float numpoles = 14;   //14 poles for A2212/13T
double timein;
double timeout;
double timer = 0;
int critical_rpm;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 7;
const int LOADCELL_SCK_PIN = 3;

Servo Motor1,Motor2;
File ForcedataFile;
File RPMdataFile;
HX711 scale;

void setup() 
{
  Serial.begin(9600);
  
  Motor1.attach(5);
  Motor2.attach(6);
  Motor1.writeMicroseconds(1000);
  Motor2.writeMicroseconds(1000);
  
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  setScales();
  //scale.set_scale(801.9);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  //scale.tare();                // reset the scale to 0
  
  while (!Serial){;}
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) 
  {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // RPM COUNTER INIT
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(INTERRUPT_PIN, interruptFired, CHANGE);

  Motor1.writeMicroseconds(1000);
  Motor2.writeMicroseconds(1000);
  delay(4000);
}

int state = 0;
void loop()
{
  if (state == 0)
  {
    Serial.println("Idle State");
    if(digitalRead(8) == 1)
    {
      state = 1;
      delay(1000);
    }
    //if(digitalRead(8) == 1)
    //{
      //state = 2;
      //delay(1000);
    //}
    if(digitalRead(9) == 1)
    {
      state = 3;
      delay(1000);
    }
  }

  if(state == 1)
  {
    Serial.println("RPM State");
    
    Motor1.writeMicroseconds(1000);
    Motor2.writeMicroseconds(1000);

    RPMdataFile = SD.open("RPMdata.txt", FILE_WRITE);
    
    delay(5000);
    Motor1.writeMicroseconds(1300);
    delay(5000);
    
    while(1)
    {
      timein = millis();
      Motor1.writeMicroseconds(1500);
      
      checkRPM();
      
      Serial.print("Time: ");
      Serial.print((timer*timeout)/1000);
      Serial.print("\t");
      Serial.print("sampletime in ms: ");
      Serial.println(timeout);
      
      if(((timer*timeout)/1000) > 10)
      {
        state = 0;
        RPMdataFile.close();
        timer = 0;
        Motor1.writeMicroseconds(1000);
        delay(1000);
        break;
      }
      
      if(digitalRead(8) == 1 || digitalRead(9) == 1)
      {
        Motor1.writeMicroseconds(1000);
        state = 0;
        delay(1000);
        break;
      }
      
      timeout = millis() - timein;
      timer++;
    }
  }

  if(state == 2)
  {
    Serial.println("Force State");
    RPMdataFile = SD.open("RPMdata.txt", FILE_WRITE);
    ForcedataFile = SD.open("forcedata.txt", FILE_WRITE);
    
    delay(5000);
    Motor1.writeMicroseconds(1300);
    delay(5000);
    
    while(1)
    {
      timein = millis();
      Motor1.writeMicroseconds(1300+((timer*timeout)/100));
      checkForce();
      checkRPM();
      Serial.print("Time: ");
      Serial.print((timer*timeout)/1000);
      Serial.print("\t");
      Serial.print("sampletime in ms: ");
      Serial.println(timeout);
      
      if(((timer*timeout)/1000) > 70)
      {
        state = 0;
        RPMdataFile.close();
        timer = 0;
        Motor1.writeMicroseconds(1000);
        delay(1000);
        break;
      }
      
      if(digitalRead(8) == 1 || digitalRead(9) == 1)
      {
        Motor1.writeMicroseconds(1000);
        state = 0;
        delay(1000);
        break;
      }
      
      timeout = millis() - timein;
      timer++;
    }
  }

  if(state == 3)
  {
    Serial.println("Torque State");
    RPMdataFile = SD.open("RPMdata.txt", FILE_WRITE);
    ForcedataFile = SD.open("forcedata.txt", FILE_WRITE);
    
    delay(5000);
    Motor1.writeMicroseconds(1200);
    delay(5000);
    
    while(1)
    {
      timein = millis();
      Motor1.writeMicroseconds(1200+((timer*timeout)/100));
      checkTorque();
      checkRPM();
      Serial.print("Time: ");
      Serial.print((timer*timeout)/1000);
      Serial.print("\t");
      Serial.print("sampletime in ms: ");
      Serial.println(timeout);
      
      if(((timer*timeout)/1000) > 50)
      {
        state = 0;
        RPMdataFile.close();
        timer = 0;
        Motor1.writeMicroseconds(1000);
        delay(1000);
        break;
      }
      
      if(digitalRead(8) == 1 || digitalRead(9) == 1)
      {
        Motor1.writeMicroseconds(1000);
        state = 0;
        delay(1000);
        break;
      }
      
      timeout = millis() - timein;
      timer++;
    }
  }
}

void interruptFired()
{
    interruptCount++;
}

// Check RPM Function
void checkRPM() 
{
  noInterrupts() ;
  interruptCount = 0;  // set variable in critical section
  interrupts() ;
  delay(100);
  noInterrupts() ;
  critical_rpm = interruptCount ;  // read variable in critical section 
  interrupts() ;
  rpm = ((critical_rpm)*(60))/(numpoles)*10;
  Serial.print("Motor RPM: ");
  Serial.print(rpm);
  Serial.print("\t");
}

// Check Force Function
void checkForce() 
{
  force = (scale.get_units(1)*9.81)/1000;
  Serial.print("Motor Force: ");
  Serial.print(force);
  Serial.print("\t");
  Serial.print((timer*timeout)/100);
  Serial.print("\t");
  
  if (RPMdataFile)
  {
     RPMdataFile.print((timer*timeout)/1000);
     RPMdataFile.print(",");
     RPMdataFile.print(rpm);
     RPMdataFile.print(",");
     RPMdataFile.println(force);
  } 
  else
  {
     // if the file didn't open, print an error:
     Serial.println("error opening forcedata.txt");
  }
}

void checkTorque() 
{
  Torque = (scale.get_units(1)*9.81)*0.1945;
  Serial.print("Motor Torque: ");
  Serial.print(Torque);
  Serial.print("\t");
  Serial.print((timer*timeout)/100);
  Serial.print("\t");
  
  if (RPMdataFile)
  {
     RPMdataFile.print((timer*timeout)/1000);
     RPMdataFile.print(",");
     RPMdataFile.print(rpm);
     RPMdataFile.print(",");
     RPMdataFile.println(Torque);
  } 
  else
  {
     // if the file didn't open, print an error:
     Serial.println("error opening forcedata.txt");
  }
}

void setScales()
{
  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)

  scale.set_scale(777.1f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale
}
