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
float numpoles = 14;   //14 poles for A2212/13T
double timein;
double timeout;
double timer = 0;
int critical_rpm;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 6;

Servo Motor1;
File dataFile;
HX711 scale;

void setup() 
{
  Serial.begin(9600);
  Motor1.attach(3);
  pinMode(9, INPUT);
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  while (!Serial){;}
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) 
  {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  dataFile = SD.open("data.txt", FILE_WRITE);

  // RPM COUNTER INIT
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(INTERRUPT_PIN, interruptFired, CHANGE);

  Motor1.writeMicroseconds(1000);
  delay(4000);
}

void loop()
{
  Motor1.writeMicroseconds(1000);
  //Serial.println(digitalRead(9));
  if(digitalRead(9) == 1)
  {
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
      if(timer > 100)
      {
        dataFile.close();
        timer = 0;
        Motor1.writeMicroseconds(1000);
        break;
      }
      timeout = millis() - timein;
      timer++;
    }
  }
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
  
  if (dataFile)
  {
     dataFile.print((timer*timeout)/1000);
     dataFile.print(",");
     dataFile.print(200);
     dataFile.print(",");
     dataFile.println(rpm);
  } 
  else
  {
     // if the file didn't open, print an error:
     Serial.println("error opening FrpmTdata.txt");
  }
}
 
void interruptFired()
{
    interruptCount++;
}
