//Library files
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <EEPROM.h>
#define _NAMIKI_MOTOR   //for Namiki 22CL-103501PG80:1
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>


//Initialisation
irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);
irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);
irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);
irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);
Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);

//Defining constants for pins:
const int executepin = 8;
const int forwardpin = 9;
const int backwardpin = 10;
const int leftpin = 11;
const int rightpin = 12;
const int buttonledpin = 13;
const int actionledpin = 3;

//Variables set to 0 to begin with.
int forwardstate = 0;
int backwardstate = 0;
int leftstate = 0;
int rightstate = 0;
int executestate = 0;
int actions[100];
int counter = 0;
int action = 0;
int traveltime = 500;

void setup() {
  //Unknown meaning, but included in demo code!
  //TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
  Omni.PIDEnable(0.31,0.01,0,10);

  //Array for storing values which correspond to actions
  int actions[100]; // Up to 100 actions (could be increased, but 100 should be enough!)
  int counter = 0;
  Serial.begin(9600); //Can use the serial monitor to see states of pins.

  //Initialising pins:
  pinMode(executepin, INPUT);
  pinMode(forwardpin, INPUT);
  pinMode(backwardpin, INPUT);
  pinMode(leftpin, INPUT);
  pinMode(rightpin, INPUT);
  pinMode(buttonledpin, OUTPUT);
  pinMode(actionledpin, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //Every cycle, check input:
  forwardstate = digitalRead(forwardpin);
  backwardstate = digitalRead(backwardpin);
  leftstate = digitalRead(leftpin);
  rightstate = digitalRead(rightpin);
  executestate = digitalRead(executepin);

  //Allows the button states to be seen on a computer
  Serial.print("Forward:"); Serial.println(forwardstate);
  Serial.print("Backward:"); Serial.println(backwardstate);
  Serial.print("Left:"); Serial.println(leftstate);
  Serial.print("Right:"); Serial.println(rightstate);
  Serial.print("Execute:"); Serial.println(executestate);
  
  //This section adds the appropriate commands to the array of integers, actions, which will be executed when the buttons are pressed:
  if (forwardstate == 1) {
    actions[counter] = 1;
    counter += 1;
    digitalWrite(buttonledpin, HIGH);
    delay(200);
    digitalWrite(buttonledpin, LOW);
  }
  if (backwardstate == 1) {
    actions[counter] = 2;
    counter += 1;
    digitalWrite(buttonledpin, HIGH);
    delay(200);
    digitalWrite(buttonledpin, LOW);
  }
  if (leftstate == 1) {
    actions[counter] = 3;
    counter += 1;
    digitalWrite(buttonledpin, HIGH);
    delay(200);
    digitalWrite(buttonledpin, LOW); 
  }
  if (rightstate == 1) {
    actions[counter] = 4;
    counter += 1;
    digitalWrite(buttonledpin, HIGH);
    delay(200);
    digitalWrite(buttonledpin, LOW); 
  }

  //For debugging, allows the actions array to be seen in serial.
  for (int i = 0; i < counter; i++)
{
    Serial.println(actions[i]);
}
  //When the execute button is pressed, the robot moves according to the traveltime in the dircections allocated by each button press in the array.
  if (executestate == 1) {
    for (int i = 0; i < counter; i++)
{
      action = actions[i];
      if (action == 1) {
         digitalWrite(actionledpin, HIGH);
         Omni.setCarAdvance(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         digitalWrite(actionledpin, LOW); 
      }
      if (action == 2) {
         digitalWrite(actionledpin, HIGH);
         Omni.setCarBackoff(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         digitalWrite(actionledpin, LOW); 
      }   
      if (action == 3) {
         digitalWrite(actionledpin, HIGH);
         Omni.setCarLeft(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         digitalWrite(actionledpin, LOW); 
      }
      if (action == 4) {
         digitalWrite(actionledpin, HIGH);
         Omni.setCarRight(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         digitalWrite(actionledpin, LOW); 
      }     
      delay(500);          
}
  }
  //Every second the program checks which buttons are being pressed.
  delay(1000);

  

}
