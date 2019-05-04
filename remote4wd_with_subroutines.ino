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
const int executepin = 6;
const int forwardpin = 9;
const int backwardpin = 10;
const int leftpin = 11;
const int rightpin = 12;
const int buttonledpin = 13;
const int actionledpin = 13;

//Global variables set to 0 to begin with.
int forwardstate = 0;
int backwardstate = 0;
int leftstate = 0;
int rightstate = 0;
int executestate = 0;
int actions[100];
int counter = 0;
int action = 0;
int actionledtoggle = 0;
int traveltime = 500;
unsigned long remote_val = 0;

#include <IRLibDecodeBase.h> // First include the decode base
#include <IRLib_P01_NEC.h>   // Now include only the protocols you wish
//#include <IRLib_P02_Sony.h>  // to actually use. The lowest numbered
//#include <IRLib_P05_Panasonic_Old.h> // must be first but others can be any order.
//#include <IRLib_P07_NECx.h>  
//#include <IRLib_P09_GICable.h>
//#include <IRLib_P11_RCMM.h>
#include <IRLibCombo.h>     // After all protocols, include this
// All of the above automatically creates a universal decoder
// class called "IRdecode" containing only the protocols you want.
// Now declare an instance of that decoder.
//IRdecode myDecoder;
IRdecodeNEC myDecoder; // Now declare an instance of NEC decoder.

// Include a receiver either this or IRLibRecvPCI or IRLibRecvLoop
#include <IRLibRecv.h> 
IRrecv myReceiver(2);  //pin number for the receiver


// You will have to set these values depending on the protocol
// and remote codes that you are using. These are from my
// Sony DVD/VCR
#define MYPROTOCOL NEC
#define IR_RIGHT 0xC03F20DF         // volume up
#define IR_LEFT 0xC03F10EF       // volume down
#define IR_EXECUTE 0xC03FC837         // centre IR_EXECUTE
#define IR_FORWARDS 0xC03F30CF     // channel up
#define IR_BACKWARDS 0xC03F08F7   // channel down
#define BUTTON_0 0xC03F48B7       // button 0
#define BUTTON_1 0xC03FA05F       // button 1
#define BUTTON_2 0xC03F609F       // button 2
#define BUTTON_3 0xC03FE01F       // button 3
#define BUTTON_4 0xC03F906F       // button 4
#define BUTTON_5 0xC03F50AF       // button 5
#define BUTTON_6 0xC03FD02F       // button 6
#define BUTTON_7 0xC03FB04F       // button 7
#define BUTTON_8 0xC03F708F       // button 8
#define BUTTON_9 0xC03FF00F       // button 9
#define BUTTON_STOP 0xC03F629D    // button stop

void setup()
{
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

  myReceiver.enableIRIn(); // Start the receiver
  Serial.println(F("Ready to receive IR signals from NEC remote"));
}

void loop()
{
  // put your main code here, to run repeatedly:
  //Every cycle, check input:
  forwardstate = digitalRead(forwardpin);
  backwardstate = digitalRead(backwardpin);
  leftstate = digitalRead(leftpin);
  rightstate = digitalRead(rightpin);
  executestate = digitalRead(executepin);
  
  if (myReceiver.getResults())
  {
    if(myDecoder.decode())
    {
      remote_val = myDecoder.value;
    }
  }
  Serial.print(F("\ndecoder value: 0x")); Serial.println(remote_val, HEX);
  //Allows the button states to be seen on a computer
  Serial.print(F("Forward:")); Serial.println(forwardstate);
  Serial.print(F("Backward:")); Serial.println(backwardstate);
  Serial.print(F("Left:")); Serial.println(leftstate);
  Serial.print(F("Right:")); Serial.println(rightstate);
  Serial.print(F("Execute:")); Serial.println(executestate);
  
  //This section adds the appropriate commands to the array of integers, actions, which will be executed when the buttons are pressed:
  if ((forwardstate == 1) || (remote_val == IR_FORWARDS))
  {
    actions[counter] = 1;
    counter += 1;
    flashledpin();
  }
  if ((backwardstate == 1) || (remote_val == IR_BACKWARDS))
  {
    actions[counter] = 2;
    counter += 1;
    flashledpin();
  }
  if ((leftstate == 1) || (remote_val == IR_LEFT))
  {
    actions[counter] = 3;
    counter += 1;
    flashledpin(); 
  }
  if ((rightstate == 1) || (remote_val == IR_RIGHT))
  {
    actions[counter] = 4;
    counter += 1;
    flashledpin(); 
  }

  //For debugging, allows the actions array to be seen in serial.
  for (int i = 0; i < counter; i++)
  {
    Serial.println(actions[i]);
  }
  //When the execute button is pressed, the robot moves according to the traveltime in the dircections allocated by each button press in the array.
  if ((executestate == 1) || (remote_val == IR_EXECUTE))
  {
    Serial.println("EXECUTING");
    for (int i = 0; i < counter; i++)
    {
      action = actions[i];
      if (action == 1) 
      {
         toggleactionledpin();
         Omni.setCarAdvance(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         toggleactionledpin();
      }
      if (action == 2)
      {
         toggleactionledpin();
         Omni.setCarBackoff(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         toggleactionledpin();
      }   
      if (action == 3)
      {
         toggleactionledpin();
         Omni.setCarLeft(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         toggleactionledpin();
      }
      if (action == 4)
      {
         toggleactionledpin();
         Omni.setCarRight(0);
         Omni.setCarSpeedMMPS(100,500);
         Omni.delayMS(traveltime,false);
         Omni.setCarSlow2Stop(1500);
         toggleactionledpin(); 
      }           
    }
  }
  //Every second the program checks which buttons are being pressed.
  delay(1000); 
  myReceiver.enableIRIn(); //Restart receiver
  remote_val = 0; 
}

void flashledpin(void)
{
    digitalWrite(buttonledpin, HIGH);
    delay(200);
    digitalWrite(buttonledpin, LOW);
}
void toggleactionledpin(void)
{
    switch(actionledtoggle)
      {       
        case 1: digitalWrite(actionledpin, LOW); actionledtoggle == 0; break;
        case 0: digitalWrite(actionledpin, HIGH); actionledtoggle == 1;  break;
      }
}
