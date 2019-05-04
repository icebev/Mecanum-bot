// Includes the Servo library
#include <Servo.h>. 

// Defines Tirg and Echo pins of the Ultrasonic Sensor
const int trigPin = 10;
const int echoPin = 11;
const int proximityLEDPin = 7;
const int pwrpin = 6;
const int interruptPin = 2;
const int potentiometerpin = A1;
volatile int power = LOW;

// Variables for the duration and the distance
int potentiometervalue;
long duration;
int distance;
int threshold; //For proximity LED

Servo myServo; // Creates a servo object for controlling the servo motor

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(proximityLEDPin, OUTPUT); //Sets proximity LED pin as Output
  pinMode(pwrpin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), toggle, RISING);
  Serial.begin(9600);
  myServo.attach(12); // Defines on which pin is the servo motor attached
}
void loop() {
  if (power == HIGH) {
    // rotates the servo motor from 15 to 165 degrees
    for(int i=15;i<=165;i += 5){  
    myServo.write(i);
    delay(30);
    distance = calculateDistance();// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    adjustthreshold();
    proximityLED();    
    Serial.print("Angle: "); Serial.println(i);
    Serial.print("Distance: "); Serial.println(distance); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    if (power == LOW)
    {
      myServo.write(90);
      return 0;
    }
    delay(500);
    }
      // Repeats the previous lines from 165 to 15 degrees
    for(int i=165;i>15; i -= 5){  
    myServo.write(i);
    delay(30);
    distance = calculateDistance();
    adjustthreshold();
    proximityLED();
    Serial.print("Angle: "); Serial.println(i);
    Serial.print("Distance: "); Serial.println(distance);
    if (power == LOW)
    {
      myServo.write(90);
      return 0;
    }
    delay(500);
    }
  }
  else {
    delay(1000); 
   }
  }

// Function for calculating the distance measured by the Ultrasonic sensor
int calculateDistance(){ 
  
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance= duration*0.034/2;
  return distance;
}

void adjustthreshold(void)
{
  potentiometervalue = analogRead(potentiometerpin);
  threshold = map(potentiometervalue, 0, 1023, 0, 100);
  Serial.print("sensor = ");
  Serial.print(potentiometervalue);
  Serial.print("\t output = ");
  Serial.println(threshold);
}
void proximityLED(void)
{
  if (distance <= threshold) {
    digitalWrite(proximityLEDPin, HIGH);
  }
  else {
    digitalWrite(proximityLEDPin, LOW);
  }
    
}
void toggle() {
  power = !power;
  digitalWrite(pwrpin, power);
  Serial.println(power);
}
