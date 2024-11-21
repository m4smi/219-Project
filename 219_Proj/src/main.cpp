#include <Arduino.h>
#include <math.h>

#define encoderA 2
#define encoderB 3

#define DIR_PIN 4    // Direction control pin
#define PWM_PIN 5    // Speed control pin (must be PWM capable)
#define ENABLE_PIN 6

#define PI 3.141592
//----Values from datasheet (10 rpm)

const float ppr_motor = 12.0;
const float gear_ratio = 98;
const float ppr_out = ppr_motor * gear_ratio;
const float cpr_out = ppr_out * 4; // 4 states from A and B

//----Pins

int pwnpin1 = 1; //pwm output pin for motor 1
int pwnpin2 = 2; //pwm output pin for motor 2
int dirpin1 = 3; //directional output for motor 1
int dirpin2 = 4; //directional output for motor 2
int sensorpin = 5; //input value from encoder

//----Variables for user
double force = 0;
double torque = 0;
double xuser = 0;
double rsect = 0.5;
float vel = 0;
float filtvel = 0;
float theta = 0;

//----Variables for speed and position cals
volatile long encoderct = 0;
unsigned long lastct = 0;
unsigned long lasttime = 0;
float rpm = 0;
bool doorknob = true;
bool lid = false;
bool clockwise = true;


//-----------------------Main
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderhandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderhandler, CHANGE);

  pinMode(pwnpin1, OUTPUT);
  pinMode(dirpin1, OUTPUT);
  pinMode(pwnpin2, OUTPUT);
  pinMode(dirpin2, OUTPUT);
  
  //initial direction is clockwise
  digitalWrite(dirpin1, clockwise ? LOW : HIGH); //might need to change to HIGH : LOW
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if (millis() - lasttime >= 100)
  {
    //----get velocity in ft/s
    speedcal();
    //----filter velocity
    filtvel = filtered_vel(vel);
    //----get pos in degrees
    theta = (encoderct * 360)/cpr_out;
    xuser = theta * (rsect * 180/PI);

  }
}

void hap_guidance()
{
  float k = 5;
  float b = 5;
  float xend = 3; //final position of door rotation
  float xequi = 0;
  float xini = 0;
  //assuming user start out at xuser = 0 and xend > 0
  

  if (doorknob) //no resistance
  {
    
    if(xuser != 0) //if user is using doorknob
    {
      //move with the user (positive force)
      //assumming xuser = xini = 0 and xend > 0
      //large assitive force at the beginning and decay as user approach xend
      if(xuser != xend)
      {
        force = k * (xend - xuser) + b * filtvel;
      }
      //if user reaches the end of the turn (90 degree)
      //assuming xuser > 0 and xini = 0
      else
      {
        force = k * (xini- xuser) - b * filtvel;
      }
    }
    else //receiving no input i.e. doorknob not moving
    {
      force = 0;
    }  
  }
  //Opening a lid
  else if (lid)
  {

  }  
  
  
}

void hap_hindrance()
{
  float k = 5;
  float b = 5;
  float xend = 3; //final position of door rotation
  float xequi = 0;
  float xini = 0;
  //assuming user start out at xuser = 0 and xend > 0
  
  //Opening a door
  if (doorknob) //no resistance
  {
    
    if(xuser != 0) //if user is using doorknob
    {
      if(xuser != xend)
      //opppose the user's force (negative force)
      //assuming xuser = xini = 0 and xend > 0
      {
        force = k * (xuser - xend) - b * filtvel;
      }
      //if user reaches the end of the turn (90 degree)
      //assuming xuser > 0 and xini = 0
      else
      {
        force = k * (xuser - xini) + b * filtvel;
      }
    }
    else //receiving no input i.e. doorknob not moving
    {
      force = 0;
    }  
  }
  //Opening a lid
  else if (lid)
  {

  } 
}

//-----------------------Functions
void encoderhandler()
{
  int A = digitalRead(encoderA);
  int B = digitalRead(encoderB);

  //Determine rotational direction based on encoder channels
  if (A == B)
  {
    encoderct++;
  }
  else
  {
    encoderct--;
  }
}

void speedcal()
{
  long changect = encoderct - lastct;
  float delta = (millis() - lasttime)/1000;

  //compute rpm
  rpm = (changect/cpr_out) * (60.0/delta);
  //converting to ft/s
  cirsect = 2 * PI * rsect;
  vel = (rpm * cirsect) * (1/60) * (1/12);

  lastct = encoderct;
}

void filtered_vel(int x)
{

}

void applytorque(float force)
{
  float motorspeed = constrain(abs(force), 0, 255); //constrain from going over analogWrite limits
  bool direction = (force > 0);

  digitalWrite(dirpin1, direction ? HIGH : LOW);
  analogWrite(pwnpin1, motorspeed);
}