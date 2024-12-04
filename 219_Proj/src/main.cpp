#include <Arduino.h>
#include <math.h>

#define encoderA 2  //interrupt
#define encoderB 3  //direction

#define dirPin_1 8    // Direction control pin for motor 1
#define dirPin_2 7    // Direction control pin for motor 2

#define pwmPin_1 5    // Speed control pin (motor 1)
#define pwmPin_2 6    // Speed control pin (motor 2)

#define PI 3.141592
//----Values from datasheet (10 rpm)

const float ppr_motor = 12.0;
const float gear_ratio = 98;
const float ppr_out = ppr_motor * gear_ratio;
const float cpr_out = ppr_out * 4; // 4 states from A and B

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

//----Variables for motors
double angular_xuser = 0;
double linear_xuser = 0;
double m1Vel = 0;
double m2Pos = 0;
double m2Pos_prev = 0;
double Tp = 0;
float m2Vel = 0;
float m2Vel_filt = 0;
float filtvel_lin = 0;
float theta = 0;

//----Dimensions
double rsect = 0.073152;
double rplly = 0.004191;
double rdkob = 0.075;
double rh = 0.0508; //[m]
double rs = 0.0762;   //[m]
double rp = 0.004191;

//----Variables for speed and position cals
volatile long encoderct = 0;
unsigned long lastct = 0;
unsigned long lasttime = 0;
static unsigned long lastprinttime = 0;
float rpm = 0;

//----States
bool guidance = false;
bool hindrance = false;
bool doorknob = true;
bool reverse = false;
bool lid = false;
bool clockwise = true;

//----Output
double force_rot = 0;
double force_lin = 0;
double torque = 0;
double duty = 0;
unsigned int output = 0;

//---- Variables for jar VE
bool slipped = false; // Bool to see if jar has slipped yet
float xslip = 0; //Position where we want jar to "slip" at

//----------------------------Functions
void hap_guidance()
{
  float k = 5;
  float b = 5;
  float xend = 3; //final position of door rotation
  float xequi = 0;
  float xuser = 0;
  //assuming user start out at xuser = 0 and xend > 0

  //Twisting a doorknob
  //no resistance
  //Assume that distance between 2 points is positive
  if (doorknob) 
  {
    
    if(angular_xuser != 0) //if user is using doorknob
    {
      //move with the user (positive force)
      //assumming xuser = xini = 0 and xend > 0
      //large assitive force at the beginning and decay as user approach xend
      if(angular_xuser < xend)
      {
        force_rot = k * (xend - angular_xuser) + b * m1Vel;
        //pushing in the doorknob
        force_lin = 0; //door is not yet opened
      }
      //if user reaches the end of the turn (90 degree)
      //assuming xuser > 0 and xini = 0
      else
      {
        //open (push in) the door
        force_lin = - b * (m1Vel); //CCW , scale with user's velocity
        //applym2_torque(force_lin);
        delay(2500);
        //------incoporate LED here
        //re-calibrate
        force_lin = k * (xuser - xequi);// "springs back" to the original position (closed door)
        force_rot = -k * (xuser - xequi); //rotate backward to the original position
        
        delay(2000);
      }
    }
    else //receiving no input i.e. doorknob not moving
    {
      force_rot = 0;
      force_lin = 0;
    }  
  }
  //Opening a lid (twisting) 
  //No resistance 
  else if (lid) //need to find xslip ([])
  {
    //if user is using the lid
    if(angular_xuser != 0)
    {
      //At beginning oppose a very minimum force against the user until they reach the slip position (xslip)
      if(xuser <= xslip)
      {
        //force pushing right CW
        force_rot = -k * (xslip - xuser); //maybe just a constant force could work
        force_lin = 0; //lid is yet opened
      }
      //user reaches slip position 
      else
      {
        //force pushing left CCW with the user
        /*the spring force will scale with the user's velocity*/
        force_rot = - b * m1Vel;
        
        //applym1_torque(force_rot);
        //----incoporate LED HERE
        delay(2500);
        //linear force push CW --> opening the lid
        force_lin = k * (xend - linear_xuser);
        //force_lin = b * (m1Vel)
        //----turn off LED
      } 
    } 
  }  
}

void hap_hindrance()
{
  float k = 5;
  float b = 5;
  float xend = 3; //final position of door rotation
  float xequi = 0;
  float xuser = 0;
  //assuming user start out at xuser = 0 and xend > 0
  
  //Twisting a doorknob
  //With resistance
  //Assume that distance between 2 points is positive
  if (doorknob) 
  {
    if(angular_xuser != 0) //if user is using doorknob
    {
      if(angular_xuser < xend)
      //opppose the user's force (negative force)
      //assuming xuser = xini = 0 and xend > 0
      {
        //rotational force
        /*"+ b * filtvel" --> make resistive force smaller if user puts 
        in more effort (bigger) (might not be true)*/

        force_rot = - k * (xend - angular_xuser) + b * m1Vel;
        //force_rot = - k * (xend - angular_xuser) - b * m1Vel; //most resistive

        //linear force
        force_lin = 0; //door is not yet opened
      }
      //if user reaches the end of the turn (90 degree)
      //assuming xuser > 0 and xini = 0
      else
      {
        //open (push in) the door
        force_lin = - b * (m2Vel_filt); //CCW , scale with user's velocity
        //applym2_torque(force_lin);
        //LED
        delay(2500);
        //re-calibrate
        force_lin = k * (xuser - xequi);// "springs back" to the original position (closed door)
        force_rot = -k * (xuser - xequi); //rotate backward to the original position
        //delay(2000);
        //LED OFF
      }
    }
    else //receiving no input i.e. doorknob not moving
    {
      force_rot = 0;
      force_lin = 0;
    }  
  }
  //Opening a lid
  else if (lid)
  {
    //Not yet reached slip point
    if(xuser <= xslip && xuser != 0)
    {
      force_rot = -k * (xslip - angular_xuser) - b * m1Vel; //CW, resistive force based on distace and user's velocity
      force_lin = 0; //lid is not yet opened
    }
    //User reached slip point
    else
    {
      //force rotates with the user (CCW)
      force_rot = - b * m1Vel; // CCW, scale with user's velocity
      delay(2500); //for smooth transition (hopefully)
      force_lin = k * (xend - linear_xuser); //CW, push open the lid (spring might be too strong?)
    }
  } 
}

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

//Calculates velocity
void m1_speedcal()
{
  long changect = encoderct - lastct;
  float delta = (millis() - lasttime)/1000;
  float cirsect = 0;

  //compute rpm
  rpm = (changect/cpr_out) * (60.0/delta);
  //converting to ft/s
  cirsect = 2 * PI * rsect;
  m1Vel = (rpm * cirsect) * (1/60) * (1/12);

  lastct = encoderct;
}

//Filters the velocity
float filtered_vel(float vel)
{
  float alpha = 0.9;
  float old_vel = 0;
  float vel_filtered = 0;

  vel_filtered = alpha * vel + (1-alpha) * old_vel;
  old_vel = vel_filtered;

  return vel_filtered;
}

//Outputs torque to the motor
void applym1_torque(double force)
{
  torque = (force * rplly * rdkob)/rsect;
  bool direction = (force > 0);

  //If force > 0 --> spins clockwise otherwise counter-clockwise
  digitalWrite(dirPin_1, direction ? LOW : HIGH);

  duty = sqrt(abs(torque) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal

  analogWrite(pwmPin_1, output);
}

void applym2_torque(double force)
{
  Tp = (rp / rs) * rh * force;  // Compute the require motor pulley torque (Tp) to generate that force

  if (force > 0) {
    digitalWrite(dirPin_2, HIGH);
  } else {
    digitalWrite(dirPin_2, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin_2, output); // output the signal
}

void setPwmFrequency(int pin, int divisor) 
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
////----------------------------Main
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  setPwmFrequency(pwmPin_1, 1);
  setPwmFrequency(pwmPin_2, 1);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  //pinMode(enablePin_1, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderhandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderhandler, CHANGE);

  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);

  pinMode(pwmPin_1, OUTPUT);
  pinMode(pwmPin_2, OUTPUT);

  pinMode(dirPin_1, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  
  //initial direction is clockwise
  analogWrite(pwmPin_1, 0);
  analogWrite(pwmPin_2, 0);

  digitalWrite(dirPin_1, clockwise ? HIGH : LOW); //convention: clockwise --> HIGH
  digitalWrite(dirPin_2, clockwise ? HIGH : LOW);

  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

}

void loop() 
{
  /* Positions countring for motor 02 */
  float begin = millis();
  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if ((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiff > 0) {       // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  updatedPos = rawPos + flipNumber * OFFSET; // need to update pos based on what most recent offset is

  //----------------------------Motor 01
  force_lin = 0;
  double forcem2 = 0;
  //----get velocity in ft/s
  m1_speedcal();

  //----get pos in degrees
  angular_xuser = (encoderct * 360)/cpr_out;
  linear_xuser = angular_xuser * (180/PI) * rsect;

  applym1_torque(force_lin);

  //----------------------------Motor 02
  double theta_s = .0106 * updatedPos - 9.0251;

  //---motor 2 position
  m2Pos = rh * (theta_s * 3.14159 / 180);
  //---motor 2 velocity
  float end = millis();
  float delta = 0;
  float delta_sec = 0;
  if (end > begin)
  {
    delta = end - begin;
    begin = end; 
  }
  delta_sec = delta/64000;
  m2Vel = double(m2Pos - m2Pos_prev) / delta_sec;
  //---filter motor 2 velocity
  m2Vel_filt = filtered_vel(m2Vel);

  m2Pos_prev = m2Pos;
  applym2_torque(forcem2);
  //----Choose a mode
  /*if (guidance)
  {
    hap_guidance();
  }
  else
  {
    hap_hindrance();
  }*/
  
  //if (millis() - lastprinttime > 1000)
  //{
  /*
    Serial.print("vel is: ");
    Serial.println(m1Vel, 3);
    Serial.print("angular pos is: ");
    Serial.println(angular_xuser, 3);
    Serial.print("linear pos is: ");
    Serial.println(linear_xuser, 3);
  */
  //}
  Serial.print("vel is: ");
  Serial.println(m2Vel_filt, 3);
  Serial.print("linear pos is: ");
  Serial.println(m2Pos, 3);
  //Serial.print("encoderct is: ");
  //Serial.println(encoderct, 3);
  Serial.println("---------------End of iteration---------------");
  delay(2500);
  lasttime = millis();
}