#include <Arduino.h>
#include <math.h>

#define encoderA 2  //interrupt
#define encoderB 3  //direction

#define dirPin_1 8    // Direction control pin for motor 1
#define dirPin_2 7    // Direction control pin

#define pwmPin_1 5    // Speed control pin (must be PWM capable)
#define pwmPin_2 6    // Speed control pin (must be PWM capable)

#define PI 3.141592
//----Values from datasheet (10 rpm)

const float ppr_motor = 12.0;
const float gear_ratio = 98;
const float ppr_out = ppr_motor * gear_ratio;
const float cpr_out = ppr_out * 4; // 4 states from A and B

//----Variables for user
double angular_xuser = 0;
double linear_xuser = 0;
float vel = 0;
float filtvel = 0;
float filtvel_lin = 0;
float theta = 0;

//----Dimensions
double rsect = 0.073152;
double rplly = 0.004191;
double rdkob = 0.075;

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
  float xini = 0;
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
        force_rot = k * (xend - angular_xuser) + b * vel;
        //pushing in the doorknob
        force_lin = 0; //door is not yet opened
      }
      //if user reaches the end of the turn (90 degree)
      //assuming xuser > 0 and xini = 0
      else
      {
        //open (push in) the door
        force_lin = - b * (vel); //CCW , scale with user's velocity
        delay(2500);
        //re-calibrate
        force_lin = k * (xuser - xequi);// "springs back" to the original position (closed door)
        force_rot = -k * (xuser - xequi); //rotate backward to the original position
        delay(2000);
      }
    }
    else //receiving no input i.e. doorknob not moving
    {
      force_rot = 0;
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
        force_rot = k * (xslip - xuser); //maybe just a constant force could work
        force_lin = 0; //lid is yet opened
      }
      //user reaches slip position 
      else
      {
        //force pushing left CCW with the user
        /*the spring force will scale with the user's velocity*/

        force_rot = - b * vel;
        delay(2000);
        //linear force push CW --> opening the lid
        force_lin = k * (xend - linear_xuser);
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

        force_rot = - k * (xend - angular_xuser) + b * filtvel;

        //linear force
        force_lin = 0; //door is not yet opened
      }
      //if user reaches the end of the turn (90 degree)
      //assuming xuser > 0 and xini = 0
      else
      {
        //open (push in) the door
        force_lin = - b * (vel); //CCW , scale with user's velocity
        delay(2500);
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
  //Opening a lid
  else if (lid)
  {
    //Not yet reached slip point
    if(xuser <= xslip && xuser != 0)
    {
      force_rot = k * (xslip - angular_xuser) + b * vel; //CW, resistive force based on distace and user's velocity
      force_lin = 0; //lid is not yet opened
    }
    //User reached slip point
    else
    {
      //force rotates with the user (CCW)
      force_rot = - b * vel; // CCW, scale with user's velocity
      delay(1500); //for smooth transition (hopefully)
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
  /*static int lastA = LOW;
  static int lastB = LOW;
  
  int A = digitalRead(encoderA);
  int B = digitalRead(encoderB);

  // Detect state changes
  if (A != lastA || B != lastB) {
    Serial.print("Encoder A changed from ");
    Serial.print(lastA);
    Serial.print(" to ");
    Serial.println(A);
    
    Serial.print("Encoder B changed from ");
    Serial.print(lastB);
    Serial.print(" to ");
    Serial.println(B);

    if (A != lastA) {
      if (A == HIGH) {
        // Rising edge of A
        if (B == LOW) {
          encoderct++;  // Clockwise
        } else {
          encoderct--;  // Counter-clockwise
        }
      }
    }

    lastA = A;
    lastB = B;
  }*/
}

//Calculates velocity
void speedcal()
{
  long changect = encoderct - lastct;
  float delta = (millis() - lasttime)/1000;
  float cirsect = 0;

  //compute rpm
  rpm = (changect/cpr_out) * (60.0/delta);
  //converting to ft/s
  cirsect = 2 * PI * rsect;
  vel = (rpm * cirsect) * (1/60) * (1/12);

  lastct = encoderct;
}

//Filters the velocity
float filtered_vel(float vel)
{
  float alpha = 0.3;
  float old_vel = 0;
  float vel_filtered = 0;

  vel_filtered = alpha * vel + alpha + (1-alpha) * old_vel;
  old_vel = vel_filtered;

  return vel_filtered;

}

//Outputs torque to the motor
void applytorque(double force)
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
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  //pinMode(enablePin_1, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderhandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderhandler, CHANGE);

  pinMode(pwmPin_1, OUTPUT);
  pinMode(dirPin_1, OUTPUT);
  //pinMode(pwmPin_2, OUTPUT);
  //pinMode(dirPin_2, OUTPUT);
  
  //initial direction is clockwise
  analogWrite(pwmPin_1, 0);
  digitalWrite(dirPin_1, clockwise ? HIGH : LOW); //convention: clockwise --> HIGH
}

void loop() 
{
  // put your main code here, to run repeatedly:
    force_lin = -10;
    //----get velocity in ft/s
    speedcal();

    //----get pos in degrees
    angular_xuser = (encoderct * 360)/cpr_out;
    linear_xuser = angular_xuser * (180/PI) * rsect;

    applytorque(force_lin);

    //----Choose a mode
    if (guidance)
    {
      hap_guidance();
    }
    else
    {
      hap_hindrance();
    }
    
    //if (millis() - lastprinttime > 1000)
    //{
      Serial.print("vel is: ");
      Serial.println(rpm, 3);
      Serial.print("angular pos is: ");
      Serial.println(angular_xuser, 3);
      Serial.print("linear pos is: ");
      Serial.println(linear_xuser, 3);
    //}
    //Serial.print("encoderct is: ");
    //Serial.println(encoderct, 3);
    Serial.println("---------------End of iteration---------------");
    delay(2500);
    lasttime = millis();
}