#include <Arduino.h>
#include <math.h>

#define encoderA 2  //interrupt
#define encoderB 3  //direction

#define dirPinRot 8    // Direction control pin for rotational motor (motor 1)
#define dirPinTrans 7    // Direction control pin for translational motor (motor 2)

#define pwmPinRot 5    // Speed control pin rotational motor (motor 1)
#define pwmPinTrans 6    // Speed control pin translational motor (motor 2)

#define PI 3.141592
//----Values from datasheet (10 rpm)

const float ppr_motor = 12.0;
const float gear_ratio = 20;
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
bool flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

//----Variables for motors
double xuser_ang = 0;
double xuser_lin = 0;
double m1Vel = 0;
double m2Pos = 0;
double m2Pos_prev = 0;
double Tp = 0;
float m2Vel = 0;
float m2Vel_filt = 0;
float filtvel_lin = 0;
float theta = 0;
float currtime = 0;
float prevtime = 0;
float delta = 0;
float delta_sec = 0;

//----Dimensions
double rsect = 0.0635;
double rplly = 0.015875;
double rdkob = 0.075;
double rh = 0.0508; //[m]
double rs = 0.0762;   //[m]
double rp = 0.004191;

//----Variables for speed and position cals
int previousEncoderState;
int currentEncoderState;
volatile long encoderct = 0;
unsigned long lastct = 0;
unsigned long lasttime = 0;
//static unsigned long lastprinttime = 0;
float rpm = 0;

//----States
bool guidance = true;
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
float xslip = 0.1; //Position where we want jar to "slip" at

enum HAPTIC_HANDLE_SETTINGS {
  DOOR = 0x1,
  JAR = 0x2,
  ASSISTIVE = 0x4,
  RESISTIVE = 0x8
};

/*****************

 MAIN CODE

*****************/

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  setPwmFrequency(pwmPinRot, 1);
  setPwmFrequency(pwmPinTrans, 1);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  
  // attachInterrupt(digitalPinToInterrupt(encoderA), encoderhandler, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderB), encoderhandler, CHANGE);

  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);

  pinMode(pwmPinRot, OUTPUT);
  pinMode(pwmPinTrans, OUTPUT);

  pinMode(dirPinRot, OUTPUT);
  pinMode(dirPinTrans, OUTPUT);
  
  //initial direction is clockwise
  analogWrite(pwmPinRot, 0);
  analogWrite(pwmPinTrans, 0);

  digitalWrite(dirPinRot, clockwise ? HIGH : LOW); //convention: clockwise --> HIGH
  digitalWrite(dirPinTrans, clockwise ? HIGH : LOW);

  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

  previousEncoderState = digitalRead(encoderB);

  Serial.println("***********************");
  pad(5);
  Serial.println("Welcome To");
  pad(4);
  Serial.println("Haptic Helper");
  Serial.println();
  pad(5);
  Serial.println("by E.L.M.S.");
  Serial.println("***********************");
}

void loop() 
{
  int userSettings = 0;

  Serial.println("Select a virtual environment:");
  pad(2);
  Serial.println("(1) Door");
  pad(2);
  Serial.println("(2) Jar");

  while(Serial.available() == 0)
  {
    int virtualEnvChoice = Serial.parseInt();

    if (Serial.available() != 0)
    {
      Serial.print("Env Choice: ");
      Serial.println(virtualEnvChoice);
    }

    switch (virtualEnvChoice)
    {
      case 0:
        break;
      case 1:
        userSettings |= HAPTIC_HANDLE_SETTINGS::DOOR;
        break;
      case 2:
        userSettings |= HAPTIC_HANDLE_SETTINGS::JAR;
        break;
      default:
        Serial.println("Unknown setting");
        return;
    };
  }
  clear();

  Serial.println("Select Mode:");
  pad(2);
  Serial.println("(1) Assistive");
  pad(2);
  Serial.println("(2) Resistive");

  while(Serial.available() == 0)
  {
    int modeChoice = Serial.parseInt();

    if(Serial.available() != 0)
    {
    Serial.print("Mode Choice: ");
    Serial.println(modeChoice);

    }

    switch(modeChoice)
    {
      case 0:
        break;
      case 1:
        userSettings |= HAPTIC_HANDLE_SETTINGS::ASSISTIVE;
        break;
      case 2:
        userSettings |= HAPTIC_HANDLE_SETTINGS::RESISTIVE;
        break;
      default:
        Serial.println("Unknown mode");
        return;
    }
  }
  clear();

  Serial.println("Setting Up System");
  Serial.println("Please Do Not Touch System Until Done");
  delay(100000);

  /*
    Case 1: Door
  */
  if(userSettings & HAPTIC_HANDLE_SETTINGS::DOOR)
  {
    apply_rot_torque(0.01);
    apply_trans_torque(5);
    delay(100000);
    apply_trans_torque(200);

    transMotorPosCount();

    double initRotPos = getRotMotorPos();
    double initTransPos = getTransMotorPos();

    double k_handle;
    double door_factor;

    /*
      Establish system parameters
    */
    if(userSettings & HAPTIC_HANDLE_SETTINGS::ASSISTIVE)
    {
      Serial.println("Assistive");
      k_handle = -0.003;
      door_factor = -0.5;
    }

    else if(userSettings & HAPTIC_HANDLE_SETTINGS::RESISTIVE)
    {
      Serial.println("Resistive");
      k_handle = 0.005;
      door_factor = 1;
    }

    else
    {
      Serial.println("ERROR: Unknown Mode");
    }

    Serial.println("Ready");

    bool isComplete = false;
    bool isRotated = false;
    while(!isComplete)
    {
      // get the translational position
      transMotorPosCount();
      
      // get the rotational position
      getRotMotorPos();

      // Serial.print("User Rotational Displacement: ");
      // Serial.println(encoderct);

      double f_doorhandle = k_handle * (initRotPos - xuser_ang) + 0.01;
      double f_door = 200;

      Serial.print("User Displacement: ");
      Serial.println(xuser_ang - initRotPos);

      /*
        Check if the door handle is rotated by x
        if so then set isRotated to true
      */
      if (!isRotated && initRotPos - xuser_ang > 0.1)
        isRotated = true;

      /*
        Check if the handle is rotated but door is not open
        if so then render opening force
        else simulation is complete
      */
      if(isRotated && xuser_lin - initTransPos > 0.5)
        isComplete = true;
      else
        f_door = door_factor * 10;

      Serial.print("Output force: ");
      Serial.println(f_doorhandle, 4);
      
      apply_rot_torque(f_doorhandle);
      apply_trans_torque(f_door);
    }
  }

  /*
    Case 2: Jar
  */
  else if(userSettings & HAPTIC_HANDLE_SETTINGS::JAR)
  {
    double xslip = 0.1;

    apply_rot_torque(-0.01);
    apply_trans_torque(-5);
    delay(100000);
    apply_trans_torque(-200);

    transMotorPosCount();

    double initRotPos = getRotMotorPos();
    double initTransPos = getTransMotorPos();

    /*
      Establish system parameters
    */
    if(userSettings & HAPTIC_HANDLE_SETTINGS::ASSISTIVE)
    {
      Serial.println("Assistive");
    }

    else if(userSettings & HAPTIC_HANDLE_SETTINGS::RESISTIVE)
    {
      Serial.println("Resistive");
    }

    else
    {
      Serial.println("ERROR: Unknown Mode");
    }

    Serial.println("Ready");

    bool isComplete = false;
    bool isOpen = false;
    while(!isComplete)
    {
      isComplete = true;
      // get the translational position
      transMotorPosCount();
      
      // get the rotational position
      getRotMotorPos();

      double f_jar_lid = -1;
      double f_jar = -200;

      if (!isOpen && xuser_ang - initRotPos > xslip)
      {
        f_jar_lid = 0;
        isOpen = true;
      }

      if (isOpen && xuser_lin - initTransPos < 0.1)
      {
        // opening force
      }
      else
        isComplete = true;

      apply_rot_torque(f_jar_lid);
      apply_trans_torque(f_jar);
    }
  }

  else
  {
    Serial.println("ERROR: Unknown Environment");
    return;
  }

  apply_rot_torque(0);
  apply_trans_torque(0);

  Serial.println("Completed");
}

/*******************

  HELPER FUNCTIONS

********************/

void pad(int n)
{
  for(int i = 0; i < n; i++)
  {
    Serial.print(" ");
  }
}

void clear()
{
  while(Serial.available())
    Serial.read();
}

//Outputs torque to the motor
void apply_rot_torque(double force)
{
  torque = (force * rplly * rdkob)/rsect;
  bool direction = (force > 0);

  //If force > 0 --> spins clockwise otherwise counter-clockwise
  digitalWrite(dirPinRot, direction ? LOW : HIGH);

  duty = sqrt(abs(torque) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal

  analogWrite(pwmPinRot, output);
}

void apply_trans_torque(double force)
{
  Tp = (rp / rs) * rh * force;  // Compute the require motor pulley torque (Tp) to generate that force
  bool direction = (force > 0);

  digitalWrite(dirPinTrans, direction ? LOW : HIGH);

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPinTrans, output); // output the signal
}

// void hap_guidance()
// {
//   float k = 5;
//   float b = 5;
//   float xend = 3; //final position of door rotation
//   float xequi_ang = 0.000;
//   float xmin_ang = 0;
//   float xmax_ang = 0;
//   float xequi_lin = 0.000;
//   float xmin_lin = -0.002;
//   float xmax_lin = 0.008;
//   float xuser = 0;
//   //assuming user start out at xuser = 0 and xend > 0

//   //Twisting a doorknob
//   //no resistance
//   //Assume that distance between 2 points is positive
//   if (doorknob) 
//   {
    
//     if(xuser_ang != 0) //if user is using doorknob
//     {
//       //move with the user (positive force)
//       //assumming xuser = xini = 0 and xend > 0
//       //large assitive force at the beginning and decay as user approach xend
//       if(xuser_ang < xmax_ang)
//       {
//         force_rot = k * (xmax_ang - xuser_ang) + b * m1Vel;
//         //pushing in the doorknob
//         force_lin = 0; //door is not yet opened
//       }
//       //if user reaches the end of the turn (90 degree)
//       //assuming xuser > 0 and xini = 0
//       else
//       {
//         //open (push in) the door
//         force_lin = - b * (m1Vel); //CCW , scale with user's velocity
//         //applym2_torque(force_lin);
//         delay(2500); //might get rid of
//       }
//     }
//     else //receiving no input i.e. doorknob not moving
//     {
//       force_rot = 0;
//       force_lin = 0;
//     }  
//   }
//   //Opening a lid (twisting) 
//   //No resistance 
//   else if (lid) //need to find xslip ([])
//   {
//     //if user is using the lid
//     if(xuser_ang != 0)
//     {
//       //At beginning oppose a very minimum force against the user until they reach the slip position (xslip)
//       if(xuser <= xslip)
//       {
//         //force pushing right CW
//         force_rot = -k * (xslip - xuser); //maybe just a constant force could work
//         force_lin = 0; //lid is yet opened
//       }
//       //user reaches slip position 
//       else
//       {
//         //force pushing left CCW with the user
//         /*the spring force will scale with the user's velocity*/
//         force_rot = - b * m1Vel;
        
//         //applym1_torque(force_rot);
//         //----incoporate LED HERE
//         delay(2500);
//         //linear force push CW --> opening the lid
//         force_lin = k * (xend - xuser_lin);
//         //force_lin = b * (m1Vel)
//         //----turn off LED
//       } 
//     } 
//   }
//   applym1_rot_torque(force_rot);
//   applym2_lin_torque(force_lin);
//   //----Recalibrate the system
//   if (doorknob)
//   {
//     Serial.println("Recalibrating...");
//     //force_lin = k * (m2Pos - xlin_equi);
//     //"springs back" to the original position (closed door)
//     force_lin = k * (abs(m2Pos - xequi_lin));// m2Pos is now pushed in (negative), abs() --> distance is positive
//     //rotate backward to the original position
//     force_rot = -k * (xuser_ang - xequi_ang); //check the sign
//     applym1_rot_torque(force_rot);
//     delay(2000);
//     applym2_lin_torque(force_lin);
//     delay(2000);
//     Serial.println("System ready!");
//   }
//   else if (lid)
//   {
//     ;
//   }

// }

// void hap_hindrance()
// {
//   float k = 5;
//   float b = 5;
//   float xend = 3; //final position of door rotation
//   float xequi = 0;
//   float xuser = 0;
//   //assuming user start out at xuser = 0 and xend > 0
  
//   //Twisting a doorknob
//   //With resistance
//   //Assume that distance between 2 points is positive
//   if (doorknob) 
//   {
//     if(xuser_ang != 0) //if user is using doorknob
//     {
//       if(xuser_ang < xend)
//       //opppose the user's force (negative force)
//       //assuming xuser = xini = 0 and xend > 0
//       {
//         //rotational force
//         /*"+ b * filtvel" --> make resistive force smaller if user puts 
//         in more effort (bigger) (might not be true)*/

//         force_rot = - k * (xend - xuser_ang) + b * m1Vel;
//         //force_rot = - k * (xend - angular_xuser) - b * m1Vel; //most resistive

//         //linear force
//         force_lin = 0; //door is not yet opened
//       }
//       //if user reaches the end of the turn (90 degree)
//       //assuming xuser > 0 and xini = 0
//       else
//       {
//         //open (push in) the door
//         force_lin = - b * (m2Vel_filt); //CCW , scale with user's velocity
//         //applym2_torque(force_lin);
//         //LED
//         delay(2500);
//         //re-calibrate
//         force_lin = k * (xuser - xequi);// "springs back" to the original position (closed door)
//         force_rot = -k * (xuser - xequi); //rotate backward to the original position
//         //delay(2000);
//         //LED OFF
//       }
//     }
//     else //receiving no input i.e. doorknob not moving
//     {
//       force_rot = 0;
//       force_lin = 0;
//     }  
//   }
//   //Opening a lid
//   else if (lid)
//   {
//     //Not yet reached slip point
//     if(xuser <= xslip && xuser != 0)
//     {
//       force_rot = -k * (xslip - xuser_ang) - b * m1Vel; //CW, resistive force based on distace and user's velocity
//       force_lin = 0; //lid is not yet opened
//     }
//     //User reached slip point
//     else
//     {
//       //force rotates with the user (CCW)
//       force_rot = - b * m1Vel; // CCW, scale with user's velocity
//       delay(2500); //for smooth transition (hopefully)
//       force_lin = k * (xend - xuser_lin); //CW, push open the lid (spring might be too strong?)
//     }
//   } 
// }

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

void encoderPos()
{
  currentEncoderState = digitalRead(encoderB);

  if(currentEncoderState != previousEncoderState)
  {
    if(digitalRead(encoderA) == currentEncoderState)
      encoderct++;
    else
      encoderct--;
  }

  previousEncoderState = currentEncoderState;
}

//Calculates velocity
void m1_speedcal()
{
  long changect = encoderct - lastct;
  float delta = (millis() - lasttime)/1000;
  float cirsect = 0;

  //compute rpm
  rpm = (changect/cpr_out) * (60.0/delta);
  //converting to m/s
  cirsect = 2 * PI * rsect;

  lastct = encoderct;

  m1Vel = (rpm * cirsect) * (1/60);
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

void transMotorPosCount()
{
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
}

double getTransMotorPos()
{
  transMotorPosCount();

  double ts = -0.0186 * updatedPos - 0.0475;
  xuser_lin = ts * (PI / 180) * rh;
  return xuser_lin;
}

double getRotMotorPos()
{
  encoderPos();
  xuser_ang = (encoderct * 360)/cpr_out;
  return xuser_ang;
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

void modeswitch()
{
  if (Serial.available())
  {
    char user_input = Serial.read();
    
    if (user_input == 's' || user_input == 'S')
    {
      if(guidance)
      {
        guidance = false;
        hindrance = true;
        Serial.println("Switched to Hindrance mode");
      }
      else
      {
        guidance = true;
        hindrance = false;
        Serial.println("Switched to Guidance mode");
      }
    }
  }
}