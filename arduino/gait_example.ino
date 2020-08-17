#include <Servo.h>
#include <math.h>

const int COXA1_SERVO  = 19;          //servo port definitions
const int FEMUR1_SERVO = 21;
const int TIBIA1_SERVO = 23;
const int COXA2_SERVO  = 25;
const int FEMUR2_SERVO = 27;
const int TIBIA2_SERVO = 29;
const int COXA3_SERVO  = 31;
const int FEMUR3_SERVO = 33;
const int TIBIA3_SERVO = 35;
const int COXA4_SERVO  = 37;
const int FEMUR4_SERVO = 39;
const int TIBIA4_SERVO = 41;
const int COXA5_SERVO  = 43;
const int FEMUR5_SERVO = 45;
const int TIBIA5_SERVO = 47;
const int COXA6_SERVO  = 49;
const int FEMUR6_SERVO = 51;
const int TIBIA6_SERVO = 53;

const int COXA_LENGTH = 51;          
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;
																		  
const long A12DEG = 209440;           //12 degrees in radians x 1,000,000
const long A30DEG = 523599;           //30 degrees in radians x 1,000,000

const int FRAME_TIME_MS = 20;         //frame time (20msec = 50Hz)

const float HOME_X[6] = {  82.0,   0.0, -82.0,  -82.0,    0.0,  82.0};  //coxa-to-toe home positions
const float HOME_Y[6] = {  82.0, 116.0,  82.0,  -82.0, -116.0, -82.0};
const float HOME_Z[6] = { -80.0, -80.0, -80.0,  -80.0,  -80.0, -80.0};

const float BODY_X[6] = { 110.4,  0.0, -110.4, -110.4,    0.0, 110.4};  //body center-to-coxa servo distances 
const float BODY_Y[6] = {  58.4, 90.8,   58.4,  -58.4,  -90.8, -58.4};
const float BODY_Z[6] = {   0.0,  0.0,    0.0,    0.0,    0.0,   0.0};

const int COXA_CAL[6]  = {2, -1, -1, -3, -2, -3};                       //servo calibration constants
const int FEMUR_CAL[6] = {4, -2,  0, -1,  0,  0};
const int TIBIA_CAL[6] = {0, -3, -3, -2, -3, -1};


//***********************************************************************
// Variable Declarations 변수 선언
//***********************************************************************

unsigned long currentTime;            //frame timer variables
unsigned long previousTime;

int mode;
int gait;
int gait_speed;
int reset_position;
int capture_offsets;

float L0, L3;                         //inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg_num;                          //positioning and walking variables
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6]   = {1,2,1,2,1,2};     //for tripod gait walking

//***********************************************************************
// Object Declarations
//***********************************************************************
PS2X ps2x;              //PS2 gamepad controller

Servo coxa1_servo;      //18 servos
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;


//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  //start serial 통신 시작
  Serial.begin(115200);
  
  //attach servos
  coxa1_servo.attach(COXA1_SERVO,610,2400);
  femur1_servo.attach(FEMUR1_SERVO,610,2400);
  tibia1_servo.attach(TIBIA1_SERVO,610,2400);
  coxa2_servo.attach(COXA2_SERVO,610,2400);
  femur2_servo.attach(FEMUR2_SERVO,610,2400);
  tibia2_servo.attach(TIBIA2_SERVO,610,2400);
  coxa3_servo.attach(COXA3_SERVO,610,2400);
  femur3_servo.attach(FEMUR3_SERVO,610,2400);
  tibia3_servo.attach(TIBIA3_SERVO,610,2400);
  coxa4_servo.attach(COXA4_SERVO,610,2400);
  femur4_servo.attach(FEMUR4_SERVO,610,2400);
  tibia4_servo.attach(TIBIA4_SERVO,610,2400);
  coxa5_servo.attach(COXA5_SERVO,610,2400);
  femur5_servo.attach(FEMUR5_SERVO,610,2400);
  tibia5_servo.attach(TIBIA5_SERVO,610,2400);
  coxa6_servo.attach(COXA6_SERVO,610,2400);
  femur6_servo.attach(FEMUR6_SERVO,610,2400);
  tibia6_servo.attach(TIBIA6_SERVO,610,2400);
    
  //clear offsets
  for(leg_num=0; leg_num<6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 1.0;

  //initialize mode and gait variables // 초기화  (gait == 걷는 모양)
  mode = 0;
  gait = 0;
  gait_speed = 1;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}


//***********************************************************************
// Main Program
//***********************************************************************
void loop() 
{
  //set up frame time
  currentTime = millis();
  if((  currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime; 


    //reset legs to home position when commanded
    if(reset_position == true)
    {
      for(leg_num=0; leg_num<6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false; 
    }
    
    //position legs using IK calculations - unless set all to 90 degrees mode
    if(mode < 99)
    {
      for(leg_num=0; leg_num<6; leg_num++)
        leg_IK(leg_num,current_X[leg_num]+offset_X[leg_num],current_Y[leg_num]+offset_Y[leg_num],current_Z[leg_num]+offset_Z[leg_num]);       
    }

    //reset leg lift first pass flags if needed
    if(mode != 4) 
    {
      leg1_IK_control = true; 
      leg6_IK_control = true; 
    }

    print_debug();                            //print debug data

    //process modes (mode 0 is default 'home idle' do-nothing mode)
    if(mode == 1)                             //walking mode
    {
      if(gait == 0) tripod_gait();            //walk using gait 0
    }
    if(mode == 99) set_all_90();              //set all servos to 90 degrees mode
  }
}


  if(ps2x.ButtonPressed(PSB_SELECT))      //set all servos to 90 degrees for calibration
  {
    mode = 99;   
  }


//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number,float X,float Y,float Z)
{
  //compute target femur-to-toe (L3) length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  //process only if reach is within possible range (not too long or too short!)
  if((L3 < (TIBIA_LENGTH+FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH-FEMUR_LENGTH)))  
  {
    //compute tibia angle
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3))/(2*FEMUR_LENGTH*TIBIA_LENGTH));
    theta_tibia = phi_tibia*RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia,0.0,180.0);
  
    //compute femur angle
    gamma_femur = atan2(Z,L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH))/(2*FEMUR_LENGTH*L3));
    theta_femur = (phi_femur + gamma_femur)*RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur,0.0,180.0);  

    //compute coxa angle
    theta_coxa = atan2(X,Y)*RAD_TO_DEG + COXA_CAL[leg_number];

    //output to the appropriate leg
    switch(leg_number)
    {
      case 0:
        if(leg1_IK_control == true)                       //flag for IK or manual control of leg
        {
          theta_coxa = theta_coxa + 45.0;                 //compensate for leg mounting
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa1_servo.write(int(theta_coxa)); 
          femur1_servo.write(int(theta_femur)); 
          tibia1_servo.write(int(theta_tibia)); 
        }
        break;
      case 1:
        theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa2_servo.write(int(theta_coxa)); 
        femur2_servo.write(int(theta_femur)); 
        tibia2_servo.write(int(theta_tibia)); 
        break;
      case 2:
        theta_coxa = theta_coxa + 135.0;                 //compensate for leg mounting
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa3_servo.write(int(theta_coxa)); 
        femur3_servo.write(int(theta_femur)); 
        tibia3_servo.write(int(theta_tibia)); 
        break;
      case 3:
        if(theta_coxa < 0)                                //compensate for leg mounting
          theta_coxa = theta_coxa + 225.0;                // (need to use different
        else                                              //  positive and negative offsets 
          theta_coxa = theta_coxa - 135.0;                //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa4_servo.write(int(theta_coxa)); 
        femur4_servo.write(int(theta_femur)); 
        tibia4_servo.write(int(theta_tibia)); 
        break;
      case 4:
        if(theta_coxa < 0)                                //compensate for leg mounting
          theta_coxa = theta_coxa + 270.0;                // (need to use different
        else                                              //  positive and negative offsets 
          theta_coxa = theta_coxa - 90.0;                 //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa5_servo.write(int(theta_coxa)); 
        femur5_servo.write(int(theta_femur)); 
        tibia5_servo.write(int(theta_tibia)); 
        break;
      case 5:
        if(leg6_IK_control == true)                       //flag for IK or manual control of leg
        {
          if(theta_coxa < 0)                              //compensate for leg mounting
            theta_coxa = theta_coxa + 315.0;              // (need to use different
          else                                            //  positive and negative offsets 
            theta_coxa = theta_coxa - 45.0;               //  due to atan2 results above!)
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa6_servo.write(int(theta_coxa)); 
          femur6_servo.write(int(theta_femur)); 
          tibia6_servo.write(int(theta_tibia)); 
        }
        break;
    }
  }
}


//***********************************************************************
// Tripod Gait
// Group of 3 legs move forward while the other 3 legs provide support
//***********************************************************************
void tripod_gait()
{
  //read commanded values from controller
  commandedX = map(ps2x.Analog(PSS_RY),0,255,127,-127);
  commandedY = map(ps2x.Analog(PSS_RX),0,255,-127,127);
  commandedR = map(ps2x.Analog(PSS_LX),0,255,127,-127);
    
  //if commands more than deadband then process 너무나 민감하게 하지 않기 위해 15 이상의
// 변경이 있을 경우에 해당 행동을 수행하도록 명령함
  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0); //total ticks divided into the two cases
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tripod_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tripod_case[leg_num] = 2;
          break;
        case 2:                               //move foot back (on the ground)
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tripod_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}




//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  //compute stride lengths
  strideX = 90*commandedX/127;
  strideY = 90*commandedY/127;
  strideR = 35*commandedR/127;

  //compute rotation trig
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  //set duration for normal and slow speed modes
  if(gait_speed == 0) duration = 1080; 
  else duration = 3240;
}


      
//***********************************************************************
// Body rotate with controller (xyz axes)
//***********************************************************************
void rotate_control()
{
  //compute rotation sin/cos values using controller inputs
  sinRotX = sin((map(ps2x.Analog(PSS_RX),0,255,A12DEG,-A12DEG))/1000000.0);
  cosRotX = cos((map(ps2x.Analog(PSS_RX),0,255,A12DEG,-A12DEG))/1000000.0);
  sinRotY = sin((map(ps2x.Analog(PSS_RY),0,255,A12DEG,-A12DEG))/1000000.0);
  cosRotY = cos((map(ps2x.Analog(PSS_RY),0,255,A12DEG,-A12DEG))/1000000.0);
  sinRotZ = sin((map(ps2x.Analog(PSS_LX),0,255,-A30DEG,A30DEG))/1000000.0);
  cosRotZ = cos((map(ps2x.Analog(PSS_LX),0,255,-A30DEG,A30DEG))/1000000.0);

  //compute Z direction move
  translateZ = ps2x.Analog(PSS_LY);
  if(translateZ > 127)
    translateZ = map(translateZ,128,255,0,TRAVEL); 
  else
    translateZ = map(translateZ,0,127,-3*TRAVEL,0);    

  for(int leg_num=0; leg_num<6; leg_num++)
  {
    //compute total distance from center of body to toe
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];

    //perform 3 axis rotations
    rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ + totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ + totalZ*sinRotX*sinRotZ - totalX;
    rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ + totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ + totalZ*sinRotX*cosRotZ - totalY;
    rotOffsetZ =  totalX*sinRotY         - totalY*sinRotX*cosRotY                                  + totalZ*cosRotX*cosRotY                                  - totalZ;

    // Calculate foot positions to achieve desired rotation
    current_X[leg_num] = HOME_X[leg_num] + rotOffsetX;
    current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY;
    current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;

    //lock in offsets if commanded
    if(capture_offsets == true)
    {
      offset_X[leg_num] = offset_X[leg_num] + rotOffsetX;
      offset_Y[leg_num] = offset_Y[leg_num] + rotOffsetY;
      offset_Z[leg_num] = offset_Z[leg_num] + rotOffsetZ + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  //if offsets were commanded, exit current mode
  if(capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}




//***********************************************************************
// Set all servos to 90 degrees
// Note: this is useful for calibration/alignment of the servos
// i.e: set COXA_CAL[6], FEMUR_CAL[6], and TIBIA_CAL[6] values in  
//      constants section above so all angles appear as 90 degrees
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90+COXA_CAL[0]); 
  femur1_servo.write(90+FEMUR_CAL[0]); 
  tibia1_servo.write(90+TIBIA_CAL[0]); 
  
  coxa2_servo.write(90+COXA_CAL[1]); 
  femur2_servo.write(90+FEMUR_CAL[1]); 
  tibia2_servo.write(90+TIBIA_CAL[1]); 
  
  coxa3_servo.write(90+COXA_CAL[2]); 
  femur3_servo.write(90+FEMUR_CAL[2]); 
  tibia3_servo.write(90+TIBIA_CAL[2]); 
  
  coxa4_servo.write(90+COXA_CAL[3]); 
  femur4_servo.write(90+FEMUR_CAL[3]); 
  tibia4_servo.write(90+TIBIA_CAL[3]); 
  
  coxa5_servo.write(90+COXA_CAL[4]); 
  femur5_servo.write(90+FEMUR_CAL[4]); 
  tibia5_servo.write(90+TIBIA_CAL[4]); 
  
  coxa6_servo.write(90+COXA_CAL[5]); 
  femur6_servo.write(90+FEMUR_CAL[5]); 
  tibia6_servo.write(90+TIBIA_CAL[5]);
}




//***********************************************************************
// Print Debug Data
//***********************************************************************
void print_debug()
{
  //output IK data
//  Serial.print(int(theta_coxa));
//  Serial.print(",");
//  Serial.print(int(theta_femur));
//  Serial.print(",");
//  Serial.print(int(theta_tibia));
//  Serial.print(",");

  //output XYZ coordinates for all legs
//  for(leg_num=0; leg_num<6; leg_num++)
//  {
//    Serial.print(int(current_X[leg_num]));
//    if(leg_num<5) Serial.print(",");
//  }
//  Serial.print("  ");
//  for(leg_num=0; leg_num<6; leg_num++)
//  {
//    Serial.print(int(current_Y[leg_num]));
//    if(leg_num<5) Serial.print(",");
//  }
//  Serial.print("  ");
//  for(leg_num=0; leg_num<6; leg_num++)
//  {
//    Serial.print(int(current_Z[leg_num]));
//    if(leg_num<5) Serial.print(",");
//  }
//  Serial.print("  ");

  //display elapsed frame time (ms) and battery voltage (V)
  currentTime = millis();
  Serial.print(currentTime-previousTime);
  Serial.print(",");
  Serial.print("\n");
}