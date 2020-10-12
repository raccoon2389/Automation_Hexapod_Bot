#include <Servo.h>
#include <math.h>


const int COXA1_SERVO  = 7;          //servo port definitions
const int FEMUR1_SERVO = 8;
const int TIBIA1_SERVO = 9;
const int COXA2_SERVO  = 4;
const int FEMUR2_SERVO = 5;
const int TIBIA2_SERVO = 6;
const int COXA3_SERVO  = 1;
const int FEMUR3_SERVO = 2;
const int TIBIA3_SERVO = 3;
const int COXA4_SERVO  = 51;
const int FEMUR4_SERVO = 52;
const int TIBIA4_SERVO = 53;
const int COXA5_SERVO  = 48;
const int FEMUR5_SERVO = 49;
const int TIBIA5_SERVO = 50;
const int COXA6_SERVO  = 45;
const int FEMUR6_SERVO = 46;
const int TIBIA6_SERVO = 47;

const int COXA_LENGTH = 51;           //leg part lengths
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;

const int TRAVEL = 30;                //translate and rotate travel limit constant
                                      
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

unsigned long currentTime;            //frame timer variables
unsigned long previousTime;

int mode = 0;
int gait;
int gait_speed = 0;
int reset_position;
int capture_offsets;


int leg_num;                          //positioning and walking variables
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY;
int commandedR = 0;
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

float L0, L3;                         //inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

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

String dd;

int leg1_IK_control, leg6_IK_control;


ros::Subscriber<std_msgs::String> sub("ww", &messageCb);

void setup()
{
  Serial.begin(57600);
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

  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  coxa1_servo.write(90);
  femur1_servo.write(90);
  tibia1_servo.write(90);
  coxa2_servo.write(90);
  femur2_servo.write(90);
  tibia2_servo.write(90);
  coxa3_servo.write(90);
  femur3_servo.write(90);
  tibia3_servo.write(90);
  coxa4_servo.write(90);
  femur4_servo.write(90);
  tibia4_servo.write(90);
  coxa5_servo.write(90);
  femur5_servo.write(90);
  tibia5_servo.write(90);
  coxa6_servo.write(90);
  femur6_servo.write(90);
  tibia6_servo.write(90);
  delay(1);
}
