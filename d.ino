#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <math.h>

ros::NodeHandle nh;

const int COXA1_SERVO = 28; //servo port definitions
const int FEMUR1_SERVO = 29;
const int TIBIA1_SERVO = 30;
const int COXA2_SERVO = 25;
const int FEMUR2_SERVO = 26;
const int TIBIA2_SERVO = 27;
const int COXA3_SERVO = 22;
const int FEMUR3_SERVO = 23;
const int TIBIA3_SERVO = 24;
const int COXA4_SERVO = 4;
const int FEMUR4_SERVO = 3;
const int TIBIA4_SERVO = 2;
const int COXA5_SERVO = 7;
const int FEMUR5_SERVO = 6;
const int TIBIA5_SERVO = 5;
const int COXA6_SERVO = 10;
const int FEMUR6_SERVO = 9;
const int TIBIA6_SERVO = 8;

const int COXA_LENGTH = 51; //leg part lengths
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;

const int TRAVEL = 30; //translate and rotate travel limit constant

const long A12DEG = 209440; //12 degrees in radians x 1,000,000
const long A30DEG = 523599; //30 degrees in radians x 1,000,000

const int FRAME_TIME_MS = 20; //frame time (20msec = 50Hz)

const float HOME_X[6] = {82.0, 0.0, -82.0, -82.0, 0.0, 82.0}; //coxa-to-toe home positions
const float HOME_Y[6] = {82.0, 116.0, 82.0, -82.0, -116.0, -82.0};
const float HOME_Z[6] = {-80.0, -80.0, -80.0, -80.0, -80.0, -80.0};

const float BODY_X[6] = {110.4, 0.0, -110.4, -110.4, 0.0, 110.4}; //body center-to-coxa servo distances
const float BODY_Y[6] = {58.4, 90.8, 58.4, -58.4, -90.8, -58.4};
const float BODY_Z[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const int COXA_CAL[6] = {2, -1, -1, -3, -2, -3}; //servo calibration constants
const int FEMUR_CAL[6] = {4, -2, 0, -1, 0, 0};
const int TIBIA_CAL[6] = {0, -3, -3, -2, -3, -1};

unsigned long currentTime; //frame timer variables
unsigned long previousTime;

int mode = 0;
int gait;
int gait_speed = 0;
int reset_position;
int capture_offsets;

int leg_num; //positioning and walking variables
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

int tripod_case[6] = {1, 2, 1, 2, 1, 2}; //for tripod gait walking

float L0, L3; //inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

Servo coxa1_servo; //18 servos
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

char dd;

int leg1_IK_control = true;
int leg6_IK_control = true;

void messageCb(const std_msgs::String &msg)
{
    if (mode < 99)
    {
        for (leg_num = 0; leg_num < 6; leg_num++)
            leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num]);
    }
    commandedR = 0;
    dd = msg.data;
    if (dd == "w")
    {
        commandedX = 0;
        commandedY = -64;
        tripod_gait();
        Serial.print("w");
    }
    else if (dd == "s")
    {
        commandedX = 0;
        commandedY = 64;
        tripod_gait();
        Serial.print("s");
    }
    else if (dd == "a")
    {
        commandedX = -64;
        commandedY = 0;
        tripod_gait();
        Serial.print("a");
    }
    else if (dd == "d")
    {
        commandedX = 64;
        commandedY = 0;
        tripod_gait();
        Serial.print("d");
    }
    else if (dd == "r")
    {
        set_all_90();
        Serial.println("set_all_90");
    }
    else if (dd == "h")
    {
        home_position();
        Serial.println("h");
    }
}

ros::Subscriber<std_msgs::String> sub("ww", &messageCb);

void setup()
{
    coxa1_servo.attach(COXA1_SERVO, 610, 2400);
    femur1_servo.attach(FEMUR1_SERVO, 610, 2400);
    tibia1_servo.attach(TIBIA1_SERVO, 610, 2400);
    coxa2_servo.attach(COXA2_SERVO, 610, 2400);
    femur2_servo.attach(FEMUR2_SERVO, 610, 2400);
    tibia2_servo.attach(TIBIA2_SERVO, 610, 2400);
    coxa3_servo.attach(COXA3_SERVO, 610, 2400);
    femur3_servo.attach(FEMUR3_SERVO, 610, 2400);
    tibia3_servo.attach(TIBIA3_SERVO, 610, 2400);
    coxa4_servo.attach(COXA4_SERVO, 610, 2400);
    femur4_servo.attach(FEMUR4_SERVO, 610, 2400);
    tibia4_servo.attach(TIBIA4_SERVO, 610, 2400);
    coxa5_servo.attach(COXA5_SERVO, 610, 2400);
    femur5_servo.attach(FEMUR5_SERVO, 610, 2400);
    tibia5_servo.attach(TIBIA5_SERVO, 610, 2400);
    coxa6_servo.attach(COXA6_SERVO, 610, 2400);
    femur6_servo.attach(FEMUR6_SERVO, 610, 2400);
    tibia6_servo.attach(TIBIA6_SERVO, 610, 2400);
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
        offset_X[leg_num] = 0.0;
        offset_Y[leg_num] = 0.0;
        offset_Z[leg_num] = 0.0;
    }
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    if (Serial.available()) //시리얼 모니터에 데이터가 입력되면
    {
        char dd;            // 입력된 데이터를 담을 변수 in_data
        dd = Serial.read(); //시리얼모니터로 입력된 데이터 in_data로 저장
        if (dd == "w")
        {
            commandedX = 0;
            commandedY = -64;
            tripod_gait();
            Serial.print("w");
        }
        else if (dd == "s")
        {
            commandedX = 0;
            commandedY = 64;
            tripod_gait();
            Serial.print("s");
        }
        else if (dd == "a")
        {
            commandedX = -64;
            commandedY = 0;
            tripod_gait();
            Serial.print("a");
        }
        else if (dd == "d")
        {
            commandedX = 64;
            commandedY = 0;
            tripod_gait();
            Serial.print("d");
        }
        else if (dd == "r")
        {
            set_all_90();
            Serial.println("set_all_90");
        }
        else if (dd == "h")
        {
            home_position();
            Serial.println("h");
        }
        nh.spinOnce();
        delay(1);
    }

void leg_IK(int leg_number, float X, float Y, float Z)
{
    //compute target femur-to-toe (L3) length
    L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
    L3 = sqrt(sq(L0) + sq(Z));

    //process only if reach is within possible range (not too long or too short!)
    if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
    {
        //compute tibia angle
        phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
        theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
        theta_tibia = constrain(theta_tibia, 0.0, 180.0);

        //compute femur angle
        gamma_femur = atan2(Z, L0);
        phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3));
        theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
        theta_femur = constrain(theta_femur, 0.0, 180.0);

        //compute coxa angle
        theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

        //output to the appropriate leg
        switch (leg_number)
        {
        case 0:
            if (leg1_IK_control == true) //flag for IK or manual control of leg
            {
                theta_coxa = theta_coxa + 45.0; //compensate for leg mounting
                theta_coxa = constrain(theta_coxa, 0.0, 180.0);
                coxa1_servo.write(int(theta_coxa));
                femur1_servo.write(int(theta_femur));
                tibia1_servo.write(int(theta_tibia));
            }
            break;
        case 1:
            theta_coxa = theta_coxa + 90.0; //compensate for leg mounting
            theta_coxa = constrain(theta_coxa, 0.0, 180.0);
            coxa2_servo.write(int(theta_coxa));
            femur2_servo.write(int(theta_femur));
            tibia2_servo.write(int(theta_tibia));
            break;
        case 2:
            theta_coxa = theta_coxa + 135.0; //compensate for leg mounting
            theta_coxa = constrain(theta_coxa, 0.0, 180.0);
            coxa3_servo.write(int(theta_coxa));
            femur3_servo.write(int(theta_femur));
            tibia3_servo.write(int(theta_tibia));
            break;
        case 3:
            if (theta_coxa < 0)                  //compensate for leg mounting
                theta_coxa = theta_coxa + 225.0; // (need to use different
            else                                 //  positive and negative offsets
                theta_coxa = theta_coxa - 135.0; //  due to atan2 results above!)
            theta_coxa = constrain(theta_coxa, 0.0, 180.0);
            coxa4_servo.write(int(theta_coxa));
            femur4_servo.write(int(theta_femur));
            tibia4_servo.write(int(theta_tibia));
            break;
        case 4:
            if (theta_coxa < 0)                  //compensate for leg mounting
                theta_coxa = theta_coxa + 270.0; // (need to use different
            else                                 //  positive and negative offsets
                theta_coxa = theta_coxa - 90.0;  //  due to atan2 results above!)
            theta_coxa = constrain(theta_coxa, 0.0, 180.0);
            coxa5_servo.write(int(theta_coxa));
            femur5_servo.write(int(theta_femur));
            tibia5_servo.write(int(theta_tibia));
            break;
        case 5:
            if (leg6_IK_control == true) //flag for IK or manual control of leg
            {
                if (theta_coxa < 0)                  //compensate for leg mounting
                    theta_coxa = theta_coxa + 315.0; // (need to use different
                else                                 //  positive and negative offsets
                    theta_coxa = theta_coxa - 45.0;  //  due to atan2 results above!)
                theta_coxa = constrain(theta_coxa, 0.0, 180.0);
                coxa6_servo.write(int(theta_coxa));
                femur6_servo.write(int(theta_femur));
                tibia6_servo.write(int(theta_tibia));
            }
            break;
        }
    }
}

void tripod_gait()
{
    if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
    {
        compute_strides();
        numTicks = round(duration / FRAME_TIME_MS / 2.0); //total ticks divided into the two cases
        for (leg_num = 0; leg_num < 6; leg_num++)
        {
            compute_amplitudes();
            switch (tripod_case[leg_num])
            {
            case 1: //move foot forward (raise and lower)
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
                current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
                if (tick >= numTicks - 1)
                    tripod_case[leg_num] = 2;
                break;
            case 2: //move foot back (on the ground)
                current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(M_PI * tick / numTicks);
                current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(M_PI * tick / numTicks);
                current_Z[leg_num] = HOME_Z[leg_num];
                if (tick >= numTicks - 1)
                    tripod_case[leg_num] = 1;
                break;
            }
        }
        //increment tick
        if (tick < numTicks - 1)
            tick++;
        else
            tick = 0;
    }
}

void compute_amplitudes()
{
    //compute total distance from center of body to toe
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

    //compute rotational offset
    rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
    rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;

    //compute X and Y amplitude and constrain to prevent legs from crashing into each other
    amplitudeX = ((strideX + rotOffsetX) / 2.0);
    amplitudeY = ((strideY + rotOffsetY) / 2.0);
    amplitudeX = constrain(amplitudeX, -50, 50);
    amplitudeY = constrain(amplitudeY, -50, 50);

    //compute Z amplitude
    if (abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
        amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
    else
        amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}

void compute_strides()
{
    //compute stride lengths
    strideX = 90 * commandedX / 127;
    strideY = 90 * commandedY / 127;
    strideR = 35 * commandedR / 127;

    //compute rotation trig
    sinRotZ = sin(radians(strideR));
    cosRotZ = cos(radians(strideR));

    //set duration for normal and slow speed modes
    if (gait_speed == 0)
        duration = 1080;
    else
        duration = 3240;
}

void set_all_90()
{
    coxa1_servo.write(90 + COXA_CAL[0]);
    femur1_servo.write(90 + FEMUR_CAL[0]);
    tibia1_servo.write(90 + TIBIA_CAL[0]);

    coxa2_servo.write(90 + COXA_CAL[1]);
    femur2_servo.write(90 + FEMUR_CAL[1]);
    tibia2_servo.write(90 + TIBIA_CAL[1]);

    coxa3_servo.write(90 + COXA_CAL[2]);
    femur3_servo.write(90 + FEMUR_CAL[2]);
    tibia3_servo.write(90 + TIBIA_CAL[2]);

    coxa4_servo.write(90 + COXA_CAL[3]);
    femur4_servo.write(90 + FEMUR_CAL[3]);
    tibia4_servo.write(90 + TIBIA_CAL[3]);

    coxa5_servo.write(90 + COXA_CAL[4]);
    femur5_servo.write(90 + FEMUR_CAL[4]);
    tibia5_servo.write(90 + TIBIA_CAL[4]);

    coxa6_servo.write(90 + COXA_CAL[5]);
    femur6_servo.write(90 + FEMUR_CAL[5]);
    tibia6_servo.write(90 + TIBIA_CAL[5]);
}

void home_position()
{
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
    }
}