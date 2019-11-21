/*------------------------------------------------------------------
Smart Robot - Line Follower 
==> Basic movement based on Nano Mouse Robot, developed by Michael Backus (http://www.akrobotnerd.com/ )
==> Line follow based on http://samvrit.tk/tutorials/pid-control-arduino-line-follower-robot/?ckattempt=1

Marcelo Jose Rovai - 06 April, 2016 - Visit: http://mjrobot.org
-------------------------------------------------------------------*/

//-------------------------------------------------------------
/* General stuff */

//--------------------------------------------------------
void calculatePID()
{
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;
}


//-------------------------------------------------------------
/* Motor stuff */
void motorStop()
{
    leftServo.writeMicroseconds(1500);
    rightServo.writeMicroseconds(1500);
    delay(200);
}

//---------------------------------------------
void motorForward()
{
    leftServo.writeMicroseconds(1500 - power);
    rightServo.writeMicroseconds(1500 + power * adj);
}

//---------------------------------------------
void motorBackward()
{
    leftServo.writeMicroseconds(1500 + power);
    rightServo.writeMicroseconds(1500 - power);
}

//------------------------------------------------
void motorTurn(int direction, int degrees)
{
    leftServo.writeMicroseconds(1500 - iniMotorPower * direction);
    rightServo.writeMicroseconds(1500 - iniMotorPower * direction);
    delay(round(adjTurn * degrees + 1));
    motorStop();
}

//---------------------------------------------------
void motorPIDcontrol()
{

    int leftMotorSpeed = 1500 - iniMotorPower - PIDvalue;
    int rightMotorSpeed = 1500 + iniMotorPower * adj - PIDvalue;

    // The motor speed should not exceed the max PWM value
    constrain(leftMotorSpeed, 1000, 2000);
    constrain(rightMotorSpeed, 1000, 2000);

    leftServo.writeMicroseconds(leftMotorSpeed);
    rightServo.writeMicroseconds(rightMotorSpeed);

    //Serial.print (PIDvalue);
    //Serial.print (" ==> Left, Right:  ");
    //Serial.print (leftMotorSpeed);
    //Serial.print ("   ");
    //Serial.println (rightMotorSpeed);
}

//-----------------------------
void drivePolygon(unsigned int time, int sides) // for motor test only
{
    for (int i = 0; i < sides; i++)
    {
        motorFwTime(time);
        motorTurn(RIGHT, 360 / sides);
    }
}

//-------------------------------------------------------------
/* definations */
int mode = 0;

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2

const int power = 500;
const int iniMotorPower = 250;
const int adj = 1;
float adjTurn = 8;

const int ledPin = 13;
const int buttonPin = 9;

// LFSensor more to the Left is "0"
const int lineFollowSensor0 = 12;
const int lineFollowSensor1 = 18;
const int lineFollowSensor2 = 17;
const int lineFollowSensor3 = 16;
const int lineFollowSensor4 = 19;

int LFSensor[5] = {0, 0, 0, 0, 0};

// PID controller
float Kp = 50;
float Ki = 0;
float Kd = 0;

float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;

#define RIGHT 1
#define LEFT -1

//-------------------------------------------------------------
/* read line sensors values 

Sensor Array 	Error Value
0 0 0 0 1	 4              
0 0 0 1 1	 3              
0 0 0 1 0	 2              
0 0 1 1 0	 1              
0 0 1 0 0	 0              
0 1 1 0 0	-1              
0 1 0 0 0	-2              
1 1 0 0 0	-3              
1 0 0 0 0	-4              

1 1 1 1 1        0 Robot found continuous line : STOPPED
0 0 0 0 0        0 Robot found no line: turn 180o
*/
void readLFSsensors()
{
    LFSensor[0] = digitalRead(lineFollowSensor0);
    LFSensor[1] = digitalRead(lineFollowSensor1);
    LFSensor[2] = digitalRead(lineFollowSensor2);
    LFSensor[3] = digitalRead(lineFollowSensor3);
    LFSensor[4] = digitalRead(lineFollowSensor4);

    if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 1))
    {
        mode = FOLLOWING_LINE;
        error = 4;
    }
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
    {
        mode = FOLLOWING_LINE;
        error = 3;
    }
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 0))
    {
        mode = FOLLOWING_LINE;
        error = 2;
    }
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 0))
    {
        mode = FOLLOWING_LINE;
        error = 1;
    }
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
    {
        mode = FOLLOWING_LINE;
        error = 0;
    }
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
    {
        mode = FOLLOWING_LINE;
        error = -1;
    }
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
    {
        mode = FOLLOWING_LINE;
        error = -2;
    }
    else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
    {
        mode = FOLLOWING_LINE;
        error = -3;
    }
    else if ((LFSensor[0] == 1) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
    {
        mode = FOLLOWING_LINE;
        error = -4;
    }
    else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
    {
        mode = STOPPED;
        error = 0;
    }
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
    {
        mode = NO_LINE;
        error = 0;
    }
}

//---------------------------------------------
void setup()
{

    Serial.begin(9600);

    // line follow sensors
    pinMode(lineFollowSensor0, INPUT);
    pinMode(lineFollowSensor1, INPUT);
    pinMode(lineFollowSensor2, INPUT);
    pinMode(lineFollowSensor3, INPUT);
    pinMode(lineFollowSensor4, INPUT);
}

void loop()
{
    readLFSsensors();

    switch (mode)
    {
    case STOPPED:
        motorStop();
        previousError = error;
        break;

    case NO_LINE:
        motorStop();
        motorTurn(LEFT, 180);
        previousError = 0;
        break;

    case FOLLOWING_LINE:
        calculatePID();
        motorPIDcontrol();
        break;
    }
}