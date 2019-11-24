#include <math.h>

// -----------------------------------------------
// -----------------------------------------------
// Global Definations
// -----------------------------------------------
// -----------------------------------------------
int mode = 0;

#define ALL_BLACK 0
#define FOLLOWING_LINE 1
#define ALL_WHITE 2

/*
const int initial_MotorPower_for_go_straight = 130;
const int initial_MotorPower_for_turning = 100;
*/
const int initial_MotorPower_for_go_straight = 200; //220;
const int initial_MotorPower_for_turning = 30;

// PID controller
float Kp = 0.17;
float Ki = 0;
float Kd = 0.001;

float error = 0, P_value = 0, I_value = 0, D_value = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;

// -----------------------------------------------
// -----------------------------------------------
// Basic motor control function
// -----------------------------------------------
// -----------------------------------------------
int Left_motor_go = 8;   //左电机前进 AIN1
int Left_motor_back = 7; //左电机后退 AIN2

int Right_motor_go = 2;   //右电机前进 BIN1
int Right_motor_back = 4; //右电机后退 BIN2

int Left_motor_pwm = 6;  //左电机控速 PWMA
int Right_motor_pwm = 5; //右电机控速 PWMB

int key = A0; //定义按键为arduino的模拟口A0

void go_straight(int left_speed, int right_speed)
{
    //左电机前进
    digitalWrite(Left_motor_go, LOW);    //左电机前进使能
    digitalWrite(Left_motor_back, HIGH); //左电机后退禁止
    analogWrite(Left_motor_pwm, left_speed);

    //右电机前进
    digitalWrite(Right_motor_go, LOW);    //右电机前进使能
    digitalWrite(Right_motor_back, HIGH); //右电机后退禁止
    analogWrite(Right_motor_pwm, right_speed);
}

void go_back(int left_speed, int right_speed)
{
    //左电机前进
    digitalWrite(Left_motor_go, HIGH);
    digitalWrite(Left_motor_back, LOW);
    analogWrite(Left_motor_pwm, left_speed);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);
    digitalWrite(Right_motor_back, LOW);
    analogWrite(Right_motor_pwm, right_speed);
}

void stop(int time)
{
    digitalWrite(Left_motor_go, LOW);
    digitalWrite(Left_motor_back, LOW);
    digitalWrite(Right_motor_go, LOW);
    digitalWrite(Right_motor_back, LOW);

    delay(time);
}

void left_rotate(int left_speed, int right_speed)
{
    //左电机后退
    digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
    digitalWrite(Left_motor_back, HIGH); //左电机后退使能
    analogWrite(Left_motor_pwm, left_speed);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, right_speed);
}

void right_rotate(int left_speed, int right_speed)
{
    //左电机前进
    digitalWrite(Left_motor_go, HIGH);  //左电机前进使能
    digitalWrite(Left_motor_back, LOW); //左电机后退禁止
    analogWrite(Left_motor_pwm, left_speed);

    //右电机后退
    digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
    digitalWrite(Right_motor_back, HIGH); //右电机后退使能
    analogWrite(Right_motor_pwm, right_speed);
}

void left(int speed)
{
    digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
    digitalWrite(Left_motor_back, HIGH); //左电机后退禁止
    analogWrite(Left_motor_pwm, speed);

    digitalWrite(Right_motor_go, LOW);    //右电机前进使能
    digitalWrite(Right_motor_back, HIGH); //右电机后退禁止
    analogWrite(Right_motor_pwm, speed / 4);
}

void right(int speed)
{
    digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
    digitalWrite(Left_motor_back, HIGH); //左电机后退禁止
    analogWrite(Left_motor_pwm, speed / 4);

    digitalWrite(Right_motor_go, LOW);    //右电机前进使能
    digitalWrite(Right_motor_back, HIGH); //右电机后退禁止
    analogWrite(Right_motor_pwm, speed);
}

void wait_for_key_action()
{
    while (digitalRead(key))
        ;                     //当按键没有被按下一直循环
    while (!digitalRead(key)) //当按键被按下时
    {
        delay(10);                   //延时10ms
        if (digitalRead(key) == LOW) //第二次判断按键是否被按下
        {
            delay(100);
            while (!digitalRead(key))
                ; //判断按键是否被松开
        }
    }
}

// -----------------------------------------------
// -----------------------------------------------
// PID stuff
// -----------------------------------------------
// -----------------------------------------------

//循迹红外引脚定义
const int A_pin = 13;
const int B_pin = 3;
const int C_pin = 10;
const int D_pin = 11;
const int E_pin = 9;
const int F_pin = 12;

//定义各个循迹红外引脚采集的数据的变量
int A;
int B;
int C;
int D;
int E;
int F;

void update_ABCDEF()
{
    A = digitalRead(A_pin);
    B = digitalRead(B_pin);
    C = digitalRead(C_pin);
    D = digitalRead(D_pin);
    E = digitalRead(E_pin);
    F = digitalRead(F_pin);
}

/* read line sensors values 

black = 1;
white = 0;

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

1 1 1 1 1        0 Robot found continuous line : ALL_BLACK
0 0 0 0 0        0 Robot found no line: turn 180o

*/

/* read line sensors values 

black = 1;
white = 0;

Sensor Array 	Error Value
0 0 (0&0) 0 1	 4              
0 0 (0&0) 1 1	 3              
0 0 (0&0) 1 0	 2              
0 0 (0&1) 1 0	 1              
0 0 (0&1) 0 0    0.5

0 0 (1&1) 0 0	 0              

0 0 (1&0) 0 0   -0.5
0 1 (1&0) 0 0	-1              
0 1 (0&0) 0 0	-2              
1 1 (0&0) 0 0	-3              
1 0 (0&0) 0 0	-4              

1 1 1 1 1        0 Robot found continuous line : ALL_BLACK
0 0 0 0 0        0 Robot found no line: turn 180o

*/

void update_errors()
{
    if ((A == 0) && (B == 0) && ((C == 0) && (D == 0)) && (E == 0) && (F == 1))
    {
        mode = FOLLOWING_LINE;
        error = 0.4;
    }
    else if ((A == 0) && (B == 0) && ((C == 0) && (D == 0)) && (E == 1) && (F == 1))
    {
        mode = FOLLOWING_LINE;
        error = 0.3;
    }
    else if ((A == 0) && (B == 0) && ((C == 0) && (D == 0)) && (E == 1) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = 0.2;
    }
    else if ((A == 0) && (B == 0) && ((C == 0) && (D == 1)) && (E == 1) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = 0.1;
    }
    else if ((A == 0) && (B == 0) && ((C == 0) && (D == 1)) && (E == 0) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = 0.01;
    }
    else if ((A == 0) && (B == 0) && ((C == 1) && (D == 1)) && (E == 0) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = 0;
    }
    else if ((A == 0) && (B == 0) && ((C == 1) && (D == 0)) && (E == 0) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = -0.01;
    }
    else if ((A == 0) && (B == 1) && ((C == 1) && (D == 0)) && (E == 0) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = -0.1;
    }
    else if ((A == 0) && (B == 1) && ((C == 0) && (D == 0)) && (E == 0) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = -0.2;
    }
    else if ((A == 1) && (B == 1) && ((C == 0) && (D == 0)) && (E == 0) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = -0.3;
    }
    else if ((A == 1) && (B == 0) && ((C == 0) && (D == 0)) && (E == 0) && (F == 0))
    {
        mode = FOLLOWING_LINE;
        error = -4;
    }
    else if ((A == 1) && (B == 1) && ((C == 1) && (D == 1)) && (E == 1) && (F == 1))
    {
        mode = ALL_BLACK;
        error = 0;
    }
    else if ((A == 0) && (B == 0) && ((C == 0) && (D == 0)) && (C == 0) && (D == 0))
    {
        mode = ALL_WHITE;
        error = 0;
    }
}

void print_information()
{
    Serial.print("A: ");
    Serial.print(A);
    Serial.print("\n");

    Serial.print("B: ");
    Serial.print(B);
    Serial.print("\n");

    Serial.print("C: ");
    Serial.print(C);
    Serial.print("\n");

    Serial.print("D: ");
    Serial.print(D);
    Serial.print("\n");

    Serial.print("E: ");
    Serial.print(E);
    Serial.print("\n");

    Serial.print("F: ");
    Serial.print(F);
    Serial.print("\n");

    Serial.print("\n");
    Serial.print("---------------------------------------");
    Serial.print("\n");
    Serial.print("\n");

    /*
    Serial.print("left_Distance:"); //输出距离（单位：厘米）
    left_distance = get_left_distance();
    Serial.print(left_distance); //显示距离
    Serial.println("cm");        //显示

    Serial.println(""); //显示

    Serial.print("right_Distance:"); //输出距离（单位：厘米）
    right_distance = get_right_distance();
    Serial.print(right_distance); //显示距离
    Serial.println("cm");         //显示
    */
}

void calculatePID()
{
    P_value = error;
    I_value = I_value + error;
    D_value = error - previousError;
    PIDvalue = (Kp * P_value) + (Ki * I_value) + (Kd * D_value);
    previousError = error;
}

int MotorSpeed = 0;
void motorPIDcontrol()
{
    if (round(PIDvalue * 10) == 0)
    {
        MotorSpeed = initial_MotorPower_for_go_straight + abs(PIDvalue);
        constrain(MotorSpeed, 0, 255);

        go_straight(MotorSpeed, MotorSpeed);
    }
    else if (PIDvalue < 0)
    {
        MotorSpeed = initial_MotorPower_for_turning + abs(PIDvalue);
        constrain(MotorSpeed, 0, 255);

        left_rotate(MotorSpeed, MotorSpeed);
    }
    else if (PIDvalue > 0)
    {
        MotorSpeed = initial_MotorPower_for_turning + abs(PIDvalue);
        constrain(MotorSpeed, 0, 255);

        right_rotate(MotorSpeed, MotorSpeed);
    }
}

// -----------------------------------------------
// -----------------------------------------------
// Timer functions
// -----------------------------------------------
// -----------------------------------------------

unsigned int get_live_seconds()
{
    return millis() / 1000;
}

/*
int full_white_report_in_a_second = 0;
int seconds_for_full_white = 0;
int temp_seconds_for_full_white = 0;
int a_full_white_report()
{
    full_white_report_in_a_second++;

    temp_seconds_for_full_white = get_live_seconds();
    if (temp_seconds_for_full_white > seconds_for_full_white)
    {
        seconds_for_full_white = temp_seconds_for_full_white;
        full_white_report_in_a_second = 0;
    }

    if (full_white_report_in_a_second > 200)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
*/
int seconds = 0;
int temp_seconds = 0;

unsigned int general_report_in_a_second = 0;
unsigned int full_white_report_in_a_second = 0;

int general_report()
{
    general_report_in_a_second++;

    temp_seconds = get_live_seconds();
    if (temp_seconds > seconds)
    {
        seconds = temp_seconds;
        general_report_in_a_second = 0;
        full_white_report_in_a_second = 0;
    }
}

unsigned int a_full_white_report()
{
    full_white_report_in_a_second++;

    temp_seconds = get_live_seconds();
    if (temp_seconds > seconds)
    {
        seconds = temp_seconds;
        full_white_report_in_a_second = 0;
    }

    //if (full_white_report_in_a_second > 200)
    //if (full_white_report_in_a_second > 10 && general_report_in_a_second > 10 && (full_white_report_in_a_second / general_report_in_a_second) > 0.8)
    if (general_report_in_a_second > 30 && (full_white_report_in_a_second / general_report_in_a_second) > 0.8)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// -----------------------------------------------
// -----------------------------------------------
// Ultrasonic sensor functions
// -----------------------------------------------
// -----------------------------------------------

const int Left_Echo = A2;
const int Left_Triger = A1;
const int Right_Echo = A4;
const int Right_Triger = A3;

int left_distance;
int right_distance;

void init_ultrasonic_sensor()
{
    pinMode(Left_Echo, INPUT);     // 定义超声波输入脚
    pinMode(Left_Triger, OUTPUT);  // 定义超声波输出脚
    pinMode(Right_Echo, INPUT);    // 定义超声波输入脚
    pinMode(Right_Triger, OUTPUT); // 定义超声波输出脚
}

int get_left_distance() // 量出前方距离
{
    digitalWrite(Left_Triger, LOW); // 给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(Left_Triger, HIGH); // 给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(15);
    digitalWrite(Left_Triger, LOW);             // 持续给触发脚低电
    float Fdistance = pulseIn(Left_Echo, HIGH); // 读取高电平时间(单位：微秒)
    Fdistance = Fdistance / 58;                 //为什么除以58等于厘米，  Y米=（X秒*344）/2
    // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
    return Fdistance;
}

int get_right_distance() // 量出前方距离
{
    digitalWrite(Right_Triger, LOW); // 给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(Right_Triger, HIGH); // 给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(15);
    digitalWrite(Right_Triger, LOW);             // 持续给触发脚低电
    float Fdistance = pulseIn(Right_Echo, HIGH); // 读取高电平时间(单位：微秒)
    Fdistance = Fdistance / 58;                  //为什么除以58等于厘米，  Y米=（X秒*344）/2
    // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
    return Fdistance;
}

void update_distance()
{
    left_distance = get_left_distance();
    right_distance = get_right_distance();
    constrain(left_distance, 0, 100);
    constrain(right_distance, 0, 100);
}

int check_if_we_are_in_tunnel()
{
    if (left_distance < 50 && right_distance < 50)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void adjust_its_position_to_the_center()
{
    int middle_value = ((left_distance + right_distance) / 2);
    if (left_distance < middle_value)
    {
        right(initial_MotorPower_for_go_straight / 2);
        //delay(500);
    }
    else if (right_distance < middle_value)
    {
        left(initial_MotorPower_for_go_straight / 2);
        //delay(500);
    }
}

void motorPIDcontrol_with_ultrasonic_sensor()
{
    if (round(PIDvalue * 10) == 0)
    {
        MotorSpeed = initial_MotorPower_for_go_straight + abs(PIDvalue);
        constrain(MotorSpeed, 0, 255);

        go_straight(MotorSpeed, MotorSpeed);
    }
    else if (PIDvalue < 0)
    {
        MotorSpeed = initial_MotorPower_for_go_straight / 2 + abs(PIDvalue);
        constrain(MotorSpeed, 0, 255);

        left(MotorSpeed);
    }
    else if (PIDvalue > 0)
    {
        MotorSpeed = initial_MotorPower_for_go_straight / 2 + abs(PIDvalue);
        constrain(MotorSpeed, 0, 255);

        right(MotorSpeed);
    }
}

void adjust_its_position_to_the_center_by_PID()
{
    int middle_value = ((left_distance + right_distance) / 2);

    if (left_distance < middle_value)
    {
        error = 0.5;
        //right(initial_MotorPower_for_go_straight / 2);
    }
    else if (right_distance < middle_value)
    {
        error = -0.5;
        //left(initial_MotorPower_for_go_straight / 2);
    }

    calculatePID();
    motorPIDcontrol_with_ultrasonic_sensor();
}

void setup()
{
    Serial.begin(9600); // open the serial port at 9600 bps:

    init_ultrasonic_sensor();

    //初始化电机驱动IO口为输出方式
    pinMode(Left_motor_go, OUTPUT);
    pinMode(Left_motor_back, OUTPUT);
    pinMode(Right_motor_go, OUTPUT);
    pinMode(Right_motor_back, OUTPUT);

    //定义按键接口为输入接口
    pinMode(key, INPUT);

    //定义四路循迹红外传感器为输入接口
    pinMode(A_pin, INPUT);
    pinMode(B_pin, INPUT);
    pinMode(C_pin, INPUT);
    pinMode(D_pin, INPUT);
    pinMode(E_pin, INPUT);
    pinMode(F_pin, INPUT);

    //按键初始化为高电平
    digitalWrite(key, HIGH);

    //四路循迹红外传感器初始化为高电平
    digitalWrite(A_pin, HIGH);
    digitalWrite(B_pin, HIGH);
    digitalWrite(C_pin, HIGH);
    digitalWrite(D_pin, HIGH);
    digitalWrite(E_pin, HIGH);
    digitalWrite(F_pin, HIGH);

    //调用按键扫描函数
    //wait_for_key_action();
}

int full_white_count = 0;
void loop()
{
    update_ABCDEF();
    update_errors();

    switch (mode)
    {
    case ALL_BLACK:
        //stop(1000);
        previousError = error;
        break;

    case ALL_WHITE:
        stop(1);

        general_report();
        if (a_full_white_report() == 1)
        {
            previousError = 0;
            error = 0;

            // when it goes to the second full white, stop ferever
            stop(100);
            full_white_count++;
            if (full_white_count >= 2)
            {
                unsigned int beginning = get_live_seconds();
                unsigned int current = beginning;
                while (1) {
                    go_straight(initial_MotorPower_for_turning*2, initial_MotorPower_for_turning*2);

                    current = get_live_seconds();
                    if ((current - beginning) >= 1) {
                        break;
                    }
                }
                while (1)
                {
                    stop(2000);
                }
            }

            // when it goes to the first full white, start the ultrasonic_sensor
            unsigned int beginning = get_live_seconds();
            unsigned int current = beginning;
            while (1)
            {
                update_distance();
                adjust_its_position_to_the_center_by_PID();

                current = get_live_seconds();
                if ((current - beginning) >= 6)
                {
                    update_ABCDEF();
                    if (C == 1 || D == 1) // adjust its position until meets black line
                    {
                        stop(100);
                        break;
                    }
                }
            }
        }

        error = 0;
        previousError = 0;
        break;

    case FOLLOWING_LINE:
        general_report();

        calculatePID();
        motorPIDcontrol();
        break;
    }
}