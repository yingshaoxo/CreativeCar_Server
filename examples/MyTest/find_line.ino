/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         tracking.c
* @author       Danny
* @version      V1.0
* @date         2017.07.26
* @brief        巡线实验
* @details
* @par History  见如下说明
*
*/
int Left_motor_go = 8;   //左电机前进 AIN1
int Left_motor_back = 7; //左电机后退 AIN2

int Right_motor_go = 2;   //右电机前进 BIN1
int Right_motor_back = 4; //右电机后退 BIN2

int Left_motor_pwm = 6;  //左电机控速 PWMA
int Right_motor_pwm = 5; //右电机控速 PWMB

int key = A0; //定义按键为arduino的模拟口A0

//循迹红外引脚定义
//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      A2                  A1                  A3                   A4
const int TrackSensorLeftPin1 = A2;  //定义左边第一个循迹红外传感器引脚为A2
const int TrackSensorLeftPin2 = A1;  //定义左边第二个循迹红外传感器引脚为A1
const int TrackSensorRightPin1 = A3; //定义右边第一个循迹红外传感器引脚为A3
const int TrackSensorRightPin2 = A4; //定义右边第二个循迹红外传感器引脚为A4

//定义各个循迹红外引脚采集的数据的变量
int A;
int B;
int C;
int D;

const int black = 1;
const int white = 0;

/**
* Function       setup
* @author        Danny
* @date          2017.07.25
* @brief         初始化配置
* @param[in]     void
* @retval        void
* @par History   无
*/
void setup()
{
    //初始化电机驱动IO口为输出方式
    pinMode(Left_motor_go, OUTPUT);
    pinMode(Left_motor_back, OUTPUT);
    pinMode(Right_motor_go, OUTPUT);
    pinMode(Right_motor_back, OUTPUT);

    //定义按键接口为输入接口
    pinMode(key, INPUT);

    //定义四路循迹红外传感器为输入接口
    pinMode(TrackSensorLeftPin1, INPUT);
    pinMode(TrackSensorLeftPin2, INPUT);
    pinMode(TrackSensorRightPin1, INPUT);
    pinMode(TrackSensorRightPin2, INPUT);

    //按键初始化为高电平
    digitalWrite(key, HIGH);

    //四路循迹红外传感器初始化为高电平
    digitalWrite(TrackSensorLeftPin1, HIGH);
    digitalWrite(TrackSensorLeftPin2, HIGH);
    digitalWrite(TrackSensorRightPin1, HIGH);
    digitalWrite(TrackSensorRightPin2, HIGH);

    //调用按键扫描函数
    Serial.begin(9600); // open the serial port at 9600 bps:
    //key_scan();
}

/**
* Function       run
* @author        Danny
* @date          2017.07.26
* @brief         小车前进
* @param[in1]    left_speed:左轮速度
* @param[in2]    right_speed:右轮速度
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int left_speed, int right_speed)
{
    //左电机前进
    digitalWrite(Left_motor_go, LOW);  //左电机前进使能
    digitalWrite(Left_motor_back, HIGH); //左电机后退禁止
    analogWrite(Left_motor_pwm, left_speed);

    //右电机前进
    digitalWrite(Right_motor_go, LOW);  //右电机前进使能
    digitalWrite(Right_motor_back, HIGH); //右电机后退禁止
    analogWrite(Right_motor_pwm, right_speed);
}

/**
* Function       brake
* @author        Danny
* @date          2017.07.25
* @brief         小车刹车
* @param[in]     time:延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void brake(int time)
{
    digitalWrite(Left_motor_go, LOW);
    digitalWrite(Left_motor_back, LOW);
    digitalWrite(Right_motor_go, LOW);
    digitalWrite(Right_motor_back, LOW);

    delay(time * 100);
}

/**
* Function       left
* @author        Danny
* @date          2017.07.25
* @brief         小车左转(左轮不动，右轮前进)
* @param[in1]    left_speed:左轮速度
* @param[in2]    right_speed:右轮速度
* @param[out]    void
* @retval        void
* @par History   无
*/
void left(int left_speed, int right_speed)
{
    //左电机停止
    digitalWrite(Left_motor_go, LOW);   //左电机前进禁止
    digitalWrite(Left_motor_back, LOW); //左电机后退禁止
    analogWrite(Left_motor_pwm, left_speed);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, right_speed);
}

/**
* Function       right
* @author        Danny
* @date          2017.07.25
* @brief         小车右转(右轮不动，左轮前进)
* @param[in1]    left_speed:左轮速度
* @param[in2]    right_speed:右轮速度
* @param[out]    void
* @retval        void
* @par History   无
*/
void right(int left_speed, int right_speed)
{
    //左电机前进
    digitalWrite(Left_motor_go, HIGH);  //左电机前进使能
    digitalWrite(Left_motor_back, LOW); //左电机后退禁止
    analogWrite(Left_motor_pwm, left_speed);

    //右电机停止
    digitalWrite(Right_motor_go, LOW);   //右电机前进禁止
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, right_speed);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.07.25
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in1]    left_speed:左轮速度
* @param[in2]    right_speed:右轮速度
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left(int left_speed, int right_speed)
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

/**
* Function       spin_right
* @author        Danny
* @date          2017.07.25
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in1]    left_speed:左轮速度
* @param[in2]    right_speed:右轮速度
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_right(int left_speed, int right_speed)
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

/**
* Function       back
* @author        Danny
* @date          2017.07.25
* @brief         小车后退
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void back(int time)
{
    //左电机后退
    digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
    digitalWrite(Left_motor_back, HIGH); //左电机后退使能
    analogWrite(Left_motor_pwm, 40);

    //右电机后退
    digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
    digitalWrite(Right_motor_back, HIGH); //右电机后退使能
    analogWrite(Right_motor_pwm, 40);

    delay(time);
}

/**
* Function       key_scan
* @author        Danny
* @date          2017.07.25
* @brief         按键检测(包含软件按键去抖)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void key_scan()
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

/**
* Function       loop
* @author        Danny
* @date          2017.07.25
* @brief         先调用setup初始化配置里面的按键扫描函数，
*                循迹模式开启
* @param[in]     void
* @retval        void
* @par History   无
*/
void loop()
{
    //检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
    //未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
    A = digitalRead(TrackSensorLeftPin1);
    B = digitalRead(TrackSensorRightPin2);
    C = digitalRead(TrackSensorRightPin1);
    D = digitalRead(TrackSensorLeftPin2);

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

    Serial.print("\n"); 
    Serial.print("---------------------------------------"); 
    Serial.print("\n"); 
    Serial.print("\n"); 

    delay(500);

/*
    if (A == white && D == white)
    {
        run(100, 100);
        Serial.println("go starght");
        delay(80);
    } else if (A == white && D == white && B == black) {
        run(100, 100);
        Serial.println("go starght");
        delay(80);
    } else if (A == white && D == white && C == black) {
        run(100, 100);
        Serial.println("go starght");
        delay(80);
    } else if (B == black && C == black) {
        run(100, 100);
        Serial.println("go starght");
        delay(80);
    } else if (A == black && D == white) {
        spin_left(60, 60);
        Serial.println("go left");
        delay(80);
    } else if (A == white && D == black) {
        spin_right(60, 60);
        Serial.println("go right");
        delay(80);
    } else if (B == black && C == white) {
        spin_left(60, 60);
        Serial.println("go left");
        delay(30);
    } else if (B == white && C == black) {
        spin_right(60, 60);
        Serial.println("go right");
        delay(30);
    }
*/
}
