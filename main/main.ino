// -----------------------------------------------
// -----------------------------------------------
// -----------------------------------------------
// -----------------------------------------------
int Left_motor_go = 8;   //左电机前进 AIN1
int Left_motor_back = 7; //左电机后退 AIN2

int Right_motor_go = 2;   //右电机前进 BIN1
int Right_motor_back = 4; //右电机后退 BIN2

int Left_motor_pwm = 6;  //左电机控速 PWMA
int Right_motor_pwm = 5; //右电机控速 PWMB

int key = A0; //定义按键为arduino的模拟口A0

//循迹红外引脚定义
const int TrackSensorLeftPin1 = 13;
const int TrackSensorLeftPin2 = 10;
const int TrackSensorRightPin1 = 11;
const int TrackSensorRightPin2 = 12;

//定义各个循迹红外引脚采集的数据的变量
int A;
int B;
int C;
int D;

void update_ABCD()
{
    A = digitalRead(TrackSensorLeftPin1);
    B = digitalRead(TrackSensorLeftPin2);
    C = digitalRead(TrackSensorRightPin1);
    D = digitalRead(TrackSensorRightPin2);
}

const int black = 1;
const int white = 0;

// -----------------------------------------------
// -----------------------------------------------
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
    init_ultrasonic_sensor();
    //wait_for_key_action();
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
void stop(int time)
{
    digitalWrite(Left_motor_go, LOW);
    digitalWrite(Left_motor_back, LOW);
    digitalWrite(Right_motor_go, LOW);
    digitalWrite(Right_motor_back, LOW);

    delay(time);
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
* Function       left_rotate
* @author        Danny
* @date          2017.07.25
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in1]    left_speed:左轮速度
* @param[in2]    right_speed:右轮速度
* @param[out]    void
* @retval        void
* @par History   无
*/
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

/**
* Function       right_rotate
* @author        Danny
* @date          2017.07.25
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in1]    left_speed:左轮速度
* @param[in2]    right_speed:右轮速度
* @param[out]    void
* @retval        void
* @par History   无
*/
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

const int distance_threshold = 40;

const int normal_speed = 120;//120; //60;
const int gentle_speed = 30;
unsigned int counter = 0;
unsigned int max_counting = 10000 * 1.5;
unsigned int second_max_counting = max_counting * 1.5;
unsigned int max_counting_for_pure_white = max_counting * 3;
int times_we_met_pure_white = 0;

void loop()
{
    update_ABCD();

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

    Serial.print("left_Distance:"); //输出距离（单位：厘米）
    left_distance = get_left_distance();
    Serial.print(left_distance); //显示距离
    Serial.println("cm");        //显示

    Serial.println(""); //显示

    Serial.print("right_Distance:"); //输出距离（单位：厘米）
    right_distance = get_right_distance();
    Serial.print(right_distance); //显示距离
    Serial.println("cm");         //显示

    //delay(500);

    /*
    if (left_distance < distance_threshold)
    {
        right_rotate(gentle_speed, gentle_speed);
        Serial.println("go right");
        delay(80);
    }
    else if (right_distance < distance_threshold)
    {
        left_rotate(gentle_speed, gentle_speed);
        Serial.println("go left");
        delay(80);
    }
    */
   ///*
    if (A == black && D == white)
    {
        counter = 0;
        while (!(B == black || C == black))
        {
            left_rotate(gentle_speed, gentle_speed);
            update_ABCD();
            counter = counter + 1;
            if (counter > max_counting)
            {
                stop(100);
                break;
            }
        }

        counter = 0;
        while (!(B == black || C == black))
        {
            right_rotate(gentle_speed, gentle_speed);
            update_ABCD();
            counter = counter + 1;
            if (counter > second_max_counting)
            {
                stop(100);
                break;
            }
        }
    }
    else if (A == white && D == black)
    {
        counter = 0;
        while (!(B == black || C == black))
        {
            right_rotate(gentle_speed, gentle_speed);
            update_ABCD();
            counter = counter + 1;
            if (counter > max_counting)
            {
                stop(100);
                break;
            }
        }

        counter = 0;
        while (!(B == black || C == black))
        {
            left_rotate(gentle_speed, gentle_speed);
            update_ABCD();
            counter = counter + 1;
            if (counter > second_max_counting)
            {
                stop(100);
                break;
            }
        }
    }
    else if (A == white && B == white && C == white && D == white)
    {
        counter = 0;
        while (A == white && B == white && C == white && D == white)
        {
            counter = counter + 1;
            go_straight(gentle_speed, gentle_speed);
            update_ABCD();
            if (counter > max_counting_for_pure_white)
            {
                stop(1000);
                times_we_met_pure_white = times_we_met_pure_white + 1;
                if (times_we_met_pure_white >= 2)
                {
                    wait_for_key_action();
                }
                break;
            }
        }

        counter = 0;
        while (A == white && B == white && C == white && D == white)
        {
            counter = counter + 1;
            go_back(gentle_speed, gentle_speed);
            update_ABCD();
            if (counter > max_counting_for_pure_white*1.5)
            {
                stop(1000);
                //wait_for_key_action();
                break;
            }
        }
    }
    else
    {
        go_straight(normal_speed, normal_speed);
    }
//    */

    /* new 1
    if (A == black && D == white)
    {
        left_rotate(normal_speed, normal_speed);
        Serial.println("go left");
        delay(80);
    }
    else if (A == white && D == black)
    {
        right_rotate(normal_speed, normal_speed);
        Serial.println("go right");
        delay(80);
    }
    else if (B == black && C == black)
    {
        go_straight(normal_speed, normal_speed);
        Serial.println("go starght");
        delay(30);
    }
    else if (B == black && C == white) {
        left_rotate(gentle_speed, gentle_speed);
        Serial.println("go left softly");
        delay(30);
    }
    else if (B == white && C == black) {
        right_rotate(gentle_speed, gentle_speed);
        Serial.println("go right softly");
        delay(30);

        go_straight(gentle_speed, gentle_speed);
        Serial.println("go starght softly");
        delay(30);
    }
    else
    {
        go_straight(gentle_speed, gentle_speed);
        Serial.println("go starght softly");
        delay(30);
    }
    */

    //  yesterday
    /*
    if (A == white && D == white)
    {
        go_straight(100, 100);
        Serial.println("go starght");
        //delay(80);
    }
    else if (A == white && D == white && B == black)
    {
        go_straight(100, 100);
        Serial.println("go starght");
        //delay(80);
    }
    else if (A == white && D == white && C == black)
    {
        go_straight(100, 100);
        Serial.println("go starght");
        //delay(80);
    }
    else if (B == black && C == black)
    {
        go_straight(100, 100);
        Serial.println("go starght");
        //delay(80);
    }
    else if (A == black && D == white)
    {
        left_rotate(60, 60);
        Serial.println("go left");
        //delay(80);
    }
    else if (A == white && D == black)
    {
        right_rotate(60, 60);
        Serial.println("go right");
        //delay(80);
    }
    else if (B == black && C == white)
    {
        left_rotate(60, 60);
        Serial.println("go left");
        //delay(30);
    }
    else if (B == white && C == black)
    {
        right_rotate(60, 60);
        Serial.println("go right");
        //delay(30);
    }
    */
}