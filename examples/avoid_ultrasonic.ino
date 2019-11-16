/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         avoid_ultrasonic.c
* @author       Danny
* @version      V1.0
* @date         2017.07.26
* @brief        超声波避障实验
* @details
* @par History  见如下说明
*
*/
//定义引脚
int Left_motor_go = 8;        //左电机前进 AIN1
int Left_motor_back = 7;      //左电机后退 AIN2

int Right_motor_go = 2;       //右电机前进 BIN1
int Right_motor_back = 4;     //右电机后退 BIN2

int Left_motor_pwm = 6;       //左电机控速 PWMA
int Right_motor_pwm = 5;      //右电机控速 PWMB

int key = A0;                 //定义按键为arduino的模拟口A0

int EchoPin = 12;             //定义回声脚为arduino上的数字12
int TrigPin = 13;             //定义触发脚为arduino上的数字13

float distance = 0;           //定义超声波所测距离为distance变量

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
  //设置串口数据传输的波特率为9600
  Serial.begin(9600);

  //初始化电机驱动IO口为输出方式
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);

  //定义按键接口为输入接口
  pinMode(key, INPUT);
  //按键初始化为高电平
  digitalWrite(key, HIGH);

  //初始化超声波引脚
  pinMode(EchoPin, INPUT);    //定义超声波输入脚
  pinMode(TrigPin, OUTPUT);   //定义超声波输出脚

  //调用按键扫描函数
  key_scan();
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
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  analogWrite(Left_motor_pwm, left_speed );

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
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
  digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
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
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void right()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  analogWrite(Left_motor_pwm, 150);

  //右电机停止
  digitalWrite(Right_motor_go, LOW);   //右电机前进禁止
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  analogWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.07.25
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  analogWrite(Left_motor_pwm, 160);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  analogWrite(Right_motor_pwm, 160);
  
  delay(time * 100);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.07.25
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in]     time:延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_right(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  analogWrite(Left_motor_pwm,160);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  analogWrite(Right_motor_pwm, 160);
  
  delay(time * 100);
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
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  analogWrite(Left_motor_pwm, 40);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  analogWrite(Right_motor_pwm, 40);

  delay(time );
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
  while (digitalRead(key));       //当按键没有被按下一直循环
  while (!digitalRead(key))       //当按键被按下时
  {
    delay(10);	                  //延时10ms
    if (digitalRead(key)  ==  LOW)//第二次判断按键是否被按下
    {
      delay(100);
      while (!digitalRead(key));  //判断按键是否被松开
    }
  }
}

/**
* Function       Distance
* @author        Danny
* @date          2017.07.26
* @brief         超声波测一次前方的距离
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void Distance()
{
  digitalWrite(TrigPin, LOW);               //给触发脚低电平2μs
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);              //给触发脚高电平10μs，这里至少是10μs
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  float Fdistance = pulseIn(EchoPin, HIGH); // 读取高电平时间(单位：微秒)
  Fdistance = Fdistance / 58;
  Serial.print("Distance:");               //输出距离（单位：厘米）
  Serial.print(Fdistance);                 //显示距离
  Serial.println("cm");
  distance = Fdistance;
}

/**
* Function       bubble
* @author        Danny
* @date          2017.07.26
* @brief         超声波测五次的数据进行冒泡排序
* @param[in1]    a:超声波数组首地址
* @param[in2]    n:超声波数组大小
* @param[out]    void
* @retval        void
* @par History   无
*/
void bubble(unsigned long *a, int n)

{
  int i, j, temp;
  for (i = 0; i < n - 1; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (a[i] > a[j])
      {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
      }
    }
  }
}

/**
* Function       Distane_test
* @author        Danny
* @date          2017.07.26
* @brief         超声波测五次，去掉最大值,最小值,
*                取平均值,提高测试准确性
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void Distance_test()
{
  unsigned long ultrasonic[5] = {0};
  int num = 0;
  while (num < 5)
  {
    Distance();
    //过滤掉测试距离中出现的错误数据大于500
    while (distance >= 500)
    {
      brake(0);
      Distance();
    }
    ultrasonic[num] = distance;
    num++;
  }
  num = 0;
  bubble(ultrasonic, 5);
  distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3;
  return;
}

/**
* Function       loop
* @author        Danny
* @date          2017.07.25
* @brief         先调用setup初始化配置里面的按键扫描函数，
*                超声波避障模式开启
* @param[in]     void
* @retval        void
* @par History   无
*/
void loop()
{
  Distance_test();
  if (distance > 50)
  {
    run(200, 200);      //当距离障碍物较远时全速前进
  }
  else if (distance >= 25 && distance <= 50)
  {
    run(100, 100);      //当快靠近障碍物时慢速前进
  }
  else if (distance < 25)
  {
    spin_right(3.5);    //当靠近障碍物时原地右转大约90度
    brake(1);
    Distance_test();    //再次测试判断前方距离
    if (distance >= 25)
    {
      run(100, 100);    //转弯后当前方距离大于25cm时前进
    }
    else if (distance < 25)
    {
      spin_left(7);    //转弯后前方距离小于25cm时向左原地转弯180度
      brake(1);
      Distance_test(); //再次测试判断前方距离
      if (distance >= 25)
      {
        run(100, 100); //转弯后当前方距离大于25cm时前进
      }
      else if (distance < 25)
      {
        spin_left(3.5);//转弯后前方距离小于25cm时向左原地转弯90度
        brake(1);
      }
    }
  }
}
