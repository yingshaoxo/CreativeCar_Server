const int Left_Echo = A2;
const int Left_Triger = A1;
const int Right_Echo = A4;
const int Right_Triger = A3;

void setup()
{
    Serial.begin(9600); // 初始化串口
    //初始化超声波引脚

    init_ultrasonic_sensor();
}

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

void loop()
{
    Serial.print("left_Distance:");    //输出距离（单位：厘米）
    Serial.print(get_left_distance()); //显示距离
    Serial.println("cm");              //显示

    Serial.println(""); //显示

    Serial.print("right_Distance:");    //输出距离（单位：厘米）
    Serial.print(get_right_distance()); //显示距离
    Serial.println("cm");               //显示

    delay(500);
}
