int Echo = 12;  
int Trig = 13; 

int Distance = 0;

void setup()
{
  Serial.begin(9600);     // 初始化串口
  //初始化超声波引脚
  pinMode(Echo, INPUT);    // 定义超声波输入脚
  pinMode(Trig, OUTPUT);   // 定义超声波输出脚
}


void Distance_test()   // 量出前方距离 
{
  digitalWrite(Trig, LOW);   // 给触发脚低电平2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
  delayMicroseconds(15);
  digitalWrite(Trig, LOW);    // 持续给触发脚低电
  float Fdistance = pulseIn(Echo, HIGH);  // 读取高电平时间(单位：微秒)
  Fdistance= Fdistance/58;       //为什么除以58等于厘米，  Y米=（X秒*344）/2
  // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
  Distance = Fdistance;
}  

void loop()
{
  Distance_test();
    Serial.print("Distance:");      //输出距离（单位：厘米）
    Serial.print(Distance);         //显示距离
    Serial.println("cm");        //显示
}


