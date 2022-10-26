#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// time
#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include <coredecls.h>                  // settimeofday_cb()
//OTA
#include <ArduinoOTA.h>

// SN74HC595 简称 N74
// ST 大平台  5
// DS 数据 4
// SH 活塞  0
//const int GND = 8;
//const int VDD = 9;
const int N74_ST_BIG = D4;
const int N74_SH_PUSH = D3;
const int N74_DS_DATA = D2;

#define HOSTNAME "LED-MATRIX"

//时间
#define TZ              8       // (utc+) 时区
#define DST_MN          0      // use 60mn for summer time in some countries中国没有夏令时，赋0


#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)
time_t now;

// int led = 0b1111111111;
const int cols[8] = {253,133,181,176,181,133,253,255};
const int row[8] = {128,64,32,176,181,133,253,255};

//8x8 逐行 阳码 十进制 顺向 c51格式 雅黑
const int led_yh[10][8]={
{255,223,47,111,111,143,255,255},/*"0",0*/
{255,223,159,223,223,223,255,255},/*"1",0*/
{255,159,239,239,159,15,255,255},/*"2",0*/
{255,159,239,159,239,143,255,255},
{255,239,207,175,15,239,255,255},/*"4",0*/
{255,143,191,159,239,143,255,255},/*"5",0*/
{255,207,191,79,47,143,255,255},/*"6",0*/
{255,15,239,223,223,191,255,255},/*"7",0*/
{255,223,47,159,111,143,255,255},/*"8",0*/
{255,223,47,111,143,143,255,255}/*"9",0*/
};

// PC2LCD 2002: 8x8 逐行 阳码 十进制 顺向 c51格式  Arial
const int led_arial[10][8]={
{255,15,111,111,111,15,255,255},/*"0",0*/
{255,223,159,223,223,223,255,255},/*"1",1*/
{255,15,239,223,191,15,255,255},/*"2",2*/
{255,15,239,207,239,15,255,255},/*"3",3*/
{255,207,175,111,15,239,255,255},/*"4",4*/
{255,143,63,15,239,31,255,255},/*"5",5*/
{255,143,127,15,111,143,255,255},/*"6",6*/
{255,15,223,223,191,191,255,255},/*"7",7*/
{255,15,111,159,111,15,255,255},/*"8",8*/
{255,31,111,15,239,31,255,255},/*"9",9*/
};

//自定义
const int led_diy[10][8] = {
{255,159,111,111,111,111,159,255},/*"0",*/
{223,159,95,223,223,223,223,143},/*"1",*/
{159,111,239,223,191,127,15,255},/*"2",*/
//{143,239,239,223,159,239,239,15},/*"3",*/
{143,239,239,223,159,239,111,143},/*"3",*/
{223,159,95,95,15,223,223,223},/*"4",*/
//{15,127,127,127,15,239,239,15},/*"5",*/
{143,127,127,31,239,239,111,159},/*"5",*/
//{15,127,127,127,15,111,111,15},/*"6",*/
{159,111,127,127,31,111,111,159},/*"6",*/
{15,239,239,223,191,191,191,191},/*"7",*/
{159,111,111,159,159,111,111,159},/*"8",*/
//{15,111,111,15,239,239,239,15}/*"9",*/
{159,111,111,15,239,239,111,159},/*"9",*/
};

const int dot[] = {255,255,159,255,159,255,255,255};/*"点",：*/
const int yue[] = {193,221,193,221,193,221,185,255};/*"月",0*/
const int nian[] = {239,192,183,65,215,129,247,247};/*"年",0*/


void pull_push_data(short num); // 数据活塞 num:0或1
void clean();
void write3Byte(int col);
void colByCol();

void digitNum();
//显示计数器
int cnt = 0;

//wifi配置声明
void WiFiConfig();
//declaring prototypes 声明
void configModeCallback (WiFiManager *myWiFiManager);
//显示时间
void showTime();
//显示日期
void showDate();
//显示年
void showYear();

void setup()
{

  Serial.begin(115200);
    // BLINKER_DEBUG.stream(Serial);
  Serial.println("start..");
  WiFiConfig();
  // Get time from network time service. ntp服务设置
  configTime(TZ_SEC, DST_SEC, "ntp1.aliyun.com");
  
  pinMode(N74_ST_BIG, OUTPUT);
  pinMode(N74_DS_DATA, OUTPUT);
  pinMode(N74_SH_PUSH, OUTPUT);

//  pinMode(GND, OUTPUT);
//  digitalWrite(GND, LOW);
//  pinMode(VDD, OUTPUT);
//  digitalWrite(VDD, HIGH);

  // OTA设置并启动 wifi连接成功后执行
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.begin();

  Serial.println(WiFi.SSID()+":"+WiFi.psk());
}

void loop()
{
  ArduinoOTA.handle();
  /**
    MSBFIRST
    00000010
    01000000
    12345678 -->对应595的Q0~Q7
    高位先发送,对应Q0~Q7是倒序关系

    LSBFIRST
    00000010
    00000010
    12345678 -->对应595的Q0~Q7
    低位先发送,对应Q0~Q7是顺序关系
  **/
  //clean();
  //逐列 检查
//  colByCol();
//digitNum();

  //显示5秒时间
  if(cnt<5000){
    //时间
    showTime();
  }else if(cnt<8000){
    //日期
    showDate();
  }else{
    //显示年
    showYear();
  }
  cnt++;
  //总共10秒一轮回，3秒显示日期
  if(cnt>10000){
    cnt = 0;
  }
  
}

void clean(){
//   digitalWrite(N74_ST_BIG, LOW);
//   shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,0);//MSBFIRST 高位先入 LSBFIRST低位先入
//   shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,0);//MSBFIRST 高位先入 LSBFIRST低位先入
//   shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,0);//MSBFIRST 高位先入 LSBFIRST低位先入
//   shiftOut(N74_DS_DATA,N74_SH_PUSH,MSBFIRST,255);//MSBFIRST 高位先入 LSBFIRST低位先入
//   digitalWrite(N74_ST_BIG, HIGH);
   digitalWrite(N74_ST_BIG, LOW);
   shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,255);//MSBFIRST 高位先入 LSBFIRST低位先入
   shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,255);//MSBFIRST 高位先入 LSBFIRST低位先入
   shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,255);//MSBFIRST 高位先入 LSBFIRST低位先入
   shiftOut(N74_DS_DATA,N74_SH_PUSH,MSBFIRST,0);//MSBFIRST 高位先入 LSBFIRST低位先入
   digitalWrite(N74_ST_BIG, HIGH);
}

//逐列
void colByCol(){
  clean();
  delay(1000);
  for(int col=0;col<9;col++){//列要多移位一次
    for(int row =0;row<8;row++){
      //逐列
      digitalWrite(N74_ST_BIG, LOW);
      //shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,255>>col);//MSBFIRST 高位先入 LSBFIRST低位先入
      write3Byte(col);//右移位，右补0，列是低电位。
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,128>>row);//MSBFIRST 位先入 LSBFIRST低位先入
      digitalWrite(N74_ST_BIG, HIGH);
      delay(5);
    }
    
  }
}
// col是移位次数
void write3Byte(int col){
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,255>>col);//MSBFIRST 高位先入 LSBFIRST低位先入 右移位，右补0，列是低电位。
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,255>>col);//MSBFIRST 高位先入 LSBFIRST低位先入
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,255>>col);//MSBFIRST 高位先入 LSBFIRST低位先入
}
void pull_push_data(short num)
{
  digitalWrite(N74_SH_PUSH, LOW);  // 活塞拉回
  digitalWrite(N74_DS_DATA, num);  // 放入数据
  digitalWrite(N74_SH_PUSH, HIGH); // 活塞推动数据
}

void WiFiConfig() {
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment for testing wifi manager
  //wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);

  //or use this for auto generated name ESP + ChipID
  wifiManager.autoConnect(HOSTNAME,"12345678");

  //Manual Wifi
  //WiFi.begin(WIFI_SSID, WIFI_PWD);
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);


  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    counter++;
  }
  Serial.println("WiFi Connected!");
  Serial.print("IP ssid: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP addr: ");
  Serial.println(WiFi.localIP());
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  
  
}
//显示时间
void showTime(){
  now = time(nullptr);
  struct tm* timeInfo;
  timeInfo = localtime(&now);
  char buff[16];

  //时间
  sprintf_P(buff, PSTR("%02d:%02d:%02d"), timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
  Serial.println(String(buff));
  //年-月-日
  sprintf_P(buff, PSTR("%04d-%02d-%02d"),timeInfo->tm_year + 1900,timeInfo->tm_mon + 1,timeInfo->tm_mday );
  Serial.println(String(buff));
  int h10 = timeInfo->tm_hour/10;
  int h0  = timeInfo->tm_hour%10;
  int m10 = timeInfo->tm_min/10;
  int m0  = timeInfo->tm_min%10;
  //要多移位一次
    for(int col =0;col<9;col++){
      //逐行
      digitalWrite(N74_ST_BIG, LOW);
      int showH10 = (led_diy[h10][col]>>4)<<4;
      int showM0 = (led_diy[m0][col]>>4)<<4;

      int d = dot[col]>>4;
      //都先右移4位，得到高4位，然后显示在前的字符，左移4位当高位，再加上显示在后的字符的高位。
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,showM0 + 15);//MSBFIRST 高位先入 LSBFIRST低位先入 分钟个位,15是补低位四个1
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,(d<<4) + (led_diy[m10][col]>>4));//MSBFIRST 高位先入 LSBFIRST低位先入 点 和分钟十位
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST, showH10 + (led_diy[h0][col]>>4) );//MSBFIRST 高位先入 LSBFIRST低位先入 小时
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,128>>col);//MSBFIRST 位先入 LSBFIRST低位先入
      digitalWrite(N74_ST_BIG, HIGH);
      delay(0.2);
    }
}

void digitNum(){
//要多移位一次
    for(int col =0;col<9;col++){
      //逐行
      digitalWrite(N74_ST_BIG, LOW);
      int x0 = led_yh[0][col]>>4;
      int x1 = led_yh[1][col]>>4;
      int x2 = led_yh[2][col]>>4;
      int x3 = led_yh[3][col]>>4;
      int x4 = led_yh[4][col]>>4;
      int x5 = led_yh[5][col]>>4;
      int d = dot[col]>>4;
      //都先右移4位，得到高4位，然后显示在前的字符，左移4位当高位，再加上显示在后的字符的高位。
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,(x4<<4)+x5);//MSBFIRST 高位先入 LSBFIRST低位先入
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,(d<<4)+x3);//MSBFIRST 高位先入 LSBFIRST低位先入
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,(x0<<4)+x1);//MSBFIRST 高位先入 LSBFIRST低位先入
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,128>>col);//MSBFIRST 位先入 LSBFIRST低位先入
      digitalWrite(N74_ST_BIG, HIGH);
      delay(0.5);
    }
    
//  }
  
}

//显示日期
void showDate(){
  now = time(nullptr);
  struct tm* timeInfo;
  timeInfo = localtime(&now);
  char buff[16];

  //年-月-日
  sprintf_P(buff, PSTR("%04d-%02d-%02d"),timeInfo->tm_year + 1900,timeInfo->tm_mon + 1,timeInfo->tm_mday );
  Serial.println(String(buff));
  int mon = timeInfo->tm_mon + 1;
  int mon10 = mon/10;
  int mon0  = mon%10;
  int d10 = timeInfo->tm_mday/10;
  int d0  = timeInfo->tm_mday%10;
  //要多移位一次
    for(int col =0;col<9;col++){
      //逐行
      digitalWrite(N74_ST_BIG, LOW);
      int showM10 = (led_diy[mon10][col]>>4)<<4;
      int showD10 = (led_diy[d10][col]>>4)<<4;

      int d = dot[col]>>4;
      //都先右移4位，得到高4位，然后显示在前的字符，左移4位当高位，再加上显示在后的字符的高位。
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,showD10 + (led_diy[d0][col]>>4) );//MSBFIRST 高位先入 LSBFIRST低位先入 日
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,yue[col]);//MSBFIRST 高位先入 LSBFIRST低位先入 月字
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST, showM10 + (led_diy[mon0][col]>>4) );//MSBFIRST 高位先入 LSBFIRST低位先入 月
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,128>>col);//MSBFIRST 位先入 LSBFIRST低位先入
      digitalWrite(N74_ST_BIG, HIGH);
      delay(0.2);
    }
}

//显示年
void showYear(){
  now = time(nullptr);
  struct tm* timeInfo;
  timeInfo = localtime(&now);
  char buff[16];

  //年-月-日
  sprintf_P(buff, PSTR("%04d-%02d-%02d"),timeInfo->tm_year + 1900,timeInfo->tm_mon + 1,timeInfo->tm_mday );
  Serial.println(String(buff));
  int y = timeInfo->tm_year + 1900;
  int hxx = y/100;
  int lxx = y%100;
  int y10 = hxx/10;
  int y0  = hxx%10;
  int t10 = lxx/10;
  int t0  = lxx%10;
  //要多移位一次
    for(int col =0;col<9;col++){
      //逐行
      digitalWrite(N74_ST_BIG, LOW);
      int showY10 = (led_diy[y10][col]>>4)<<4;
      int showT10 = (led_diy[t10][col]>>4)<<4;

      int d = dot[col]>>4;
      //都先右移4位，得到高4位，然后显示在前的字符，左移4位当高位，再加上显示在后的字符的高位。
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,nian[col] );//MSBFIRST 高位先入 LSBFIRST低位先入 年字
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,showT10 + (led_diy[t0][col]>>4));//MSBFIRST 高位先入 LSBFIRST低位先入 年低位
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST, showY10 + (led_diy[y0][col]>>4) );//MSBFIRST 高位先入 LSBFIRST低位先入 年高位
      shiftOut(N74_DS_DATA,N74_SH_PUSH,LSBFIRST,128>>col);//MSBFIRST 位先入 LSBFIRST低位先入
      digitalWrite(N74_ST_BIG, HIGH);
      delay(0.2);
    }
}
