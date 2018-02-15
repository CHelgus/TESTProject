#ifdef __cplusplus  //=========================темпереатура чипа
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
#include <WiFi.h>                                               //wifi library
#include <WiFiUdp.h>                                            // wifi library 2
#include <WiFiMulti.h>                                          // wifi multi connect library
#include <Wire.h>
#include <TimeLib.h>      //библиотека времени C
#include <Adafruit_BMP085.h>                                    //
#include "RTClib.h"
#include "SHT21.h"
#define SDA_PIN 21                                              //Connect VCC of the BMP085&SHT21 sensor to 3.3V (NOT 5.0V!), Connect GND to Ground
#define SCL_PIN 22
#define PIN_A 18
#define PIN_B 19
#define SERIAL1_RXPIN 14
#define SERIAL1_TXPIN 13
#define SERIAL2_RXPIN 16
#define SERIAL2_TXPIN 17
//unsigned long prevMillis = 0;
uint8_t temprature_sens_read();
RTC_DS1307 rtc;                                                 //Connect SCL to i2c clock - pin 22 esp32 thats Analog 5,Connect SDA to i2c data - pin 21 esp32 thats Analog 4
SHT21 SHT21;
Adafruit_BMP085 bmp;
HardwareSerial Serial1(1);
HardwareSerial Serial2(2);                                              // pin 16=RX, pin 17=TX
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};   //mh-z19 готовая посылка для запроса MH-Z19
byte cmdstartdust[19] = {0xAA, 0xB4, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0xAB};
byte cmdgetdust[19] =   {0xAA, 0xB4, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0xAB};
byte cmdstartfen[9] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
byte cmdgetfen[9] =  {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
boolean dust = false;
boolean fen = false;
boolean trig = false;
boolean sem0, sem1, sem2, sem;
byte cmdfen[9] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
unsigned char response[9];//mh-z19
unsigned char responsesds[10];
unsigned char responsefen[10];
unsigned long LastTime, Time1, Time2, Time3;                                   // счетчики
int inter67 = 67000;
int inter31 = 31000;
int inter17 = 17000;
int inter7 = 7000;
int inter5 = 5000;
int inter1 = 1000;
int tyear, tday, tmonth, thour, tminute, tsecond;
uint8_t temp_farenheit;                                                 //for chip's Temp
float temp_celsius;
String input_string = "";//эту фигню нужно переделать....
String temp_out = "";
String hum_out = "";
String batt_out = "";
IPAddress timeServerIP;                          //for NTP
const char* ntpServerName = "ru.pool.ntp.org";
int TIMEZONE = 2;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
WiFiUDP udp;
WiFiMulti WiFiMulti;
unsigned int  localPort = 2390;      // local port to listen for UDP packets
unsigned long ntp_time = 0;
long  t_correct        = 0;
unsigned long cur_ms   = 0;
unsigned long ms1      = 0;
unsigned long ms2      = 10000000UL;
unsigned long t_cur    = 0;
unsigned int err_count = 0;
unsigned long realtime = 0;
uint16_t     vdd       = 0;
unsigned long timing;
unsigned long curMillis;
long tempi = 0;
long humi = 0;
long barin = 0;
unsigned int ppm = 0;
unsigned int dust25 = 0;
unsigned int dust10 = 0;
unsigned int fenolF = 0;
unsigned int fenolA = 0;
void Connect(void)  {   // ----------подключение к сетке
  WiFiMulti.addAP("WOLFIES", "messwiththebestdieliketherest");
  WiFiMulti.addAP("inok_open", "open__inok");
  WiFiMulti.addAP("AndroidAP", "123456789");
  Serial.println("Connecting Wifi...");
  if (WiFiMulti.run() == WL_CONNECTED)     {
    Serial.println("");
    Serial.print("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}
bool GetNTP(void)                               {     //--ПОЛУЧЕНИЕ ВРЕМЕНИ
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  delay(1000);
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println(">>>>>>>>>>No packet yet");     return false;
  } else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    udp.read(packetBuffer, NTP_PACKET_SIZE); // Читаем пакет в буфер
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);// 4 байта начиная с 40-го сождержат таймстамп времени - число секунд
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]); // от 01.01.1900
    unsigned long secsSince1900 = highWord << 16 | lowWord;          // Конвертируем два слова в переменную long
    const unsigned long seventyYears = 2208988800UL;                 // Конвертируем в UNIX-таймстамп (число секунд от 01.01.1970
    unsigned long epoch = secsSince1900 - seventyYears;
    ntp_time = epoch + TIMEZONE * 3600; // Делаем поправку на местную тайм-зону
    Serial.print("Unix time = ");
    Serial.println(ntp_time);
    time_t utcCalc = ntp_time;
    tyear = year(utcCalc);
    tmonth = month(utcCalc);
    tday = day(utcCalc);
    thour = hour(utcCalc);
    tminute = minute(utcCalc);
    tsecond = second(utcCalc);
    rtc.adjust(DateTime(tyear, tmonth, tday, thour, tminute, tsecond));
    Serial.print( year(utcCalc )) ;   Serial.print( ":" );
    Serial.print( month(utcCalc )) ;    Serial.print( ":" );
    Serial.print( day(utcCalc )) ;    Serial.print( ":" );
    Serial.print( hour(utcCalc )) ;   Serial.print( ":" );
    Serial.print( minute(utcCalc ) );   Serial.print( ":" );
    Serial.println( second(utcCalc ) );
  }
  return true;//true
}
void tryNTP()                                   {     //------------ПОПЫТКА ПОДКЛЮЧИТСЯ К НТП
  cur_ms = millis();                          // начало отсчета
  t_cur = cur_ms / 1000;                       // в  секундах
  if ( cur_ms < ms2 || (cur_ms - ms2) > 60000 ) // каждые 60 секунд считываем время в интернете
  { err_count++;                         // Делаем три  попытки синхронизации с интернетом
    if ( GetNTP() > 0 )
    { ms2 = cur_ms;
      err_count = 0;
      t_correct = ntp_time - t_cur;
    }
  }
  if ( err_count > 10 )
  {
    Serial.println("NTP connect false");
  }
}
unsigned long sendNTPpacket(IPAddress& address) {     //* Посылаем запрос NTP серверу на заданный адрес
  Serial.println("sending NTP packet...");
  memset(packetBuffer, 0, NTP_PACKET_SIZE);// Очистка буфера в 0
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode     // Формируем строку зыпроса NTP сервера
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  packetBuffer[12]  = 49; // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  udp.beginPacket(address, 123); // Посылаем запрос на NTP сервер (123 порт)
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
/*void cleport() {
  if (Serial1.available() > 0) {
    char c = Serial.read();
    if (c == '0') {
      return;
    }
  }
  }*/
void sendTime()                                 {
  DateTime now = rtc.now();
  int month = now.month();
  int day = now.day();
  int dayOfTheWeek = now.dayOfTheWeek();
  int hour = now.hour();
  int minute = now.minute();
  int second(now.second());
  String phour, pmin, psec, pweek, pmon, pday;
  if (hour < 10)
  { phour = String(hour, DEC);
    phour = "0" + phour;
  }
  else {
    phour = String(hour, DEC);
  }
  if (minute < 10) {
    pmin = String(minute  , DEC);
    pmin = "0" + pmin;
  } else {
    pmin = String(minute, DEC);
  }
  if (second < 10) {
    psec = String(second, DEC);
    psec = "0" + psec;
  } else {
    psec = String(second, DEC);
  }
  switch (dayOfTheWeek) {
    case 0: pweek = "\xB2\xDe\xE1\xDA\xE0\xD5\xE1\xD5\xDD\xEC\xD5";   break; // Воскресенье
    case 1: pweek = "\xBF\xDE\xDD\xD5\xD4\xD5\xDB\xEC\xDD\xD8\xDA";   break; //Понедельник
    case 2: pweek = "\xB2\xE2\xDE\xE0\xDD\xD8\xDA";             break;
    case 3: pweek = "\xC1\xE0\xD5\xD4\xD0";                     break;
    case 4: pweek = "\xC7\xD5\xE2\xD2\xD5\xE0\xD3";             break;
    case 5: pweek = "\xBF\xEF\xE2\xDD\xD8\xE6\xD0";             break;
    case 6: pweek = "\xC1\xE3\xD1\xD1\xDE\xE2\xD0";             break;
  }
  switch (month) {
    case 1: pmon = "\xCF\xDD\xD2\xD0\xE0\xEC\xEF";                  break;
    case 2: pmon = "\xC4\xD5\xD2\xE0\xD0\xDB\xEF";              break; //цуко c  я на конце
    case 3: pmon = "\xBC\xD0\xE0\xE2";                          break;
    case 4: pmon = "\xB0 \xDF \xE0\xD5\xDB\xEF";                break;
    case 5: pmon = "\xBC\xD0\xEF";                              break;
    case 6: pmon = "\xB8\xEE\xDD\xEF";                          break;
    case 7: pmon = "\xB8\xEE\xDB\xEF";                          break;
    case 8: pmon = "\xB0\xD2\xD3\xE3\xE1\xE2";                  break;
    case 9: pmon = "\xC1\xD5\xDD\xE2\xEF\xD1\xE0\xEF";          break;
    case 10: pmon = "\xBE\xDA\xE2\xEF\xD1\xE0\xEF";             break;
    case 11: pmon = "\xBD\xDE\xEF\xD1\xE0\xEF";                 break;
    case 12: pmon = "\xB4\xD5\xDA\xD0\xD1\xE0\xEF";             break;
  }
  select(0);//---------------------------------------------------------------------------------------------------------------------   000   ----------------
  Serial1.print((String)"t8.txt=\"" + phour  + "\"" + char(255) + char(255) + char(255));
  Serial1.print((String)"t10.txt=\"" + pmin  + "\"" + char(255) + char(255) + char(255));//Serial1.print((String)"t12.txt=\"" + psec  + "\"" + char(255) + char(255) + char(255));
  Serial1.print((String)"t14.txt=\"" + pweek + "\"" + char(255) + char(255) + char(255));
  Serial1.print((String)"t15.txt=\"" + day   + "\"" + char(255) + char(255) + char(255));
  Serial1.print((String)"t16.txt=\"" + pmon  + "\"" + char(255) + char(255) + char(255));
}
void getHumTemIn()  	                          {     //--SHT21 TEMP&HUM --inside-
  float temp;
  float hum;
  hum = SHT21.getHumidity();
  humi = (round(hum * 10)) / 10; //humi = humi / 10;
  temp = SHT21.getTemperature();
  tempi = round(temp * 10) / 10;
  Serial.print("READY TempIn&HumIn="); Serial.print(tempi); Serial.print(" "); Serial.println(humi);
}
void getProcTem(void)                         	{  	  //--temperature of chip esp32
  temp_farenheit = temprature_sens_read();
  temp_celsius = ( temp_farenheit - 32 ) / 1.8;
  Serial.print("READY Proc Temperature= "); Serial.println(temp_celsius);
}
void getBarIn()    		                          {  	  //--BMP 180- Ath Pressure
  float bar;
  bar = bmp.readSealevelPressure();
  bar = (bar / 1000) * 7.50063755419211;
  barin = round(bar * 10) / 10;
  Serial.print("READY Ath Pressure=  "); Serial.println(barin);   //---------------------------------------------------------end of sendBarin
}
void getCo2In()  		                            {     //--Co2In
  select(2);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.write(cmd, 9);
  memset(response, 0, 9);
  Serial1.readBytes(response, 9);

  int i; //--control
  byte crc = 0;
  for (i = 1; i < 8; i++) crc += response[i];
  crc = 255 - crc;
  crc++;  //=====control

  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) )
  { char raw[32];
    sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X", response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7], response[8]);
    Serial.print(">>>>>>>>>>CO2 error= "); Serial.println( raw);
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    ppm = (256 * responseHigh) + responseLow;
    //char raw[32];
    //sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X", response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7], response[8]);
    //Serial.print(" CO2 = "); Serial.println( raw);
    Serial.print("READY CO2= "); Serial.println(ppm);
  }
}  //-----------------------------------------------------------end of sendCo2In
void getDustIn()  	                          	{  	  //--DUSTIn
  if (dust == false)  {
    select(3);
    while (Serial1.available())   {
      Serial1.read();
    }
    Serial1.write(cmdstartdust, 19);
    (dust = true);
  } else {
    select(3);
    while (Serial1.available()) {
      Serial1.read();
    }
    Serial1.write(cmdgetdust, 19);    //SDS-021
    memset(responsesds, 0, 10);
    Serial1.readBytes(responsesds, 10);

    int i;
    byte crc = 0;
    for (i = 2; i < 8; i++) crc += responsesds[i];
    Serial.print("<><><DUST  CRC - ");
    Serial.println(crc);

    if ( !(responsesds[0] == 0xAA && responsesds[1] == 0xC0 && responsesds[8] == crc && responsesds[9] == 0xAB) )
    { char raw[40];
      sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", responsesds[0], responsesds[1], responsesds[2], responsesds[3], responsesds[4], responsesds[5], responsesds[6], responsesds[7], responsesds[8], responsesds[9]);
      Serial.print(">>>>>>>>>>DUST error = "); Serial.println(raw);
    } else {
      unsigned int responseHigh25 =  (int) responsesds[3];
      unsigned int responseLow25 = (int) responsesds[2];
      dust25 = ((256 * responseHigh25) + (responseLow25)) / 10;
      unsigned int responseHigh10 = (unsigned int) responsesds[5];
      unsigned int responseLow10 = (unsigned int) responsesds[4];
      dust10 = ((256 * responseHigh10) + (responseLow10)) / 10; //devide to 10
      Serial.print("READY Dust 2,5 = "); Serial.println(dust25);
      Serial.print("READY Dust  10 = "); Serial.println(dust10);
      // char raw[36];
      //   printf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", responsesds[0], responsesds[1], responsesds[2], responsesds[3], responsesds[4], responsesds[5], responsesds[6], responsesds[7], responsesds[8], responsesds[9]);
      //  Serial.print(" DUST = "); Serial.println(raw);
    }
  }
}
void getFenol()                                {     //-----------------------------------------------------------sendCo2In
  select(1);
  if (fen == false)  {
    Serial1.write(cmdstartfen, 9);
    (fen = true);
  } else {
    select(1);
    while (Serial1.available()) {
      Serial1.read();
    }
    Serial1.write(cmdgetfen, 9);                  //ch2o
    memset(responsefen, 0, 9);
    Serial1.readBytes(responsefen, 9);
    int i;
    byte crc = 0;
    for (i = 1; i < 8; i++) crc += responsefen[i];
    crc = 255 - crc;
    crc++;
    Serial.print("<><><FENOL CRC - ");
    Serial.println(crc);
    if ( !(responsefen[0] == 0xFF && responsefen[1] == 0x86 && responsefen[8] == crc) )
    { char raw[36];
      sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X", responsefen[0], responsefen[1], responsefen[2], responsefen[3], responsefen[4], responsefen[5], responsefen[6], responsefen[7], responsefen[8]);
      Serial.print(">>>>>>>>>>FENOL error = "); Serial.println(raw);
    } else {
      unsigned int responseHighF = (unsigned int) responsefen[2];
      unsigned int responseLowF = (unsigned int) responsefen[3];
      fenolF = (256 * responseHighF) + (responseLowF);
      Serial.print("READY Fenol = "); Serial.println(fenolF);
      unsigned int responseHighA = (unsigned int) responsefen[6];
      unsigned int responseLowA = (unsigned int) responsefen[7];
      fenolA = (256 * responseHighA) + (responseLowA);
      Serial.print("READY LVA gas= "); Serial.println(fenolA);
      //     char raw[36];
      //    sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X", responsefen[0], responsefen[1], responsefen[2], responsefen[3], responsefen[4], responsefen[5], responsefen[6], responsefen[7], responsefen[8]);
      //    Serial.print(" Fenol = "); Serial.println(raw);
    }
  }
}
void showAllNo()                                {     //--show all no Serial
  String tempdat = ("Tempin.txt=\"" + String(tempi) + "\"" );
  String humdat = ("Humin.txt=\"" + String(humi) + "\"" );
  String bardat = "t3.txt=\"" + String(barin) + "\"";
  String temppdat = ("t9.txt=\"" + String(temp_celsius) + "\"" );
  String co2dat = ("CO2.txt=\"" + String(ppm) + "\"" );
  String dustdat25 = ("tD25.txt=\"" + String(dust25) + "\"" );
  String dustdat10 = ("D10.txt=\"" + String(dust10) + "\"" );
  String fendatA = ("LVA.txt=\"" + String(fenolA) + "\"" );
  String fendatF = ("CH2O.txt=\"" + String(fenolF) + "\"" );
  //Serial.print("in port CO2 = ");Serial.println(co2dat);

  select(0);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print(tempdat); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print(humdat); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available())  {
    Serial1.read();
  }
  Serial1.print(bardat); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available())    {
    Serial1.read();
  }
  Serial1.print(temppdat); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print(co2dat); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print(dustdat25); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print(dustdat10); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print(fendatA); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
  while (Serial1.available()) {
    Serial1.read();
  }
  Serial1.print(fendatF); Serial1.write(0xFF);  Serial1.write(0xFF);  Serial1.write(0xFF);
}
void select(int x)                              {     ///============On\\Off
  switch (x) {
    case 0:
      { digitalWrite(PIN_A, LOW);
        digitalWrite(PIN_B, LOW);
      }
      break;
    case 1:
      { digitalWrite(PIN_A, HIGH);
        digitalWrite(PIN_B, LOW);
      }
      break;
    case 2:
      { digitalWrite(PIN_A, LOW);
        digitalWrite(PIN_B, HIGH);
      }
      break;
    case 3:
      { digitalWrite(PIN_A, HIGH);
        digitalWrite(PIN_B, HIGH);
      }
      break;
  }
}
void remote()                                   {     // data from remote
  while (Serial2.available() > 0)
  { char c = Serial2.read();
    if (c == '#')
    { int ind1, ind2;
      ind1 = input_string.indexOf(':');
      temp_out = input_string.substring(0, ind1);
      ind2 = input_string.indexOf(':', ind1 + 1 );
      hum_out = input_string.substring(ind1 + 1, ind2);
      batt_out = input_string.substring(ind2 + 1);
      select(0);//-=-------------------------------------------------0000-----------
      Serial1.print((String)"t4.txt=\"" + temp_out + "\"" + char(255) + char(255) + char(255));
      Serial1.print((String)"t12.txt=\"" + hum_out + "\"" + char(255) + char(255) + char(255));
      Serial1.print((String)"t28.txt=\"" + batt_out + "\"" + char(255) + char(255) + char(255));
      Serial.print("READY remote sensors: ");
      Serial.print(temp_out);
      Serial.print("-");
      Serial.print(hum_out);
      Serial.print("-");
      Serial.println(batt_out);
      input_string = "";
    }
    else {
      input_string += c;
    }
  }
}
void setup()                                    {     //------------------------------------SETUP
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  Serial2.begin(9600, SERIAL_8N1, SERIAL2_RXPIN, SERIAL2_TXPIN);                                                 // pin 16=RX, pin 17=TX
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial1.print((String)"rest" + char(255) + char(255) + char(255));
  SHT21.begin();
  if (! rtc.begin())    {
    Serial.println(">>>>>>>>>>Couldn't find RTC");
    while (1);
  }
  if (! rtc.isrunning())  { //---------------------------------------------------------здесь влупить подключение по NTP и проверку часов
    Serial.println(">>>>>>>>>>>RTC is NOT running!");
  }
  bmp.begin();
  if (!bmp.begin())      {
    Serial.println(">>>>>>>>>>>Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }
  Connect();
  tryNTP();
  LastTime = millis();
  Time1 = millis();
  Time2 = millis();
  Serial.println("<><><><><> lets running <><><><><><><>");
}
void loop()                                     {           //-------------------------LOOP
  if (millis() - LastTime > inter5)  {
    LastTime = millis();
    sendTime();
    showAllNo();
  }
  if (millis() - Time2 > inter31)  {
    Time2 = millis();
    remote();
    getProcTem();
    getBarIn();
    getHumTemIn();
  }
  if (millis() - Time1 > inter67)  {
    Time1 = millis();
    getFenol();
    getDustIn();
    getCo2In();
  }
}//--------------------------------------------------------------------------------END LOOP
/*А    Б  В    Г   Д  Е   Ж   З   И   Й    К   Л  М   Н   О    П   Р   С   Т   У   Ф  Х   Ц   Ч    Ш   Щ   Ъ   Ы   Ь   Э   Ю   Я
  B0  B1  B2  B3  B4  B5  B6  B7  B8  B9  BA  BB  BC  BD  BE  BF  C0  C1  C2  C3  C4  C5  C6  C7  C8  C9  CA  CB  CC  CD  CE  CF
  а   б   в   г   д   е   ж   з   и   й   к   л   м   н   о   п   р   с   т   у   ф   х   ц   ч   ш   щ   ъ   ы   ь   э   ю   я
  D0  D1  D2  D3  D4  D5  D6  D7  D8  D9  DA  DB  DC  DD  DE  DF  E0  E1  E2  E3  E4  E5  E6  E7  E8  E9  EA  EB  EC  ED  EE  EF

  void sendCommand(const char* cmd)
  {
  while (nexSerial.available())
  {
      nexSerial.read();
  }

  nexSerial.print(cmd);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  }

  void serialEvent(void){
  if (Serial.available() > 0) {
    char c = Serial.read();
      if (c == '0') {
        soTrue = !soTrue;
      }
  }
  }

*/
