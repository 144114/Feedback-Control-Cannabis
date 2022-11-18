// Simple Kalman
#include <SimpleKalmanFilter.h>

// Real Time Clock
#include <ThreeWire.h>  
#include <RtcDS1302.h>

// Sht3x
#include "Adafruit_SHT31.h"
#include <Wire.h>

// LCD  
#include <LiquidCrystal.h>

//PID
#include <PID_v1.h>

// Variable

//Real Time Clock
ThreeWire myWire(A3, A2, 10); // SCA = A4, SCL = A5, CE
RtcDS1302<ThreeWire> Rtc(myWire);
uint8_t month, day, hour, minute, second;
uint8_t second_threshold = 0;
uint16_t year;

// Water Pump
int water_pin = 1;

// Lightblub
int light_pin = 2;

// sht3x
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Fan
int pwm_pin = 3;
int moA_pin = 4;
int moB_pin = 5;
int pwm = 0;

// Soil Humid
#define soilHumid_pin A0

// LCD
const int rs = 12, en = 11, d4 = 9, d5 = 8, d6 = 7, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int lcd_counter  = 0;

//PID
double Setpoint, Input = 0, Output;
double Kp=1, Ki=1.5, Kd=0.11;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Kalman
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

//Theshold

int soilHumid_below_water_threshold = 0;
int soilHumid_above_water_threshold = 0;
int soilHumid_below_fan_threshold = 0;
int soilHumid_above_fan_threshold = 0;

float max_water_humid = 150;
float max_water_dry = 1023;


int areatemp_below_water_threshold = 0;
int areatemp_above_water_threshold = 0;
int areatemp_below_fan_threshold = 0;
int areatemp_above_fan_threshold = 0;


void setup () {
  Serial.begin(9600);
  //if (1){
    Rtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) Rtc.SetDateTime(compiled);
  //}

  if (!sht31.begin(0x44)){
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  
  //if (1){
  Setpoint = 26;
  myPID.SetMode(AUTOMATIC);
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  //lcd.print("hello");
  
  
  
  pinMode(water_pin,OUTPUT);
  //pinMode(light_pin,OUTPUT);
  pinMode(pwm_pin,OUTPUT);
  pinMode(moA_pin,OUTPUT);
  pinMode(moB_pin,OUTPUT);
  

}
void loop () {
  
  RtcDateTime now = Rtc.GetDateTime();

  if (now.Second() != second_threshold){
    second_threshold = now.Second();
    //printDateTime(now);
    float passed_soil_hum = soil_humidity();
    float passed_area_temp = sht3x_tem();
    float passed_area_humid = sht3x_hum();
    lcd_func(passed_area_temp,passed_area_humid,passed_soil_hum,lcd_counter);
      lcd_counter += 1;
      if(now.Second() % 2 == 0){
     
      }

    Input = sht3x_tem();
    float measured_value = Input + random(-100,100)/100.0;
    float estimated_value = simpleKalmanFilter.updateEstimate(measured_value);
    myPID.Compute();
    Serial.println(String(Setpoint) +","+String(Input)+","+String(Output)+","+String(measured_value)+","+String(estimated_value));
    //pwm = 255;
    //fan();
    //  water_pump();
   }
    
  
  //else{
    
  //}

}

void printDateTime(const RtcDateTime& dt){
  year   = (int)dt.Year();
  month  = (int)dt.Month();
  day    = (int)dt.Day();
  hour   = (int)dt.Hour();
  minute = (int)dt.Minute();
  second = (int)dt.Second();
  
  String str = String(day) + 
               "/" + String(month) + 
               "/" + String(year) + 
               "   " + String(hour) + 
               ":" + String(minute) + 
               ":" + String(second);
  Serial.println(str);
}

void lcd_func(float lcd_area_temp,float lcd_area_humid,float lcd_soil_humid,int counter){

  if (counter == 1){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Area Temp");
    lcd.setCursor(0,1);
    lcd.print(lcd_area_temp);
  }

  if (counter == 2){
    lcd.setCursor(0,0);
    lcd.print("Area Humid");
    lcd.setCursor(0,1);
    lcd.print(lcd_area_humid);
  }
      
  if (counter >= 3){
    lcd.setCursor(0,0);
    lcd.print("Soil Humid");
    lcd.setCursor(0,1);
    lcd_soil_humid = 100 - map(lcd_soil_humid,max_water_humid,max_water_dry,0,100);
    lcd.print(lcd_soil_humid);
    lcd_counter = 0;
  }

}

float soil_humidity(){
  float soilhumid_val = analogRead(soilHumid_pin);
  //Serial.println("Soil Humidity : ");
  //Serial.print(soilhumid_val);
  return soilhumid_val;
}

float sht3x_hum(){
  float area_humid = sht31.readHumidity();
    if (isnan(area_humid)) { 
      area_humid=0.0;   
    }
  //Serial.println("Area Humidity : ");
  //Serial.print(area_humid);
  return area_humid;
}

float sht3x_tem(){
  float area_temp = sht31.readTemperature();
  if (isnan(area_temp)) {
    area_temp = 0.0;
  }
  //Serial.println("Area Tempareture : ");
  //Serial.print(area_temp);
  return area_temp;
}

void fan(){
  analogWrite(pwm_pin, pwm);
  digitalWrite(moA_pin, HIGH);
  digitalWrite(moB_pin, LOW);

}

void lightbulb(){
  digitalWrite(light_pin,HIGH);
  Serial.println("light is on");

}

void water_pump(){
  digitalWrite(water_pin,HIGH);
  Serial.println("water pump");
}
