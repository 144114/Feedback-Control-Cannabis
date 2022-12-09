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

// fan
int motorA_fan_pin = 0;
int motorB_fan_pin = 1;
#define pwm_fan_pin A1
int pwm_fan = 130;

// Lightblub
int light_pin = 2;

// sht3x
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// water_pin
int water_pwm_pin = 3;
int motorA_water_pin = 4;
int motorB_water_pin = 5;
int pwm_water = 120;

// Soil Humid
#define soilHumid_pin A0

// LCD
const int rs = 12, en = 11, d4 = 9, d5 = 8, d6 = 7, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int lcd_counter  = 0;

//Kalman
SimpleKalmanFilter simpleKalmanFilter(2.5, 4, 0.05); // Soil humidity
SimpleKalmanFilter simpleKalmanFilter_2(0.3, 0.2, 0.01); // Area Temp
SimpleKalmanFilter simpleKalmanFilter_3(2, 3, 0.01); // Area Humidity
float passed_soil_hum = 0;
float passed_area_temp = 0;
float passed_area_humid = 0;
float estimated_value = simpleKalmanFilter.updateEstimate(passed_soil_hum);
float estimated_value_2 = simpleKalmanFilter_2.updateEstimate(passed_area_temp);
float estimated_value_3 = simpleKalmanFilter_3.updateEstimate(passed_area_humid);


const long SERIAL_REFRESH_TIME = 100;
long refresh_time;
int want = 1;
//Threshold

int soilHumid_below_water_threshold = 0;
int soilHumid_above_water_threshold = 0;
int soilHumid_below_fan_threshold = 0;
int soilHumid_above_fan_threshold = 0;
int soilhumid_threshold = 65;

float max_water_humid = 150;
float max_water_dry = 1023;


int areatemp_threshold = 24;
int areatemp_above_water_threshold = 20;
int areatemp_below_fan_threshold = 0;
int areatemp_above_fan_threshold = 0;

//PID
double Setpoint, Input, Output;
double Kp = 1.98, Ki = 0.62, Kd = 0.17;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double Setpoint_2, Input_2, Output_2;
double Kp_2 = 2.56, Ki_2 = 0.8 , Kd_2 = 0.1 ;
PID myPID_2(&Input_2, &Output_2, &Setpoint_2, Kp_2, Ki_2, Kd_2, DIRECT);

void setup () {
  Serial.begin(9600);
  //if (1){
    Rtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) Rtc.SetDateTime(compiled);
  //}

  if (!sht31.begin(0x44)){
    //Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  
  //if (1){
  Setpoint = 26;
  myPID.SetMode(AUTOMATIC);
  myPID_2.SetMode(AUTOMATIC);
  
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  //lcd.print("hello");
  
  
  //pinMode(water_pin,OUTPUT);
  pinMode(light_pin,OUTPUT);
  //pinMode(water_pwm_pin,Output);
  pinMode(motorA_water_pin,OUTPUT);
  pinMode(motorB_water_pin,OUTPUT);
  
  //pinMode(pwm_fan_pin,Output_2);
  pinMode(motorA_fan_pin,OUTPUT);
  pinMode(motorB_fan_pin,OUTPUT);
  
  now = Rtc.GetDateTime();
  if (now.Month() == 11){
    if (now.Day() >= 19 && now.Day() <= 26){
      areatemp_threshold = 24;
      soilhumid_threshold = 65;
    }
    else{
      areatemp_threshold = 26;
      soilhumid_threshold = 50;
    }
  }
  else{
    areatemp_threshold = 26;
    soilhumid_threshold = 50;
  }

  Input = sht3x_tem();
  Input_2 = soil_humidity();
  Setpoint = areatemp_threshold;
  Setpoint_2 = soilhumid_threshold;
  myPID.SetMode(AUTOMATIC);
  myPID_2.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(120,255);
  myPID_2.SetOutputLimits(100,255);
}
void loop () {
  
  RtcDateTime now = Rtc.GetDateTime();
  
  while (want == 1){
    for (int i = 0; i < 20; i++){
      passed_soil_hum = soil_humidity();
      passed_area_temp = sht3x_tem();
      passed_area_humid = sht3x_hum();
      estimated_value = simpleKalmanFilter.updateEstimate(passed_soil_hum);
      estimated_value_2 = simpleKalmanFilter_2.updateEstimate(passed_area_temp);
      estimated_value_3 = simpleKalmanFilter_3.updateEstimate(passed_area_humid);
      Serial.println("F_M_soil_hum E_soil_hum M_A_temp E_A_temp M_A_hum E_A_hum");
      Serial.println(String(passed_soil_hum)+" "+String(estimated_value)+" "+String(passed_area_temp)+" "+String(estimated_value_2)+" "+String(passed_area_humid)+" "+String(estimated_value_3));
    }
    want = 0;
  }


  if (now.Second() != second_threshold){
    second_threshold = now.Second();
    //printDateTime(now);
    passed_soil_hum = soil_humidity();
    passed_area_temp = sht3x_tem();
    passed_area_humid = sht3x_hum();
      lcd_counter += 1;
      if(now.Second() % 2 == 0){
     
      }

    Input = estimated_value_2; // Area Temp
    Input_2 = estimated_value; // Soil Humid
    
    float measured_value = Input + random(-100,100)/100.0;
    estimated_value = simpleKalmanFilter.updateEstimate(passed_soil_hum); // Soil Humid

    float measured_value_2 = Input_2 + random(-100,100)/100.0;
    estimated_value_2 = simpleKalmanFilter_2.updateEstimate(passed_area_temp); // Area Temp

    estimated_value_3 = simpleKalmanFilter_3.updateEstimate(passed_area_humid); // Area Humid

    Input = estimated_value_2; // Area Temp
    Input_2 = estimated_value; // Soil Humid
    
    myPID.Compute();
    myPID_2.Compute();

    Serial.println(String(areatemp_threshold) + " " + String(soilhumid_threshold) + " " + String(255 - Output) + " " + String(Output_2) + " " + String(Setpoint) + " " + String(Setpoint_2));
    //Serial.println(String(Setpoint) +","+String(Input)+","+String(Output)+","+String(measured_value)+","+String(estimated_value));
    //Serial.println("M_soil_hum E_soil_hum M_A_temp E_A_temp M_A_hum E_A_hum");
    //Serial.println(String(passed_area_temp)+" "+String(estimated_value_2));
    //Serial.println(String(passed_soil_hum)+" "+String(estimated_value)+" "+String(passed_area_temp)+" "+String(estimated_value_2)+" "+String(passed_area_humid)+" "+String(estimated_value_3));
   }

   if (now.Month() == 11){
    if (now.Day() >= 19 && now.Day() <= 26){
      if (now.Hour() >= 6 && now.Hour() <= 10){
        lightbulb_on();
      }
      else{
        lightbulb_off();
      }
    }
    else{
      if(now.Hour() >= 6 && now.Hour() <= 21){
        lightbulb_on();
      }
      else{
        lightbulb_off();
      }
    }
   }
   else{
    if(now.Hour() >= 6 && now.Hour() <= 21){
      lightbulb_on();
   }
    else{
      lightbulb_off();
   }

   }
   
  //else{
    lcd_func(passed_area_temp,passed_area_humid,passed_soil_hum,estimated_value,estimated_value_2,estimated_value_3,lcd_counter);
    pwm_fan = 255 - Output;
    pwm_water = Output_2;
    
    analogWrite(pwm_fan_pin, pwm_fan);
    digitalWrite(motorA_fan_pin, HIGH);
    digitalWrite(motorB_fan_pin, LOW);
    
    
    analogWrite(water_pwm_pin, pwm_water);
    digitalWrite(motorA_water_pin, HIGH);
    digitalWrite(motorB_water_pin, LOW);
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

void lcd_func(float lcd_area_temp,float lcd_area_humid,float lcd_soil_humid,float lcd_estimated_value,float lcd_estimated_value_2,float lcd_estimated_value_3, int counter){

  if (counter == 1){
    lcd.clear();
    lcd.setCursor(0,0);
    if (lcd_area_temp == 0){
      lcd.print("Error A temp");
    } else {
      lcd.print("Area Temp");
    }
    lcd.setCursor(0,1);
    lcd.print(lcd_area_temp);
  }

  if (counter == 2){
    lcd.clear();
    lcd.setCursor(0,0);
    if (lcd_area_humid == 0){
      lcd.print("Error A humid");
    } else {
      lcd.print("Area Humid");
    }
    lcd.setCursor(0,1);
    lcd.print(lcd_area_humid);
  }
      
  if (counter == 3){
    lcd.clear();
    lcd.setCursor(0,0);
    if (lcd_soil_humid == 0){
      lcd.print("Error S humid");
    } else {
      lcd.print("Soil Humid");
    }
    lcd.setCursor(0,1);
    lcd.print(lcd_soil_humid);
  }

  if (counter == 4){
    lcd.clear();
    lcd.setCursor(0,0);
    if (lcd_estimated_value == 0){
      lcd.print("Error ESH");
    } else {
      lcd.print("ESH");
    }
    lcd.setCursor(0,1);
    lcd.print(lcd_estimated_value);
  }

    if (counter == 5){
    lcd.clear();
    lcd.setCursor(0,0);
    if (lcd_estimated_value_2 == 0){
      lcd.print("Error EAT");
    } else {
      lcd.print("EAT");
    }
    lcd.setCursor(0,1);
    lcd.print(lcd_estimated_value_2);
  }

  if (counter >= 6){
    lcd.clear();
    lcd.setCursor(0,0);
    if (lcd_estimated_value_3 == 0){
      lcd.print("Error EAS");
    } else {
      lcd.print("EAS");
    }
    lcd.setCursor(0,1);
    lcd.print(lcd_estimated_value_3);
    lcd_counter = 0;
  }

}

float soil_humidity(){
  float soilhumid_val = analogRead(soilHumid_pin);
  soilhumid_val = 100.0 - map(soilhumid_val,max_water_humid,max_water_dry,0,1000)/100.0 - 44;
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

void water_pump(){

  
}

void lightbulb_on(){
  digitalWrite(light_pin,HIGH);
  //Serial.println("light is on");
}

void lightbulb_off(){
  digitalWrite(light_pin,LOW);
  //Serial.println("light is on");

}

void fan(){
  
}
