//#define ESP32 0



#include <Arduino.h>
// Include the library:
#include <TM1637Display.h>
#include "RTClib.h"

#include <NTPClient.h>

#ifdef ESP32
    #include <WiFi.h>
#else
    #include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>

#include <RotaryEncoder.h>

#include "private.h"

// Device can be found on the network using this name
#define NAME "Puzzle_3"


WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);


// Define the connections pins:
#define CLK_DIG D5
#define DIO_DIG D6

#ifdef ESP32
    #define TOUCH_PIN 17
#else
    #define TOUCH_PIN D7
#endif


RTC_DS3231 rtc;
// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN D0
#define CLOCK_SCL D1
#define CLOCK_SDA D2

#define PIN_A   18 //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define PIN_B   19 //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON  25 //ky-040 sw  pin, add 100nF/0.1uF capacitors between pin & ground!!!

int16_t position = 0;

//RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);

bool rotated = false;
/*
void encoderISR()
{
    encoder.readAB();
    rotated = true; 
    //Serial.print("ROTATED");
}

bool pushed = false;
void encoderButtonISR()
{
  encoder.readPushButton();
  if(pushed == false)
    {
        pushed = true; 
    }
}*/

int numCounter = 0;

uint8_t cur_min, cur_hour;

uint16_t year = 2022;
uint8_t month = 3;
uint8_t day = 14;

uint8_t seconds;

uint8_t alarm_hour; 
uint8_t alarm_min;

uint8_t new_alarm_hour; 
uint8_t new_alarm_min;

TM1637Display display(CLK_DIG, DIO_DIG); //set up the 4-Digit Display.

DateTime alarm_time;
DateTime datetime;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

enum state
{
    SHOW_TIME = 0,
    SHOW_ALARM = 1,
    SHOW_ALARM_CHANGE_HOURS,
    SHOW_ALARM_CHANGE_MINS
} clc_state;

enum alarm
{
    ALARM_OFF = 0,
    ALARM_ON
} alarm_state;


bool touched = false;
void IRAM_ATTR isr() {
    if(touched == false)
    {
        touched = true; 
    }

    Serial.print("HIGH");
  
}

void IRAM_ATTR onAlarm() {
    Serial.println("ALARM");
    alarm_state = ALARM_ON;  
}

void setup()
{

    Serial.begin(115200);
    delay(1000);


    display.setBrightness(0x0a); //set the diplay to maximum brightness
    pinMode(TOUCH_PIN, INPUT_PULLUP);
    attachInterrupt(TOUCH_PIN, isr, FALLING);
/*
    encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

    attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISR,       CHANGE);  //call encoderISR()    every high->low or low->high changes
    attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes
*/


    // Making it so, that the alarm will trigger an interrupt
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
    pinMode(CLOCK_SCL, INPUT_PULLUP);
    pinMode(CLOCK_SDA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

    Wire.begin(CLOCK_SDA, CLOCK_SCL);
    
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
    }


    Serial.print("Connecting to ");
    Serial.println(SSID);
    // Set name passed to AP while connection
    WiFi.setHostname(NAME);
    // Connect to AP
    WiFi.begin(SSID, PWD);
    // Wait while not connected
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    // Print IP
    Serial.println("\nWiFi connected.\nIP Adress: ");
    Serial.println(WiFi.localIP());

    timeClient.begin();
    //summer time...
    timeClient.setTimeOffset(7200);
 
  
    Serial.println("lets set the time!");

    if(timeClient.update())
    {
        Serial.print("Updating RTC time..");
    }

    cur_min = timeClient.getMinutes();
    cur_hour = timeClient.getHours();
    seconds = timeClient.getSeconds();  

    datetime = DateTime(year, month, day, cur_hour, cur_min, seconds);

    rtc.adjust(datetime);

    if(cur_min < 10)
            display.showNumberDecEx(cur_min, 0b01000000, true, 2U, 2);
    else
        display.showNumberDecEx(cur_min, 0b01000000, false, 2U, 2);

    if(cur_hour < 10)
        display.showNumberDecEx(cur_hour, 0b01000000, true, 2U, 0);
    else
        display.showNumberDecEx(cur_hour, 0b01000000, false, 2U, 0);

    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);


    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    alarm_time = DateTime(year, month, day, alarm_hour, alarm_min, 0);
    alarm_state = ALARM_OFF;

    clc_state = SHOW_TIME;
    
    // following line sets the RTC to the date &amp; time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date &amp; time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  
}

 unsigned long time_alarm_show;
 bool show_alarm = false;
 bool switch_state = false;

void loop()
{/*
    switch(clc_state)
    {
        case SHOW_TIME:
        {
            DateTime now = rtc.now();
            if(now.minute() != cur_min || switch_state)
            {
                
                cur_min = now.minute();
                if(cur_min < 10)
                    display.showNumberDecEx(cur_min, 0b01000000, true, 2U, 2);
                else
                    display.showNumberDecEx(cur_min, 0b01000000, false, 2U, 2);
                if(now.hour() != cur_hour || switch_state)
                {
                    if(timeClient.update())
                    {
                        Serial.print("Updating RTC time..");
                        cur_min = timeClient.getMinutes();
                        cur_hour = timeClient.getHours();
                        seconds = timeClient.getSeconds(); 

                        datetime = DateTime(year, month, day, cur_hour, cur_min, seconds);

                        rtc.adjust(datetime);

                    }
                    cur_hour = now.hour();
                    if(now.hour() < 10)
                        display.showNumberDecEx(cur_hour, 0b01000000, true, 2U, 0);
                    else
                        display.showNumberDecEx(cur_hour, 0b01000000, false, 2U, 0);

                    switch_state = false;
                }
            
                Serial.print(now.year(), DEC);
                Serial.print('/');
                Serial.print(now.month(), DEC);
                Serial.print('/');
                Serial.print(now.day(), DEC);
                Serial.print(" (");
                Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
                Serial.print(") ");
                Serial.print(now.hour(), DEC);
                Serial.print(':');
                Serial.print(now.minute(), DEC);
                Serial.print(':');
                Serial.print(now.second(), DEC);
                Serial.println();
            
            }
            if(pushed && !alarm_state)
            {
                pushed = false;
                clc_state = SHOW_ALARM;
                switch_state = true;
            }
            break;
        }
        case SHOW_ALARM:
        {
            
            if(switch_state)
            {
                time_alarm_show = millis();
                switch_state = false;
            }
            else 
            {
                if(millis()-time_alarm_show > 5000)
                {
                    clc_state = SHOW_TIME;
                    switch_state = true;
                }
            }
            // show alarm time.
            if(alarm_min < 10)
                display.showNumberDecEx(alarm_min, 0b01000000, true, 2U, 2);
            else
                display.showNumberDecEx(alarm_min, 0b01000000, false, 2U, 2);
            if(alarm_hour < 10)
                    display.showNumberDecEx(alarm_hour, 0b01000000, true, 2U, 0);
            else
                display.showNumberDecEx(alarm_hour, 0b01000000, false, 2U, 0);

            if(pushed && !alarm_state)
            {
                pushed = false;
                clc_state = SHOW_ALARM_CHANGE_HOURS;
                switch_state = true;
            }
            break;
        }
        case SHOW_ALARM_CHANGE_HOURS:
        {
            if(switch_state)
            {
                new_alarm_hour = alarm_hour;
                encoder.setPosition(new_alarm_hour);
                time_alarm_show = millis();
                switch_state = false;
            }
            if (rotated)
            {
                time_alarm_show = millis();
                Serial.print(encoder.getPosition());
                new_alarm_hour = encoder.getPosition();
                rotated = false;
                if(new_alarm_hour < 10)
                    display.showNumberDecEx(new_alarm_hour, 0b01000000, true, 2U, 0);
                else
                    display.showNumberDecEx(new_alarm_hour, 0b01000000, false, 2U, 0);
            }
            else if (pushed)
            {
                pushed = false;
                clc_state = SHOW_ALARM_CHANGE_MINS;
                switch_state = true;
            }
            else 
            {
                if(millis()-time_alarm_show > 3000)
                {
                    clc_state = SHOW_TIME;
                    switch_state = true;
                }
            }
            break;
        }

        case SHOW_ALARM_CHANGE_MINS:
        {
            if(switch_state)
            {
                Serial.print("changed to mins");
                new_alarm_min = alarm_min;
                encoder.setPosition(new_alarm_min);
                time_alarm_show = millis();
                switch_state = false;
            }
            if (rotated)
            {
                time_alarm_show = millis();
                new_alarm_min = encoder.getPosition();
                rotated = false;
                if(new_alarm_min < 10)
                    display.showNumberDecEx(new_alarm_min, 0b01000000, true, 2U, 2);
                else
                    display.showNumberDecEx(new_alarm_min, 0b01000000, false, 2U, 2);
            }
            else if (pushed && !alarm_state)
            {
                pushed = false;
                alarm_hour = new_alarm_hour;
                alarm_min = new_alarm_min;
                alarm_time = DateTime(year, month, day, alarm_hour, alarm_min, 0);

                //set the alarm time: 
                // schedule an alarm 10 seconds in the future
                rtc.clearAlarm(1);
                if(!rtc.setAlarm1(
                        alarm_time,
                        DS3231_A1_Hour // this mode triggers the alarm when the seconds match. See Doxygen for other options
                )) {
                    Serial.println("Error, alarm wasn't set!");
                }else {
                    Serial.println("Alarm will happen at");
                }
                clc_state = SHOW_TIME;
                switch_state = true;
            }
            else 
            {
                if(millis()-time_alarm_show > 5000)
                {
                    clc_state = SHOW_TIME;
                    switch_state = true;
                }
            }
            break;
        }
        default:
            break;

    }
    
    if(touched == true)
    {   touched = false; 
        if(alarm_state)
        {
            Serial.print("SNOOZE");
            rtc.clearAlarm(1);
            if(!rtc.setAlarm1(
                    rtc.now() + TimeSpan(10*60),
                    DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
            )) {
                Serial.println("Error, alarm wasn't set!");
            }else {
                Serial.println("Alarm will happen at");
            }
        }
        Serial.println("HIGH");
    }
    else if (pushed == true)
    {
        if(alarm_state)
        {
            Serial.print("Set alarm off");
            alarm_state = ALARM_OFF;
            pushed = false;
        }
    }*/
/*
    // the value at SQW-Pin (because of pullup 1 means no alarm)
    Serial.print(" SQW: ");
    Serial.print(digitalRead(CLOCK_INTERRUPT_PIN));
    // whether a alarm happened happened
    Serial.print(" Alarm1: ");
    Serial.print(rtc.alarmFired(1));

        // put your main code here, to run repeatedly:
        //timeClient.update();

    if(rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
        Serial.println("Alarm cleared");
    }*/

        //Serial.println(timeClient.getFormattedTime())

    delay(1000);
    Serial.println("loop");
    
}