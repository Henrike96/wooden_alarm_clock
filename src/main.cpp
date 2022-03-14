/*#include <Arduino.h>

#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define SSID "Vodafone-69BF"
#define PWD "HLGHNHgLnTQGgccN"

// Device can be found on the network using this name
#define NAME "Puzzle_3"

WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);


void setup() {
        Serial.begin(9600);
  // put your setup code here, to run once:
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


}

void loop() {
  // put your main code here, to run repeatedly:
  timeClient.update();

  Serial.println(timeClient.getFormattedTime());

  delay(1000);
}*/
#include <Arduino.h>
// Include the library:
#include <TM1637Display.h>
#include "RTClib.h"

#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Device can be found on the network using this name
#define NAME "Puzzle_3"

#define SSID "Vodafone-69BF"
#define PWD "HLGHNHgLnTQGgccN"

WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

RTC_DS3231 rtc;
// Define the connections pins:
#define CLK_DIG 33
#define DIO_DIG 32

#define TOUCH_PIN 17

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 16


int numCounter = 0;

uint8_t cur_min, cur_hour;

uint16_t year = 2022;
uint8_t month = 3;
uint8_t day = 14;

uint8_t seconds;

TM1637Display display(CLK_DIG, DIO_DIG); //set up the 4-Digit Display.



char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


bool touched = false;
void IRAM_ATTR isr() {
    if(touched == false)
    {
        touched = true; 
    }
  
}

void IRAM_ATTR onAlarm() {
    Serial.println("ALARM");
  
}

void setup()
{
    display.setBrightness(0x0a); //set the diplay to maximum brightness
    pinMode(TOUCH_PIN, INPUT_PULLUP);
    attachInterrupt(TOUCH_PIN, isr, FALLING);
    Serial.begin(9600);


    // Making it so, that the alarm will trigger an interrupt
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

    
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

  
    Serial.println("lets set the time!");

    timeClient.update();

    cur_min = timeClient.getMinutes();
    cur_hour = timeClient.getHours();
    seconds = timeClient.getSeconds();  

    DateTime datetime = DateTime(year, month, day, cur_hour, cur_min, seconds);

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

    // schedule an alarm 10 seconds in the future
    if(!rtc.setAlarm1(
            rtc.now() + TimeSpan(10),
            DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm will happen in 10 seconds!");
    }

    
    // following line sets the RTC to the date &amp; time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date &amp; time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  
}


void loop()
{

    DateTime now = rtc.now();

    if(now.minute() != cur_min)
    {
        
        cur_min = now.minute();
        if(cur_min < 10)
            display.showNumberDecEx(cur_min, 0b01000000, true, 2U, 2);
        else
            display.showNumberDecEx(cur_min, 0b01000000, false, 2U, 2);
        if(now.hour() != cur_hour)
        {
            if(timeClient.update())
            {
                cur_min = timeClient.getMinutes();
                cur_hour = timeClient.getHours();
                seconds = timeClient.getSeconds(); 

                DateTime datetime = DateTime(year, month, day, cur_hour, cur_min, seconds);

                rtc.adjust(datetime);

            }
            cur_hour = now.hour();
            if(now.hour() < 10)
                display.showNumberDecEx(cur_hour, 0b01000000, true, 2U, 0);
            else
                display.showNumberDecEx(cur_hour, 0b01000000, false, 2U, 0);
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
    if(touched == true)
    {   touched = false; 
        Serial.println("HIGH");
    }
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
    
}