//#define ESP32 0



#include <Arduino.h>
// Include the library:
#include <TM1637Display.h>
#include "RTClib.h"

#include <NTPClient.h>
#include <Timezone.h> 


#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>

#include <RotaryEncoder.h>

#include "private.h"
#include "mytime.h"

// Device can be found on the network using this name
#define NAME "Puzzle_3"


void setup_Time(void);
int get_TimeOffset(void);
void get_Time(void);
void update_Time(void);

StaticJsonDocument<5000> docWeather;
char api_str[120];
HTTPClient http;
WiFiClient client;
WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org");


// Define the connections pins:
#define CLK_DIG 14
#define DIO_DIG 12

#define TOUCH_PIN 0


TM1637Display display(CLK_DIG, DIO_DIG);

RTC_DS3231 rtc;
// the pin that is connected to SQW
//#define CLOCK_INTERRUPT_PIN 13
#define CLOCK_SCL 5
#define CLOCK_SDA 4

bool rotated = false;


int numCounter = 0;

uint8_t cur_min, cur_hour;

uint8_t seconds;

uint8_t alarm_hour; 
uint8_t alarm_min;


enum cause{
    nocause = 0,
    generalCheck, 
    touch
}wakeupCause;

//TM1637Display display(CLK_DIG, DIO_DIG); //set up the 4-Digit Display.

DateTime alarm_time;
DateTime datetime;

int timeOffset;
int newtimeOffset;

bool touched = false;

DateTime timeNow;

#define FPM_SLEEP_MAX_TIME           0xFFFFFFF
 
// Required for LIGHT_SLEEP_T delay mode
extern "C" {
#include "user_interface.h"
}



void IRAM_ATTR isr() {

    wakeupCause = touch;

    Serial.print("HIGH");
    detachInterrupt(digitalPinToInterrupt(TOUCH_PIN));
}
/*
void IRAM_ATTR at2oclock() {
    Serial.println("ALARM");
    rtc.clearAlarm(1);
    rtc.disableAlarm(1);
    wakeupCause = generalCheck;
    detachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN));
}
*/
void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(CLK_DIG, INPUT_PULLUP);
    pinMode(DIO_DIG, INPUT_PULLUP);
    //pinMode(CLOCK_INTERRUPT_PIN, INPUT);
    pinMode(TOUCH_PIN, INPUT);

    display.setBrightness(0x0a); //set the diplay to maximum brightness



    pinMode(CLOCK_SCL, INPUT_PULLUP);
    pinMode(CLOCK_SDA, INPUT_PULLUP);

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

    setup_Time();

    timeOffset = get_TimeOffset();
    if(!(timeOffset == 3600 || timeOffset == 7200))
    {
        Serial.print("Wrong time offset!");
    }
    Serial.print("Time offset");
    Serial.print(timeOffset);

    timeClient.begin();
    //summer time...
    timeClient.setTimeOffset(timeOffset);
 
  
    Serial.println("lets set the time!");

    if(timeClient.update())
    {
        Serial.print("Updating RTC time..");
    }

    cur_min = timeClient.getMinutes();
    cur_hour = timeClient.getHours();
    seconds = timeClient.getSeconds();  

    timeClient.end();

    datetime = DateTime(2022, 3, 14, cur_hour, cur_min, seconds);

    rtc.adjust(datetime);

    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

/*
    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    alarm_min = 35;
    alarm_hour = 15;
    alarm_time = DateTime(year, month, day, alarm_hour, alarm_min, 0);

    if(!rtc.setAlarm1(
        alarm_time,
        DS3231_A1_Hour // this mode triggers the alarm when the seconds match. See Doxygen for other options
        )) {
            Serial.println("Error, alarm wasn't set!");
        }else {
            Serial.println("Alarm will happen at");
            Serial.println(alarm_hour);
            Serial.println(alarm_min);
    }
*/

    display.setBrightness(0,0);

    if(cur_min < 10)
            display.showNumberDecEx(cur_min, 0b01000000, true, 2U, 2);
    else
        display.showNumberDecEx(cur_min, 0b01000000, false, 2U, 2);

    if(cur_hour < 10)
        display.showNumberDecEx(cur_hour, 0b01000000, true, 2U, 0);
    else
        display.showNumberDecEx(cur_hour, 0b01000000, false, 2U, 0);

    wakeupCause = nocause;

    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), isr, ONHIGH);

    //attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), at2oclock, ONLOW);


    delay(1000);
    
}

 unsigned long time_alarm_show;
 bool show_alarm = false;
 bool switch_state = false;

void callback() {
  Serial.println("Callback");

}

void loop()
{
    Serial.print("Going to sleep");
    //attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), at2oclock, ONLOW);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), isr, ONHIGH);
         //wifi_station_disconnect(); //not needed
    gpio_pin_wakeup_enable(GPIO_ID_PIN(TOUCH_PIN), GPIO_PIN_INTR_HILEVEL);
    //gpio_pin_wakeup_enable(GPIO_ID_PIN(CLOCK_INTERRUPT_PIN), GPIO_PIN_INTR_LOLEVEL);
    wifi_set_opmode(NULL_MODE);
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    wifi_fpm_open();
    wifi_fpm_set_wakeup_cb(callback);
    wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
    delay(1000);


    Serial.print("Woke up again!");
    switch(wakeupCause)
    {
        case generalCheck:
            Serial.print("General Check!");
            delay(1000);
            timeNow = rtc.now();
            if(WiFi.status() != WL_CONNECTED)
            {
                WiFi.begin(SSID, PWD);
                // Wait while not connected
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(500);
                    Serial.print(".");
                }
            }
            //newtimeOffset = get_TimeOffset();
            /*
            //timeClient.begin();
            timeClient.setTimeOffset(timeOffset);

            if(timeClient.update())
            {
                Serial.print("Updating RTC time..");
        
                cur_min = timeClient.getMinutes();
                cur_hour = timeClient.getHours();
                seconds = timeClient.getSeconds();  

                uint16_t year = 2022;
                uint8_t month = 3;
                uint8_t day = 14;

                datetime = DateTime(year, month, day, cur_hour, cur_min, seconds);

                rtc.adjust(datetime);

            }
            timeClient.end();
            delay(10000);
            alarm_min = 36;
            alarm_hour = 15;
            alarm_time = DateTime(2022, 3, 14, alarm_hour, alarm_min, 0);
            if(!rtc.setAlarm1(alarm_time, DS3231_A1_Hour )) 
            {
                Serial.println("Error, alarm wasn't set!");
            }else {
                Serial.println("Alarm will happen at");
                Serial.println(alarm_hour);
                Serial.println(alarm_min);
            }*/
            delay(3000);
            break;
        case touch:
            Serial.print("Touch event");
            // turn on display
            display.setBrightness(7, 1);
            timeNow = rtc.now();
            cur_min = timeNow.minute();
            cur_hour = timeNow.hour();
            if(cur_min < 10)
                display.showNumberDecEx(cur_min, 0b01000000, true, 2U, 2);
            else
                display.showNumberDecEx(cur_min, 0b01000000, false, 2U, 2);
            if(timeNow.hour() < 10)
                display.showNumberDecEx(cur_hour, 0b01000000, true, 2U, 0);
            else
                display.showNumberDecEx(cur_hour, 0b01000000, false, 2U, 0);

            while(digitalRead(TOUCH_PIN))
            {
                timeNow = rtc.now();
                if(timeNow.minute() != cur_min)
                {
                    cur_min = timeNow.minute();
                    if(cur_min < 10)
                        display.showNumberDecEx(cur_min, 0b01000000, true, 2U, 2);
                    else
                        display.showNumberDecEx(cur_min, 0b01000000, false, 2U, 2);
                    if(timeNow.hour() != cur_hour)
                    {
                        cur_hour = timeNow.hour();

                        if(timeNow.hour() < 10)
                            display.showNumberDecEx(cur_hour, 0b01000000, true, 2U, 0);
                        else
                            display.showNumberDecEx(cur_hour, 0b01000000, false, 2U, 0);
                    }
                }
                delay(1000);
            }
            attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), isr, ONHIGH);
            //attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), at2oclock, ONLOW);
            delay(3000);
            // turn off display
            display.setBrightness(0, 0);
            display.showNumberDecEx(cur_min, 0b01000000, true, 2U, 2);
            display.showNumberDecEx(cur_hour, 0b01000000, false, 2U, 0);
            break;
        default:
        wakeupCause = nocause;
            break; 
    }
    wakeupCause = nocause;
    timeNow = rtc.now();
    Serial.print(timeNow.year(), DEC);
    Serial.print('/');
    Serial.print(timeNow.month(), DEC);
    Serial.print('/');
    Serial.print(timeNow.day(), DEC);
    Serial.print(" (");
    Serial.print(") ");
    Serial.print(timeNow.hour(), DEC);
    Serial.print(':');
    Serial.print(timeNow.minute(), DEC);
    Serial.print(':');
    Serial.print(timeNow.second(), DEC);
    Serial.println();
    delay(1000);
}
    

   

void setup_Time(void)
{
    // extract time zone...
    strcpy(api_str, "http://api.openweathermap.org/data/2.5/weather?q=Velber,de&cnt=3&units=metric&appid=");
    strcat(api_str, API);
}

int get_TimeOffset(void)
{
    http.begin(client, api_str);
    int httpCode = http.GET();
    int offset = 0;
    if(httpCode > 0)
    {
        DeserializationError errorhttp = deserializeJson(docWeather, http.getString());
        if(errorhttp)
        {
            Serial.print("deserialize Json failed");
        }
        else
        {
            offset = docWeather["timezone"];
        }
    }
    return offset;
}