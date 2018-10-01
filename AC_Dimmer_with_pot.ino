// Written by Scott Driscoll, public domain - share, copy, etc. No warranty.
// 2018, Sept 30
// Youtube with part links: https://youtu.be/DzqvI6oM0-c

// This script dims an LED thru a triac according to time provided by a real time clock (DS3231) in order to simulate a
// skylight. It doesn't take into account time of year.

// requires TimerOne & RTC_DS3231 library.

// Sunrise to sunset is 7:30am to 7:21pm on Oct 1st
// going to ramp from off (MORNING_START) to full brightness (MORNING_STOP), then back down EVENING_START to EVENING_STOP

float MORNING_START = 6.75; // military time, 24 hours
float MORNING_STOP = 9.0;
float EVENING_START = 17.0; // 18 = 6pm
float EVENING_STOP = 20.0; // 20 = 8pm

/*
float MORNING_START = 1; // military time, 24 hours
float MORNING_STOP = 11;
float EVENING_START = 13; // 18 = 6pm
float EVENING_STOP = 23; // 20 = 8pm
*/

/*
 AC_Dimmer_with_pot
 
 This sketch is a sample sketch using the ZeroCross Tail(ZCT) to generate a sync
 pulse to drive a PowerSSR Tail(PSSRT) for dimming ac lights.
 
 Download TimerOne.h from the Arduino library and place in the folder applicable
 to your operating system.
 
 Connection to an Arduino Duemilanove:
 1. Connect the C terminal of the ZeroCross Tail to digital pin 2 with a 10K ohm pull up to 5V.
 2. Connect the E terminal of the ZeroCross Tail to GND.
 3. Connect the center terminal of a 10K ohm potentiometer to ADC0. Also connnect the CW 
 terminal to 5V and the CCW terminal to GND.
 4. Connect the PowerSSR Tail +in terminal to Digital pin 4 and the -in terminal to GND.
 
*/


#include <TimerOne.h>
#include <Wire.h>
#include "RTClib.h"

#include <TimerOne.h>

String inString = ""; 

RTC_DS3231 rtc;

bool led_on = false;
bool testmode = false;
unsigned long print_cycle_time = 2000;

volatile int i=0;               // Variable to use as a counter
volatile boolean zero_cross=0;  // Boolean to store a "switch" to tell us if we have 
                                // crossed zero
int PSSR1 = 4;                  // PSSRT connected to digital pin 4
volatile unsigned long dim = 8200;                   // Default Dimming level (0-128)  0 = on, 128 = off if 
// 107 is lowest value                                // unable to find pot
unsigned long freqStep = 1000000;   // was 73           // Adjust this value for full off PSSRT when pot
                                // is fully CCW; you may have flicker at max CW end
int pot = 0;                    // External potentiometer connected to analog input pin 0
int LED = 0;                    // Arduino board LED on digital pin 13

int is_on = 0;
float brightness_setpoint = 50.0;

bool triac_on = false;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

float compare_time = 0.0;
int j = 0;

void setup()
{
  #ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  



  Serial.begin(115200);
  Serial.println("Start");
 pinMode(LED_BUILTIN, OUTPUT);
 pinMode(4, OUTPUT);                              // Set PSSR1 pin as output
 attachInterrupt(0, zero_cross_detect, RISING);   // Attach an Interupt to Pin 2 
                                                 // (interupt 0) for Zero Cross Detection
 Timer1.initialize(freqStep);
 Timer1.attachInterrupt(dim_check,freqStep);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
    if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

// This gets called when the AC signal cross zero and sets a delay for when the triac should be turned on thru Timer1.
// The longer the delay, the dimmer the LED as the lower percentage of each cycle the triac is held on.
void zero_cross_detect() 
{
   zero_cross = 1;
   // set the boolean to true to tell our dimming function that a zero cross has occured
   Timer1.setPeriod(dim);
} 



// Timer1 triggered
void dim_check() {     
  if (!triac_on) {
      if (dim < 7999) {
        digitalWrite(PSSR1, HIGH);
      }  
      triac_on = true;   
      Timer1.setPeriod(8000 - dim);     // call this function again to turn off the triac signal near the end of the cycle. 
      // the total cycle is about 8000us, so I just subtract my dim from 7500 to get the off time.
  }
  else {
    digitalWrite(PSSR1, LOW); 
    triac_on = false; 
    Timer1.setPeriod(1000000);
  }   
}



unsigned long last_run;
unsigned long last_fake_time_increment;
void loop()                        // Main loop
{
  int incoming_byte;
   /* if (Serial.available() > 0) {
      incoming_byte = Serial.read();
      Serial.println(incoming_byte);
      if (incoming_byte == 117) { // u
         brightness_setpoint += .1;
      }
      else {
        brightness_setpoint -= .1;
      }
      
    }
    */

  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
    if (inChar == '\n') {
      //Serial.print("Value:");
      //Serial.println(inString.toInt());
      //Serial.print("String: ");
      //Serial.println(inString);
      // clear the string for new input:
      
      brightness_setpoint = (float)(inString.toInt()) / 10.0;
      inString = "";
    }
  }

    
  //dim = linearize(analogRead(pot)/10);         // Read the pot value
   // brightness_setpoint = mapf(analogRead(pot), 0, 1024, 0, 100);
    
    // every 2 seconds, poll RTC to get time and adjust brightness.
    if (millis() - last_run > print_cycle_time) {
      set_brightness();
      last_run = millis();
  
    }
    //compare_time = 19.0;

    // speed thru day for testing
    
    if (testmode) {
      if (millis() - last_fake_time_increment > 10) {
        compare_time += .01;
        if (compare_time > 24.0) {
          compare_time = 0.0;
        }
        last_fake_time_increment = millis();
      }
    }
}

// this function takes about 1ms. Not sure if either this function, which communicates with the RTC,
// will cause a flicker in the dimmer. I suspect the interrupt will interrupt this function. Not
// sure if it will mess up the communication with the RTC. Might need to call this function from the
// interrupt (dim_check) before or after the on pulse is triggered: if on pulse is due past 50% of cycle, 
// call early, if on pulse is less than 50% call after.
void set_brightness() {
  
  DateTime now = rtc.now(); // this call takes 1ms
  if (!testmode) {
    compare_time = now.hour(); // this function doesn't take any time
    compare_time += (float)(now.minute())/ 60.0; // this gets us a single float for the hour with a decimal for percent 
  }
  
  //compare_time = 14.0;
  // thru the hour based on the minutes.
  // get time of day 
  //Serial.println(compare_time);

 
  // ramp up time
  if (compare_time >= MORNING_START && compare_time < MORNING_STOP) {
    brightness_setpoint = mapf(compare_time, MORNING_START, MORNING_STOP, 0.0, 100.0);
    
  }
  else if (compare_time >= MORNING_STOP && compare_time < EVENING_START) {
    // full brightness time
    brightness_setpoint = 100.0;
    
  }
  else if (compare_time >= EVENING_START && compare_time < EVENING_STOP) {
    // ramp down
    brightness_setpoint = mapf(compare_time, EVENING_START, EVENING_STOP, 100.0, 0.0);
    
  }
  else { // off otherwise
    
    brightness_setpoint = 0;
  }
 
 
  
  // convert to a dim value needed by dimmer
  dim = linearize(brightness_setpoint);


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
    
    Serial.print("MORNING_START: ");
    Serial.print(MORNING_START);
    Serial.print(", MORNING_STOP: ");
    Serial.print(MORNING_STOP);
    Serial.print(", EVENING_START: ");
    Serial.print(EVENING_START);
    Serial.print(", EVENING_STOP: ");
    Serial.println(EVENING_STOP);
    
    Serial.print("Current brightness (0- 100): ");
    Serial.println(brightness_setpoint);
    //Serial.print(", on trigger delay per cycle (ms): ");
    //Serial.println(dim);
    Serial.println();
    
    
}

// takes an input 0-100 and creates an output that makes the LED behave more linearly according to how humans perceive the light.
// smaller return is brighter
// 8100 is off, 500 is brightest setting
float linearize(float i) {
  unsigned long output = 8100; // effectively off
  if (i <= 0.09) {
    return 8000;
  }
  if (i < 10.0) {
    return mapf(i, .09, 9.99, 7900, 7500);
    
  }
  else if (i < 50.0) {
   return mapf(i, 10, 50.0, 7500, 6000);
  }
  else if (i< 90.0) {
    return mapf(i, 50, 90.0, 6000, 3000);
  }
  else if (i<98.0) {
    return mapf(i, 90, 98.0, 3000, 500.0);
  }
  else 
    return 500.0;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



