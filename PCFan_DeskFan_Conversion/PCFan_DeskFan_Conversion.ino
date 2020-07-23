//Include libraries
#include <PWM.h>              //Changes PWM frequencies on certain pins
#include <Stepper.h>          //Lets you control stepper motors!
#include <FastLED.h>          //Lets you communicate with I.A. LEDs

//Pins for stepper motor connection
const byte stepIn1 = 8;               //Pins on included stepper module
const byte stepIn2 = 10;
const byte stepIn3 = 9;
const byte stepIn4 = 11;

//Pins for Fan PWM signals
const byte fanPin = 3;

//Pins for buttons or knobs
const byte btnLeftPin = 4;
const byte btnRightPin = 5;
const byte speedKnobPin = A0;

//Variables for FastLED library
#define NUM_LEDS 5
#define DATA_PIN 7
#define MAX_BRIGHTNESS 255

byte hue = 0;                   //Global hue for LEDs
byte sat = 255;                 //Global saturation for LEDs
byte val = MAX_BRIGHTNESS;      //Global brightness for LEDs

CRGB fanLEDs[NUM_LEDS];         //Container for addressing LED strip

//Variables for stepper motor usage
const int stepsPerRevolution = 2038;    //Steps for a 28BYJ-48 stepper motor
int stepperPosition = 0;                //Relative position of the motor shaft
int stepperSpeed = 6;                   //Speed at which consecutive steps happen

Stepper oscillatorMotor(stepsPerRevolution, stepIn1, stepIn2, stepIn3, stepIn4);
// ^Initialize stepper motor

//Variables needed for fan speed control
const int32_t pwmFrequency = 25000;     //Frequency of the pin in Hz
byte fanSpeed = 75;                     //Fan speed as a percentage (0-255);


void setup() 
{
  //Initialize Serial Monitor
  Serial.begin(9600);
  
  //Initialize button/knob pins
  pinMode(btnLeftPin, INPUT_PULLUP);
  pinMode(btnRightPin, INPUT_PULLUP);
  pinMode(speedKnobPin, INPUT);

  //Initialize fastLED library on our container
  FastLED.addLeds<WS2812B, DATA_PIN>(fanLEDs, NUM_LEDS);
  FastLED.clear();                                        //Turn off any lingering lights
  
  //Begin adjusting the PWM frequency
  InitTimersSafe();
  
  bool success = SetPinFrequencySafe(fanPin, pwmFrequency);   //Check if successful PWM change
  if(success)
  {
    Serial.println("Changing of PWM frequency successful.");
  }

  //Set default stepper rotation speed
  oscillatorMotor.setSpeed(stepperSpeed);
  
}

void loop() 
{
  
}
