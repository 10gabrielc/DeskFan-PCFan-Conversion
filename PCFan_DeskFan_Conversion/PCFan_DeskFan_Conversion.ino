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
const byte btnOscillatePin = 6;
const byte speedKnobPin = A0;
const int voltLowBnd = 0;
const int voltUpBnd = 1023;

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
bool isOscillating = false;             //Variable to determine if fan should oscillate
int stepsPerLoop = 5;

Stepper oscillatorMotor(stepsPerRevolution, stepIn1, stepIn2, stepIn3, stepIn4);
// ^Initialize stepper motor

//Variables needed for fan speed control
const int32_t pwmFrequency = 25000;     //Frequency of the pin in Hz
int fanSpeed = 75;                     //Fan speed as a percentage (0-255);
const int fanSpdLowBnd = 50;
const int fanSpdUpBnd = 255;

//Variables needed for millis() timing, values in milliseconds
double currentTime = 0;                       //Current system run time
double changeSpeedLast = 0;             //Last time the change speed function ran
const double changeSpeedDelay = 200;          //Delay to wait between changing speed

double oscillateToggleLast = 0;               //Last time oscillation toggle was checked
const double oscillateToggleDelay = 100;      //Delay to wait between checking toggle
double oscillateLast = 0;                     //Last time fan oscillated
const double oscillateDelay = 10;              //Delay between stepper motor oscillations

//Variables needed for button status/debouncing
bool btnOscillatePressed = false;
bool btnOscLast = false;

void setup() 
{
  //Initialize Serial Monitor
  Serial.begin(9600);
  
  //Initialize button/knob pins
  pinMode(btnLeftPin, INPUT_PULLUP);
  pinMode(btnRightPin, INPUT_PULLUP);
  pinMode(speedKnobPin, INPUT);

  //Initialize fastLED library on our container
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(fanLEDs, NUM_LEDS);
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

  UpdateFanSpeed();
}

void loop() 
{
  currentTime = millis();     //Update current system time

  //Check for speed changes after delay period
  if((currentTime - changeSpeedLast) >= changeSpeedDelay)
  {
    //Check current speed setting on knob and update fan speed accordingly
    UpdateFanSpeed();
    changeSpeedLast = millis();
  }

  if((currentTime - oscillateToggleLast) >= oscillateToggleDelay)
  {
    //Check to see if the button to turn on oscillation is pressed
    btnOscillatePressed = digitalRead(btnOscillatePin);       //Get button status

    //Check for button press, but utilize debounce
    if(btnOscillatePressed == 0 && btnOscLast == 0)
    {
      isOscillating = !isOscillating;
      //btnOscLast = 1;               //excluding debounce for now
    }

    oscillateToggleLast = millis();
  }

  if((currentTime - oscillateLast) >= oscillateDelay)
  {
    Oscillate();
    oscillateLast = millis();
  }
  
}

//Function that runs at startup. Fans need high initial speed percentage
void FirstStartup()
{
  //Set fan speed to high speed to kick start them
  pwmWrite(fanPin, 200);
  delay(1500);

  //Get the speed that is set by the potentiometer knob
  UpdateFanSpeed();

  //Set the current stepper motor position to its midpoint of rotation.
  stepperPosition = stepsPerRevolution / 4;
}

//Function that reads analog voltage, converts to value from 0-255
int CalculateFanSpeed()
{
  int voltage = analogRead(speedKnobPin);       //Analog read checks voltage level on A0
  int speedConversion = map(voltage, voltLowBnd, voltUpBnd, fanSpdLowBnd, fanSpdUpBnd); //Translation using map function
  return speedConversion;                              //Send back new speed
}

//Function that changes the speed of the fans using PWM at 25KHz
void UpdateFanSpeed()
{
  fanSpeed = CalculateFanSpeed();   //Update global fan speed variable
  pwmWrite(fanPin, fanSpeed);       //Update fan speed
  Serial.println(fanSpeed);
}

//Function for oscillating the fan head
void Oscillate()
{
  //Check if fan has reached an oscillatory bound
  if(stepperPosition <= 0 || stepperPosition >= (stepsPerRevolution/2))
  {
    stepsPerLoop = -stepsPerLoop; //Invert the direction of rotation
  }

  oscillatorMotor.step(stepsPerLoop);
}

//Function that changes the color of the LEDs (static)
void setLEDcolors()
{
  
}
