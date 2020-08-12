//Included libraries
#include <PWM.h>              //Changes PWM frequencies on certain pins
#include <Stepper.h>          //Lets you control stepper motors
//#include <FastLED.h>          //Lets you communicate with I.A. LEDs
#include <DHT.h>              //Lets you utilize DHT11 temp sensor

//Pins for stepper motor connection
const byte stepIn1 = 7;               //Pins on included stepper module
const byte stepIn2 = 8;
const byte stepIn3 = 9;
const byte stepIn4 = 10;

//Pins for Fan PWM signals
const byte fanPin = 3;

//Pins for buttons or knobs
const int btnLeftPin = A5;
const int btnRightPin = A4;
const byte btnOscillatePin = 12;
const byte speedKnobPin = A0;
const int voltLowBnd = 0;
const int voltUpBnd = 1017;

//Pins for RGB LED
const byte ledRedPin = 5;
const byte ledGreenPin = 6;
const byte ledBluePin = 11;

//Variables for FastLED library
//#define NUM_LEDS 5
//#define DATA_PIN 7
//#define MAX_BRIGHTNESS 255

//byte hue = 0;                   //Global hue for LEDs
//byte sat = 255;                 //Global saturation for LEDs
//byte val = MAX_BRIGHTNESS;      //Global brightness for LEDs

//CRGB fanLEDs[NUM_LEDS];         //Container for addressing LED strip

//Variables for stepper motor usage
const int stepsPerRev = 2038;             //Steps for a 28BYJ-48 stepper motor
int stepperPosition = 0;                  //Relative position of the motor shaft
int stepperSpeed = 3;                     //Speed at which consecutive steps happen
bool isOscillating = false;               //Variable to determine if fan should oscillate
int stepsPerLoop = 5;

Stepper oscillatorMotor(stepsPerRev, stepIn1, stepIn3, stepIn2, stepIn4);
// ^Initialize stepper motor

//Variables needed for fan speed control
const int32_t pwmFrequency = 25000;     //Frequency of the pin in Hz
int fanSpeed = 75;                      //Fan speed as a percentage (0-255);
const int fanSpdLowBnd = 50;
const int fanSpdUpBnd = 255;

//Variables needed for millis() timing, values in milliseconds
double currentTime = 0;                       //Current system run time
double changeSpeedLast = 0;                   //Last time the change speed function ran
const double changeSpeedDelay = 200;          //Delay to wait between changing speed
double oscillateToggleLast = 0;               //Last time oscillation toggle was checked
const double oscillateToggleDelay = 500;      //Delay to wait between checking toggle
double oscillateLast = 0;                     //Last time fan oscillated
const double oscillateDelay = 25;             //Delay between stepper motor oscillations
const double tempUpdateDelay = 4000;          //Delay between temp reads
double tempUpdateLast = 0;                    //Last time temperature was read
const double turnLRDelay = 250;               //Delay between checking LR buttons
double turnLRLast = 0;                        //Last time buttons were checked
const double stepCoolDelay = 600000;          //10 minute max runtime for stepper motor
double stepCoolLast = 0;                      //Last time stepper motor was turned on

//Variables needed for button status/debouncing
bool btnOscillatePressed = false;
bool btnOscLast = false;
bool btnTurnLeftPressed = false;
bool btnLeftLast = false;
bool btnTurnRightPressed = false;
bool btnRightLast = false;

//Variables needed for DHT library usage
#define DHTPIN 2        //Data pin for DHT sensor
#define DHTTYPE DHT11   //Type of DHT Sensor used
DHT tempSensor(DHTPIN, DHTTYPE);

void setup() 
{
  //Initialize Serial Monitor
  Serial.begin(9600);
  
  //Initialize button/knob pins
  pinMode(btnLeftPin, INPUT_PULLUP);
  pinMode(btnRightPin, INPUT_PULLUP);
  pinMode(btnOscillatePin, INPUT_PULLUP);
  pinMode(speedKnobPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);

  //Initialize fastLED library on our container
  //FastLED.addLeds<WS2812B, DATA_PIN, GRB>(fanLEDs, NUM_LEDS);
  //FastLED.clear();                                        //Turn off any lingering lights
  
  //Begin adjusting the PWM frequency
  InitTimersSafe();
  
  bool success = SetPinFrequencySafe(fanPin, pwmFrequency);   //Check if successful PWM change
  if(success)
  {
    //If PWM update successful, blink the onboard UNO LED multiple times
    for(int i = 0; i <4; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }
  
  //Set default stepper rotation speed
  oscillatorMotor.setSpeed(stepperSpeed);

  //Enable usage of the DHT sensor
  tempSensor.begin();
  
  //Run function for initial fan startup
  FirstStartup();
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
      btnOscLast = 1;
      if(isOscillating == true)
      {
        stepCoolLast = currentTime;
      }
      else
      {
        StepperPowerDown();
      }
    }

    oscillateToggleLast = millis();
  }

  if((isOscillating == true) && (currentTime - stepCoolLast) >= stepCoolDelay)
  {
    //Stop the stepper motor from running after 10 minutes of running
    //Gives it time to cool down (before melting attached hot glue!)
    isOscillating = false;
    StepperPowerDown();
  }
  
  if(btnOscLast == 1)     //Debounce oscillation toggle button
  {
    if(digitalRead(btnOscillatePin) == HIGH)
    {
      btnOscLast = 0;
      delay(25);
    }
  }

  if((isOscillating == true) && ((currentTime - oscillateLast) >= oscillateDelay))
  {
    Oscillate();
    oscillateLast = millis();
  }

  if((currentTime - turnLRLast) >= turnLRDelay)
  {
    //Turn the fan head left or right if buttons are pressed
    if(digitalRead(btnLeftPin) == LOW)
    {
      TurnLeft();
    }
    else if(digitalRead(btnRightPin) == LOW)
    {
      TurnRight();
    }
    turnLRLast = currentTime;
  }

  if((currentTime - tempUpdateLast) >= tempUpdateDelay)
  {
    //Update the temperature read from the DHT sensor
    DisplayTemperature();
    tempUpdateLast = currentTime;
  }
}

//Function that runs at startup. Fans need high initial speed percentage
void FirstStartup()
{
  //Set fan speed to high speed to kick start them
  pwmWrite(fanPin, 200);
  delay(3500);

  //Get the speed that is set by the potentiometer knob
  UpdateFanSpeed();

  //Set the current stepper motor position to its midpoint of rotation.
  stepperPosition = (stepsPerRev/4)/2;

  //Make the LED go rainbow to show first startup is done (blocking method)
  for(int redVal = 0; redVal <=255; redVal++)
  {
    analogWrite(ledRedPin, redVal);
    delay(1);
  }
  for(int greenVal = 0; greenVal <=255; greenVal++)
  {
    analogWrite(ledGreenPin, greenVal);
    delay(1);
  }
  for(int redVal = 255; redVal >=0; redVal--)
  {
    analogWrite(ledRedPin, redVal);
    delay(1);
  }
  for(int blueVal = 0; blueVal <=255; blueVal++)
  {
    analogWrite(ledBluePin, blueVal);
    delay(1);
  }
  for(int greenVal = 255; greenVal >=0; greenVal--)
  {
    analogWrite(ledGreenPin, greenVal);
    delay(1);
  }
  for(int redVal = 0; redVal <=255; redVal++)
  {
    analogWrite(ledRedPin, redVal);
    delay(1);
  }
  for(int blueVal = 255; blueVal >=0; blueVal--)
  {
    analogWrite(ledBluePin, blueVal);
    delay(1);
  }
  for(int redVal = 255; redVal >=0; redVal--)
  {
    analogWrite(ledRedPin, redVal);
    delay(1);
  }
}

//Function that reads analog voltage, converts to value from 0-255
int CalculateFanSpeed()
{
  int voltage = analogRead(speedKnobPin);       //Analog read checks voltage level on A0
  //Serial.println(voltage);
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
  if((stepperPosition < 0) || (stepperPosition > (stepsPerRev/4)))
  {
    stepsPerLoop = -stepsPerLoop; //Invert the direction of rotation
    delay(1000);
  }
  stepperPosition += stepsPerLoop;
  oscillatorMotor.step(stepsPerLoop);
  delay(1);
}

void TurnLeft()
{
  //This will turn the fan left, regardless of its saved position
  //Helps to recenter fan
  int tempSteps = abs(stepsPerLoop);
  oscillatorMotor.step(-tempSteps);
  delay(1);
}

void TurnRight()
{
  //This will turn the fan right, regardless of its saved position
  //Helps to recenter fan
  int tempSteps = abs(stepsPerLoop);
  oscillatorMotor.step(tempSteps);
  delay(1);
}

void StepperPowerDown()
{
  //Turn all the pins going to the stepper motor controller LOW
  //Stops them from wasting power/creating heat
  digitalWrite(stepIn1, LOW);
  digitalWrite(stepIn2, LOW);
  digitalWrite(stepIn3, LOW);
  digitalWrite(stepIn4, LOW);
}

void DisplayTemperature()
{
  //Update the temperature read from the DHT sensor
  float humidity = tempSensor.readHumidity();
  float temperatureF = tempSensor.readTemperature(true);
  int hif = (int) tempSensor.computeHeatIndex(temperatureF, humidity);

  //Translate the temmperature value into a color on an RGB LED
  const int tempLowerBound = 77;
  const int tempUpperBound = 90;

  int tempColorRed = map(hif, tempLowerBound, tempUpperBound, 0, 255);
  int tempColorGreen = map(hif, tempUpperBound, tempLowerBound, 0, 255);

  if(tempColorRed > 255)
    tempColorRed = 255;
  else if(tempColorRed < 0)
    tempColorRed = 0;

  if(tempColorGreen > 255)
    tempColorGreen = 255;
  else if(tempColorGreen < 0)
    tempColorGreen = 0;
  
  analogWrite(ledRedPin, tempColorRed);
  analogWrite(ledGreenPin, tempColorGreen);

  Serial.print("Current humidity: ");
  Serial.println(humidity);
  Serial.print("Current Temperature: ");
  Serial.println(temperatureF);
  Serial.print("Heat index: ");
  Serial.println(hif);
}
