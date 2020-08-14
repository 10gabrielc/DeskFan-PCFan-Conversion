//Included libraries
#include <PWM.h>                //Changes PWM frequencies on certain pins
#include <DHT.h>                //Lets you utilize DHT11 temp sensor
#include <FastLED.h>            //Lets you control individually addressable LEDs

//Pins used on the Arduino UNO
const byte fanPin = 3;            //Pin for PC Fan PWM connection
const byte speedKnobPin = A0;     //Variable voltage pin on potentiometer
const byte btnModePin = 8;        //Button to change modes for the LEDs
const byte btnSettingPin = 9;     //Button to change a setting of a mode for the LEDs

//Variables needed for fan speed control
const int32_t pwmFrequency = 25000;     //Frequency of the pin in Hz
const int fanSpdLowBnd = 50;            //Minimum PWM fan speed
const int fanSpdUpBnd = 255;            //Maximum PWM fan speed
const int voltLowBnd = 0;               //Minimum voltage reading value
const int voltUpBnd = 1017;             //Maximum voltage reading value
int fanSpeed = 75;                      //Fan speed as a percentage (0-255);

//Variables needed for button usage
bool btnModeLast = 0;             //Stores the last state of the mode button
bool btnSettingLast = 0;          //Stores last state of setting button

//Variables needed for various coded modes
byte currentMode = 0;             //Stores current mode being displayed on LEDs          
byte staticColor = 0;             //Stores which static color choice to display
bool rainbowShiftOn = false;

//Variables needed for millis() timing, values in milliseconds
double changeSpeedLast = 0;                   //Last time the change speed function ran
const double changeSpeedDelay = 200;          //Delay to wait between changing speed
const double tempUpdateDelay = 4000;          //Delay between temp reads
double tempUpdateLast = 0;                    //Last time temperature was read
double rainbowShiftLast = 0;                  //Last time rainbow shift ran
const double rainbowShiftDelay = 25;          //Delay betwen rainbow shift loops running
double btnModeCheckLast = 0;                  //Last time mode button was checked for a press
const double btnModeCheckDelay = 300;         //Delay between mode button press checks
double btnSettingCheckLast = 0;               //Last time settings button was checked for press
const double btnSettingCheckDelay = 300;      //Delay between setting button press checks

//Variables needed for DHT library usage
#define DHTPIN 2                        //Data pin for DHT sensor
#define DHTTYPE DHT11                   //Type of DHT Sensor used
DHT tempSensor(DHTPIN, DHTTYPE);        //Create DHT11 sensor object

//Variables and objects needed for LEDs
#define NUM_LEDS 9                      //Total number of LEDs
#define DATA_PIN 7                      //Arduino pin used for data
#define MAX_BRIGHTNESS 255              //Maximum brightness possible (0-255)
CRGB allLEDs[NUM_LEDS];                 //Create object to represent LED chain
byte globalHue = 0;                     //Stores current LED hue
byte globalSat = 255;                   //Stores current LED saturation
byte globalVal = MAX_BRIGHTNESS/2;      //Stores current LED brightness


//Color Definitions for FastLED library
#define colorRed 0
#define colorOrange 18
#define colorYellow 30
#define colorGreen 90
#define colorMiku 100
#define colorBlue 128
#define colorPurple 150
#define colorPink 180

void setup() 
{
  //Initialize button/knob pins
  pinMode(speedKnobPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(btnModePin, INPUT_PULLUP);
  pinMode(btnSettingPin, INPUT_PULLUP);

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
  
  //Enable usage of the DHT sensor
  tempSensor.begin();

  //Initialize FastLED library/chain
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(allLEDs, NUM_LEDS);
  FastLED.clear();        //Sets all LEDs to black
  FastLED.show();         //Pushes new LED data to LEDs
  
  //Run function for initial fan startup
  FirstStartup();
}

void loop() 
{
  double currentTime = millis();     //Store current system time in double variable

  //Check for speed changes after delay period
  if((currentTime - changeSpeedLast) >= changeSpeedDelay)
  {
    //Check current speed setting on knob and update fan speed accordingly
    UpdateFanSpeed();
    changeSpeedLast = millis();
  }

  if((currentTime - tempUpdateLast) >= tempUpdateDelay)
  {
    //Update the temperature read from the DHT sensor
    DisplayTemperature();
    tempUpdateLast = currentTime;
  }

  if((currentTime - btnModeCheckLast) >= btnModeCheckDelay)
  {
    if((digitalRead(btnModePin) == 0) && (btnModeLast == 0))
    {
      //Button is pressed, and we must change modes
      if(currentMode == 1)
        currentMode = 0;          //Roll mode back to zero
      else    
        currentMode++;            //Change to next mode

      switch(currentMode)         //Change LED behavior based off current mode
      {
        case 0:
          StaticColorLED();       //Set static color using function
          rainbowShiftOn = false; //Disable Rainbow effect
          break;
        case 1:
          rainbowShiftOn = true;  //Enable rainbow effect loop
          break;
        default:
          FastLED.clear();
          FastLED.show();
      }

      btnModeLast = 1;      //Set last mode to on to enable debouncing
    }
  }

  if(btnModeLast = 1)      //Handle debouncing of mode button
  {
    if(digitalRead(btnModePin == 1))
    {
      btnModeLast = 0;
      delay(50);
    }
  }

  if((currentTime-btnSettingCheckLast)>=btnSettingCheckDelay)
  {
    if((digitalRead(btnSettingPin) == 0) && (btnSettingLast == 0))
    {
      //Do something different based on the current mode
      switch(currentMode)         //Change LED behavior based off current mode
      {
        case 0:
          StaticColorLED();       //Call function again so that it changes color
        case 1:
          rainbowShiftOn = !rainbowShiftOn;   //Toggle the rainbow shifting on or off
          break;
        default:
          FastLED.clear();
          FastLED.show();
      }

      btnSettingLast = 1;     //Set last state to on to enable debouncing
    }
  }

  if(btnSettingLast = 1)      //Handle debouncing of setting button
  {
    if(digitalRead(btnSettingPin == 1))
    {
      btnSettingLast = 0;
      delay(50);
    }
  }
  
  if((rainbowShiftOn==true) && ((currentTime-rainbowShiftLast)>=rainbowShiftDelay))
  {
    RainbowShiftLED();
    rainbowShiftLast = currentTime;
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

  //Make the LEDs go rainbow for a loop
  for(int i = 0; i<256;i++)
  {
    RainbowShiftLED();
  }
  FastLED.clear();
  FastLED.show();
}

//Function that reads analog voltage, converts to value from 0-255
int CalculateFanSpeed()
{
  int voltage = analogRead(speedKnobPin);       //Analog read checks voltage level on A0
  int speedConversion = map(voltage, voltLowBnd, voltUpBnd, fanSpdLowBnd, fanSpdUpBnd); //Translation using map function
  return speedConversion;                       //Send back new speed
}

//Function that changes the speed of the fans using PWM at 25KHz
void UpdateFanSpeed()
{
  fanSpeed = CalculateFanSpeed();   //Update global fan speed variable
  pwmWrite(fanPin, fanSpeed);       //Update fan speed
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

  //Serial.println(tempColorRed);
  //Serial.println(tempColorGreen);
  //Serial.print("Current humidity: ");
  //Serial.println(humidity);
  //Serial.print("Current Temperature: ");
  //Serial.println(temperatureF);
  //Serial.print("Heat index: ");
  //Serial.println(hif);
}

void StaticColorLED()
{
  //LED colors are changed based on a pre-generated set of colors
  byte hueToSet = 0;
  byte satToSet = globalSat;
  switch(staticColor)
  {
    case 0:
      hueToSet = colorRed;      //Red color
      break;
    case 1:
      hueToSet = colorOrange;   //Orange color
      break;
    case 2:
      hueToSet = colorYellow;   //Yellow color
      break;
    case 3:
      hueToSet = colorGreen;    //Green color
      break;
    case 4:
      hueToSet = colorMiku;     //Miku color
      break;
    case 5:
      hueToSet = colorBlue;     //Blue color
      break;
    case 6:
      hueToSet = colorPurple;    //Purple color
      break;
    case 7:
      hueToSet = colorPink;     //Pink color
      break;
    case 8:                     
      satToSet = 0;             //White color using minimum saturation
      break;
    default:                    //How did we get here?
      FastLED.clear();          //Set LEDs to black
  }
  fill_solid(allLEDs, NUM_LEDS, CHSV(hueToSet, satToSet, globalVal));
  FastLED.show();

  if(staticColor == 8)
    staticColor = 0;
  else
    staticColor++;
}

void RainbowShiftLED()
{
  //Increment the hue of the leds in the same line they are set!
  fill_solid(allLEDs, NUM_LEDS, CHSV(globalHue++, globalSat, globalVal));
  FastLED.show();
}
