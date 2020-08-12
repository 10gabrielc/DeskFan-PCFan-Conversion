//Included libraries
#include <PWM.h>              //Changes PWM frequencies on certain pins
#include <DHT.h>              //Lets you utilize DHT11 temp sensor

//Pins for Fan PWM signals
const byte fanPin = 3;

//Pins for buttons or knobs
const byte speedKnobPin = A0;
const int voltLowBnd = 0;
const int voltUpBnd = 1017;

//Variables needed for fan speed control
const int32_t pwmFrequency = 25000;     //Frequency of the pin in Hz
int fanSpeed = 75;                      //Fan speed as a percentage (0-255);
const int fanSpdLowBnd = 50;
const int fanSpdUpBnd = 255;

//Variables needed for millis() timing, values in milliseconds
double currentTime = 0;                       //Current system run time
double changeSpeedLast = 0;                   //Last time the change speed function ran
const double changeSpeedDelay = 200;          //Delay to wait between changing speed
const double tempUpdateDelay = 4000;          //Delay between temp reads
double tempUpdateLast = 0;                    //Last time temperature was read

//Variables needed for button status/debouncing

//Variables needed for DHT library usage
#define DHTPIN 2        //Data pin for DHT sensor
#define DHTTYPE DHT11   //Type of DHT Sensor used
DHT tempSensor(DHTPIN, DHTTYPE);

void setup() 
{
  //Initialize Serial Monitor
  Serial.begin(9600);
  
  //Initialize button/knob pins
  pinMode(speedKnobPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

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
