/*
PRE-Vent Ventilator Project
Developed by NSWC Panama City in response to the COVID-19 Pandemic
*/          

// Date Last Modified: 10/25/2020
  

#include <Servo.h>
#include <TFT_HX8357.h>
TFT_HX8357 tft = TFT_HX8357();


#define SWAP(a, b) {uint16_t tmp = a; a = b; b = tmp;}

uint8_t Orientation = 1;    // 0 Portrait 1 Landscape   2 Inverted Portrait 3 Inverted Landscape

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFC00
#define blankingColor BLACK

// -- -- -- -- Pin Definitions -- -- -- -- //

#define RPMSensorPin          21
//#define relayPin            45
#define tonePin               47
#define motorPWMOut           12
#define tidalArmPin            9
#define valueSwitchPin         7
#define alarmSwitchPin         6

#define tidalVolume_Pot_Pin   A10
#define O2_Pot_Pin            A11
#define IE_Pot_Pin            A12
#define motorSpeedPotPin      A13
#define O2sensorPin           A14
#define pressureSensorPin     A15


/// -- -- -- -- -- -- -- -- -- -- -- -- -- //


// -- -- -- -- Preset Ranges of Values -- -- -- -- //

#define pressureMin -75.0   // -75 cm H2O to 75 cm H20
#define pressureMax  75.0

//#define minimumServoVal 75 // minimum safe PWM value
#define O2Baseline 20.8

#define testDelayTime 60000

/// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- ///


///////////////////////////////
// -- -- -- -- Need to be sorted -- -- -- -- //

int ID;
int changeCounter = 0;
int setIERatio = 0;
int FiO2 = 0;
int prev_FiO2 = 0;
int prev_displayFiO2 = 0;
int tidalVolume = 0;
int prev_tidalVolume = 0;
float prev_displayTidalVolume = 0.0;
int IERatio = 0;
int prev_displayIERatio = 0;
int prev_IERatio = 0;
int motorPWM = 180;
int prev_motorPWM = 0;
int motorPWMOffset = 0;
int tuning = -20;
int pwmOut = 0;
volatile int counter = 0;
volatile int hit = 0;
int mlOffest = 0;

float alpha = 0.0;
float RPM = 0.0;
float CycleTime = 1.0;
float BPM = 0.0;
float prev_BPM = 0.0;
int   setBPM = 0;
int   prev_setBPM = 0;
float prev_displayBPM = 0.0;
int   prev_displaySetBPM = 0;
float motorIdleTime = 0.0;
float inspirationTime = 0;
float prev_insperationTime = 0.0;
float prev_displayInsperationTime = 0.0;
float inspiredTimeSetpoint = 0;
float BPMSetpoint = 0;

float inspSensitivity = 0.1;
//float inspTimeDeviation = 0.0;
float RMV_LPM = 0.0;
float Oxygen_LPM = 0.0;
float prev_Oxygen_LPM = 0.0;
float prev_displayOxygen_LPM = 0.0;
float momentArmOffset = 0.0;
float pressure = 0.0;
float PIPpressure = 0.0;
float prevPressure = 1.0;
float prev_pressureAverage = 0.0;
//float avgPressure = 0.0;
//float actualO2 = 0.0;
//float avgActualO2 = 0.0;
unsigned long previousScreenUpdateTime = 0;

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- //




// -- -- -- -- Average Variables -- -- -- -- //

const int numOxyReadings = 10;
const int numPressureReadings = 10;
const int numInspiredReadings = 20;

float o2Readings[numOxyReadings];            //Averaging Arrays
float pressureReadings[numPressureReadings];
float inspiredTimeReadings[numInspiredReadings];

int o2Index = 0;                          //Array Indices
int pressureIndex = 0;
int inspiredTimeIndex = 0;

float o2Total = 0.0;                      //Totaling Variables
float pressureTotal = 0.0;
float inspiredTimeTotal = 0.0;

float o2Average = 0.0;                    //Averaging Variables
float prev_o2Average = 0.0;
float prev_displayO2Average = 0.0;
float pressureAverage = 0.0;
float prev_displayPressureAverage = 0;
float inspiredTimeAverage = 0.0;

float O2calibrationFactor = 1;
float pressureCalibrationFactor = 1;

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- //



// -- -- -- -- Timing Variables -- -- -- -- //

unsigned long currentTime = 0;
//unsigned long steadyRunningTime = 10000;

unsigned long alarmDelayTime = testDelayTime;//300000;     // Alarm delay from start                     ------> DELAY VAR
unsigned long alarmDelayOffset = 0;       // Alarm time offset if reset occurs later    ------> RESET VAR

unsigned long lockInTime = testDelayTime;//300000;        // lock in time at 5 mins                     ------> DELAY VAR

unsigned long inspDelay = testDelayTime;//300000;         // set time to lock in inspiration            ------> DELAY VAR
unsigned long inspOffset = 0;             // inspiration offset if reset occurs later   ------> RESET VAR

unsigned long  previousTime = 0;          // Time Logging Variables
//unsigned long  previousTime2 = 0;
unsigned long  endCycleTime = 0;
unsigned long  startCycleTime = 0;
unsigned long  lastInterruptTime = 0;

volatile unsigned long last_interrupt_time = 0;

//unsigned long rollingPressureTime = 10000;
//unsigned long rollingPressureLowTime = 0;

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- //



// -- -- -- -- Booleans -- -- -- -- //

bool inspSet = false;
bool inputChanged = false;
bool state = false;
bool pressureTooLow = false;
bool motor = true;

//volatile bool screenBool = false;           // Booleans made volitile if used with an interrupt
//volatile bool temp = false;
volatile bool switchHit = false;
bool prevSet = false;

// -- -- -- -- -- -- -- -- -- -- -- //
//////////////////////////////////////
/// -- -- -- -- Alarms -- -- -- -- ///

bool pressureAlarm = false;
bool O2Alarm = false;
bool inspirationAlarm = false;
bool alarm = false;
bool locked = false;
bool wait = false;


/// -- -- -- -- -- -- -- -- -- -- ///

/// -- -- Display Variables -- -- ///
int tidalVolumeDisplay = 0;
int FiO2Display        = 0;
int IERatioDisplay     = 0;
int setBPMDisplay      = 0;
/// -- -- -- -- -- -- -- -- -- -- ///


//Button trigger(21, 50, true, false);                        // <--------------------------- OBJECT FROM DEBOUNCING LIBRARY FOR POLLING --------- ||

Servo tidalArm;                              // Initialize the object for the tidal volume servo arm




void setup(void) /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
  
  // Setting up the PWM frequency
  int myEraser = 7;                                         // this is 111 in binary and is used as an eraser
  TCCR1B &= ~myEraser;                                      // this operation (AND plus NOT),  set the three bits in TCCR1B to 0
  
  int myPrescaler = 2;                                      // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.
  TCCR1B |= myPrescaler;                                    //this operation (OR), replaces the last three bits in TCCR1B with our new value 011

  tidalArm.attach(tidalArmPin);
  tidalArm.writeMicroseconds(1000);
  
  attachInterrupt(digitalPinToInterrupt(RPMSensorPin), switchISR,  FALLING);

  analogWrite(motorPWMOut, 0); // set motor PWM to zero if fault

  pinMode(valueSwitchPin, INPUT_PULLUP);
  pinMode(alarmSwitchPin, INPUT_PULLUP);
  
  
  // -- -- -- -- Initializing the averaging arrays -- -- -- -- //
  
  for (int reading = 0; reading < numOxyReadings; reading++){                // Initialize the o2Readings array with zeros
    
    o2Readings[reading] = 0;
    
  }
  
  for (int reading = 0; reading < numPressureReadings; reading++){        // Initialize pressureReadings array with zeros
    
    pressureReadings[reading] = 0;
    
  }

  for (int reading = 0; reading < numInspiredReadings; reading++){        // Initialize the inspiredTimeReadings array with zeros
    
    inspiredTimeReadings[reading] = 0;
    
  }
  
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- //


  
  // -- -- -- -- INITIALIZE THE SCREEN -- -- -- -- //
  
  tft.begin();
  tft.setRotation(Orientation);
  tft.fillScreen(BLACK);
  tft.setTextWrap(false);
  
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- //
  
  Serial.begin(9600);
  Serial.println("Starting up...");
  Serial.println();


  splashScreen();
  splashO2Warning();
  calibrateO2();
  calibratePressure();
  tft.fillScreen(BLACK);

  tft.setCursor(100, 110);
  tft.setTextColor(YELLOW);
  tft.setTextSize(3);
  tft.print("SET DESIRED BPM");
  tft.setCursor(160, 160);
  tft.print("TO BEGIN");
  tft.fillRect(385,0,480,130, BLACK);
  tft.fillRect(385,0,480,30, YELLOW);
  delay(500);
  tft.fillRect(385,0,480,30, BLACK);
  delay(500);
  tft.fillRect(385,0,480,30, YELLOW);
  delay(500);
  tft.fillRect(385,0,480,30, BLACK);
  delay(500);
  tft.fillRect(385,0,480,30, YELLOW);
  delay(500);
  tft.fillRect(385,0,480,30, BLACK);
  delay(500);
  tft.fillRect(385,0,480,30, YELLOW);
  delay(500);
  tft.fillRect(385,0,480,30, BLACK);
  delay(500);
  tft.fillRect(385,0,480,30, YELLOW);
  delay(500);
  tft.fillRect(385,0,480,30, BLACK);
  delay(500);
  tft.fillScreen(BLACK);
  paintScreen();
  editValues();
  setValues();
  
  
}////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






void loop() //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{

  // Alarm Timer Offset
  if(digitalRead(alarmSwitchPin)){
    alarmDelayOffset = millis();
  }
  
  // Jason Edit for Border
  if(digitalRead(alarmSwitchPin)){
    tft.drawRect(0, 0, 480, 320, RED);
    tft.drawRect(1, 1, 478, 318, RED);
    tft.drawRect(2, 2, 476, 316, RED);
  }
  else
  {
    tft.drawRect(0, 0, 480, 320, BLACK);
    tft.drawRect(1, 1, 478, 318, BLACK);
    tft.drawRect(2, 2, 476, 316, BLACK);
  }
  
  currentTime = millis();

  // If alarm enabled show "Alarms Active" on screen
  if (millis() > alarmDelayTime + alarmDelayOffset)
  {
    if (!locked)
    {
      tft.fillRect(0, 305, 220, 28, blankingColor);  //Blanking
    }

    tft.setCursor(0, 305);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.print("Alarms Active");
    locked = true;
       
  } 
  // Otherwise show "Alarms Deactivated"
  else 
  {

    if (locked)
    {
      tft.fillRect(0, 305, 220, 28, blankingColor);  //Blanking
    }   

    tft.setCursor(0, 305);
    tft.setTextColor(MAGENTA);
    tft.setTextSize(2);
    tft.print("Alarms Deactivated");
    locked = false;  

  }
  
  // If the switch has not been hit within a reasonable amount of time, ramp up the PWM to get the motor spinning.
  if (currentTime - previousTime > 6000  && state == false && wait == false) 
  {

    analogWrite(motorPWMOut, pwmOut);
    motorPWMOffset = motorPWMOffset + 1;
    //Serial.println(motorPWMOffset);

  }
  
  updateScreen();

  // Check for user mode request                                                        NEW FUNCTIONALITY WITH MODES
  if (digitalRead(valueSwitchPin))
  {
    editValues();
    prevSet = true;
    
  }
  else
  {
    setValues();
    prevSet = false;
  }
  
  //                                                                                        BEGIN CHECK OF INPUT
  /*- - - - - - - - - - - - - - -*/

  if(!prevSet)
  {
    //                                                                                        BEGIN READING POTENTIOMETERS
    
    int tempTidalVolume = analogRead(tidalVolume_Pot_Pin);
    int tempTidalVolume2 = map(tempTidalVolume, 0, 1013, 4, 30);                              // 1023 reduced to 1013 to improve input error
    tidalVolume = tempTidalVolume2 * 50;
    
    int tempFiO2 = analogRead(O2_Pot_Pin);
    FiO2 = map(tempFiO2, 0, 1013, 21, 100);
    
    int tempIERatio = analogRead(IE_Pot_Pin);
    IERatio = tempIERatio * 5.3 / 1013 + 1;    // 1 to 6     5.3 to bias up

    int tempSetBPM = map(analogRead(motorSpeedPotPin), 20, 1013, 7, 40);

    if(tempSetBPM < 8)
    {
      motorPWM = 0;
      //wait = true;
    } 
    else 
    {
      motorPWM = 180;
      //wait = false;
    }
      
    if(setBPM != tempSetBPM)
    {
      setBPM = tempSetBPM;
    }
    
    //                                                                                                 END READING POTENTIOMETERS
  }

  // This section detects a change on one of the potentiometers. If there's a change it updates the corresponding value and resets the alarm offset
  if( (abs(tidalVolume - prev_tidalVolume) >= 1) || abs(FiO2 - prev_FiO2) >= 1 || abs(IERatio - prev_IERatio) >= 1 || abs(motorPWM - prev_motorPWM) >= 1 )
  {
    changeCounter++;
      
    if (changeCounter > 99 )
    {
      inputChanged = true;
      prev_tidalVolume = tidalVolume;
      prev_FiO2 = FiO2;
      prev_IERatio = IERatio;
      prev_motorPWM = motorPWM;
      // motorPWMOffset = 0;
      
      inspSet = false;
      inspOffset = millis();
      //Serial.println("Reset");

      for (int reading = 0; reading < numPressureReadings; reading++)
      {  // Initialize the o2Readings and pressureReadings arrays with zeros
        
        pressureReadings[reading] = 0;
        
      }

      
      for (int reading = 0; reading < numInspiredReadings; reading++)
      {  // Initialize the o2Readings and pressureReadings arrays with zeros
        
        inspiredTimeReadings[reading] = 0;
        
      }

      pressureIndex = 0;
      inspiredTimeIndex = 0;
      
      pressureTotal = 0.0;
      inspiredTimeTotal = 0.0;
      
      pressureAverage = 0.0;
      inspiredTimeAverage = 0.0;
      
      //Serial.println("Change detected Reseting Alarm Delay Offset, Averages, and Setpoint");
      alarmDelayOffset = millis();
      //Serial.print("New alarm reference: "); Serial.println(alarmDelayOffset);
      
    }

  } 
  else 
  {
  changeCounter = 0;
  }

  /*- - - - - - - - - - - - - - -*/
  //                                                                                        END CHECK OF INPUT








  //                                                                                                    BEGIN PRESSURE CHECK
  /*- - - - - - - - - - - - - - -*/

  if(pressure < 2.5){
    if(millis() > alarmDelayTime + alarmDelayOffset){
      pressureAlarm = true;
    }
  } else {
    pressureAlarm = false;
  }

  /*- - - - - - - - - - - - - - -*/
  //                                                                                                    END PRESSURE CHECK





  //                                                                                                    BEGIN INSPIRED TIME CHECK AND MOTOR UPDATE
  /*- - - - - - - - - - - - - - -*/
  if((abs(inspiredTimeSetpoint - inspiredTimeAverage) > .2) && inspSet)
  {
    inspirationAlarm = true;
  } 
  else
  {
    inspirationAlarm = false;
  }

  pwmOut = motorPWM + motorPWMOffset;
  if (pwmOut > 255)
  {
    pwmOut = 255;
  }

  if (pwmOut < minimumPWMVal())
  {
    pwmOut = minimumPWMVal();
  }

  if(motorPWM == 0)
  {
    pwmOut = 0;
  }

  //Serial.println(pwmOut);
  
  if(motor)
  {
    analogWrite(motorPWMOut, pwmOut); // update motor PWM
  }
  
  motorIdleTime = ((inspirationTime * IERatio) - inspirationTime) * 1000;   // in milliseconds

  /*- - - - - - - - - - - - - - -*/
  //                                                                                                    END INSPIRED TIME CHECK AND MOTOR UPDATE



  //                                                                                                    BEGIN O2 ERROR CHECK
  /*- - - - - - - - - - - - - - -*/
  
  //int tempO2 = analogRead(O2_Pot_Pin); // < --- End use
  
  float O2error =  abs(FiO2 - o2Average);
  //Serial.print(O2error); Serial.print(" + "); Serial.println(FiO2);
  if((O2error > 5.0) && (millis() > alarmDelayTime + alarmDelayOffset))
  {
    O2Alarm = true;
  }
  else
  {
    O2Alarm = false;
  }
  
  /*- - - - - - - - - - - - - - -*/
  //                                                                                                    END 02 ERROR CHECK



  //                                                                                                    BEGIN ALARM CHECK
  /*- - - - - - - - - - - - - - -*/
  if(O2Alarm || pressureAlarm || inspirationAlarm){
    if( millis() > alarmDelayTime + alarmDelayOffset){
      alarm = true;
      tone(tonePin, 1000, 1000);  // tone(pin, frequency(Hz), duration(ms));
      
    }
    } else {
    alarm = false;
    noTone(tonePin);
  }
  /*- - - - - - - - - - - - - - -*/
  //                                                                                                    END ALARM CHECK
  

  //                                                                                                    BEGIN TRIGGER CHECK
  /*- - - - - - - - - - - - - - -*/

  if(switchHit)
  {         // ensures trigger has completed
    
    if (state == false)
    {
      updatePIP();
      float readPressure = analogRead(pressureSensorPin);
      pressure = (mapfloat(readPressure, 102, 922, pressureMin, pressureMax) + pressureCalibrationFactor); //ensures that the pressure gets checked
      Serial.println(pressure);
    }

    if(counter < 3)
    {
      counter++;
      if(counter > 3)
      {
        counter = 3;
      }
      updateCalcs();
      //updateScreen();
    }
    
  }
  /*- - - - - - - - - - - - - - -*/
  //                                                                                                    END TRIGGER CHECK
  
  updatePressureAverage();
  
  if(counter >= 3)
  {
    if(switchHit)    // hit the switch  --> I'm in the pocket
    {
      if (state == false){    //  sets the timer for entering hold
        previousTime = millis();    // record time as soon as the switch is hit
      }
      currentTime = millis();            // get the most accurate time
      if(currentTime - previousTime > motorIdleTime) // we have waited our motorIdleTime
      {
        previousTime = currentTime;

        if(!wait)
        {
          analogWrite(motorPWMOut, pwmOut);        // motor on
        }
        
        startCycleTime = millis();
        switchHit = false;
        state = false;
      }
      else
      {
        if(motorIdleTime > 250)
        {

          if(!wait)
          {
            analogWrite(motorPWMOut, 0);
          }
          
          motor = false;
          
        }

        if(state == false)
        {
          endCycleTime = millis();
          updateO2Average();
          updateCalcs();
          updateInspiredAverage();

          state = true;
        }
        else
        {
          updateCalcs();
        }
      }
    }
  }
}////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void updateCalcs()
{

  CycleTime = (endCycleTime - startCycleTime) / 1000.0; // = time for 1 revolution (Inhale & Exhale) in ms
  
  inspirationTime = (CycleTime / 2.0) + alpha;  // in ms
  
  if(inspirationTime > 3.0){
    inspirationTime = 3.0;
  }

  
  BPM = 60.0 / (float(CycleTime) + float(motorIdleTime / 1000.0));
  
  RMV_LPM = tidalVolume / 1000.0 * BPM;
  
  Oxygen_LPM = RMV_LPM * (1.0 - (1.0 - (FiO2 / 100.0)) / 0.79);
  
  momentArmOffset = tidalVolume / 403.6544;
  
}


void switchISR()
{
  //Serial.println("Hit");
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
    //hit++;
    //Serial.println(hit);
    switchHit = true;
  }
  last_interrupt_time = interrupt_time;
}


void updatePressureAverage()
{
  
  // subtract the last reading:
  pressureTotal = pressureTotal - pressureReadings[pressureIndex];
  
  // read from the sensor:
  float readPressure = analogRead(pressureSensorPin);
  pressureReadings[pressureIndex] = ( mapfloat(readPressure, 102, 922, pressureMin, pressureMax) + pressureCalibrationFactor );
  
  // add the reading to the total:
  pressureTotal = pressureTotal + pressureReadings[pressureIndex];

  // advance to the next position in the array:
  pressureIndex = pressureIndex + 1;
  
  // if we're at the end of the array...
  if (pressureIndex >= numPressureReadings) {
    // ...wrap around to the beginning:
    pressureIndex = 0;
  }

  // calculate the average:
  pressureAverage = pressureTotal / numPressureReadings;
  

}


void updateO2Average()
{

  // subtract the last reading:
  o2Total = o2Total - o2Readings[o2Index];

  // read from the sensor:
  float readO2 = analogRead(O2sensorPin);
  o2Readings[o2Index] = ( mapfloat(readO2, 130, 910, 20.8, 100.0) / O2calibrationFactor );

  if( o2Readings[o2Index] > 99.99 ){
    o2Readings[o2Index] = 99.99;
  }
  
  // add the reading to the total:
  o2Total = o2Total + o2Readings[o2Index];

  // advance to the next position in the array:
  o2Index = o2Index + 1;

  // if we're at the end of the array...
  if (o2Index >= numOxyReadings) {
    // ...wrap around to the beginning:
    o2Index = 0;
  }

  // calculate the average:
  o2Average = o2Total / numOxyReadings;
  
}


void calibrateO2()
{
  int count = 0;
  float O2TotalCal = 0.0;
  float O2AvgCal = 0.0;
  float statusBarOffset = 2;
  
  tft.setCursor(65, 40);
  tft.setTextColor(GREEN);
  tft.setTextSize(3);
  tft.print("Calibrating sensors.");
  tft.setCursor(130, 90);
  tft.print("Please wait.");

  tft.fillRect(38, 260, 402, 20, WHITE); // (x , y , width, height)
  
  while (count < 100){
    float readO2         = analogRead(O2sensorPin);
    float startingO2     = ( mapfloat(readO2, 130, 910, 20.8, 100.0) );

    O2TotalCal = O2TotalCal + startingO2;
    delay(100);

    tft.fillRect(40 + statusBarOffset, 262, 2, 16, GREEN);
    statusBarOffset = statusBarOffset + 2;

    count++;
  }
  

  O2AvgCal = O2TotalCal / count;
  
  // hard coded to 1.3 for a fresh sensor. Down the line it should
  float O2cal = 1.3; // be implemented with proper procedure  --->  O2AvgCal / 20.8;
  
  O2calibrationFactor = O2cal;

  tft.setCursor(160,140);
  tft.setTextSize(2);
  tft.print("O2 Calibrated");
  tft.setCursor(70,160);
  tft.print("Ambient O2 Detected:  "); tft.print(O2AvgCal/1.3); tft.print("%");
  
}


void calibratePressure()
{

  int count = 0;
  float pressureTotalCal = 0.0;
  float pressureAvgCal = 0.0;
  float statusBarOffset = 200;

  while (count < 100){
    float readPressure         = analogRead(pressureSensorPin);
    float startingPressure     = ( mapfloat(readPressure, 102, 922, pressureMin, pressureMax) );

    pressureTotalCal = pressureTotalCal + startingPressure;
    delay(100);
    
    tft.fillRect(38 + statusBarOffset, 262, 2, 16, GREEN);
    statusBarOffset = statusBarOffset + 2;

    count++;
  }

  pressureAvgCal = -(pressureTotalCal / count);

  pressureCalibrationFactor = pressureAvgCal;

  tft.setCursor(140,220);
  tft.print("Pressure zeroed");
  delay(1500);
  
}


void updateInspiredAverage()
{
  inspiredTimeTotal = inspiredTimeTotal - inspiredTimeReadings[inspiredTimeIndex];  //subtract the last reading
  
  inspiredTimeReadings[inspiredTimeIndex] = inspirationTime;                        // add value into the array
  inspiredTimeTotal = inspiredTimeTotal + inspiredTimeReadings[inspiredTimeIndex];  // add to total
  inspiredTimeIndex = inspiredTimeIndex + 1;                                        // advance index
  if (inspiredTimeIndex >= numInspiredReadings) 
  {
    // ...wrap around to the beginning:
    inspiredTimeIndex = 0;
  }
  inspiredTimeAverage = inspiredTimeTotal / numInspiredReadings;                    // calculate inspiredTime average

  if (counter >= 3 && !wait)
  {
     motorPWMOffset = motorPWMOffset + 6 * (setBPM - BPM);                          // P Control
     if (motorPWMOffset < -(180 - minimumPWMVal()))
     {
      motorPWMOffset = -(180 - minimumPWMVal());
     }
     if(motorPWMOffset > 75)
     {
      motorPWMOffset = 75;
     }
    Serial.print("Motor Error: "); Serial.println(setBPM - BPM);
  }
  
  if ((millis() >  inspDelay + inspOffset))
  {
    if(inspSet == false)
    {
      inspiredTimeSetpoint = inspiredTimeAverage;
      setIERatio = IERatio;
      inspSet = true;
    } 
  }
}
  

// Tuning PWM for tidal volume
int minimumPWMVal(){

  int multiplier = 0;

  if (tidalVolume < 550) {
    multiplier = 0;
    } else if (tidalVolume < 1050) {
    multiplier = 1;
    } else {
    multiplier = 2;
  }

  switch (tidalVolume) {

    case 150:

    return 75 + tuning * multiplier;
    
    break;

    case 200:

    
    return 75 + tuning * multiplier;

    break;
    
    case 250:

    
    return 84 + tuning * multiplier;

    break;
    
    case 300:

    
    return 85 + tuning * multiplier;

    break;
    
    case 350:

    
    return 85 + tuning * multiplier;

    break;
    
    case 400:

    
    return 85 + tuning * multiplier;

    break;
    
    case 450:

    
    return 85 + tuning * multiplier;

    break;
    
    case 500:

    
    return 91 + tuning * multiplier;

    break;
    
    case 550:

    
    return 92 + tuning * multiplier;

    break;
    
    case 600:

    
    return 94 + tuning * multiplier;

    break;
    
    case 650:

    
    return 95 + tuning * multiplier;

    break;
    
    case 700:

    
    return 96 + tuning * multiplier;
    
    break;
    
    case 750:

    
    return 96 + tuning * multiplier;

    break;

    case 800:

    
    return 97 + tuning * multiplier;

    break;

    case 850:
    
    
    return 106 + tuning * multiplier;
    
    break;

    case 900:

    
    return 110 + tuning * multiplier;

    break;

    case 950:

    
    return 117 + tuning * multiplier;  // Can't go lower than 1.6

    break;
    
    case 1000:
    
    
    return 120 + tuning * multiplier;     // + 0.2

    break;
    
    case 1050:
    
    
    return 121 + tuning * multiplier;

    break;
    
    case 1100:
    
    
    return 125 + tuning * multiplier;     //  + 0.3

    break;
    
    case 1150:
    
    
    return 130 + tuning * multiplier;

    break;
    
    case 1200:
    
    
    return 130 + tuning * multiplier;  // + 0.4

    break;
    
    case 1250:
    
    
    return 130 + tuning * multiplier;

    break;

    case 1300:
    
    
    return 130 + tuning * multiplier;   // +  0.5

    break;

    case 1350:
    
    
    return 135 + tuning * multiplier;
    

    break;

    case 1400:
    
    
    return 143 + tuning * multiplier;  // + 0.5

    break;

    case 1450:
    
    
    return 142 + tuning * multiplier;

    break;

    case 1500:
    
    
    return 147 + tuning * multiplier;
    

    break;

    default:

    break;
    
  }
}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void splashScreen()
{
  tft.setCursor(100, 90);
  tft.setTextSize(6);
  tft.setTextColor(WHITE);
  tft.print("PRE-Vent");
  tft.setCursor(205, 200);
  tft.setTextSize(3);
  tft.print("v1.0");
  delay(2000);
  tft.fillScreen(BLACK);
}


void paintScreen()
{

  tft.setCursor(10, 3);
  tft.setTextSize(3);
  tft.setTextColor(CYAN);
  tft.print("TIDAL");
  tft.setCursor(0, 28);
  tft.print("VOLUME");
  tft.setCursor(35, 102);
  tft.setTextSize(3);
  tft.print("mL");
  
  tft.setCursor(140, 17);
  tft.setTextSize(4);
  tft.setTextColor(GREEN);
  tft.print("Fi0");
  tft.setCursor(215, 32);
  tft.setTextSize(2);
  tft.print("2");
  tft.setCursor(160, 102);
  tft.setTextSize(4);
  tft.print("%");
  
  tft.setCursor(270, 17);
  tft.setTextSize(4);
  tft.setTextColor(YELLOW);
  tft.print("I:E");
  tft.setCursor(260, 102);
  tft.setTextSize(3);
  tft.print("RATIO");
  
  tft.setCursor(275, 140);          
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.print("INSP");
  tft.setCursor(275, 165);
  tft.print("TIME");
  tft.setCursor(280, 245);
  tft.print("sec");
  
  tft.setCursor(30, 160);
  tft.setTextSize(4);
  tft.setTextColor(GREEN);
  tft.print("O");
  tft.setCursor(54, 175);
  tft.setTextSize(2);
  tft.print("2");
  tft.setCursor(15, 245);
  tft.setTextSize(3);
  tft.print("LPM");
  
  tft.setCursor(160, 160);
  tft.setTextSize(4);
  tft.setTextColor(GREEN);
  tft.print("O");
  tft.setCursor(183, 175);
  tft.setTextSize(2);
  tft.print("2");
  tft.setCursor(170, 245);
  tft.setTextSize(3);
  tft.print("%");
  
  tft.setCursor(385, 17);
  tft.setTextSize(4);
  tft.setTextColor(YELLOW);
  tft.print("RATE");
  tft.setCursor(400, 102);            
  tft.setTextSize(3);
  tft.setTextColor(YELLOW);
  tft.print("BPM");

  tft.setCursor(385, 160);
  tft.setTextSize(3);
  tft.setTextColor(CYAN);
  tft.print("PRESS");
  tft.setCursor(385, 245);
  tft.setTextSize(3);
  tft.print("cmH");
  tft.setCursor(438, 252);
  tft.setTextSize(2);
  tft.print("2");
  tft.setCursor(451, 245);
  tft.setTextSize(3);
  tft.print("O");

  tft.setCursor(280, 285);
  tft.print("PIP =");
  
  
  tft.fillRect(239, 0, 2, 320, WHITE);  // Divider line
  
}


void updateServo()
{

  int servoVal = ((((tidalVolume)/415.9803)-0.360594)/0.003937)+975;  // offset by 25 to dial in exact tidal volume
  
  //write the appropriate value based on desired tidal volume
  tidalArm.writeMicroseconds(servoVal);
  
}


void editValues()
{
  
  //                                                                   BEGIN READING POTENTIOMETERS
  /*- - - - - - - - - - - - - - -*/
  
  int tempTidalVolume = analogRead(tidalVolume_Pot_Pin);
  int tempTidalVolume2 = map(tempTidalVolume, 0, 1013, 4, 30);                              // 1023 reduced to 1013 to improve input error
  tidalVolumeDisplay = tempTidalVolume2 * 50;
  
  int tempFiO2 = analogRead(O2_Pot_Pin);
  FiO2Display = map(tempFiO2, 0, 1013, 21, 100);
  
  int tempIERatio = analogRead(IE_Pot_Pin);
  IERatioDisplay = tempIERatio * 5.3 / 1013 + 1;    // 1 to 6     5.3 to bias up

  int tempSetBPM = map(analogRead(motorSpeedPotPin), 20, 1013, 7, 40);

  if(tempSetBPM < 8)
  {
    motorPWM = 0;
    //wait = true;
  } 
  else 
  {
    motorPWM = 180;
    //wait = false;
  }
    
  if(setBPMDisplay != tempSetBPM)
  {
    setBPMDisplay = tempSetBPM;
  }
  

  /*- - - - - - - - - - - - - - -*/
  //                                                                                                 END READING POTENTIOMETERS
  
}


void setValues()
{
  tidalVolume = tidalVolumeDisplay;
  FiO2        = FiO2Display;
  IERatio     = IERatioDisplay;
  setBPM      = setBPMDisplay;
}


void updateScreen()
{
  
  unsigned long screenUpdateInterval = 100;

  if(!prevSet)
  {
    if(millis() - previousScreenUpdateTime > screenUpdateInterval){   
      
      previousScreenUpdateTime = millis();

      updateServo();
      if(tidalVolume != prev_displayTidalVolume)
      {
        // updateServo();
        tft.fillRect(9, 65, 96, 28, blankingColor);    //Blanking    -->    TIDAL VOLUME
        tft.setCursor(9, 65);
        tft.setTextColor(CYAN);
        tft.setTextSize(4);
        tft.print(tidalVolume);
        prev_displayTidalVolume = tidalVolume;
      }
      
      
      if(FiO2 != prev_displayFiO2)
      {
        tft.fillRect(150, 65, 73, 28, blankingColor);  //Blanking    -->    FiO2
        tft.setCursor(150, 65);
        tft.setTextColor(GREEN);
        tft.setTextSize(4);
        tft.print(FiO2);
        prev_displayFiO2 = FiO2;
      }
  
  
      if(IERatio != prev_displayIERatio)
      {
        tft.fillRect(270, 65, 75, 28, blankingColor);  //Blanking    -->    I:E RATIO
        tft.setCursor(270, 65);
        tft.setTextColor(YELLOW);
        tft.setTextSize(4);
        tft.print("1:");
        tft.print(IERatio, 1);
        prev_displayIERatio = IERatio;
      }
  
  
      if(inspirationTime != prev_displayInsperationTime)
      {
        if(inspirationAlarm){
          tft.fillRect(255, 205, 99, 28, RED);  //Blanking    -->    INSPIRATION TIME
          } else {
          tft.fillRect(255, 205, 99, 28, blankingColor);  //Blanking    -->    INSPIRATION TIME
        }
        tft.setCursor(255, 205);
        tft.setTextColor(WHITE);
        tft.setTextSize(4);
        tft.print(inspirationTime, 2);                                          //255, 205, 99, 28, blankingColor
        prev_displayInsperationTime = inspirationTime;
      }
  
      if(BPM != prev_displayBPM)
      {
        tft.fillRect(402, 130, 92, 28, blankingColor);  //Blanking    -->    BREATHS PER MINUTE
        tft.setCursor(402, 130);
        tft.setTextColor(YELLOW);
        tft.setTextSize(2);                                                     //385, 66, 92, 28,
        if(BPM > 90.0){
          BPM = 90.0;
        }
        tft.print(BPM, 1);
        
        prev_displayBPM = BPM;
      }
  
      if(setBPM != prev_setBPM)
      {
        tft.fillRect(405, 60, 92, 28, blankingColor);
        tft.setCursor(405,60);
        tft.setTextColor(YELLOW);
        tft.setTextSize(4);
        if (setBPM != 7){
          tft.print(setBPM);
        } else {
          tft.print("OFF");
        }
        prev_displaySetBPM = setBPM;
        
      }
      
      if(Oxygen_LPM != prev_displayOxygen_LPM)
      {
        tft.fillRect(5, 205, 99, 28, blankingColor);  //Blanking    -->    Oxygen_LPM
        tft.setCursor(5, 205);
        tft.setTextColor(GREEN);
        tft.setTextSize(4);
        if(Oxygen_LPM > 50.0){
          Oxygen_LPM = 50.0;
        }
        tft.print(Oxygen_LPM, 1);
        prev_displayOxygen_LPM = Oxygen_LPM;
      }
      
      
      if((o2Average != prev_displayO2Average) || O2Alarm)
      {
        if(O2Alarm){
          tft.fillRect(130, 205, 99, 28, RED);  //Blanking    -->    Oxygen %
          } else {
          tft.fillRect(130, 205, 99, 28, blankingColor);  //Blanking    -->    Oxygen %
        }
        tft.setCursor(130, 205);
        tft.setTextColor(GREEN);
        tft.setTextSize(4);
        tft.print(o2Average, 1);
        prev_displayO2Average = o2Average;
      }
      
  
      if(pressureAverage != prev_displayPressureAverage)
      {
        if(pressureAlarm){
          tft.fillRect(385, 205, 99, 28, RED);
          } else {
          tft.fillRect(385, 205, 99, 28, blankingColor);  //Blanking    -->    pressureAverage
        }
        tft.setCursor(385, 205);
        tft.setTextColor(CYAN);
        tft.setTextSize(4);
        //  if(pressureAverage > 90.0){
        //    pressureAverage = 90.0;
        //  }
        tft.print(pressureAverage, 1);
        prev_displayPressureAverage = pressureAverage;
      }
  
      if(PIPpressure < pressureAverage){
        PIPpressure = pressureAverage;
      }  
    }
  }
  else
  {
      if(millis() - previousScreenUpdateTime > screenUpdateInterval){   
      
      previousScreenUpdateTime = millis();
      
      if(tidalVolumeDisplay != prev_displayTidalVolume)
      {
        updateServo();
        tft.fillRect(9, 65, 96, 28, blankingColor);    //Blanking    -->    TIDAL VOLUME
        tft.setCursor(9, 65);
        tft.setTextColor(CYAN);
        tft.setTextSize(4);
        tft.print(tidalVolumeDisplay);
        prev_displayTidalVolume = tidalVolumeDisplay;
      }
      
      
      if(FiO2Display != prev_displayFiO2)
      {
        tft.fillRect(150, 65, 73, 28, blankingColor);  //Blanking    -->    FiO2
        tft.setCursor(150, 65);
        tft.setTextColor(GREEN);
        tft.setTextSize(4);
        tft.print(FiO2Display);
        prev_displayFiO2 = FiO2Display;
      }
  
  
      if(IERatioDisplay != prev_displayIERatio)
      {
        tft.fillRect(270, 65, 75, 28, blankingColor);  //Blanking    -->    I:E RATIO
        tft.setCursor(270, 65);
        tft.setTextColor(YELLOW);
        tft.setTextSize(4);
        tft.print("1:");
        tft.print(IERatioDisplay, 1);
        prev_displayIERatio = IERatioDisplay;
      }
  
  
      if(inspirationTime != prev_displayInsperationTime)
      {
        if(inspirationAlarm){
          tft.fillRect(255, 205, 99, 28, RED);  //Blanking    -->    INSPIRATION TIME
          } else {
          tft.fillRect(255, 205, 99, 28, blankingColor);  //Blanking    -->    INSPIRATION TIME
        }
        tft.setCursor(255, 205);
        tft.setTextColor(WHITE);
        tft.setTextSize(4);
        tft.print(inspirationTime, 2);                                          //255, 205, 99, 28, blankingColor
        prev_displayInsperationTime = inspirationTime;
      }
  
      if(BPM != prev_displayBPM)
      {
        tft.fillRect(402, 130, 92, 28, blankingColor);  //Blanking    -->    BREATHS PER MINUTE
        tft.setCursor(402, 130);
        tft.setTextColor(YELLOW);
        tft.setTextSize(2);                                                     //385, 66, 92, 28,
        if(BPM > 90.0){
          BPM = 90.0;
        }
        tft.print(BPM, 1);
        
        prev_displayBPM = BPM;
      }
  
      if(setBPMDisplay != prev_setBPM)
      {
        tft.fillRect(405, 60, 92, 28, blankingColor);
        tft.setCursor(405,60);
        tft.setTextColor(YELLOW);
        tft.setTextSize(4);
        if (setBPMDisplay != 7){
          tft.print(setBPMDisplay);
        } else {
          tft.print("OFF");
        }
        prev_displaySetBPM = setBPMDisplay;
        
      }
      
      if(Oxygen_LPM != prev_displayOxygen_LPM)
      {
        tft.fillRect(5, 205, 99, 28, blankingColor);  //Blanking    -->    Oxygen_LPM
        tft.setCursor(5, 205);
        tft.setTextColor(GREEN);
        tft.setTextSize(4);
        if(Oxygen_LPM > 50.0){
          Oxygen_LPM = 50.0;
        }
        tft.print(Oxygen_LPM, 1);
        prev_displayOxygen_LPM = Oxygen_LPM;
      }
      
      
      if((o2Average != prev_displayO2Average) || O2Alarm)
      {
        if(O2Alarm){
          tft.fillRect(130, 205, 99, 28, RED);  //Blanking    -->    Oxygen %
          } else {
          tft.fillRect(130, 205, 99, 28, blankingColor);  //Blanking    -->    Oxygen %
        }
        tft.setCursor(130, 205);
        tft.setTextColor(GREEN);
        tft.setTextSize(4);
        tft.print(o2Average, 1);
        prev_displayO2Average = o2Average;
      }
      
  
      if(pressureAverage != prev_displayPressureAverage)
      {
        if(pressureAlarm){
          tft.fillRect(385, 205, 99, 28, RED);
          } else {
          tft.fillRect(385, 205, 99, 28, blankingColor);  //Blanking    -->    pressureAverage
        }
        tft.setCursor(385, 205);
        tft.setTextColor(CYAN);
        tft.setTextSize(4);
        //  if(pressureAverage > 90.0){
        //    pressureAverage = 90.0;
        //  }
        tft.print(pressureAverage, 1);
        prev_displayPressureAverage = pressureAverage;
      }
  
      if(PIPpressure < pressureAverage){
        PIPpressure = pressureAverage;
      }  
    }
  }
}


void splashO2Warning()
{


  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3);

  tft.setCursor(65, 120);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.print("If O2 has been used");
  tft.setCursor(65, 160);
  tft.print("Turn off PRE-Vent");
  tft.setCursor(65, 200);
  tft.print("Turn off O2");
  tft.setCursor(65, 240);
  tft.print("Pump BVM 15 times!");
  tft.setCursor(65, 280);
  tft.print("Power PRE-Vent back on");

  delay(500);
  tft.fillRect(0, 0, 480, 120, BLACK);
  delay(250);
  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3); 
  delay(500);
  tft.fillRect(0, 0, 480, 120, BLACK);
  delay(250);
  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3); 
  delay(500);  
  tft.fillRect(0, 0, 480, 120, BLACK);
  delay(250);
  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3); 
  delay(500);  
  tft.fillRect(0, 0, 480, 120, BLACK);
  delay(250);
  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3); 
  delay(500);
  tft.fillRect(0, 0, 480, 120, BLACK);
  delay(250);
  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3);   
  delay(500);
  tft.fillRect(0, 0, 480, 120, BLACK);
  delay(250);
  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3);  
  delay(500);
  tft.fillRect(0, 0, 480, 120, BLACK);
  delay(250);
  fillStopsign(380, 50, 3);
  fillStopsign(40, 50, 3);
  delay(5000);
  tft.fillScreen(BLACK);  

  
}


void updatePIP()
{
    if(PIPpressure != prevPressure){
      tft.fillRect(385, 285, 99, 28, blankingColor);
      tft.setCursor(385, 285);
      tft.setTextColor(CYAN);
      tft.setTextSize(3);
      tft.print(PIPpressure, 1);
      prevPressure = PIPpressure;
      PIPpressure = 0;
    }
}


void fillStopsign(int posx, int posy, int scale)
{

  int x = posx;
  int y = posy;
  
  int dx = 10*scale;
  int dy = dx*1.4;
  int index = 1;

  while (index <= 3){
    tft.fillTriangle(posx - dx, posy, posx + dx, posy, posx, posy - dy, RED);
    tft.fillTriangle(posx - dx, posy, posx + dx, posy, posx, posy + dy, RED);

    posx = posx + dx;
    index = index + 1;
    
  }

  tft.fillRect(x, y - dy, dx*2, dy, RED);
  tft.fillRect(x, y, dx*2, dy, RED);

  tft.setCursor(x - 5, y - 10);
  tft.setTextColor(WHITE);
  tft.setTextSize(scale);
  tft.print("STOP");
  
 
  
}
