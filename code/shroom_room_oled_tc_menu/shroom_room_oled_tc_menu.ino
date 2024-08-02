/*
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>. 
*/

/* This piece of software controls the Cave Box. 
  Please refer to https://sholtosworkshop.com/ for documentation. 
  There is also a Contact section if you want to ask questions about the code.

  Compile as ESP32 -WROOM-DA_MODULE
  or ESP32 Dev Module on boards with holes and without overhanging esp antenna

  Version 1.02 has outputs for relays programmed in.
  Version 1.03 uses AHT20
  Version 1.04 has a more sophisticated fan error prcedure.

Changed ubuntu vrsion to 22 steps taken to get to compile:

faeinterval hardcoded vals to ints not floats
updated tcmenu and dependencies
esp32 back to v2



*/
#define FIRMWARE_VERSION "1.04"
//#define USE_PCB_OLED_V2 //Second milled board, first to use oled screen
//#define USE_PCB_OLED_MODIFIED_PERFBOARD // 1st board made with perfboard, OLED added later
#define USE_PCB_OLED_V3 //3rd milled board, has oled screen and bought mister board.

// #define USE_DHT22_SENSOR
#define USE_AHT20_SENSOR

#include "shroom_room_oled_tc_menu_menu.h"
#include "Wire.h"
#include "EEPROM.h"
#ifdef USE_DHT22_SENSOR
  #include "DHT.h"
#endif
#ifdef USE_AHT20_SENSOR
  #include <Adafruit_AHTX0.h>
#endif
#include <PID_v1.h>
#include <FastLED.h>



#define PRESET_ARRAY_ADDRESS 28 //Address after tcMenu items, needs to be updated if more EERPOM using variables are created with tcMenu.

//// Pin Definitions ////
//OLED Clk 22, data 21, enc A 23, B 19, btn 25, all of these set in TCMENU

#ifdef USE_PCB_OLED_V2
  const int DHT_PIN = 18;
  const int MISTER_PIN = 17; 
  const int FAN_PIN = 16;
  const int FAN_TACH_PIN = 4;
  const int WS2812_PIN = 5;
  const int PROG_FREQ_PIN = 33;
#endif

#ifdef USE_PCB_OLED_V3
  const int DHT_PIN = 17;
  const int MISTER_PIN = 18; 
  const int FAN_PIN = 16;
  const int FAN_TACH_PIN = 4;
  const int WS2812_PIN = 5;
  const int PROG_FREQ_PIN = 33;
  #ifdef USE_AHT20_SENSOR
    const int AHT_SCL_PIN = 27;
    const int AHT_SDA_PIN = 17;
  #endif


  //Following are output pins to connect SSR to to use the Cave Box as a controller for AC equipement
  const int EXT_FAN_PIN = 32; //pin 6 on header red LED testing
  const int EXT_HUMIDIFIER_PIN = 33; //pin 5 on header, green LED for testing
  const int EXT_LIGHT_PIN = 26; //pin 4 on header, yellow LED for testing
#endif

#ifdef USE_PCB_OLED_MODIFIED_PERFBOARD
  const int DHT_PIN = 4;
  const int MISTER_PIN = 16; 
  const int FAN_PIN = 18;
  const int FAN_TACH_PIN = 32;
  const int WS2812_PIN = 27;
  const int PROG_FREQ_PIN = 33;
#endif

const int LOOP_TIME = 2000; //The control loop only updates after this much time has passed.

//// LED Stuff ////
const int LED_LOOP_TIME = 20; // How often the led animation updates.
const int NUM_LEDS = 1;
const int LED_ARRAY_LENGTH = 255;
//Colours that show the state of the device:
const int WARNING_COLOUR = 0;
const int FAN_COLOUR = 192;
const int MIST_COLOUR = 224;
const int MILLIS_PER_DAY = 86400000.0;
unsigned long lightOnTimeMillis = 0;
int colour = 85;
int RHColour = 0;
int sinArray[LED_ARRAY_LENGTH];
int ledLoopPos = 0;
int newColour = 85;
int minBrightness = 50;
bool LEDStateToggle = 0; //Stops the LED timer from repeatedly switching on or off the LED.

/// Mushroom Growing Parameters ////
unsigned long fanOnTime = 2000; //After misting starts fan stays on this long to disperse water vapour.
bool activateFan = 0; //If true fan turns on.

double RHSetPoint; //%  Relative humidity setpoint for PID.
char RHString[11], tempString[11], newRHMsg[11]; //Stores the relative humidity as a string
bool isMisterOn;
bool fanTachState = 1;
bool fanErrorTriggered = 0; //Fan Error only triggers once after restarting to indicate something might be wrong.
int loopCount;

double RH, T; //Stores the current relative humidity and temperature values.

//// PID Stuff ////
double RHInput, RHOutput; //Relative humidity
//Set initial PID params,overwritten when eeprom loaded.
double RHKp = 1000, RHKi = 0.01, RHKd = 0;

//Some variables needed for blocks of code that run after set intervals
unsigned long loopStartTime, ledStartTime, fanOnStartTime, fanTachStartTime, PIDOnStartTime;
unsigned long fanTachInterval = 0;

/// Fresh air exchange variables
//FAEOnStartTime measures how long its been since FAE started.
//FAEOnIntervalStartTime is how long since the last FAE cycle started.
//FAEInterval is how long between FAE cycles.
//FAEOnTime is how long a cycle lasts.
unsigned long FAEOnStartTime, FAEOnIntervalStartTime, FAEOnTime;
bool FAEOn = 0; // When this is true fresh air is exchanged.

struct preset { //https://arduino.stackexchange.com/questions/25945/how-to-read-and-write-eeprom-in-esp8266 useful 
  double RHSetPoint;
  unsigned long FAEInterval;

} presetArr[] = {
  {0.0, 0},
  {0.0, 0},
  {0.0, 0},
  {0.0, 0},
  {0.0, 0},
  {0.0, 0},
  {0.0, 0},
  {0.0, 0},
  {0.0, 0},
  {0.0, 0}
};

CRGB leds[NUM_LEDS];

#ifdef USE_DHT22_SENSOR
  DHT dht(DHT_PIN, DHT22);
#endif

#ifdef USE_AHT20_SENSOR
  Adafruit_AHTX0 aht;
  TwoWire I2CAHT = TwoWire(1); // Use 2nd i2c interface for sensor.
#endif
  
PID RHPID(&RHInput, &RHOutput, &RHSetPoint, RHKp, RHKi, RHKd, DIRECT); 

const char ErrorWarningPgm[] PROGMEM = "WARNING";

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  setupMenu();
  Wire.begin();

  #ifdef USE_DHT22_SENSOR
    dht.begin();
  #endif
  
  #ifdef USE_AHT20_SENSOR
    I2CAHT.begin(AHT_SDA_PIN, AHT_SCL_PIN, 100000);
//    aht.begin(&I2CAHT);
//    Serial.println("i2c init");
    if (! aht.begin(&I2CAHT)) {
      Serial.println("Could not find AHT? Check wiring");
    }
  #endif
  
  pinMode(MISTER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(FAN_TACH_PIN, INPUT_PULLUP);
  pinMode(PROG_FREQ_PIN, OUTPUT);

  pinMode(EXT_FAN_PIN, OUTPUT);
  pinMode(EXT_HUMIDIFIER_PIN, OUTPUT);
  pinMode(EXT_LIGHT_PIN, OUTPUT);

  //tell the PID to range between 0 and the full window size
  RHPID.SetOutputLimits(0, LOOP_TIME);
  RHPID.SetSampleTime(LOOP_TIME);
  //turn on pid for humidity controller
  RHPID.SetMode(AUTOMATIC);
  
  // // ledc used to generate squarewave to control the mister. TODO see if there is optimum duty cycle.
  ledcSetup(0, 109000, 8); //channel 0, 109kHz, 8 bits resolution, though perhaps require much less
  ledcAttachPin(MISTER_PIN, 0);

  FastLED.addLeds<WS2812, WS2812_PIN, GRB>(leds, NUM_LEDS); 
  //Generate array containing oscillating brightness values for the led.
  float angleStep = 6.283/(float)LED_ARRAY_LENGTH;
  for (int i = 0; i <= LED_ARRAY_LENGTH; i++){ 
    sinArray[i] = int((sin(i*angleStep)+1.25)*255./(1. + 1.25));
  }
  

  //load state from EEPROM.
  EEPROM.get(PRESET_ARRAY_ADDRESS, presetArr);
  menuMgr.load();
  //Copy values from the TCMenu vars to mushroom house ones:
  changeRHSetpoint((double)menuRHSetPoint.getIntValueIncludingOffset());
  RHKp = (double)menuPValue.getAsFloatingPointValue();
  RHKi = (double)menuIValue.getAsFloatingPointValue();
  RHKd = (double)menuDValue.getAsFloatingPointValue();
  RHPID.SetTunings(RHKp, RHKi, RHKd);
  
  setFAEInterval(); //convert stored FAEPerHour to interval in millis
  FAEOnTimeCallBack(20); // Converts value from EEPROM in mins to ms.
  
  initController();
}

void loop() {

  taskManager.runLoop(); //Updates the screen
  

  //read values from the sensor, if it returns 1 it's not giving values so restart loop() and try again.
  if (updateDHT() == 1) { 
    return;  
  }
  
  if (menuControllerBool.getBoolean()) {
     //These functions run continuously if controler is on.
    outputPID(); // Controls the mister.
    updateLED(); // Has to be before checkHumidityAcheivable and fanControl else it will turn of the WARNING_COLOUR before the controller switched off. 
    checkHumidityAcheivable(); //Ensures that the setpoint has been reached fairly recently
    fanControl(); 
  }


// To measure loop time: typically ~5us, sometimes rising to ~30us with the control loop and tc menu running.
//  digitalWrite(PROG_FREQ_PIN, LOW);
//  delay(1);
//  digitalWrite(PROG_FREQ_PIN, HIGH);

}


/// Shroom Room Controll functions ////
void initController() {
  //Resets the timers and puts the Cave Box into a state ready to begin humidifying.
  // Should be called whenever it starts working, or if the Cave Boxs parameters have been changed.
  loopStartTime = millis();
  ledStartTime = millis();
  fanOnStartTime = millis();
  fanTachStartTime = millis();
  PIDOnStartTime = millis();
  FAEOnIntervalStartTime = millis();
  isMisterOn = 0;
  loopCount = 0;
  lightOntimeCallback(22);
}


void deinitController() {
  //When the controller is switched off this function should be called to make sure everything is off.
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(EXT_FAN_PIN, LOW);
  
  ledcWrite(0, 0);// 0% duty cycle
  digitalWrite(EXT_HUMIDIFIER_PIN, LOW);
  
  isMisterOn = 0;
  leds[0] = CHSV(colour, 255, 0);
  FastLED.show();
  digitalWrite(EXT_LIGHT_PIN, LOW);
}

int updateDHT() {
  //This function measures the temp and RH and displays them, calculates the PID output, and sends the variables to the Serial Monitor for plotting. 
  // return 2 indicates fresh readings, 1 means error, 0 is nothing happened.  
  if (millis() - loopStartTime > LOOP_TIME) { // Following section runs every 2 seconds.
    loopStartTime += LOOP_TIME;

    #ifdef USE_DHT22_SENSOR
      RH = dht.readHumidity();
      T = dht.readTemperature();
    #endif
    
    #ifdef USE_AHT20_SENSOR
      sensors_event_t humidity;
      sensors_event_t temp;
      aht.getEvent(&humidity, &temp);
      RH = humidity.relative_humidity;
      T = temp.temperature;
    #endif
    
    
    // Check if any reads failed and exit early (to try again).
    if (isnan(RH) || isnan(T) || RH == 0.0 || T == 0.0 ) {
      Serial.println(F("Failed to read from humidity sensor!"));
//      LEDWarningColour();
      newColour = WARNING_COLOUR;
      strcpy(RHString, "Error");
      strcpy(tempString, "Error");
      menuRHString.setTextValue(RHString);
      menuTString.setTextValue(tempString);
      return 1; //Restart the loop and hopefully get a result next time.
    }

    RHColour = map((int)RH, 0, 100, 32, 160); //LED shows humidity,orange is low to blue is high.
    newColour = RHColour;
    
    sprintf(RHString, " %d%%", int(RH)); //Convert relative humidity to string to display on OLED.
    menuRHString.setTextValue(RHString); //Show the value on OLED.
    sprintf(tempString, " %d°C", int(T)); //Convert relative humidity to string to display on OLED.
    menuTString.setTextValue(tempString);

    RHInput = RH;
    RHPID.Compute();
    
    Serial.print(F("H:"));
    Serial.print(RHInput);
    Serial.print(",");
    Serial.print("sp:");
    Serial.print(presetArr[menuPresets.getCurrentValue()].RHSetPoint);
    Serial.print(",");
    Serial.print("p:");
    Serial.println(RHOutput/10.);//scaled so similar to other variable for plotting
  
    return 2; // 2 indicates fresh readings  
  }
  return 0;
}

void outputPID() {
  //runs continuously when controller is running
  //turn analog PID output into very slow pwm to drive the ultrasonic mister:
  if (RHOutput > millis() - loopStartTime){
    increaseRH();
  }
  else{
    decreaseRH();
  }
}

void checkHumidityAcheivable() {
  // Ensures that the setpoint has been reached fairly recently and isn't unacheivable due to low water,
  // a broken vapouriser disk or driver, a setpoint thats too high for a large box ect.
  // When the controller initialises it stores the time PIDOnStartTime.
  // If after 2 hours (arbitrary) the set point hasn't been reached set the current RH as the setpoint
  // and display a message stating that the controller was unable to reach the asked for setpoint
  // so it has been changed. 2*60*60*1000 = 7200000 milliseconds = 2 hours
  
  if (RH >= presetArr[menuPresets.getCurrentValue()].RHSetPoint) {
    // If RH>=setpoint we know it's acheivable so reset the timer. If it can't be acheived later on
    // probablly the water needs refilling.
    PIDOnStartTime = millis();
  }
  if (millis() - PIDOnStartTime >= 7200000.) { // 7200000.
    RHErrorState();
  }
}

void updateLED() {
  // Check if light should be on TODO test for longer intervals.
  
  if (millis()%MILLIS_PER_DAY < lightOnTimeMillis && LEDStateToggle == 0) { //So LED should be on according to timer.
    menuEnableLED.setBoolean(1);
    LEDStateToggle = 1; //So it only switches once per cycle and doesn't keep turning LED back on if user switches it off manually.
  }
  if (millis()%MILLIS_PER_DAY > lightOnTimeMillis && LEDStateToggle == 1) { //So LED should be off according to timer. {
    menuEnableLED.setBoolean(0);
    LEDStateToggle = 0; //So it only switches once per cycle and doesn't keep turning LED back on if user switches it off manually.
  }
  
  //The LED pulsing and changing colour smoothly is handled here.
  if (menuLEDShowRH.getBoolean()) { // If we want colour to display RH:
    if (millis() - ledStartTime > LED_LOOP_TIME){
      ledStartTime += LED_LOOP_TIME;
      //smoothly transition from the current colour to newColour
      if (colour < newColour) {colour += 1;}
      if (colour > newColour) {colour -= 1;}
      
      if (menuEnableLED.getBoolean()) { // If the user wants the LED on:
        leds[0] = CHSV(colour, 255, sinArray[ledLoopPos]);
        FastLED.show(); 
        digitalWrite(EXT_LIGHT_PIN, HIGH);
      }
      else {
        leds[0] = CHSV(0, 0, 0);
        FastLED.show(); 
        digitalWrite(EXT_LIGHT_PIN, LOW);
      }
      
      ledLoopPos += 1;
      if (ledLoopPos == LED_ARRAY_LENGTH) {ledLoopPos = 0;}
  
    }
  }
  else { // if menuLEDShowRH false we just want the LED to be pure white.
    if (menuEnableLED.getBoolean()) {
      leds[0] = CHSV(255, 0, 255); 
      FastLED.show();
      digitalWrite(EXT_LIGHT_PIN, HIGH);
    }
    else {
      leds[0] = CHSV(0, 0, 0);
      FastLED.show(); 
      digitalWrite(EXT_LIGHT_PIN, LOW);
    }
  }
}

void fanControl() {
    //Fan tach section, measures the time period of the fans tach output to ensure it's operating correctly. 
  if (!activateFan || !menuenableFan.getBoolean()){
    //i.e if fan is not supposed to be on continually keep fanTachStartTime low else when the tach time period is
    //measured the interval will have started the last time the fan was used which is likely to be more than the
    //interval that starts the error state.
    fanTachStartTime = millis();
  }
  //fanTachInterval stores the amount of time since the fan tach signal changed state or the preceding if block stopped running
  fanTachInterval = millis() - fanTachStartTime; 
  
  if (digitalRead(FAN_TACH_PIN) != fanTachState) {
    fanTachState = !fanTachState;
    fanTachStartTime = millis();
  }
  //The interval that starts the error is set in the following section and is a bit arbitrary, it governs how long 
  //after the fan tach signal stops the error state triggers.
  if (activateFan && fanTachInterval > 1000 && menuenableFan.getBoolean() && !fanErrorTriggered) { 
    //if fan should be on but its more than 1000ms since the tach changed state flag an error. 
    // If fan error has already triggered and been cleared then fan is working its just the sensing 
    // thats gone wrong and I don't want devices to brick because of this.
    fanErrorState();
    return;
  }

  //Start FAE if its been more than FAEInterval since it last happened
  if (millis() - FAEOnIntervalStartTime > presetArr[menuPresets.getCurrentValue()].FAEInterval && menuFAEPerHour.getAsFloatingPointValue() != 0){
    //Start fresh air exchange cycle
    FAEOnStartTime = millis();
    FAEOnIntervalStartTime = millis();
    FAEOn = 1;
    activateFan = 1;
//    Serial.print("Fae Started ");
//    Serial.println(millis());
  }
  
  // Once FAE has been on for required amount of time switch it off by setting FAEOn = 0
  if (millis() - FAEOnStartTime > FAEOnTime) {
//    if (FAEOn == 1) {
//      Serial.print("Fae end ");
//      Serial.println(millis());
//    }
    FAEOn = 0;
  }

  //if humidity is v. high, so might start condensing fan is switched on
  if (RH > menuMaxRH.getAsFloatingPointValue()) { 
    activateFan = 1;
    //Serial.println("maxRH Ecd");
  }
    //If mister only just switched off keep fan on for fanOnTime before switching off
  //And if the relative humidity is too high don't turn fan off.
  // If FAEOn = 0 FAE cycle has finished so fan switches off now here too.
  if (millis() - fanOnStartTime > fanOnTime && RH <= menuMaxRH.getAsFloatingPointValue() && !FAEOn){ 
    activateFan = 0;
  }

  //actually control the fan here, after all the different things that can affect it have had their say via activateFan
  if (activateFan && menuenableFan.getBoolean()){
    digitalWrite(FAN_PIN, HIGH);
    digitalWrite(EXT_FAN_PIN, HIGH);
  }
  else {
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(EXT_FAN_PIN, LOW);
  }
}

void updateDisplayNonLoop() {
  //Updates the display when I don't want to poll func in void loop.
  // Runs taskmanager until it's time for it to update the screen again.
  // Shouldn't be used anywhere time sensitive as it introduces a delay.
  //Probably there is a more elegant solution than this but it works.
  for (int i=0; i<100; i++) {
    taskManager.runLoop();
    delay(1);
  }
}

void measureAirExchangeTime() {
  // This function attempts to measure the time taken for the fan to move the chambers volume of air.
  // It first records the ambiant humidity outside the chamber, then humidifies to the setpoint, leaves if for a 
  // while for the air to mix in the chamber a bit, then runs the fan until the humidity is half way between ambiant and
  // the humidity measured after mixing. Its probably not extremely accurate but gets close enough.
  
  //Following displays some text on the screen.
  auto drawable = renderer.getDeviceDrawable();
  gfx.clear(); 
  drawable->startDraw();

  drawable->drawText(Coord(0,0), u8g2_font_6x10_tf, 0, "Calibrating FAE.");
  drawable->drawText(Coord(0,10), u8g2_font_6x10_tf, 0, "Before continuing");
  drawable->drawText(Coord(0,20), u8g2_font_6x10_tf, 0, "ensure sensor is at");
  drawable->drawText(Coord(0,30), u8g2_font_6x10_tf, 0, "ambiant humidity.");
  drawable->endDraw();

  updateDisplayNonLoop();

  
  Serial.println("FAE fan on time Measurement started.");
  initController();
  while (updateDHT() != 2){ }//Just loop in this while until updateDHT returns 2 meaning fresh readings
  
  double initialRH = RH; //Store external RH
  Serial.print("initial RH ");
  Serial.println(initialRH);
  
  //For this to work RHSetPoint must be greater than external RH, Ideally the setpoint you're going to be running at.
  digitalWrite(FAN_PIN, HIGH);
  digitalWrite(EXT_FAN_PIN, HIGH);
  
  while(RH < presetArr[menuPresets.getCurrentValue()].RHSetPoint){
    if (updateDHT() == 1) { continue; }
    outputPID();
  }
  
  // Leave it to mix 
  Serial.println("mixing");
  delay(5000);
  deinitController();
  delay(25000);
  // Measure current RH
  Serial.println("measuring RH");
  while (updateDHT() != 2){ }//Just loop in this while until updateDHT returns 2 meaning fresh readings
  double finalRH = 0.5*(initialRH + RH);
  Serial.print("final RH ");
  Serial.println(finalRH);
  
  unsigned long measureAirExchangeStartTime = millis();
  Serial.print("Start time ");
  Serial.println(measureAirExchangeStartTime);
  digitalWrite(FAN_PIN, HIGH);
  digitalWrite(EXT_FAN_PIN, HIGH);
  initController();
  while (RH > finalRH){
    while (updateDHT() != 2){ }//Just loop in this while until updateDHT returns 2 meaning fresh readings   
  }
  
  unsigned long newFAEOnTime = millis() - measureAirExchangeStartTime;
  if (newFAEOnTime <= 60000.) { newFAEOnTime = 60001.;} //So its never less than 1 minute.
  menuFAEOnTime.setFromFloatingPointValue(newFAEOnTime/60000.0); //Sends value to screen in minutes. tcmenu turns to 0 as rounded down, then sends it back to FAEOnTime
  Serial.println(millis());
  FAEOnTime = newFAEOnTime; //FAEOnTime is updated with rounding error'd number in the callback, so give it the correct value here.
  Serial.print("Air exchange time ");
  Serial.println(FAEOnTime);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(EXT_FAN_PIN, LOW);
  SaveSettingsCallback(17); //Save new value.
}

void increaseRH() {
  //turns on mister and fan.
  if (!isMisterOn) {
    //Serial.println("incRH ");
    if (isMisterOn == 0) {
      // This section only runs the first time mister is switched on in a cycle.
      activateFan = 1;
      fanOnStartTime = millis();
    }
    activateFan = 1;
    fanOnStartTime = millis();
    ledcWrite(0, 119);// 119/255=47% duty cycle
    digitalWrite(EXT_HUMIDIFIER_PIN, HIGH);
    isMisterOn = 1;
  }
}

void decreaseRH() {
  //turns off mister and fan after a delay of fanOnTime
  if (isMisterOn) { 
    //Serial.println("decRH");
    
    ledcWrite(0, 0);// 0% duty cycle
    digitalWrite(EXT_HUMIDIFIER_PIN, LOW);
    isMisterOn = 0;
  }
}

void setFAEInterval() {
  //Calculate the number of milliseconds interval that FAEPerHour defines.
  if (menuFAEPerHour.getAsFloatingPointValue() == 0.0) { 
    presetArr[menuPresets.getCurrentValue()].FAEInterval = 0;
  }
  else {
    presetArr[menuPresets.getCurrentValue()].FAEInterval = 3600000.0/menuFAEPerHour.getAsFloatingPointValue(); //= millisPerHour/FAEPerHour
  }
}

void onFanDialogFinished(ButtonType btnPressed, void* /*userdata*/) {        
  if(btnPressed == BTNTYPE_ACCEPT) {
    menuControllerBool.setBoolean(1);
  }
}

void LEDWarningColour() {
  leds[0] = CHSV(WARNING_COLOUR, 255, 255);
  FastLED.show();
}

void fanErrorState() {
  Serial.println("Fan Error");
  //turn off the controller
  menuControllerBool.setBoolean(0);
  //Make the error noticable by changing the colour to solid red.
  LEDWarningColour();
  fanErrorTriggered = 1; //Now the error wont trigger again until the Cave Box is restarted.

  // Produces dialog describing the issue
  withMenuDialogIfAvailable([] (MenuBasedDialog* dlg) {
      dlg->setButtons(BTNTYPE_NONE, BTNTYPE_ACCEPT);
      dlg->show(ErrorWarningPgm, false, onFanDialogFinished);
      dlg->copyIntoBuffer("Fan Error.");
  });
}

// this is called when the RHErrorState dialog is dismissed.
void onRHDialogFinished(ButtonType btnPressed, void* /*userdata*/) {        
  if(btnPressed == BTNTYPE_ACCEPT) {
    menuControllerBool.setBoolean(1);
  }
}

void RHErrorState() {
  // If setpoint cannot be reached it displays an acheivable setpoint.
  
  Serial.println("Setpoint Unatainable");
  menuControllerBool.setBoolean(0);//turn off the controller
  //Make the error noticable by changing the colour to solid red.
  LEDWarningColour();
 // Serial.println("LED red");

  
  sprintf(newRHMsg, "Max RH = %d%%", int(RH - 1.));
  withMenuDialogIfAvailable([] (MenuBasedDialog* dlg) {
    dlg->setButtons(BTNTYPE_NONE, BTNTYPE_ACCEPT);
    dlg->showRam("RH Too Low", false, onRHDialogFinished);
    dlg->copyIntoBuffer(newRHMsg);
  });
}

void onFactoryResetDialogFinished(ButtonType btnPressed, void* /*userdata*/) {        
  if(btnPressed == BTNTYPE_CANCEL) {
    //Serial.println("cnl");
  }
  
  if(btnPressed == BTNTYPE_OK) {
    Serial.println("factory Reset");

    //menuFAEPerHour.setFromFloatingPointValue(5.); 
    menuFAEOnTime.setFromFloatingPointValue(7.);
    FAEOnTime = menuFAEOnTime.getAsFloatingPointValue()*60.*1000.;
    
    
    menuPValue.setFromFloatingPointValue(2000.0);
    menuIValue.setFromFloatingPointValue(0.01);
    menuDValue.setFromFloatingPointValue(0.0);
    RHKp = (double)menuPValue.getAsFloatingPointValue();
    RHKi = (double)menuIValue.getAsFloatingPointValue();
    RHKd = (double)menuDValue.getAsFloatingPointValue();
    RHPID.SetTunings(RHKp, RHKi, RHKd);
    menuEnableLED.setBoolean(1);
    menuControllerBool.setBoolean(1);
    menuenableFan.setBoolean(1);
    menuLEDShowRH.setBoolean(1);
    menuLightOnTime.setFromFloatingPointValue(24.0);
    menuMaxRH.setFromFloatingPointValue(96.0);
    
    menuPresets.setCurrentValue(1);
    
    // FAEInterval (ms) = 60*60*1000/FAEPerHr
    // All parameters are from Stamets Growing Gourmet and Medicinal Mushrooms
    // Where a range of values stated I chose the middle, and I rounded down.
    preset newPresetArr[] = {
    {95.0, 450000}, //Pin 95% 8FAE, very high humidity and lots of fresh air exchange
    {85.0, 600000}, //Fruit 85% 6FAE, moderate humidity and FAE
    {87.0, 900000}, //Oyster 87% 4FAE
    {92.0, 600000}, //Lions Mane 92% 6FAE
    {70.0, 600000}, //Shiitake 70% 6FAE
    {92.0, 900000}, //Reishi 92% 4FAE as Stamets doesn't specify.
    {85.0, 600000}, //Custom1 85% 6FAE, moderate humidity and FAE User can decide.
    {85.0, 600000}, //Custom2 85% 6FAE, moderate humidity and FAE User can decide.
    {85.0, 600000}, //Custom3 85% 6FAE, moderate humidity and FAE User can decide.
    {85.0, 600000}, //Custom4 85% 6FAE, moderate humidity and FAE User can decide.
    {85.0, 600000}, //Custom5 85% 6FAE, moderate humidity and FAE User can decide.
    {85.0, 600000}  //Custom6 85% 6FAE, moderate humidity and FAE User can decide.
    };
    memcpy(presetArr, newPresetArr, sizeof(presetArr));
    
    changeRHSetpoint(presetArr[menuPresets.getCurrentValue()].RHSetPoint);
    menuFAEPerHour.setFromFloatingPointValue(3600000.0/presetArr[menuPresets.getCurrentValue()].FAEInterval); //millisPerHr/millisPerInterval gives air exchanges per hour.
    
    SaveSettingsCallback(17);
  }
}

void changeRHSetpoint(double newSetPoint){
  //using a pointer to menuPresets.getCurrentValue()].RHSetPoint in PID declaration doesn't update the 
  // value when changed later, so this function ties together all the places the setpoint is stored.
  RHSetPoint = newSetPoint; //tells pid
  presetArr[menuPresets.getCurrentValue()].RHSetPoint = newSetPoint; //puts new value in active preset struct.
  menuRHSetPoint.setFromFloatingPointValue(newSetPoint);// Updates TCmenu variable.
  MaxRHCallback(25); // Ensures that max RH is greater than the new RH
  // Setpoint has changed so restart the timer that counts how long it's taken to get to the setpoint.
  PIDOnStartTime = millis();
}

//// TC Menu Callback functions ////
void CALLBACK_FUNCTION factoryReset(int id) {
  withMenuDialogIfAvailable([] (MenuBasedDialog* dlg) {
    dlg->setButtons(BTNTYPE_CANCEL, BTNTYPE_OK);
    dlg->showRam("Factory Reset", false, onFactoryResetDialogFinished);
    dlg->copyIntoBuffer("Are You Sure?");
  });
}


void CALLBACK_FUNCTION FAEChangedCallback(int id) {
  setFAEInterval();
//  Serial.print("FAE interval ");
//  Serial.println(FAEInterval);
}


void CALLBACK_FUNCTION RHChangedCallback(int id) {
  changeRHSetpoint((double)menuRHSetPoint.getIntValueIncludingOffset());
}


void CALLBACK_FUNCTION controllerEnableCallback(int id) {
  //Serial.println("cntrl enbl callback");
  if (menuControllerBool.getBoolean()){
    initController();
  }
  else {
    deinitController();
  }
}



void CALLBACK_FUNCTION enableLEDCallback(int id) {
    if (menuEnableLED.getBoolean()){
      ledStartTime = millis();
    }
    else {
      leds[0] = CHSV(colour, 255, 0);
      FastLED.show();
      digitalWrite(EXT_LIGHT_PIN, LOW);
    }
}



void CALLBACK_FUNCTION DValueCallback(int id) {
  RHKd = (double)menuDValue.getAsFloatingPointValue();
  RHPID.SetTunings(RHKp, RHKi, RHKd);
}


void CALLBACK_FUNCTION PValueCallback(int id) {
  RHKp = (double)menuPValue.getAsFloatingPointValue();
  RHPID.SetTunings(RHKp, RHKi, RHKd);
}


void CALLBACK_FUNCTION IValueCallback(int id) {
  RHKi = (double)menuIValue.getAsFloatingPointValue();
  RHPID.SetTunings(RHKp, RHKi, RHKd);
}



void CALLBACK_FUNCTION SaveSettingsCallback(int id) {
  EEPROM.put(PRESET_ARRAY_ADDRESS, presetArr);
  //glArduinoEeprom.writeArrayToRom(PRESET_ARRAY_ADDRESS, presetArr, sizeof presetArr);
  menuMgr.save();
  EEPROM.commit();
  withMenuDialogIfAvailable([] (MenuBasedDialog* dlg) {
    dlg->setButtons(BTNTYPE_NONE, BTNTYPE_OK);
    dlg->showRam("Settings Saved", false);
    dlg->copyIntoBuffer(" ");
  });
}


void CALLBACK_FUNCTION FAEOnTimeCallBack(int id) {
    FAEOnTime = menuFAEOnTime.getAsFloatingPointValue()*60.*1000.; //Convert to milliseconds from minutes.
}



void CALLBACK_FUNCTION CalibrateFAECallback(int id) {
  initController();
  measureAirExchangeTime();
}



void CALLBACK_FUNCTION lightOntimeCallback(int id) {
    //Convert hrs to millis:
    lightOnTimeMillis = menuLightOnTime.getAsFloatingPointValue()*60.*60.*1000.;
    ledStartTime = millis();
    
}



void CALLBACK_FUNCTION LEDShowRHCallback(int id) {
    ledStartTime = millis();
}


void CALLBACK_FUNCTION PresetsCallback(int id) {
    //Update TCMenu with the presets vars
    changeRHSetpoint(presetArr[menuPresets.getCurrentValue()].RHSetPoint);
    menuFAEPerHour.setFromFloatingPointValue(3600000.0/presetArr[menuPresets.getCurrentValue()].FAEInterval); //millisPerHr/millisPerInterval gives air exchanges per hour.

}


void CALLBACK_FUNCTION MaxRHCallback(int id) {
    if (menuMaxRH.getAsFloatingPointValue() <= presetArr[menuPresets.getCurrentValue()].RHSetPoint) {
      menuMaxRH.setFromFloatingPointValue(presetArr[menuPresets.getCurrentValue()].RHSetPoint + 5.0);
    }

    if (menuMaxRH.getAsFloatingPointValue() > 98.0) {
      menuMaxRH.setFromFloatingPointValue(98.0);
    }
}



void CALLBACK_FUNCTION PrintSettingsCallback(int id) {
    Serial.print("RH ");
    Serial.println(RH);
    Serial.print("RH Setpoint ");
    Serial.println(RHSetPoint);
    Serial.print("menuRHSetPoint ");
    Serial.println(menuRHSetPoint.getAsFloatingPointValue());
    Serial.print("Temp C ");
    Serial.println(T);
    Serial.println();
    
    Serial.print("FAEOnTime (ms) ");
    Serial.println(FAEOnTime);
    Serial.print("menuFAEOnTime (mins) ");
    Serial.println(menuFAEOnTime.getAsFloatingPointValue());

    Serial.print("FAE Interval ");
    Serial.println(presetArr[menuPresets.getCurrentValue()].FAEInterval);
    Serial.print("");
    Serial.println();
    Serial.print("");
    Serial.println();
    Serial.print("");
    Serial.println();
    
    Serial.print("RHKp ");
    Serial.println(RHKp);
    Serial.print("RHKi ");
    Serial.println(RHKi);
    Serial.print("RHKd ");
    Serial.println(RHKd);
    Serial.println();

    Serial.print("enable LED ");
    Serial.println(menuEnableLED.getBoolean());
    Serial.print("Controller On ");
    Serial.println(menuControllerBool.getBoolean());

    Serial.print("enable fan ");
    Serial.println(menuenableFan.getBoolean());
    Serial.print("LED show RH ");
    Serial.println(menuLEDShowRH.getBoolean());
    
    Serial.print("light on time ");
    Serial.println(menuLightOnTime.getAsFloatingPointValue());
    
    Serial.print("max RH ");
    Serial.println(menuMaxRH.getAsFloatingPointValue());
    Serial.print("");
    Serial.println();
    Serial.print("");
    Serial.println();
    Serial.print("");
    Serial.println();
    Serial.print("");
    Serial.println();
    Serial.print("");
    Serial.println();
    Serial.print("");
    Serial.println();
}



void CALLBACK_FUNCTION firmwareVersionCallback(int id) {
  withMenuDialogIfAvailable([] (MenuBasedDialog* dlg) {
    dlg->setButtons(BTNTYPE_NONE, BTNTYPE_OK);
    dlg->showRam("Software Version", false);
    dlg->copyIntoBuffer(FIRMWARE_VERSION);
  });
}
