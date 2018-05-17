// Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
// This is a human-readable summary of (and not a substitute for) the license.
// Disclaimer
//
// You are free to:
// Share — copy and redistribute the material in any medium or format
// Adapt — remix, transform, and build upon the material
// The licensor cannot revoke these freedoms as long as you follow the license terms.
//
// Under the following terms:
// Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
// NonCommercial — You may not use the material for commercial purposes.
// ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
// No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
//
// Notices:
// You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.
// No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
//
// github: https://github.com/gusgonnet/waterTemperatureControl
// hackster: XXXXXXXXXXXXXXX - XXXXXXXXXXXXXXXX
//
// Free for personal use.
//
// https://creativecommons.org/licenses/by-nc-sa/4.0/
//
// Uses this library for the DS18b20 sensor: https://github.com/LukeUSMC/ds18b20-photon
// Datasheet: http://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf
//

#include "Particle-OneWire.h"
#include "DS18B20.h"
#include "NCD4Relay.h"
#include "PietteTech_DHT.h"
#include "elapsedMillis.h"
#include "FiniteStateMachine.h"

#define APP_NAME "waterTemperatureControl"
String VERSION = "Version 0.03";

SYSTEM_MODE(AUTOMATIC);

/*******************************************************************************
 * changes in version 0.01:
       * Initial version
 * changes in version 0.02:
       * particle share link: https://go.particle.io/shared_apps/5aea7c1e5ae4df19f3000d87 (with particle build libraries for ds18)
       * particle share link: https://go.particle.io/shared_apps/5afce71c04e41960b7000816 (with library for ds18: https://github.com/LukeUSMC/ds18b20-photon)
       * adding second ds18b20 sensor for sensing ambient temperature on D4
       * adding DHT22 sensor for sensing ambient temperature and humidity on D5
 * changes in version 0.03:
       * particle share link: https://go.particle.io/shared_apps/5afd72bb04e419d0e3000f40
       * adding cloud function setOnOff() to set this prj on or off
 *******************************************************************************/

// Argentina time zone GMT-3
const int TIME_ZONE = -3;

/*******************************************************************************
 declare FSM states with proper enter, update and exit functions
 we currently have the following states:
  - init     : when the system boots up it starts in this state.
               After some time (ten seconds), the system transitions to the idle state.
  - idle     : the water temperature is being monitored.
               No relays are activated.
  - cooling  : adds cold water to cool down the water temperature
  - warming  : adds hot water to raise the water temperature
  - filling  : adding water
  - emptying : removing water

*******************************************************************************/
State initState = State(initEnterFunction, initUpdateFunction, initExitFunction);
State offState = State(offEnterFunction, offUpdateFunction, offExitFunction);
State idleState = State(idleEnterFunction, idleUpdateFunction, idleExitFunction);
State coolingState = State(coolingEnterFunction, coolingUpdateFunction, coolingExitFunction);
State warmingState = State(warmingEnterFunction, warmingUpdateFunction, warmingExitFunction);
State fillingState = State(fillingEnterFunction, fillingUpdateFunction, fillingExitFunction);
State emptyingState = State(emptyingEnterFunction, emptyingUpdateFunction, emptyingExitFunction);

//initialize state machine, start in state: init
FSM stateMachine = FSM(initState);

// Sample FSM every now and then - 100 milliseconds means 10 times per second
#define QUICK_LOOP_INTERVAL 100
elapsedMillis quickLoopTimer;

// FSM states constants
#define STATE_INIT "Initializing"
#define STATE_OFF "Off"
#define STATE_IDLE "Idle"
#define STATE_COOLING "Cooling"
#define STATE_WARMING "Warming"
#define STATE_FILLING "Filling"
#define STATE_EMPTYING "Emptying"
String state = STATE_INIT;
String command = "";

// timers work on millis, so we adjust the value with this constant
#define MILLISECONDS_TO_MINUTES 60000
#define MILLISECONDS_TO_SECONDS 1000

// 10 seconds for the init cycle, so sensor samples get stabilized
// units: seconds
int initTimeout = 10;

// a state has to have a minimum duration time: this is now one minute
// units: minutes
int minimumTimeInState = 1;

/*******************************************************************************
 temperature sensor and variables
*******************************************************************************/
//Sets Pin D2 for Temp Sensor
DS18B20 ds18b20 = DS18B20(D2);

// Sample temperature sensor every 30 seconds
#define TEMPERATURE_SAMPLE_INTERVAL 30 * MILLISECONDS_TO_SECONDS
elapsedMillis temperatureSampleInterval;

//temperature related variables
#define INVALID -1
double temperatureCurrent = INVALID;
double temperatureTarget = 30.0;

//sensor difference with real temperature (if none set to zero)
//use this variable to align measurements with your existing displays
double temperatureCalibration = 0;

//you can change this to your liking
double temperatureMargin = 2; //0.25

// Celsius is the default unit, set this boolean to true if you want to use Fahrenheit
const bool useFahrenheit = false;

/*******************************************************************************
 temperature sensor and variables for ambient sensing
*******************************************************************************/
//Sets Pin D4 for Ambient Temp Sensor
DS18B20 ds18b20_3 = DS18B20(D4);
double temperatureCurrent3 = INVALID;

/*******************************************************************************
 DHT sensor for ambient sensing
*******************************************************************************/
#define DHTTYPE DHT22 // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN 5      // Digital pin for communications
void dht_wrapper();   // must be declared before the lib initialization
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);
bool bDHTstarted; // flag to indicate we started acquisition
double temperatureCurrent4 = INVALID;
double humidityCurrent4 = INVALID;
// This wrapper is in charge of calling the DHT sensor lib
void dht_wrapper() { DHT.isrCallback(); }

/*******************************************************************************
 relay variables
*******************************************************************************/
NCD4Relay relayController;
#define RELAY_MIXING_PUMP 1
#define RELAY_DRAIN_PUMP 2
#define RELAY_COLD_WATER_VALVE 3
#define RELAY_HOT_WATER_VALVE 4

#define MAX_SENSOR D0
#define MIN_SENSOR D1

/*******************************************************************************
 structure for writing settings in eeprom
 https://docs.particle.io/reference/firmware/photon/#eeprom
*******************************************************************************/
//randomly chosen value here. The only thing that matters is that it's not 255
// since 255 is the default value for uninitialized eeprom
// value 143 will be used in version 0.1
#define EEPROM_VERSION 143
#define EEPROM_ADDRESS 0

struct EepromMemoryStructure
{
  uint8_t version = EEPROM_VERSION;
  double temperatureTarget;
  double temperatureCalibration;
};
EepromMemoryStructure eepromMemory;

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup()
{

  // publish startup message with firmware version
  Particle.publish(APP_NAME, VERSION, PRIVATE);

  // declare cloud variables
  // https://docs.particle.io/reference/firmware/photon/#particle-variable-
  // Up to 20 cloud variables may be registered and each variable name is limited to a maximum of 12 characters.
  Particle.variable("temperature", temperatureCurrent);
  Particle.variable("target", temperatureTarget);
  Particle.variable("calibration", temperatureCalibration);
  // This variable informs the state of the system
  Particle.variable("state", state);

  // declare cloud functions
  // https://docs.particle.io/reference/firmware/photon/#particle-function-
  // Up to 15 cloud functions may be registered and each function name is limited to a maximum of 12 characters.
  Particle.function("setTarget", setTarget);
  Particle.function("setCalbrtion", setCalibration);
  Particle.function("setOnOff", setOnOff);

  /*******************************************************************************
   cloud variables and functions for the ambient temperature
  *******************************************************************************/
  Particle.variable("tempAmbient", temperatureCurrent3);

  Particle.variable("tempAmbieDHT", temperatureCurrent4);
  Particle.variable("humiAmbieDHT", humidityCurrent4);

  Time.zone(TIME_ZONE);

  relayController.setAddress(0, 0, 0);

  // this is needed if you are flashing new versions while there is one or more relays activated
  // they will remain active if we don't turn them off
  relayController.turnOffAllRelays();

  //restore settings from eeprom, if there were any saved before
  readFromEeprom();
}

/*******************************************************************************
 * Function Name  : loop
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
void loop()
{
  quickLoop();
}

/*******************************************************************************
 * Function Name  : quickLoop
 * Description    : this function runs every 100 milliseconds (10 times a second)
                     to save power
 *******************************************************************************/
void quickLoop()
{

  // is time up? no, then come back later
  if (quickLoopTimer < QUICK_LOOP_INTERVAL)
  {
    return;
  }

  // time is up, reset timer
  quickLoopTimer = 0;

  // read the temperature sensors
  readTemperature();

  // update the FSMs
  // the FSM is the heart of the program - all actions are defined by its states
  stateMachine.update();

  // command takes in an asynchronous call from a particle cloud function
  // the FSM addressed the command with the stateMachine.update() call, so we can flush it here
  command = "";
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 CONTROL FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : setOnOff
 * Description    : this function sets the system on or off
 * Parameters     : String parameter: on/ON/On or off/OFF/Off
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setOnOff(String parameter)
{

  // validate percentage
  if (!((parameter.equalsIgnoreCase("on")) or (parameter.equalsIgnoreCase("off"))))
  {
    Particle.publish(APP_NAME, "ERROR: Invalid command: " + parameter + ". Try on or off");
    return -1;
  }

  command = parameter;

  return 0;
}

/*******************************************************************************
 * Function Name  : setTarget
 * Description    : this function sets the target temperature
 * Parameters     : String parameter: the target temperature
                     Value has to be between 10 and 70. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setTarget(String parameter)
{

  double localValue = atoi(parameter);

  if ((localValue >= 10) && (localValue <= 70))
  {
    temperatureTarget = localValue;
    Particle.publish(APP_NAME, "Setting temperature target to " + double2string(temperatureTarget), PRIVATE);
    saveSettingsInEeprom();
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set temperature target to " + parameter + " (10<=value<=70)", PRIVATE);
  return -1;
}

/*******************************************************************************
 * Function Name  : setCalibration
 * Description    : this function sets the calibration of the temperature
 * Parameters     : String parameter: the calibration value
                     Value has to be between -50 and 50. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setCalibration(String parameter)
{

  double localValue = atoi(parameter);

  if ((localValue >= -50) && (localValue <= 50))
  {

    // rollback previous calibration adjustment
    temperatureCurrent = temperatureCurrent - temperatureCalibration;

    // store new calibration value
    temperatureCalibration = localValue;

    // apply new calibration adjustment
    temperatureCurrent = temperatureCurrent + temperatureCalibration;

    Particle.publish(APP_NAME, "Setting temperature calibration to " + double2string(temperatureCalibration), PRIVATE);
    saveSettingsInEeprom();
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set temperature calibration to " + parameter + " (-50<=value<=50)", PRIVATE);
  return -1;
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 SENSOR FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : readTemperature
 * Description    : reads the temperature sensor in D2 at every TEMPERATURE_SAMPLE_INTERVAL
 * Return         : none
 *******************************************************************************/
void readTemperature()
{

  // is time up? no, then come back later
  if (temperatureSampleInterval < TEMPERATURE_SAMPLE_INTERVAL)
  {
    return;
  }

  //is time up, reset timer
  temperatureSampleInterval = 0;

  getTemp();
  getTempAmb();
  getTempAmbDHT();

  Particle.publish(APP_NAME, "TAmb: " + double2string(temperatureCurrent3) + ", TAmbDHT: " + double2string(temperatureCurrent4) + ", HAmbDHT: " + double2string(humidityCurrent4), PRIVATE);
}

/*******************************************************************************
 * Function Name  : getTemp
 * Description    : reads the DS18B20 sensor
 * Return         : nothing
 *******************************************************************************/
void getTemp()
{

  int dsAttempts = 0;
  double temperatureLocal = INVALID;

  if (!ds18b20.search())
  {
    ds18b20.resetsearch();
    temperatureLocal = ds18b20.getTemperature();
    while (!ds18b20.crcCheck() && dsAttempts < 4)
    {
      dsAttempts++;
      if (dsAttempts == 3)
      {
        delay(1000);
      }
      ds18b20.resetsearch();
      temperatureLocal = ds18b20.getTemperature();
      continue;
    }
    dsAttempts = 0;

    if (useFahrenheit)
    {
      temperatureLocal = ds18b20.convertToFahrenheit(temperatureLocal);
    }

    // calibrate values
    temperatureLocal = temperatureLocal + temperatureCalibration;

    // if reading is valid, take it
    if ((temperatureLocal != INVALID) && (ds18b20.crcCheck()))
    {
      temperatureCurrent = temperatureLocal;
    }
  }
}

/*******************************************************************************
 * Function Name  : getTempAmb
 * Description    : reads the third DS18B20 sensor (ambient)
 * Return         : nothing
 *******************************************************************************/
void getTempAmb()
{

  int dsAttempts = 0;
  double temperatureLocal = INVALID;

  if (!ds18b20_3.search())
  {
    ds18b20_3.resetsearch();
    temperatureLocal = ds18b20_3.getTemperature();
    while (!ds18b20_3.crcCheck() && dsAttempts < 4)
    {
      dsAttempts++;
      if (dsAttempts == 3)
      {
        delay(1000);
      }
      ds18b20_3.resetsearch();
      temperatureLocal = ds18b20_3.getTemperature();
      continue;
    }
    dsAttempts = 0;

    if (useFahrenheit)
    {
      temperatureLocal = ds18b20_3.convertToFahrenheit(temperatureLocal);
    }

    // if reading is valid, take it
    if ((temperatureLocal != INVALID) && (ds18b20_3.crcCheck()))
    {
      temperatureCurrent3 = temperatureLocal;
    }
  }
}

/*******************************************************************************
 * Function Name  : getTempAmbDHT
 * Description    : reads the temperature of the DHT22 sensor
 * Return         : none
 *******************************************************************************/
void getTempAmbDHT()
{

  // start the sample
  if (!bDHTstarted)
  {
    DHT.acquireAndWait(5);
    bDHTstarted = true;
  }

  //still acquiring sample? go away and come back later
  if (DHT.acquiring())
  {
    return;
  }

  //I observed my dht22 measuring below 0 from time to time, so let's discard that sample
  if (DHT.getCelsius() < 0)
  {
    //reset the sample flag so we can take another
    bDHTstarted = false;
    return;
  }

  if (useFahrenheit)
  {
    temperatureCurrent4 = DHT.getFahrenheit();
  }
  else
  {
    temperatureCurrent4 = DHT.getCelsius();
  }

  humidityCurrent4 = DHT.getHumidity();

  //reset the sample flag so we can take another
  bDHTstarted = false;
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 FINITE STATE MACHINE FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * FSM state Name        : init 
 * Description           : when the system boots starts in this state to stabilize sensors.
                            After some time (ten seconds), the system transitions to the off state.
*******************************************************************************/
void initEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_INIT);
}
void initUpdateFunction()
{
  // is time up? no, then come back later
  if (stateMachine.timeInCurrentState() < (initTimeout * MILLISECONDS_TO_SECONDS))
  {
    return;
  }

  stateMachine.transitionTo(offState);
}
void initExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : off 
 * Description           : the system does not do anything - it's OFF
*******************************************************************************/
void offEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_OFF);
}
void offUpdateFunction()
{
  // on command received?
  if (command == "on")
  {
    stateMachine.transitionTo(idleState);
  }
  // off command received?
  if (command == "off")
  {
    Particle.publish(APP_NAME, "The system is off already", PRIVATE);
  }
}
void offExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : idle
 * Description           : 
*******************************************************************************/
void idleEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_IDLE);
}
void idleUpdateFunction()
{
  // off command received?
  if (command == "off")
  {
    stateMachine.transitionTo(offState);
  }
  // on command received?
  if (command == "on")
  {
    Particle.publish(APP_NAME, "The system is on already", PRIVATE);
  }

  // is there too much water? empty a bit
  if (MAX_SENSOR == 1)
  {
    stateMachine.transitionTo(emptyingState);
  }
  // is there too litle water? add some
  if (MIN_SENSOR == 0)
  {
    stateMachine.transitionTo(fillingState);
  }
  // is it too hot? cool it down
  if (temperatureCurrent > (temperatureTarget + temperatureMargin))
  {
    stateMachine.transitionTo(coolingState);
  }
  // is it too cold? warm it up
  if (temperatureCurrent < (temperatureTarget - temperatureMargin))
  {
    stateMachine.transitionTo(warmingState);
  }
}
void idleExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : cooling
 * Description           : 
*******************************************************************************/
void coolingEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_COOLING);
  // turn on the recirculation (mixing) pump
  turnOnRelay(RELAY_MIXING_PUMP);
  // open the cold water valve
  turnOnRelay(RELAY_COLD_WATER_VALVE);
}
void coolingUpdateFunction()
{
  // off command received?
  if (command == "off")
  {
    stateMachine.transitionTo(offState);
  }
  // on command received?
  if (command == "on")
  {
    Particle.publish(APP_NAME, "The system is on already", PRIVATE);
  }

  // is it cool enough? go back to idle
  if (temperatureCurrent <= temperatureTarget)
  {
    stateMachine.transitionTo(idleState);
  }
  // is there too much water? empty a bit
  if (MAX_SENSOR == 1)
  {
    stateMachine.transitionTo(emptyingState);
  }

  //TODO: set a maximum timer as safety just in case level sensors go wrong
}
void coolingExitFunction()
{
  // turn off the recirculation (mixing) pump
  turnOffRelay(RELAY_MIXING_PUMP);
  // close the cold water valve
  turnOffRelay(RELAY_COLD_WATER_VALVE);
}

/*******************************************************************************
 * FSM state Name        : warming
 * Description           : 
*******************************************************************************/
void warmingEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_WARMING);
  // turn on the recirculation (mixing) pump
  turnOnRelay(RELAY_MIXING_PUMP);
  // open the hot water valve
  turnOnRelay(RELAY_HOT_WATER_VALVE);
}
void warmingUpdateFunction()
{
  // off command received?
  if (command == "off")
  {
    stateMachine.transitionTo(offState);
  }
  // on command received?
  if (command == "on")
  {
    Particle.publish(APP_NAME, "The system is on already", PRIVATE);
  }

  // is it warm enough? go back to idle
  if (temperatureCurrent >= temperatureTarget)
  {
    stateMachine.transitionTo(idleState);
  }
  // is there too much water? empty a bit
  if (MAX_SENSOR == 1)
  {
    stateMachine.transitionTo(emptyingState);
  }

  //TODO: set a maximum timer as safety just in case level sensors go wrong
}
void warmingExitFunction()
{
  // turn off the recirculation (mixing) pump
  turnOffRelay(RELAY_MIXING_PUMP);
  // close the hot water valve
  turnOffRelay(RELAY_HOT_WATER_VALVE);
}

/*******************************************************************************
 * FSM state Name        : filling
 * Description           : 
*******************************************************************************/
void fillingEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_FILLING);
  // turn on the recirculation (mixing) pump
  turnOnRelay(RELAY_MIXING_PUMP);
  // open the hot water valve
  turnOnRelay(RELAY_HOT_WATER_VALVE);
  // open the cold water valve
  turnOnRelay(RELAY_COLD_WATER_VALVE);
}
void fillingUpdateFunction()
{
  // off command received?
  if (command == "off")
  {
    stateMachine.transitionTo(offState);
  }
  // on command received?
  if (command == "on")
  {
    Particle.publish(APP_NAME, "The system is on already", PRIVATE);
  }

  // is there enough water? transition to idle
  if (MIN_SENSOR == 1)
  {
    stateMachine.transitionTo(idleState);
  }

  // this is a safety measure in case something goes wrong with the MIN sensor
  // this would be too much water, but let's exit from this state nonetheless
  if (MAX_SENSOR == 1)
  {
    stateMachine.transitionTo(idleState);
  }

  //TODO: set a maximum timer just in case both level sensors go wrong
}
void fillingExitFunction()
{
  // turn off the recirculation (mixing) pump
  turnOffRelay(RELAY_MIXING_PUMP);
  // close the hot water valve
  turnOffRelay(RELAY_HOT_WATER_VALVE);
  // close the cold water valve
  turnOffRelay(RELAY_COLD_WATER_VALVE);
}

/*******************************************************************************
 * FSM state Name        : emptying
 * Description           : 
*******************************************************************************/
void emptyingEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_EMPTYING);
  // turn on the drain pump
  turnOnRelay(RELAY_DRAIN_PUMP);
}
void emptyingUpdateFunction()
{
  // off command received?
  if (command == "off")
  {
    stateMachine.transitionTo(offState);
  }
  // on command received?
  if (command == "on")
  {
    Particle.publish(APP_NAME, "The system is on already", PRIVATE);
  }

  // is there enough water? transition to idle
  if (MAX_SENSOR == 0)
  {
    stateMachine.transitionTo(idleState);
  }

  // this is a safety measure in case something goes wrong with the MAX sensor
  if (MIN_SENSOR == 0)
  {
    stateMachine.transitionTo(idleState);
  }

  //TODO: set a maximum timer just in case both level sensors go wrong
}
void emptyingExitFunction()
{
  // turn off the drain pump
  turnOffRelay(RELAY_DRAIN_PUMP);
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 HELPER FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : double2string
 * Description    : return the string representation of the double number
                     passed as parameter with 2 decimals
 * Return         : the string
 *******************************************************************************/
String double2string(double doubleNumber)
{
  String stringNumber = String(doubleNumber);

  //return only 2 decimals
  // Example: show 19.00 instead of 19.000000
  stringNumber = stringNumber.substring(0, stringNumber.length() - 4);

  return stringNumber;
}

/*******************************************************************************
 * Function Name  : setState
 * Description    : sets the state of the system and publishes the change
 * Return         : none
 *******************************************************************************/
void setState(String newState)
{
  state = newState;
  Particle.publish(APP_NAME, "FSM entering " + newState + " state", PRIVATE);
}

/*******************************************************************************
 * Function Name  : turnOnRelay
 * Description    : turns on the specified relay
 * Return         : none
 *******************************************************************************/
void turnOnRelay(int relay)
{
  relayController.turnOnRelay(relay);
  Particle.publish(APP_NAME, "Turning on relay " + String(relay), PRIVATE);
}

/*******************************************************************************
 * Function Name  : turnOffRelay
 * Description    : turns off the specified relay
 * Return         : none
 *******************************************************************************/
void turnOffRelay(int relay)
{
  relayController.turnOffRelay(relay);
  Particle.publish(APP_NAME, "Turning off relay " + String(relay), PRIVATE);
}

/*******************************************************************************/
/*******************************************************************************/
/*******************          EEPROM FUNCTIONS         *************************/
/********  https://docs.particle.io/reference/firmware/photon/#eeprom         **/
/********                                                                     **/
/********  wear and tear discussion:                                          **/
/********  https://community.particle.io/t/eeprom-flash-wear-and-tear/23738/5 **/
/*******************************************************************************/
/*******************************************************************************/

/*******************************************************************************
 * Function Name  : readFromEeprom
 * Description    : retrieves the settings from the EEPROM memory
 * Return         : none
 *******************************************************************************/
void readFromEeprom()
{

  EepromMemoryStructure myObj;
  EEPROM.get(EEPROM_ADDRESS, myObj);

  //verify this eeprom was written before
  // if version is 255 it means the eeprom was never written in the first place, hence the
  // data just read with the previous EEPROM.get() is invalid and we will ignore it
  if (myObj.version == EEPROM_VERSION)
  {

    temperatureTarget = myObj.temperatureTarget;
    temperatureCalibration = myObj.temperatureCalibration;

    // Particle.publish(APP_NAME, "Read settings from EEPROM");
  }
}

/*******************************************************************************
 * Function Name  : saveSettingsInEeprom
 * Description    : This function saves interesting data to EEPROM
 * Return         : none
 *******************************************************************************/
void saveSettingsInEeprom()
{

  //store variables in the struct type that will be saved in the eeprom
  eepromMemory.version = EEPROM_VERSION;
  eepromMemory.temperatureTarget = temperatureTarget;
  eepromMemory.temperatureCalibration = temperatureCalibration;

  //then save
  EEPROM.put(EEPROM_ADDRESS, eepromMemory);

  // Particle.publish(APP_NAME, "Stored settings on EEPROM");
}
