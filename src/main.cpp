#include <Homie.h>
#include <EEPROM.h>
#include "Valve.h"
#include "Pump.h"
#include <ArduinoJson.h>
#include <HX711_ADC.h>

const int PIN_VALVE_IN_PRIMARY = 27;
const int PIN_VALVE_IN_SECONDARY = 25;
const int PIN_VALVE_OUT_FLUSH = 12;
const int PIN_VALVE_OUT_DRAIN = 32;
const int PIN_VALVE_OUT_WATER = 4;

const int PIN_PUMP = 2;

const unsigned int PIN_FLOWMETER = 17;
const unsigned int PIN_WATERSENSOR = 16;
const unsigned int PIN_LEVELSWITCH = 21;

const int PIN_HX711_DT = 35;
const int PIN_HX711_SCK = 14;
float last_known_weight = 0.0;

// How often send to mqtt in seconds
const int PUPLISHING_INTERVAL = 1;
unsigned long lastPublished = 0;

// How often save to eeprom in seconds
const int SAVING_INTERVAL = 20;
unsigned long lastSaved = 0;

volatile long flow_pulse_counter = 0;
long flow_pulses = 0;
int pulsesPerMl = 7; // 1000/7055
int MAX_TIME_WITHOUT_FLOW = 10;


enum podactions {REFILL, DRAIN, FLUSH, WATER, IDLE, CLEAN};
enum podactions POD_ACTION = IDLE;


int RUN_ACTION_FOR_SECONDS = 0;
int RUN_ACTION_FOR_PULSES = 0;
unsigned long ACTION_RUN_START_AT = 0;
long ACTION_RUN_PULSES_START_AT = 0;


HomieNode valvesNode("valves", "Valves", "valve");
HomieNode flowNode("flow", "Flow", "sensor");
HomieNode pumpNode("pump", "Pump", "pump");
HomieNode sensorNode("sensors", "Sensors", "sensor");
HomieNode scaleNode("scale", "Scale", "scale");

HomieNode actions("actions", "Actions", "action");

Valve inPrimary(&PIN_VALVE_IN_PRIMARY, "in-primary", "Inlet primary", &valvesNode);
Valve inSecondary(&PIN_VALVE_IN_SECONDARY, "in-secondary", "Inlet secondary", &valvesNode);
Valve outDrain(&PIN_VALVE_OUT_DRAIN, "out-drain", "Outlet drain", &valvesNode);
Valve outFlush(&PIN_VALVE_OUT_FLUSH, "out-flush", "Outlet flush", &valvesNode);
Valve outWater(&PIN_VALVE_OUT_WATER, "out-water", "Outlet water", &valvesNode);

Pump pump(&PIN_PUMP, &pumpNode);
HX711_ADC LoadCell(PIN_HX711_DT, PIN_HX711_SCK);

bool scaleResetHandler(const HomieRange& range, const String& value);
bool flowResetHandler(const HomieRange& range, const String& value);
bool refillHandler(const HomieRange& range, const String& value);
bool drainHandler(const HomieRange& range, const String& value);
bool cleanHandler(const HomieRange& range, const String& value);
bool waterHandler(const HomieRange& range, const String& value);
void refill(int maxMl, int maxSeconds);
void drain(int maxMl, int maxSeconds);
void clean(int maxMl, int maxSeconds);
void water(int maxMl, int maxSeconds);
void idle();
bool levelReached();
bool waterDetected();
long getFlowCounter();
void saveFlowCounter(long value);
float getWeight();
void saveWeight(float value);


void loopHandler() {

  // Pull data from LoadCell 
  static boolean newDataReady = 0;
  float weight = 0.0;
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    weight = LoadCell.getData();
    weight += last_known_weight;
  }
  
  if (POD_ACTION != IDLE) {
      
      // To Do
      // Always check if some water flows... Otherwise stop
      if (POD_ACTION == REFILL || POD_ACTION == CLEAN) {
          
          if(levelReached()) {
            actions.setProperty("events").send("Stopped action due to level reached event.");
            idle();
            POD_ACTION = IDLE;
            return;
          }

      }

      // Max time reached
      // Valid for all actions
      if (millis() - ACTION_RUN_START_AT >= RUN_ACTION_FOR_SECONDS * 1000UL) {
          actions.setProperty("events").send("Stopped action due to time reached event.");
          idle();
          POD_ACTION = IDLE;
          return;
      }

      // Max Flow reached
      // Valid for all actions
      if (abs(flow_pulse_counter - ACTION_RUN_PULSES_START_AT) >= RUN_ACTION_FOR_PULSES) {
          actions.setProperty("events").send("Stopped action due to max ml reached event.");
          idle();
          POD_ACTION = IDLE;
          return;
      }

  }

  if (millis() - lastPublished >= PUPLISHING_INTERVAL * 1000UL || lastPublished == 0) {
    inPrimary.publish();
    inSecondary.publish();
    outDrain.publish();
    outFlush.publish();
    outWater.publish();
    pump.publish();
    sensorNode.setProperty("level-switch").send(String(levelReached()));
    sensorNode.setProperty("water-detector").send(String(waterDetected()));
  
    flowNode.setProperty("pulses").send(String(flow_pulse_counter));
    flowNode.setProperty("volume").send(String(flow_pulse_counter/pulsesPerMl));
    
    scaleNode.setProperty("weight").send(String(weight));

    lastPublished = millis();

  }

  if (millis() - lastSaved >= SAVING_INTERVAL * 1000UL || lastSaved == 0) {

    saveWeight(weight);
    saveFlowCounter(flow_pulse_counter);
    lastSaved = millis();
  }



}


IRAM_ATTR void flow () {
  if(POD_ACTION == REFILL) {
    flow_pulse_counter++;
  } else {
    flow_pulse_counter--;
  }
}


void setup() {

  Serial.begin(74880);
  Serial << endl << endl;

  Homie_setFirmware("pod-controller", "1.0.0");
  Homie.setLoopFunction(loopHandler);
  Homie.setLedPin(22, HIGH);

  pinMode(PIN_FLOWMETER, INPUT_PULLUP);
  pinMode(PIN_WATERSENSOR, INPUT);
  pinMode(PIN_LEVELSWITCH, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOWMETER), flow, RISING);


  LoadCell.begin();
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(25.95); // set calibration factor (float)
    Serial.println("HX711 startup is complete");
  }

  last_known_weight = getWeight();
  Serial.println("Loaded weight from eeprom: " + String(last_known_weight));
  if (isnan(last_known_weight)) {
    last_known_weight = 0.0;
  }

  long pulses = getFlowCounter();
  Serial.println("Loaded pulses from eeprom: " + String(pulses));
  if(pulses != -1) {
    flow_pulse_counter = pulses;
  }

  actions.advertise("events")
                    .setName("Events")
                    .setDatatype("string");
                    
  actions.advertise("refill")
                    .setName("Refill")
                    .setDatatype("string")
                    .settable(refillHandler);

  actions.advertise("drain")
                  .setName("Drain")
                  .setDatatype("string")
                  .settable(drainHandler);

  actions.advertise("clean")
                  .setName("Clean")
                  .setDatatype("string")
                  .settable(cleanHandler);

  actions.advertise("water")
                .setName("Water")
                .setDatatype("string")
                .settable(waterHandler);

  flowNode.advertise("reset")
                  .setName("Reset")
                  .setDatatype("boolean")
                  .settable(flowResetHandler);

  scaleNode.advertise("reset")
                  .setName("Reset")
                  .setDatatype("boolean")
                  .settable(scaleResetHandler);

  Homie.setup(); 



}

void loop() {
  Homie.loop();
}

bool levelReached() {
  return !digitalRead(PIN_LEVELSWITCH);
}

bool waterDetected() {
  return !digitalRead(PIN_WATERSENSOR);
}

void refill(int maxMl, int maxSeconds) {

  // In Secondary -> open
  // Out Flush    -> open
  pump.run(true);
  inSecondary.open(true);
  outFlush.open(true);

  POD_ACTION = REFILL;
  ACTION_RUN_START_AT = millis();
  RUN_ACTION_FOR_SECONDS = maxSeconds;
  RUN_ACTION_FOR_PULSES = maxMl * pulsesPerMl;
  ACTION_RUN_PULSES_START_AT = flow_pulse_counter;

  actions.setProperty("events").send("Start action REFILL. max ml: " + String(RUN_ACTION_FOR_PULSES/pulsesPerMl) + " max time in seconds: " + String(RUN_ACTION_FOR_SECONDS));

}

void clean(int maxMl, int maxSeconds) {
  // Just for development
  return;
  // In Secondary   -> open
  // Out Drain      -> open
  pump.run(true);
  inSecondary.open(true);
  outDrain.open(true);

  POD_ACTION = CLEAN;
  ACTION_RUN_START_AT = millis();
  RUN_ACTION_FOR_SECONDS = maxSeconds;
  RUN_ACTION_FOR_PULSES = maxMl * pulsesPerMl;
  ACTION_RUN_PULSES_START_AT = flow_pulse_counter;

  actions.setProperty("events").send("Start action CLEAN. max ml: " + String(RUN_ACTION_FOR_PULSES/pulsesPerMl) + " max time in seconds: " + String(RUN_ACTION_FOR_SECONDS));

}

void drain(int maxMl, int maxSeconds) {

  // In Primary   -> open
  // Out Drain    -> open
  pump.run(true);
  inPrimary.open(true);
  outDrain.open(true);

  POD_ACTION = DRAIN;
  ACTION_RUN_START_AT = millis();
  RUN_ACTION_FOR_SECONDS = maxSeconds;
  RUN_ACTION_FOR_PULSES = maxMl * pulsesPerMl;
  ACTION_RUN_PULSES_START_AT = flow_pulse_counter;

  actions.setProperty("events").send("Start action DRAIN. max ml: " + String(RUN_ACTION_FOR_PULSES/pulsesPerMl) + " max time in seconds: " + String(RUN_ACTION_FOR_SECONDS));
}

void water(int maxMl, int maxSeconds) {

  // In Primary   -> open
  // Out Drain    -> open
  pump.run(true);
  inPrimary.open(true);
  outWater.open(true);

  POD_ACTION = DRAIN;
  ACTION_RUN_START_AT = millis();
  RUN_ACTION_FOR_SECONDS = maxSeconds;
  RUN_ACTION_FOR_PULSES = maxMl * pulsesPerMl;
  ACTION_RUN_PULSES_START_AT = flow_pulse_counter;

  actions.setProperty("events").send("Start action WATER. max ml: " + String(RUN_ACTION_FOR_PULSES/pulsesPerMl) + " max time in seconds: " + String(RUN_ACTION_FOR_SECONDS));
}

void idle() {

  pump.run(false);
  inPrimary.open(false);
  inSecondary.open(false);
  outFlush.open(false);
  outDrain.open(false);
  outWater.open(false);

  POD_ACTION = IDLE;
  ACTION_RUN_START_AT = 0;
  RUN_ACTION_FOR_SECONDS = 0;
  ACTION_RUN_PULSES_START_AT = 0;
  RUN_ACTION_FOR_PULSES = 0;

  saveFlowCounter(flow_pulse_counter);

}

long getFlowCounter() {
  long pulses = 0;
  EEPROM.begin(512);
  EEPROM.get(0, pulses);
  EEPROM.end();
  return pulses;
}

void saveFlowCounter(long value) {
  EEPROM.begin(512);
  EEPROM.put(0, value);
  EEPROM.commit();
  EEPROM.end();
}

float getWeight() {
  float weight = 0;
  EEPROM.begin(512);
  EEPROM.get(4, weight);
  EEPROM.end();
  return weight; 
}

void saveWeight(float value) {
  EEPROM.begin(512);
  EEPROM.put(4, value);
  EEPROM.commit();
  EEPROM.end();
}



bool refillHandler(const HomieRange& range, const String& value) {

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, value);

  Serial.println("Refill request");
  Serial.println(serializeJson(doc, Serial));

  int maxMl  = doc["maxMl"];
  int maxSeconds  = doc["maxSeconds"];

  refill(maxMl, maxSeconds);

  return true;

}

bool drainHandler(const HomieRange& range, const String& value) {

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, value);

  Serial.println("Drain request");
  Serial.println(serializeJson(doc, Serial));

  int maxMl  = doc["maxMl"];
  int maxSeconds  = doc["maxSeconds"];

  drain(maxMl, maxSeconds);

  return true;

}

bool cleanHandler(const HomieRange& range, const String& value) {

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, value);

  Serial.println("Clean request");
  Serial.println(serializeJson(doc, Serial));

  int maxMl  = doc["maxMl"];
  int maxSeconds  = doc["maxSeconds"];

  clean(maxMl, maxSeconds);

  return true;

}

bool waterHandler(const HomieRange& range, const String& value) {

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, value);

  Serial.println("Water request");
  Serial.println(serializeJson(doc, Serial));

  int maxMl  = doc["maxMl"];
  int maxSeconds  = doc["maxSeconds"];

  water(maxMl, maxSeconds);

  return true;

}

bool flowResetHandler(const HomieRange& range, const String& value) {

  if (value != "true") return false;
  flow_pulse_counter = 0;
  saveFlowCounter(flow_pulse_counter);
  return true;

}

bool scaleResetHandler(const HomieRange& range, const String& value) {
  if (value != "true") return false;
  last_known_weight = 0.0;
  saveWeight(last_known_weight);
  return true;
}