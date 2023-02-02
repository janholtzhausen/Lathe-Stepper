#include <TMC2130Stepper.h>
#include "FastAccelStepper.h"

#define EN_PIN          17  
#define DIR_PIN         21  
#define STEP_PIN        22  
#define CS_PIN          15  
#define MOSI            13   
#define MISO            12   
#define SCK             14   
#define R_SENSE         0.11f 
#define POT_PIN         34
#define HOLDRATIO       0.2f
#define STARTCURRENT    400
#define MAXSPEED        30000


TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI, MISO, SCK);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

int normalized_analog_value = 0;

void setup() {
    SPI.begin();
    Serial.begin(115200);
    while(!Serial);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    
    driver.begin();    
    driver.setCurrent(STARTCURRENT, R_SENSE, HOLDRATIO);
    driver.microsteps(16);
    
   

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(EN_PIN);
    stepper->setAutoEnable(true);
    stepper->setDelayToEnable(50);
    stepper->setAcceleration(1000);
  }

}

void steppersetter(int mappedVal ){
  int range = map(mappedVal, 0, 4095, 0, 6);
  switch (range) {
    case 0:  
      driver.setCurrent(200, R_SENSE, 0.5);
      driver.high_speed_mode(0);
      driver.stealthChop(1);
    case 1:  
      driver.setCurrent(400, R_SENSE, 0.4);
      driver.high_speed_mode(0);
      driver.stealthChop(1);
    case 2:  
      driver.setCurrent(600, R_SENSE, 0.3);
      driver.high_speed_mode(0);
      driver.stealthChop(0);
    case 3:  
      driver.setCurrent(800, R_SENSE, 0.3);
      driver.high_speed_mode(0);
      driver.stealthChop(0);
    case 4:  
      driver.setCurrent(1000, R_SENSE, 0.3);
      driver.high_speed_mode(1);
      driver.stealthChop(0);
    case 5:  
      driver.setCurrent(1200, R_SENSE, 0.2);
      driver.high_speed_mode(1);
      driver.stealthChop(0);
    case 6:  
      driver.setCurrent(1400, R_SENSE, 0.1);
      driver.high_speed_mode(1);
      driver.stealthChop(0);
  }
  digitalWrite(EN_PIN, LOW);
  driver.stop_enable(0);
  stepper->setSpeedInHz(mappedVal);
}

void loop() {
  int mappedVal = map(analogRead(POT_PIN),0,4095,0,MAXSPEED);
  steppersetter(mappedVal);
  stepper->runForward();
}