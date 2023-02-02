#include <TMC2130Stepper.h>
#include "FastAccelStepper.h"
#include <SPI.h>

#define EN_PIN           17 // Enable
#define DIR_PIN          21 // Direction
#define STEP_PIN         22 // Step
#define CS_PIN           15  // Chip select
#define MOSI          13 // Master Out Slave In (MOSI)
#define MISO          12 // Master In Slave Out (MISO)
#define SCK           14 // Slave Clock (SCK)

#define R_SENSE 0.11f // Match to your driver
                     // SilentStepStick series use 0.11
                     // UltiMachine Einsy and Archim2 boards use 0.2
                     // Panucatt BSD2660 uses 0.1
                     // Watterott TMC5160 uses 0.075

#define POT_PIN           34
#define DEADZONE 20


TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI, MISO, SCK);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

int normalized_analog_value = 0;

void setup() {
    SPI.begin();
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Step1");
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    
    driver.begin();             // Initiate pins and registeries
    //driver.rms_current(200);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driver.setCurrent(600, 0.11, 0.3);
    driver.stealthChop(0);      // Enable extremely quiet stepping
    driver.microsteps(16);
    driver.high_speed_mode(0);
   

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(EN_PIN);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    // stepper->setSpeedInHz(3000);  // the parameter is us/step !!!
    stepper->setAcceleration(1000);
    //stepper->move(10000);
  }

}

void steppersetter(int mappedVal ){
  digitalWrite(EN_PIN, LOW);
  driver.stop_enable(0);
  stepper->setSpeedInHz(mappedVal);
}

void loop() {
  normalized_analog_value = analogRead(POT_PIN) - 512;
  if(abs(normalized_analog_value)-DEADZONE < 0){
    // Serial.println(normalized_analog_value);
    // Serial.println("In Dead Zone");
    driver.stop_enable(1);
    digitalWrite(EN_PIN, HIGH);
  }
    else {
    if(normalized_analog_value > 0){
//      Serial.print("Running forward at ");
      int mappedVal = map(abs(normalized_analog_value),DEADZONE,512,10,32767);
//      Serial.println(mappedVal);
      steppersetter(mappedVal);
      stepper->runForward();
    }
    if(normalized_analog_value < 0){
//      Serial.print("Running backward at ");
      int mappedVal = map(abs(normalized_analog_value),DEADZONE,512,10,32767);
//      Serial.println(mappedVal);
      steppersetter(mappedVal);
      stepper->runBackward();
    }
  }

}