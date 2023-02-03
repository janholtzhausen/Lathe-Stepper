#include <TMC2130Stepper.h>
#include "FastAccelStepper.h"

#define EN_PIN          23  
#define DIR_PIN         21  
#define STEP_PIN        22  
#define CS_PIN          15  
#define MOSI            13   
#define MISO            12   
#define SCK             14   
#define R_SENSE         0.11f 
#define POT_PIN         34
#define A_PIN           33
#define Z_PIN           32

#define HOLDRATIO       0.2f
#define STARTCURRENT    200
#define MINCURRENT      300
#define MEDCURRENT      600
#define HIGHCURRENT     800
#define MAXCURRENT      1000
#define MAXSPEED        20000

TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI, MISO, SCK);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

int32_t startA = 0;
int32_t endZ = 0;
bool zTrigger = false;
bool aTrigger = false;

unsigned long button_time = 0;  
unsigned long last_button_time = 0; 
unsigned long previousMillis = 0;  
const long interval = 1000; 

void IRAM_ATTR setStartA()
{
   button_time = millis();
   if (button_time - last_button_time > 250)
      { aTrigger = true;
         last_button_time = button_time;
      }
}
void IRAM_ATTR setEndZ()
{
   button_time = millis();
   if (button_time - last_button_time > 250)
      { zTrigger = true;
         last_button_time = button_time;
      }
}

void setup() {
    SPI.begin();
    Serial.begin(115200);
    while(!Serial);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    
    driver.begin();    
    driver.high_speed_mode(0);
    driver.microsteps(16);
    driver.setCurrent(STARTCURRENT, R_SENSE, HOLDRATIO);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(EN_PIN);
    stepper->setAutoEnable(true);
    stepper->setDelayToEnable(5000);
    stepper->setDelayToDisable(1000);
    stepper->setSpeedInUs(100); 
    stepper->setAcceleration(15000);
  }

   pinMode(A_PIN, INPUT_PULLUP);
   attachInterrupt(A_PIN, setStartA, FALLING);

   pinMode(Z_PIN, INPUT_PULLUP);
   attachInterrupt(Z_PIN, setEndZ, FALLING);

}

void loop() {
   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
   }
Serial.print("startA ===> ");
Serial.print(startA);
Serial.print("      endZ ===> ");
Serial.println(endZ);
if (zTrigger) {
   endZ = stepper->getCurrentPosition();
   zTrigger = false;
   }
if (aTrigger) {
   // stepper->setCurrentPosition(0);
   startA = stepper->getCurrentPosition();
   aTrigger = false;
   }


   if (endZ == 0) {
      stepper->runForward();
   }
   if (endZ != 0 && startA == 0) {   
      stepper->runBackward();
   }

   if (endZ != 0 && startA != 0) {   

      stepper->moveTo(endZ, true);
         if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
         }
      stepper->moveTo(startA, true);
   }
 
  }