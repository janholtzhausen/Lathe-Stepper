#include <TMC2130Stepper.h>
#include "FastAccelStepper.h"


#define ONBOARD_LED     2
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
#define RUNSTART_BTN    3
#define RUNSTOP_BTN     19
#define JOGR_BTN        23
#define JOGL_BTN        5

#define HOLDRATIO       0.2f
#define STARTCURRENT    400
#define MINCURRENT      200
#define MAXCURRENT      1000
#define SPEEDSTEPS      20
#define MAXSPEED        20000

TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, MOSI, MISO, SCK);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;


int32_t startA = 0;
int32_t endZ = 0;
bool zTrigger = false;
bool aTrigger = false;
bool runStart = false;
bool runStop = false;
bool jogLeft = false;
bool jogRight = false;

unsigned long button_time = 0;  
unsigned long last_button_time = 0; 
unsigned long previousMillis = 0;  
const long interval = 1000; 


void IRAM_ATTR leftJog()
{
   button_time = millis();
   if (button_time - last_button_time > 250)
      { jogLeft = true;
         last_button_time = button_time;
      }
}
void IRAM_ATTR rightJog()
{
   button_time = millis();
   if (button_time - last_button_time > 250)
      { jogRight = true;
         last_button_time = button_time;
      }
}

void IRAM_ATTR bounceStart()
{
   button_time = millis();
   if (button_time - last_button_time > 250)
      { runStart = true;
         last_button_time = button_time;
      }
}
void IRAM_ATTR bounceStop()
{
   button_time = millis();
   if (button_time - last_button_time > 250)
      { runStop = true;
         last_button_time = button_time;
      }
}

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

   pinMode(A_PIN, INPUT_PULLUP);
   pinMode(Z_PIN, INPUT_PULLUP);
   pinMode(RUNSTART_BTN, INPUT_PULLUP);
   pinMode(RUNSTOP_BTN, INPUT_PULLUP);
   pinMode(JOGR_BTN, INPUT_PULLUP);
   pinMode(JOGL_BTN, INPUT_PULLUP);




    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    pinMode(ONBOARD_LED, OUTPUT);
    
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
    
    stepper->setAcceleration(32000);
  }

   pinMode(JOGR_BTN, INPUT_PULLUP);
   attachInterrupt(JOGR_BTN, rightJog, FALLING);

   pinMode(JOGL_BTN, INPUT_PULLUP);
   attachInterrupt(JOGL_BTN, leftJog, FALLING);

   pinMode(A_PIN, INPUT_PULLUP);
   attachInterrupt(A_PIN, setStartA, FALLING);

   pinMode(Z_PIN, INPUT_PULLUP);
   attachInterrupt(Z_PIN, setEndZ, FALLING);


   pinMode(RUNSTART_BTN, INPUT_PULLUP);
   attachInterrupt(RUNSTART_BTN, bounceStart, FALLING);

   pinMode(RUNSTOP_BTN, INPUT_PULLUP);
   attachInterrupt(RUNSTOP_BTN, bounceStop, FALLING);



   adcAttachPin(POT_PIN);
}

void loop() {

   int potValue = map(analogRead(POT_PIN),0,4095,0,SPEEDSTEPS);
   int mappedSpd = map(potValue,0,SPEEDSTEPS,0,MAXSPEED);
   int mappedCur = map(potValue,0,SPEEDSTEPS,MINCURRENT,MAXCURRENT);
   stepper->setSpeedInHz(mappedSpd); 
   driver.setCurrent(mappedCur, R_SENSE, HOLDRATIO);


   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
   }

/* Serial.print("Stepper Speed ===> ");
Serial.print(mappedSpd);
Serial.print("  Stepper Current ===> ");
Serial.print(mappedCur);
Serial.print("  startA ===> ");
Serial.print(startA);
Serial.print("  endZ ===> ");
Serial.print(endZ);
Serial.print("  Position ===> ");
Serial.println(stepper->getCurrentPosition());
*/
if (jogLeft) {
   Serial.println("JogLeft");
   stepper->move(1000);
   jogLeft = false;
   }
if (jogRight) {
   Serial.println("JogRight");
   stepper->move(-1000);
   jogRight = false;
   } 

if (runStart) {
   stepper->runForward();
   runStart = false;
   }
if (runStop) {
   startA = false;
   endZ = false;
   stepper->forceStopAndNewPosition(0);
   runStop = false;
   }



if (zTrigger) {
   endZ = stepper->getCurrentPosition();
   zTrigger = false;
   }
if (aTrigger) {
   startA = stepper->getCurrentPosition();
   aTrigger = false;
   }

   if (endZ != 0 && startA == 0) {   
      stepper->runBackward();
   }

   if (endZ != 0 && startA != 0) {   
      digitalWrite(ONBOARD_LED, HIGH);
      stepper->moveTo(endZ, true);
         if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
         }
      stepper->moveTo(startA, true);
   }
 
  }