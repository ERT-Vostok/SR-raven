#include <Adafruit_BMP280.h>

#define INIT_SAMPLES 200
#define TRIGGER_PIN 10 // CHANGE THAT
#define LED_PIN 13
#define BURN_TIME 2000 //ms
#define TRIGGER_ALTITUDE 1 //m //TEST VALUES
#define TRIGGER_THRESHOLD 0.5 //m
Adafruit_BMP280 bmp;

// Holds the value of the pressure for the zero altitude ( in hPa )
float initPressure, alt;
// Tell if we passed once the trigger altitude
boolean hasPassedTriggerAlt;
// Tell if we already triggered
boolean hasTriggered;

void blink(int time){
  digitalWrite(LED_PIN, HIGH);
  delay(time);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Serial ready, starting BMP...");
  blink(500);
  

  if(!bmp.begin(0x76)){
    Serial.println("Couldn't find the BMP, check wiring.");
    while(1) delay(1000);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  initPressure = 0;
  getInitPressure();
  hasPassedTriggerAlt = false;
  alt = 0;

  Serial.print("BMP ready. Altitude 0 pressure : ");
  Serial.print(initPressure); Serial.println(" hPa.");
}

void loop() { 
  alt = bmp.readAltitude(initPressure);

  //Serial.println(alt);
  delay(1);
  
  if((alt > TRIGGER_ALTITUDE + TRIGGER_THRESHOLD) && !hasPassedTriggerAlt && !hasTriggered){
    hasPassedTriggerAlt = true;
    Serial.println("Threshold reached");
  } else if((alt <= TRIGGER_ALTITUDE) && hasPassedTriggerAlt && !hasTriggered) {
    Serial.print("2nd event trigger now! Altitude : "); Serial.println(alt);
    trigger();
    blink(1000);
    hasTriggered = true;
  } else if(hasTriggered){
    while(1) delay(1000); // lock in infinite loop when done triggering
  }
  
}

void getInitPressure(){
  initPressure = 0;
  for(int i = 0; i<INIT_SAMPLES; i++){
    initPressure = initPressure + (bmp.readPressure() / 100); // bmp return the pressure in Pa and we want the initPressure in hPa
  }

  initPressure = initPressure / INIT_SAMPLES;
}

void trigger(){
  digitalWrite(TRIGGER_PIN, HIGH);
  delay(BURN_TIME);
  digitalWrite(TRIGGER_PIN, LOW);
}
