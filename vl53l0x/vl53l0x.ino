#include "Adafruit_VL53L0X.h"

#define SHT_LOX1 5

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);
  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  // initing LOX1
  
  if(!lox.begin()) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // power 
  Serial.println(F("VL53L0X booted succesfully...\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
}
