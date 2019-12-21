#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// DFPlayer Mini (MP3 player)
HardwareSerial mySoftwareSerial(1);
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  //DFPlayer Mini
  mySoftwareSerial.begin(9600, SERIAL_8N1, 16, 17);
  myDFPlayer.begin(mySoftwareSerial);
  myDFPlayer.volume(30);
  myDFPlayer.play(1);
  delay(10);
}
void loop() {
  if (myDFPlayer.available()) {
    //myDFPlayer.next();   
  }
}
