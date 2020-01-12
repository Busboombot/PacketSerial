#include <Arduino.h>
#include <MessageProcessor.h>


SerialDataProcessor sdp(Serial1);

void setup() {
  
  Serial1.begin(115200);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(20, OUTPUT);
}

int loopCounter = 0;
int blinkCount = 0;
bool loopToggle = false;

void loop() {
  // send data only when you receive data:

  if( ++loopCounter > (1<<20) ){
    digitalWriteFast(LED_BUILTIN, (loopToggle = !loopToggle));
    loopCounter = 0;
    blinkCount ++;
    if(blinkCount%10 == 0){
      //Serial.print("Blink "); Serial.println(blinkCount);
    }
  }

  sdp.update();

}


