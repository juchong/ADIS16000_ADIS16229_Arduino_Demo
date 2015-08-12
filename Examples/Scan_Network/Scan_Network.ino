#include <ADIS16000.h>
#include <SPI.h>

ADIS16000 VIBE(7,2,4); //CS,DR,RST
String inputString = "";
boolean stringComplete = false;
int sensid = 0;

void setup()
{
  Serial.begin(115200); // Initialize serial output via USB
  VIBE.configSPI(); // Configure SPI communication to IMU  
  inputString.reserve(200);
  Serial.println("Start scan? (y/n): ");
}

void loop()
{
  if(stringComplete){
    if(inputString == "y\n" || inputString == "Y\n"){
      Serial.print(inputString);
      if(VIBE.readProdID() == 0x3E80) {
        Serial.println("Found ADIS16000!");
        VIBE.setDataReady();
      }
      else {
        Serial.println("ERORR: Failed to connect to ADIS16000");
        Serial.println("Returned data may be invalid");
      }
      Serial.println("Scanning...");
      for(int i = 1; i < 7; i++){
        sensid = VIBE.pollSensor(i);
        if(sensid == 1){
          Serial.print("FOUND A SENSOR ON ID ");
          Serial.print(i);
          Serial.println(" ");
        }
        else {
          Serial.print("No sensor found on ID ");
          Serial.print(i);
          Serial.println(" ");
        }
      }
    }
    inputString = "";
    stringComplete = false;
    Serial.println(" ");
    Serial.println("Start scan? (y/n): ");
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}


