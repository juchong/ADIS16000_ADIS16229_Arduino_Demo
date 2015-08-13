#include <ADIS16000.h>
#include <SPI.h>

ADIS16000 VIBE(7,2,4); //CS,DR,RST
String inputString = "";
boolean sensorFound = false;
int sensid = 0;
int streamSensor = 99;
boolean validSensor[6] = {0,0,0,0,0,0};

void setup()
{
  Serial.begin(115200); // Initialize serial output via USB
  VIBE.configSPI(); // Configure SPI communication to IMU  
  Serial.println("Start scan? [y/Y]");
  flushReceive();
}

void loop()
{
  // Wait for user input...
  while(Serial.available() == 0) {
  }
  inputString = Serial.readString();
  
  if(inputString == "y" || inputString == "Y") {
    //Serial.println(inputString);
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
      if(sensid == 1) {
        Serial.print("FOUND A SENSOR ON ID ");
        Serial.print(i);
        Serial.println(" ");
        sensorFound = true;
        validSensor[i] = true; 
      }
      else {
        Serial.print("No sensor found on ID ");
        Serial.print(i);
        Serial.println(" ");
      }
    }
    if (sensorFound == false) {
      Serial.println("No Sensors Found");
      Serial.println("Retry scan? [y/Y]");
      flushReceive();
    }

    Serial.println(" ");
    if(sensorFound == true) {
      Serial.println("Sensors were found.");
    }
    while(sensorFound == true) {
      Serial.println(" ");
      Serial.println("Select sensor to stream from [1 - 6]");
      flushReceive();
      while(Serial.available() == 0) {
      }
      streamSensor = Serial.parseInt();
      if(streamSensor == 1 || streamSensor == 2 || streamSensor == 3 || streamSensor == 4 || streamSensor == 5 || streamSensor == 6) {
        if(validSensor[streamSensor] == true) {
          int16_t * data;
          int16_t intData;
          data = VIBE.readFFTBuffer(streamSensor);
          Serial.println("X: ");
          for(int i = 0; i < 256; i++) {
            intData = data[i];
            Serial.printf("%3X", intData);
            Serial.printf(",");
          }
          Serial.println("Y: ");
          for(int i = 256; i < 512; i++) {
            intData = data[i];
            Serial.printf("%3X", intData);
            Serial.printf(",");
          }
        }
        else
        {
          Serial.println("Invalid input. Try again.");
          flushReceive();
        }
      }      
    }    
  }
}

void flushReceive() {
  while(Serial.available())
  Serial.read();
}
  



