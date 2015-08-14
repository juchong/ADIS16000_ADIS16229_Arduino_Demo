#include <ADIS16000.h>
#include <SPI.h>

ADIS16000 VIBE(7,2,4); //CS,DR,RST
String inputString = "";
String addSensor = "";
boolean sensorFound = false;
int sensid = 0;
int streamSensor = 99;
boolean validSensor[6] = {0,0,0,0,0,0};
boolean reScan = false;

void setup()
{
  Serial.begin(115200); // Initialize serial output via USB
  VIBE.configSPI(); // Configure SPI communication to IMU  
  Serial.println("Start scan? [y/Y]");
  flushReceive();
  VIBE.regWrite(REC_CTRL1, 0x00);
}

void loop()
{
  // Wait for user input...
  if(reScan == false){
    while(Serial.available() == 0) {
    }
    inputString = Serial.readString();
  }
  
  if(inputString == "y" || inputString == "Y" || reScan == true) {
    //Serial.println(inputString);
    inputString == "";
    reScan = false;
    if(VIBE.readProdID() == 0x3E80) {
      Serial.println("Found ADIS16000!");
      VIBE.setDataReady();
    }
    else {
      Serial.println("ERORR: Failed to connect to ADIS16000");
      Serial.println("Returned data may be invalid");
    }
    Serial.println("Scanning for sensors...");
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
      Serial.println("Add a sensor? [y/n]");
      flushReceive();
      while(Serial.available() == 0) {
      }
      addSensor = Serial.readString();
      if(addSensor == "y" || addSensor == "Y"){
        Serial.println("What ID should be assigned to the new sensor? [1 - 6]");
        while(Serial.available() == 0) {
        }
        int newID = Serial.parseInt();
        if(newID == 1 || newID == 2 || newID == 3 || newID == 4 || newID == 5 || newID == 6) {
          int addStatus = VIBE.addSensor(newID);
          if(addStatus == 1){
            Serial.println("Sensor was added successfully!");
            Serial.println("Scan for sensors again? [y/Y]");
            sensorFound == true;
          }
          else {
            Serial.println("ERROR: The sensor was not added successfully");
            Serial.println("Scan for sensors again? [y/Y]");
          }
        }
        else {
          Serial.println("ERROR: The ID entered is not valid. Try again.");
        }
      }
      else {
        reScan = true;
      }
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
          VIBE.regWrite(PAGE_ID, streamSensor);
          VIBE.regWrite(REC_CTRL1, 0x1102);
          VIBE.regWrite(GLOB_CMD_G, 0x02);
          
          uint16_t bufferxy[2][256];
          VIBE.requestFFTData(streamSensor);
          int dataWritten = VIBE.readFFTBuffer(streamSensor, bufferxy);
          if (dataWritten == 1) {
            Serial.println("X: ");            
            for(int i = 0; i < 256; i++) {
              //Serial.printf("%3X", intData);
              Serial.print(bufferxy[0][i]);
              Serial.print(",");
            }
            
            Serial.println(" ");
            Serial.println("Y: ");            
            for(int i = 0; i < 256; i++) {
              //Serial.printf("%3X", intData);
              Serial.print(bufferxy[1][i]);
              Serial.print(",");
            }
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
  



