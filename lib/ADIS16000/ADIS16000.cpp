////////////////////////////////////////////////////////////////////////////////////////////////////////
//  August 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16000.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16000 Digital MEMS 
//  Vibration Sensor with Embedded RF Transceiver to an 8-Bit Atmel-based Arduino development board. 
//  Functions for SPI configuration, reads and writes, and scaling are included. 
//
//  This example is free software. You can redistribute it and/or modify it
//  under the terms of the GNU Lesser Public License as published by the Free Software
//  Foundation, either version 3 of the License, or any later version.
//
//  This example is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
//  FOR A PARTICULAR PURPOSE.  See the GNU Lesser Public License for more details.
//
//  You should have received a copy of the GNU Lesser Public License along with 
//  this example.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ADIS16000.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS and RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16000::ADIS16000(int CS, int DR, int RST) {
  _CS = CS;
  _RST = RST;
  _DR = DR;

  SPI.begin(); // Initialize SPI bus
  configSPI(); // Configure SPI

//Set default pin states
  pinMode(_CS, OUTPUT); // Set CS pin to be an output
  pinMode(_RST, OUTPUT); // Set RST pin to be an output
  pinMode(_DR, INPUT); // Set CS pin to be an output
  digitalWrite(_CS, HIGH); // Initialize CS pin to be high
  digitalWrite(_RST, HIGH); // Initialize RST pin to be high
  setDataReady();
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16000::~ADIS16000() {
  // Close SPI bus
  SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting _RST pin low for delay (in ms).
////////////////////////////////////////////////////////////////////////////
int ADIS16000::hardwareResetDUT(uint8_t ms) {
  digitalWrite(_RST, LOW);
  delay(100);
  digitalWrite(_RST, HIGH);
  delay(ms);
  return 1;
}

int ADIS16000::softwareResetDUT() {
  regWrite(GLOB_CMD_G, 0x80);
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16000::configSPI() {
  SPI.setBitOrder(MSBFIRST); // Per the datasheet
  SPI.setClockDivider(SPI_CLOCK_DIV8); // Config for  1MHz (ADIS16448 max 2MHz)
  SPI.setDataMode(SPI_MODE3); // Clock base at one, sampled on falling edge
  return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16000::regRead(uint8_t regAddr) {
//Read registers using SPI
  
  // Write register address to be read
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(regAddr); // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(15); // Delay to not violate read rate (40us)

  // Read data from requested register
  digitalWrite(_CS, LOW); // Set CS low to enable device
  uint8_t _msbData = SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData = SPI.transfer(0x00); // Send (0x00) and place lower byte into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(15); // Delay to not violate read rate (40us)
  
  int16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

  return(_dataOut);
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16000::regWrite(uint8_t regAddr, int16_t regData) {

  // Write register address and data
  uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

  // Split words into chars
  uint8_t highBytehighWord = (highWord >> 8);
  uint8_t lowBytehighWord = (highWord & 0xFF);
  uint8_t highBytelowWord = (lowWord >> 8);
  uint8_t lowBytelowWord = (lowWord & 0xFF);

  // Write highWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
  SPI.transfer(lowBytehighWord); // Write low byte from high word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (40us)

  // Write lowWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
  SPI.transfer(lowBytelowWord); // Write low byte from low word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  return(1);
}

////////////////////////////////////////////////////////////////////////////
// Reads the product ID of the SPI device.
// Returns the product ID in LSBs (Default: 0x3E80)
////////////////////////////////////////////////////////////////////////////

int16_t ADIS16000::readProdID() {
  regWrite(PAGE_ID, 0x00);
  int16_t prodid = regRead(PROD_ID_G);
  return(prodid);
}

////////////////////////////////////////////////////////////////////////////
// Reads whether the sensor requested responds with an ID equal to the one
//  requested.
// Returns 1 if the sensor responds, 0 if it does not.
////////////////////////////////////////////////////////////////////////////

int ADIS16000::testSensor(uint8_t sensorAddr){
  regWrite(PAGE_ID, sensorAddr); // Set page to sensorAddr
  uint16_t sensData = regRead(SENS_ID); // Read the SENS_ID register
  // Debug: Print raw sensor data
  //Serial.print("sensData: ");
  //Serial.printf("%3X",sensData);
  //Serial.println(" ");
  uint16_t HexID = 0xAD00;
  HexID = HexID + sensorAddr; // Mask the sensor address with the HEXID
  if(HexID != sensData) // The result should match the sensor address specified
    return 0;
  else
    return 1;
}

////////////////////////////////////////////////////////////////////////////
// Adds a sensor to the network and assigns the desired sensor address.
// Returns the result of testSensor().
////////////////////////////////////////////////////////////////////////////

int ADIS16000::addSensor(uint8_t sensorAddr) {
  regWrite(PAGE_ID, 0x00); // Set page to 0 (gateway)
	regWrite(GLOB_CMD_G, 0x01); // Add sensor to network command
	regWrite(CMD_DATA, sensorAddr); // Assign sensor ID
  delay(2000);
  int status = testSensor(sensorAddr);
  return status;
}

////////////////////////////////////////////////////////////////////////////
// Removes a sensor from the network and clears its address from the 
//  network table.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////

int ADIS16000::removeSensor(uint8_t sensorAddr) {
  regWrite(PAGE_ID, 0x00); // Set page to 0 (gateway)
	regWrite(CMD_DATA, sensorAddr); // Set CMD_DATA to the desired sensor
	regWrite(GLOB_CMD_G, 0x8000); // Remove the sensor listed in CMD_DATA from the network
	return 1;
}

////////////////////////////////////////////////////////////////////////////
// Saves gateway settings to memory (EEPROM).
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////

int ADIS16000::saveGatewaySettings() {
	regWrite(PAGE_ID, 0x00);
	regWrite(GLOB_CMD_G, 0x40);
	return 1;
}

////////////////////////////////////////////////////////////////////////////
// Saves sensor settings to memory (EEPROM).
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////

int ADIS16000::saveSensorSettings(uint8_t sensorAddr) {
	regWrite(PAGE_ID, sensorAddr);
	regWrite(GLOB_CMD_S, 0x40);
	regWrite(PAGE_ID, 0x00);
	regWrite(GLOB_CMD_G, 0x02);
	return 1;
}

////////////////////////////////////////////////////////////////////////////
// Polls a single sensor on the network and returns whether it is active.
// Returns 1 if the sensor responds, 0 if it does not.
////////////////////////////////////////////////////////////////////////////

int ADIS16000::pollSensor(uint8_t sensorAddr){
  regWrite(PAGE_ID, 0x00);
  regWrite(CMD_DATA, sensorAddr);
  regWrite(GLOB_CMD_G, 0x2000);
  delay(1500);
  int status = testSensor(sensorAddr);
  return status;
}

int ADIS16000::initRealTimeSampling() {
  uint16_t rcWord = regRead(REC_CTRL1);
  uint16_t wWord = rcWord & 0xFFFC;
  wWord = wWord | 0x03;
  regWrite(REC_CTRL1, wWord);
  regWrite(GLOB_CMD_S, 0x800);
  delayMicroseconds(100000);
  regWrite(GLOB_CMD_G, 0x02);
  return 1;
}

int ADIS16000::stopRealTimeSampling() {
  regWrite(GLOB_CMD_S, 0x800);
  return 1;
}

////////////////////////////////////////////////////////////////////////////
// Requests FFT data from the specified sensor. 
// Returns 1 when the sensor finishes acquiring data.
////////////////////////////////////////////////////////////////////////////

int ADIS16000::requestFFTData(uint8_t sensorAddr) {
  regWrite(PAGE_ID, sensorAddr); // Set page to selected sensor registers
  regWrite(GLOB_CMD_S,0x800); // Load "start recording data" command to buffer
  regWrite(PAGE_ID, 0x00); // Set page to gateway registers
  regWrite(CMD_DATA, sensorAddr); // Write sensor to be sent commands
  regWrite(GLOB_CMD_G, 0x02); // Update settings of the sensor in CMD_DATA
  delay(1500); // THIS SHOULD BE INTERRUPT DRIVEN

  return 1;
}

////////////////////////////////////////////////////////////////////////////
// Reads FFT data from the output buffers and loads them into a buffer 
//  allocated on the host MCU.
// Returns 1 when the buffer has been loaded.
// NOTE: The buffer must be declared in main() as: uint16_t bufferxy[2][256];
////////////////////////////////////////////////////////////////////////////

int ADIS16000::readFFTBuffer(uint8_t sensorAddr, uint16_t bufferxy[][256]) {
  regWrite(PAGE_ID, sensorAddr);
  regWrite(BUF_PNTR, 0x00); // Reset buffer pointer
  delay(100);

	for (int i = 0; i < 256; i++) {
		bufferxy[0][i] = regRead(X_BUF);
	}
  delay(500);
  regWrite(PAGE_ID, sensorAddr);
  regWrite(BUF_PNTR, 0x00);
  delay(100);
  Serial.println(" ");

  for (int i = 0; i < 256; i++) {
    bufferxy[1][i] = regRead(Y_BUF);
  }
  /*
  Serial.print("XDEV: ");
  for (int i = 0; i < 256; i++) {
    Serial.print(bufferxy[0][i]);
    Serial.print("-");
  }
  Serial.println(" ");

  Serial.print("YDEV: ");
  for (int i = 0; i < 256; i++) {
    Serial.print(bufferxy[1][i]);
    Serial.print("-");
  }
  Serial.println(" ");
  */
	return 1;
}

////////////////////////////////////////////////////////////////////////////
// Configures Data Ready functionality.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////

int ADIS16000::setDataReady() {
	regWrite(PAGE_ID, 0x00);
	regWrite(GPO_CTRL, 0x08);
  return 1;
}

int ADIS16000::setPeriodicMode(uint16_t interval, uint8_t scalefactor, uint8_t sensorAddr) {
  regWrite(PAGE_ID, sensorAddr);
  regWrite(UPDAT_INT, interval);
  regWrite(INT_SCL, scalefactor);
  regWrite(GLOB_CMD_S, 0x800);
  return 1;
}

float ADIS16000::scaleTime(int16_t sensorData, int gRange) {
  int lsbrange = 0;
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData; // Else return the raw number
  switch (gRange) {
    case 1:
      lsbrange = 0.0305;
      break;
    case 5:
      lsbrange = 0.1526;
      break;
    case 10:
      lsbrange = 0.3052;
      break;
    case 20:
      lsbrange = 0.6104;
      break;
    default:
      lsbrange = 0.0305;
      break;
  }
  float finalData = signedData * lsbrange;
  return finalData;
}

float ADIS16000::scaleFFT(int16_t sensorData, int gRange) {
  int lsbrange = 0;
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData; // Else return the raw number
  switch (gRange) {
    case 1:
      lsbrange = 0.0153;
      break;
    case 5:
      lsbrange = 0.0763;
      break;
    case 10:
      lsbrange = 0.1526;
      break;
    case 20:
      lsbrange = 0.3052;
      break;
    default:
      lsbrange = 0.0153;
      break;
  }
  float finalData = signedData * lsbrange;
  return finalData;
}

float ADIS16000::scaleSupply(int16_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData; // Else return the raw number
  float finalData = signedData * 0.00044; // Multiply by accel sensitivity (250 uG/LSB)
  return finalData;
}

float ADIS16000::scaleTemp(int16_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData; // Else return the raw number
  float finalData = signedData * 0.0815; // Multiply by accel sensitivity (250 uG/LSB)
  return finalData;
}
