////////////////////////////////////////////////////////////////////////////////////////////////////////
//  August 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16000.h
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

#include "Arduino.h"
#include <SPI.h>

// Uncomment for DEBUG mode
//#define DEBUG

// User Register Memory Map from Table 10 for PAGE_ID = 0x0000 (ADIS16000)
#define PAGE_ID 		0x00
#define NETWORK_ID	 	0x02
#define FLASH_CNT_G 	0x04
#define NW_ERROR_STAT	0x06
#define TX_PWR_CTRL_G	0x08
#define RSSI_G			0x0A
#define TEMP_OUT_G		0x0C
#define SUPPLY_OUT_G	0x0E
#define TEST_MODE		0x10
#define GLOB_CMD_G		0x12
#define CMD_DATA		0x14
#define PROD_ID_G		0x16
#define LOT_ID1_G		0x1A
#define LOT_ID2_G		0x1C
#define SERIAL_NUM_G	0x24
#define GPO_CTRL		0x2A
#define DIAG_STAT_G		0x2C

// User Register Memory Map from Table 11 for PAGE_ID >= 0x0001 (ADIS16229)
#define SENS_ID			0x02
#define FLASH_CNT_S		0x04
#define X_BUF			0x06
#define Y_BUF			0x08
#define TEMP_OUT_S		0x0A
#define SUPPLY_OUT_S	0x0C
#define FFT_AVG1		0x0E
#define FFT_AVG2		0x10
#define BUF_PNTR		0x12
#define REC_PNTR		0x14
#define X_SENS			0x16
#define Y_SENS			0x18
#define REC_CTRL1		0x1A
#define REC_CTRL2		0x1C
#define ALM_F_LOW		0x20
#define ALM_F_HIGH		0x22
#define ALM_X_MAG1		0x24
#define ALM_Y_MAG1		0x26
#define ALM_X_MAG2		0x28
#define ALM_Y_MAG2		0x2A
#define ALM_PNTR		0x2C
#define ALM_S_MAG		0x2E
#define ALM_CTRL		0x30
#define AVG_CNT			0x32
#define DIAG_STAT_S		0x34
#define GLOB_CMD_S		0x36
#define ALM_X_STAT		0x38
#define ALM_Y_STAT		0x3A
#define ALM_X_PEAK		0x3C
#define ALM_Y_PEAK		0x3E
#define TIME_STMP_L		0x40
#define TIME_STMP_H		0x42
#define ALM_X_FREQ		0x44
#define ALM_Y_FREQ		0x46
#define PROD_ID_S		0x48
#define REC_FLSH_CNT	0x4A
#define REC_INFO1		0x4C
#define REC_INFO2		0x4E
#define REC_CNTR		0x50
#define PKT_TIME_L		0x52
#define PKT_TIME_H		0x54
#define PKT_ERROR_STAT	0x56
#define TX_PWR_CTRL_S	0x58
#define RSSI_S			0x5A
#define RF_MODE			0x5C
#define UPDAT_INT		0x5E
#define INT_SCL			0x60
#define USER_SCR		0x64
#define LOT_ID1_S		0x68
#define LOT_ID2_S		0x6A

// ADIS16000/ADIS16229 Class Definition
class ADIS16000{

public:
  typedef int16_t *buffer[256][2];

	// ADIS16000 Constructor (ChipSelect, Reset Pins)
	ADIS16000(int CS, int DR, int RST);

	// Destructor
	~ADIS16000();

	// Performs hardware reset. Delay in milliseconds. Returns 1 when complete.
	int hardwareResetDUT(uint8_t ms);

  // Performs software reset.
  int softwareResetDUT();

	// Sets SPI bit order, clock divider, and data mode. Returns 1 when complete.
	int configSPI();

	// Read register (two bytes) Returns signed 16 bit data.
 	int16_t regRead(uint8_t regAddr);

	// Write register (two bytes). Returns 1 when complete.
	int regWrite(uint8_t regAddr, int16_t regData);

  // Read the ADIS16000 product ID.
  int16_t readProdID();

  // Performs a test to determine whether the sensor written in sensorData is present.
  int testSensor(uint8_t sensorData);

	// Add sensor to network. Returns 1 when complete.
	int addSensor(uint8_t sensorAddr);

	// Remove sensor from network. Returns 1 when complete.
	int removeSensor(uint8_t sensorAddr);

	// Save configuration settings for the gateway. Returns 1 when complete.
	int saveGatewaySettings();

	// Save configuration settings for the selected sensor. Returns 1 when complete.
	int saveSensorSettings(uint8_t sensorAddr);

  // Polls a single sensor on the network and returns whether it is active.
  int pollSensor(uint8_t sensorAddr);

  int initRealTimeSampling();

  int stopRealTimeSampling();

  // Requests FFT data from the specified sensor. 
  int requestFFTData(uint8_t sensorAddr);

	// Reads FFT data from the output buffers and loads them into a buffer allocated on the host MCU.
	int readFFTBuffer(uint8_t sensorAddr, uint16_t bufferxy[][256]);

	// Configures Data Ready functionality.
	int setDataReady();

  int setPeriodicMode(uint16_t interval, uint8_t scalefactor, uint8_t sensorAddr);

	// Scales single time sample. Returns acceleration in mg.
	float scaleTime(int16_t sensorData, int gRange);

	// Scales single FFT sample. Returns acceleration in mg.
	float scaleFFT(int16_t sensorData, int gRange);

	// Scales supply voltage. Returns voltage in mV.
	float scaleSupply(int16_t sensorData);

	// Scales sensor temperature. Returns temperature in C.
	float scaleTemp(int16_t sensorData);

private:
	int _CS;
	int _RST;
  int _DR;

};
