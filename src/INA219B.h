/*
Name:		INA219B.h

Description: INA219B TI i2c output current/voltage/power monitor Library

Created : 15 August 2020

Author : Didier Coyman

MIT License
Copyright(c) 2020 Didier  Coyman
Permission is hereby granted, free of charge, to any person obtaining a copy
of this softwareand associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :
The above copyright noticeand this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _LIB_INA219B_
#define _LIB_INA219B_

#include "Arduino.h"

/** I2C address **/
typedef enum INA219_I2C_ADDRESS {
	I2C_ADDR_40 = 0x40, // Default i2c address
	I2C_ADDR_41 = 0x41,
	I2C_ADDR_44 = 0x44,
	I2C_ADDR_45 = 0x45
};

/** registers **/
#define INA219_CONFIG_REG 	0x00 // Configuration Register address
#define INA219_SHUNT_REG    0x01 // Shunt Voltage Register address
#define INA219_BUS_REG    	0x02 // Bus Voltage Register address
#define INA219_POWER_REG     	0x03 // Power Register address
#define INA219_CURRENT_REG 	0x04 // Current flowing through Shunt address
#define INA219_CALIBRATION_REG     	0x05 // Calibration Register address

/** register bit configurations **/
#define INA219_RESET   0x8000 // Reset bit

/** mask for bus voltage range **/
#define INA219_CONFIG_BVOLTAGERANGE_MASK 0x2000 // Bus Voltage Range Mask

/** bus voltage range values **/
typedef enum INA219_BUSVRANGE {
	RANGE_16V = 0x0000, // 0-16V Range
	RANGE_32V = 0x2000, // 0-32V Range
};

/** mask for gain bits **/
#define INA219_PGA_GAIN_MASK 0x1800 // Gain Mask

/** values for gain bits **/
typedef enum INA219_PGA_GAIN {
	GAIN_1_40MV = 0x0000,  // Gain 1, 40mV Range
	GAIN_2_80MV = 0x0800,  // Gain 2, 80mV Range
	GAIN_4_160MV = 0x1000, // Gain 4, 160mV Range
	GAIN_8_320MV = 0x1800, // Gain 8, 320mV Range
};

/** mask for bus ADC resolution bits **/
#define INA219_BADCRES_MASK 0x0780

/** values for bus ADC resolution **/
typedef enum INA219_BUS_ADC_RES {
	BUS_RES_9BIT = 0x0000, // 9-bit bus res = 0..511
	BUS_RES_10BIT = 0x0080, // 10-bit bus res = 0..1023
	BUS_RES_11BIT = 0x0100, // 11-bit bus res = 0..2047
	BUS_RES_12BIT = 0x0180, // 12-bit bus res = 0..4097
	BUS_RES_2S = 0x0480, // 2 x 12-bit bus samples averaged together
	BUS_RES_4S = 0x0500, // 4 x 12-bit bus samples averaged together
	BUS_RES_8S = 0x0580, // 8 x 12-bit bus samples averaged together
	BUS_RES_16S = 0x0600, // 16 x 12-bit bus samples averaged together
	BUS_RES_32S = 0x0680, // 32 x 12-bit bus samples averaged together
	BUS_RES_64S = 0x0700, // 64 x 12-bit bus samples averaged together
	BUS_RES_128S = 0x0780, // 128 x 12-bit bus samples averaged together
};

/** mask for shunt ADC resolution bits **/
#define INA219_CONFIG_SADCRES_MASK 0x0078 // Shunt ADC Resolution and Averaging Mask

/** values for shunt ADC resolution **/
typedef enum INA219_SHUNT_ADC_RES{
	SHUNT_RES_9BIT = 0x0000, // 1 x 9-bit shunt sample
	SHUNT_RES_10BIT = 0x0008, // 1 x 10-bit shunt sample
	SHUNT_RES_11BIT = 0x0010, // 1 x 11-bit shunt sample
	SHUNT_RES_12BIT = 0x0018, // 1 x 12-bit shunt sample
	SHUNT_RES_2S = 0x0048, // 2 x 12-bit shunt samples averaged together
	SHUNT_RES_4S = 0x0050, // 4 x 12-bit shunt samples averaged together
	SHUNT_RES_8S = 0x0058, // 8 x 12-bit shunt samples averaged together
	SHUNT_RES_16S = 0x0060, // 16 x 12-bit shunt samples averaged together
	SHUNT_RES_32S = 0x0068, // 32 x 12-bit shunt samples averaged together
	SHUNT_RES_64S = 0x0070, // 64 x 12-bit shunt samples averaged together
	SHUNT_RES_128S = 0x0078, // 128 x 12-bit shunt samples averaged together
};

/** mask for operating mode bits **/
#define INA219_CONFIG_MODE_MASK 0x0007 // Operating Mode Mask

/** values for operating mode **/
typedef enum INA219_OPERATION_MODE{
	POWERDOWN = 0x00, // power down
	SVOLT_TRIGGERED = 0x01, // shunt voltage triggered
	BVOLT_TRIGGERED = 0x02, // bus voltage triggered
	SANDBVOLT_TRIGGERED = 0x03, // shunt and bus voltage triggered
	ADCOFF = 0x04, // ADC off
	SVOLT_CONTINUOUS = 0x05, // shunt voltage continuous
	BVOLT_CONTINUOUS = 0x06, // bus voltage continuous
	SANDBVOLT_CONTINUOUS = 0x07, // shunt and bus voltage continuous
};

/*!
 *   @brief  Class that stores state and functions for interacting with INA219
 *  current/power monitor IC
 */
class INA219B {
public:
	INA219B(INA219_I2C_ADDRESS i2caddr = I2C_ADDR_40);
	bool begin();
	void reset();
	float RShunt = 0.1;
	void setBusADCRes(INA219_BUS_ADC_RES resolution);
	void setShuntADCRes(INA219_SHUNT_ADC_RES resolution);
	void setOperationMode(INA219_OPERATION_MODE mode);
	void setPGAGain(INA219_PGA_GAIN gain);
	void setBusRange(INA219_BUSVRANGE range);
	void setCalibration(uint16_t calibrationValue);
	float getBusVoltage_V();
	float getBusVoltage_V_Raw();
	float getShuntVoltage_mV();
	float getCurrent_mA();
	float getCurrent_microA();
	float getCurrent_mA_Raw();
	float getPower_mW();
	bool getOverflow();
	void startSingleMeasurement();

private:
	int ina219_i2caddr = -1;
	bool checkDevice();
	uint16_t calibrationRegister;

	void writeRegister(uint8_t reg, uint16_t val);
	uint16_t readRegister(uint8_t reg);
};

#endif

