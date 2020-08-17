/*
Name:		INA219B.cpp

Description : INA219B TI i2c output current / voltage / power monitor Library

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

#include "INA219B.h"
#include <Wire.h>

/*
 *  Instantiates a new INA219 class
 *  @param addr the I2C address the device can be found on. Default is 0x40
 */
INA219B::INA219B(INA219_I2C_ADDRESS i2caddr)
{
	ina219_i2caddr = i2caddr;
}

bool INA219B::begin()
{
	Wire.begin();
	return checkDevice();
}

bool INA219B::checkDevice()
{
	Wire.beginTransmission(ina219_i2caddr);
	if (Wire.endTransmission() == 0) {
		return true;
	}
	return false;
}

void INA219B::reset()
{
	writeRegister(INA219_CONFIG_REG, INA219_RESET);
}

// Read the Overflow flag from the register
bool INA219B::getOverflow() {
	uint16_t val;
	val = readRegister(INA219_BUS_REG);
	bool ovf = (val & 1);
	return ovf;
}


// Set the max bus range voltage
void INA219B::setBusRange(INA219_BUSVRANGE range)
{
	uint16_t currentConfReg = readRegister(INA219_CONFIG_REG);
	currentConfReg &= ~(INA219_CONFIG_BVOLTAGERANGE_MASK);
	currentConfReg |= range;
	writeRegister(INA219_CONFIG_REG, currentConfReg);
}

// Define the gain and range of operation
void INA219B::setPGAGain(INA219_PGA_GAIN gain) {
	uint16_t currentConfReg = readRegister(INA219_CONFIG_REG);
	currentConfReg &= ~(INA219_PGA_GAIN_MASK);
	currentConfReg |= gain;
	writeRegister(INA219_CONFIG_REG, currentConfReg);
}

// Set the operation mode continuous or triggered, shunt volatge, bus voltage or both
void INA219B::setOperationMode(INA219_OPERATION_MODE mode)
{
	uint16_t currentConfReg = readRegister(INA219_CONFIG_REG);
	currentConfReg &= ~(INA219_CONFIG_MODE_MASK);
	currentConfReg |= mode;
	writeRegister(INA219_CONFIG_REG, currentConfReg);
}

void INA219B::startSingleMeasurement()
{
	setOperationMode(SANDBVOLT_TRIGGERED);
	uint16_t convertionReady = 0x0000;
	while (!convertionReady)
	{
		convertionReady = ((readRegister(INA219_BUS_REG)) & 0x0002); // checks convertion ready bit
	}
}

// Set the calibration value for the current and power register
void INA219B::setCalibration(uint16_t calibrationValue)
{
	calibrationRegister = calibrationValue;
	writeRegister(INA219_CALIBRATION_REG, calibrationValue);
}

// Set the resolution or the number of samples averaged for the bus register
void INA219B::setBusADCRes(INA219_BUS_ADC_RES resolution)
{
	uint16_t currentConfReg = readRegister(INA219_CONFIG_REG);
	currentConfReg &= ~(INA219_BADCRES_MASK);
	currentConfReg |= resolution;
	writeRegister(INA219_CONFIG_REG, currentConfReg);
}

// Set the resolution or the number of samples averaged for the shunt register
void INA219B::setShuntADCRes(INA219_SHUNT_ADC_RES resolution)
{
	uint16_t currentConfReg = readRegister(INA219_CONFIG_REG);
	currentConfReg &= ~(INA219_CONFIG_SADCRES_MASK);
	currentConfReg |= resolution;
	writeRegister(INA219_CONFIG_REG, currentConfReg);
}

// Read the Bus Voltage from the register and return the converted value in volt, LSB = 4mV
float INA219B::getBusVoltage_V()
{
	uint16_t currentRegValue = readRegister(INA219_BUS_REG);
	// Shift to the right 3 to drop CNVR (convertion ready bit) and OVF (Overflow bit) and multiply by LSB
	currentRegValue = (currentRegValue >> 3);
	return (currentRegValue * 0.004);
}

// Read the Shunt Voltage from the register and return the converted value in volt, LSB = 0.01mV
float INA219B::getShuntVoltage_mV()
{
	int16_t currentRegValue = readRegister(INA219_SHUNT_REG);
	return (currentRegValue * 0.01);
}

// Read the Current from the register and return the converted value in mA, LSB is calculated
float INA219B::getCurrent_mA()
{
	if (calibrationRegister)
	{
		// risk of device reset during sharp load, re-apply the calibration by precaution
		writeRegister(INA219_CALIBRATION_REG, calibrationRegister);
		uint16_t currentRegValue = readRegister(INA219_CURRENT_REG);
		if (currentRegValue != 0)
		{
			if (RShunt <= 0) return 0.0;
			return (0.04096 / (calibrationRegister * RShunt)) * currentRegValue * 1000;
		}
	}
	return 0.0;
}

// Read the Current from the register and return the converted value in mA, LSB is calculated
float INA219B::getPower_mW()
{
	if (calibrationRegister)
	{
		// risk of device reset during sharp load, re-apply the calibration by precaution
		writeRegister(INA219_CALIBRATION_REG, calibrationRegister);

		uint16_t currentRegValue = readRegister(INA219_POWER_REG);
		if (currentRegValue != 0)
		{
			if (RShunt <= 0) return 0.0;
			// Power LSB which is 20 times the Current LSB for a power value in watts
			return (0.04096 / (calibrationRegister * RShunt)) * 20 * currentRegValue * 1000;
		}
	}
}

// Read the Current from the register and return the converted value in mA, LSB is calculated
float INA219B::getCurrent_microA()
{
	if (calibrationRegister)
	{
		// risk of device reset during sharp load, re-apply the calibration by precaution
		writeRegister(INA219_CALIBRATION_REG, calibrationRegister);

		int16_t currentRegValue = readRegister(INA219_CURRENT_REG);
		if (currentRegValue != 0)
		{
			if (RShunt <= 0) return 0.0;
			return (0.04096 / (calibrationRegister * RShunt)) * currentRegValue * 1000000;
		}
	}
	return 0.0;
}

// *** Private function ***
void INA219B::writeRegister(uint8_t reg, uint16_t val)
{
	Wire.beginTransmission(ina219_i2caddr);
	uint8_t lVal = val & 255;
	uint8_t hVal = val >> 8;
	Wire.write(reg);
	Wire.write(hVal);
	Wire.write(lVal);
	Wire.endTransmission();
}

uint16_t INA219B::readRegister(uint8_t reg)
{
	uint8_t MSByte = 0, LSByte = 0;
	uint16_t regValue = 0;
	Wire.beginTransmission(ina219_i2caddr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(ina219_i2caddr, 2);
	if (Wire.available())
	{
		MSByte = Wire.read();
		LSByte = Wire.read();
	}
	regValue = (MSByte << 8) + LSByte;
	return regValue;
}


