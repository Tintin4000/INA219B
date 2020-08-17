/*
Name:		ContinuousMode.ino

Description: Example usage of the INA219B library using the continuous mode measurement.

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

#include <INA219B.h>

INA219B ina219b;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);

	if (ina219b.begin()) Serial.println("I2C connected");

	ina219b.reset();
	ina219b.setPGAGain(GAIN_1_40MV);
	ina219b.setBusRange(RANGE_16V);
	ina219b.setCalibration(32768);
	ina219b.setOperationMode(SANDBVOLT_CONTINUOUS);
	ina219b.setBusADCRes(BUS_RES_8S);
	ina219b.setShuntADCRes(SHUNT_RES_8S);
}

// the loop function runs over and over again until power down or reset
void loop() {
	Serial.printf("VBus Voltage = %fV\n", ina219b.getBusVoltage_V());
	Serial.printf("Shunt Voltage = %fmV\n", ina219b.getShuntVoltage_mV());
	Serial.printf("Current = %4.3fmA\n", ina219b.getCurrent_mA());
	Serial.printf("Current = %6.0fmicroA\n", ina219b.getCurrent_microA());
	Serial.printf("Power = %4.3fmW\n", ina219b.getPower_mW());

	ina219b.setOperationMode(POWERDOWN);
	delay(5000);
	ina219b.setOperationMode(SANDBVOLT_CONTINUOUS);
	delay(100);
}
