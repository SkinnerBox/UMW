/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_ACCELEROMETER_ADXL345_H_
#define _AEROQUAD_ACCELEROMETER_ADXL345_H_

#include <Accelerometer.h>
#include <SensorsStatus.h>

#define ACCEL_ADDRESS 0x53

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

void initializeAccel() {

  vehicleState |= ACCEL_DETECTED;

  delay(10);
  writeRegister ( ACCEL_ADDRESS, 0x2D, 1<<3 ); //  register: Power CTRL  -- value: Set measure bit 3 on
  writeRegister ( ACCEL_ADDRESS, 0x31, 0x0B ); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  writeRegister ( ACCEL_ADDRESS, 0x2C, 0x09 ); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
}
  
void measureAccel() {

    byte msbRegisters[3] = { 0x33, 0x35, 0x37 };
    byte lsbRegisters[3] = { 0x32, 0x34, 0x36 };

    //read the x, y, and z values from the IMU's registers
    for (byte axis = XAXIS, lsb=0, msb=0; axis <= ZAXIS; axis++) 
    {
      msb = readRegister(ACCEL_ADDRESS, msbRegisters[axis] );
      lsb = readRegister(ACCEL_ADDRESS, lsbRegisters[axis] );
      meterPerSecSec[axis] = ((msb << 8) | lsb);
    }

}

void measureAccelSum() {

    byte msbRegisters[3] = { 0x33, 0x35, 0x37 };
    byte lsbRegisters[3] = { 0x32, 0x34, 0x36 };

    //read the x, y, and z values from the IMU's registers
    for (byte axis = XAXIS, lsb=0, msb=0; axis <= ZAXIS; axis++) 
    {
      msb = readRegister(ACCEL_ADDRESS, msbRegisters[axis] );
      lsb = readRegister(ACCEL_ADDRESS, lsbRegisters[axis] );
      accelSample[axis] += ((msb << 8) | lsb);
    }
    accelSampleCount++;
}

void evaluateMetersPerSec() {
	
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }
  accelSampleCount = 0;		
}

void computeAccelBias() {
  
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = fabs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}



#endif
