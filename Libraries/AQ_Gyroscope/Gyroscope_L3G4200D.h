/*  AeroQuad v3.0.1 - June 2012
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


  Implemented by Lars Karlslund
*/


#ifndef _AEROQUAD_GYROSCOPE_L3G3200D_H_
#define _AEROQUAD_GYROSCOPE_L3G3200D_H_


#include <Gyroscope.h>
#include <SensorsStatus.h>
#include <Wire.h>

#define GYRO_CALIBRATION_TRESHOLD 8

#define GYRO_ADDRESS 105


#define GYRO_RATE 2000
//#define GYRO_RATE 500


#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_CTRL_REG4 0x23
#define GYRO_CTRL_REG5 0x24

// Axis inversion: -1 = invert, 1 = don't invert
int gyroAxisInversionFactor[3] = {1,-1,-1};

/*
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
}*/

void initializeGyro() {
  // Found at http://bildr.org/2011/06/l3g4200d-arduino/
  //vehicleState |= GYRO_DETECTED;
  sendByteI2C(GYRO_ADDRESS, 0x0f);
  if (readByteI2C(GYRO_ADDRESS) == 0b11010011) {
    vehicleState |= GYRO_DETECTED;
  }

  // Enable x, y, z and turn off power down:
  //writeRegister(GYRO_ADDRESS, CTRL_REG1, 0b00001111);
	updateRegisterI2C(GYRO_ADDRESS, GYRO_CTRL_REG1, 0b10011111);
	delay(5);
  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  //writeRegister(GYRO_ADDRESS, GYRO_CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  //writeRegister(GYRO_ADDRESS, GYRO_CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(GYRO_RATE == 250){
    updateRegisterI2C(GYRO_ADDRESS, GYRO_CTRL_REG4, 0b10000000);
    //gyroScaleFactor = radians(1.0 / 10);
	gyroScaleFactor = radians(0.00875);
  }
  else if(GYRO_RATE == 500)
  {
    updateRegisterI2C(GYRO_ADDRESS, GYRO_CTRL_REG4, 0b10010000);
    //gyroScaleFactor = radians(1.0 / 17.5);
	gyroScaleFactor = radians(0.0175);
  }else
  {
    writeRegister(GYRO_ADDRESS, CTRL_REG4, 0b00110000);
    //gyroScaleFactor = radians(-1.0 / 20);
	gyroScaleFactor = radians(-0.07);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  //writeRegister(GYRO_ADDRESS, CTRL_REG5, 0b00000010);
  
	delay(5);
  // High pass filter enabled 
  updateRegisterI2C(GYRO_ADDRESS, GYRO_CTRL_REG5, 0b00000010);

  delay(10); 

}
  


// Read raw values from sensor (inverted if required)
void readGyroRaw(int *gyroRaw) 
{
    sendByteI2C(GYRO_ADDRESS, 0x80 | 0x28); // 0x80 autoincrement from 0x28 register
    Wire.requestFrom(GYRO_ADDRESS,6);
    
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
        gyroRaw[axis] = gyroAxisInversionFactor[axis] * readReverseShortI2C();
    }
	
/*
    //byte msbRegisters[3] = { 0x29, 0x2B, 0x2D };
    //byte lsbRegisters[3] = { 0x28, 0x2A, 0x2C };
	
	byte msbRegisters[3] = { 0x2B, 0x29, 0x2D };
    byte lsbRegisters[3] = { 0x2A, 0x28, 0x2C };

    //read the x, y, and z values from the IMU's registers
    for (byte axis = XAXIS, lsb=0, msb=0; axis <= ZAXIS; axis++) 
    {
      msb = gyroAxisInversionFactor[axis] * readRegister(105, msbRegisters[axis] );
      lsb = gyroAxisInversionFactor[axis] * readRegister(105, lsbRegisters[axis] );
      gyroRaw[axis] = ((msb << 8) | lsb);
    }
	*/
}

void measureGyro() {
    int gyroRaw[3];
    readGyroRaw(gyroRaw);
    
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      gyroRate[axis] = (gyroRaw[axis] - gyroZero[axis]) * gyroScaleFactor;
  }
  
  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}


void measureGyroSum() {
    int gyroRaw[3];
    readGyroRaw(gyroRaw);
    
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
        gyroSample[axis] += gyroRaw[axis];
    }
    
    gyroSampleCount++;
}


void evaluateGyroRate() {
    for (byte axis = 0; axis <= ZAXIS; axis++) {
        gyroRate[axis] = ((gyroSample[axis] / gyroSampleCount) - gyroZero[axis]) * gyroScaleFactor;
        gyroSample[axis] = 0;
    }
    
    gyroSampleCount = 0;
    
    // Measure gyro heading
    long int currentTime = micros();
    if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
        gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
    }
    gyroLastMesuredTime = currentTime;
}


boolean calibrateGyro() {

  int gyro_address = 0XD2;
  int findZero[FINDZERO];
  int diff = 0; 
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      sendByteI2C(gyro_address, 0x80 | (0x28+axis*2));
      Wire.requestFrom(gyro_address,2);
      findZero[i] = readReverseShortI2C();
      delay(10);
    }
    int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
  if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
    gyroZero[axis] = tmp;
  } 
  else {
    return false; //Calibration failed.
  }
  }
  return true;
}


#endif