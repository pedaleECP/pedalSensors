/****************************************************************************
* ITG3200_DUE.cpp - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors                  *
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with ITG-3200 Breakout                             *
* SCL     -> pin 21     (no pull up resistors)                              *
* SDA     -> pin 20     (no pull up resistors)                              *
* CLK & GND -> pin GND                                                      *
* INT       -> not connected  (but can be used)                             *
* VIO & VDD -> pin 3.3V                                                     *
*****************************************************************************/

// This library implementation has been adapted to work on Arduino DUE by PedalSensor team 

#include "FIMU_ITG3200_DUE.h"


ITG3200_DUE::ITG3200_DUE() {
  setGains(1.0,1.0,1.0);
  setOffsets(0.0,0.0,0.0);
  setRevPolarity(0,0,0);
  //Wire.begin();       //Normally this code is called from setup() at user code
                        //but some people reported that joining I2C bus earlier
                        //apparently solved problems with master/slave conditions.
                        //Uncomment if needed.
}

void ITG3200_DUE::init(unsigned int  address) {
  // Uncomment or change your default ITG3200_DUE initialization
  
  // fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values)
  init(address, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);
  
  // slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, true, true);
  
  // fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, true, true);
  
  // slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, true, true);
}

void ITG3200_DUE::init(unsigned int address, byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, bool _ITGReady, bool _INTRawDataReady) {
  _dev_address = address;
  setSampleRateDiv(_SRateDiv);
  setFSRange(_Range);
  setFilterBW(_filterBW);
  setClockSource(_ClockSrc);
  setITGReady(_ITGReady);
  setRawDataReady(_INTRawDataReady);  
  delay(GYROSTART_UP_DELAY);  // startup 
}

byte ITG3200_DUE::getDevAddr() {
  /*readmem(WHO_AM_I, 1, &_buff[0]); 
  return _buff[0];  */
  return _dev_address;
}

void ITG3200_DUE::setDevAddr(unsigned int  _addr) {
  writemem(WHO_AM_I, _addr); 
  _dev_address = _addr;
}

byte ITG3200_DUE::getSampleRateDiv() {
  readmem(SMPLRT_DIV, 1, &_buff[0]);
  return _buff[0];
}

void ITG3200_DUE::setSampleRateDiv(byte _SampleRate) {
  writemem(SMPLRT_DIV, _SampleRate);
}

byte ITG3200_DUE::getFSRange() {
  readmem(DLPF_FS, 1, &_buff[0]);
  return ((_buff[0] & DLPFFS_FS_SEL) >> 3);
}

void ITG3200_DUE::setFSRange(byte _Range) {
  readmem(DLPF_FS, 1, &_buff[0]);   
  writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) ); 
}

byte ITG3200_DUE::getFilterBW() {  
  readmem(DLPF_FS, 1, &_buff[0]);
  return (_buff[0] & DLPFFS_DLPF_CFG); 
}

void ITG3200_DUE::setFilterBW(byte _BW) {   
  readmem(DLPF_FS, 1, &_buff[0]);
  writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_DLPF_CFG) | _BW)); 
}

bool ITG3200_DUE::isINTActiveOnLow() {  
  readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ACTL) >> 7);
}

void ITG3200_DUE::setINTLogiclvl(bool _State) {
  readmem(INT_CFG, 1, &_buff[0]);
  writemem(INT_CFG, ((_buff[0] & ~INTCFG_ACTL) | (_State << 7))); 
}

bool ITG3200_DUE::isINTOpenDrain() {  
  readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_OPEN) >> 6);
}

void ITG3200_DUE::setINTDriveType(bool _State) {
  readmem(INT_CFG, 1, &_buff[0]);
  writemem(INT_CFG, ((_buff[0] & ~INTCFG_OPEN) | _State << 6)); 
}

bool ITG3200_DUE::isLatchUntilCleared() {    
  readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}

void ITG3200_DUE::setLatchMode(bool _State) {
  readmem(INT_CFG, 1, &_buff[0]);
  writemem(INT_CFG, ((_buff[0] & ~INTCFG_LATCH_INT_EN) | _State << 5)); 
}

bool ITG3200_DUE::isAnyRegClrMode() {    
  readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void ITG3200_DUE::setLatchClearMode(bool _State) {
  readmem(INT_CFG, 1, &_buff[0]);
  writemem(INT_CFG, ((_buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4)); 
}

bool ITG3200_DUE::isITGReadyOn() {   
  readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}

void ITG3200_DUE::setITGReady(bool _State) {
  readmem(INT_CFG, 1, &_buff[0]);
  writemem(INT_CFG, ((_buff[0] & ~INTCFG_ITG_RDY_EN) | _State << 2)); 
}

bool ITG3200_DUE::isRawDataReadyOn() {
  readmem(INT_CFG, 1, &_buff[0]);
  return (_buff[0] & INTCFG_RAW_RDY_EN);
}

void ITG3200_DUE::setRawDataReady(bool _State) {
  readmem(INT_CFG, 1, &_buff[0]);
  writemem(INT_CFG, ((_buff[0] & ~INTCFG_RAW_RDY_EN) | _State)); 
}

bool ITG3200_DUE::isITGReady() {
  readmem(INT_STATUS, 1, &_buff[0]);
  return ((_buff[0] & INTSTATUS_ITG_RDY) >> 2);
}

bool ITG3200_DUE::isRawDataReady() {
  readmem(INT_STATUS, 1, &_buff[0]);
  return (_buff[0] & INTSTATUS_RAW_DATA_RDY);
}

void ITG3200_DUE::readTemp(float *_Temp) {
  readmem(TEMP_OUT,2,_buff);
  *_Temp = 35 + (((_buff[0] << 8) | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32  
}

void ITG3200_DUE::readGyroRaw(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ){
  readmem(GYRO_XOUT, 6, _buff);
  *_GyroX = ((_buff[0] << 8) | _buff[1]);
  *_GyroY = ((_buff[2] << 8) | _buff[3]); 
  *_GyroZ = ((_buff[4] << 8) | _buff[5]);
}

void ITG3200_DUE::readGyroRaw(int16_t *_GyroXYZ){
  readGyroRaw(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200_DUE::setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol) {
  polarities[0] = _Xpol ? -1 : 1;
  polarities[1] = _Ypol ? -1 : 1;
  polarities[2] = _Zpol ? -1 : 1;
}

void ITG3200_DUE::setGains(float _Xgain, float _Ygain, float _Zgain) {
  gains[0] = _Xgain;
  gains[1] = _Ygain;
  gains[2] = _Zgain;
}

void ITG3200_DUE::setOffsets(int16_t _Xoffset, int16_t _Yoffset, int16_t _Zoffset) {
  offsets[0] = _Xoffset;
  offsets[1] = _Yoffset;
  offsets[2] = _Zoffset;
}

void ITG3200_DUE::zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
  int16_t xyz[3]; 
  float tmpOffsets[] = {0,0,0};

  for (int16_t i = 0;i < totSamples;i++){
    delay(sampleDelayMS);
    readGyroRaw(xyz);
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];  
  }
  setOffsets(-tmpOffsets[0] / totSamples, -tmpOffsets[1] / totSamples, -tmpOffsets[2] / totSamples);
}

void ITG3200_DUE::readGyroRawCal(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ) {
  readGyroRaw(_GyroX, _GyroY, _GyroZ);
  *_GyroX += offsets[0];
  *_GyroY += offsets[1];
  *_GyroZ += offsets[2];
}

void ITG3200_DUE::readGyroRawCal(int16_t *_GyroXYZ) {
  readGyroRawCal(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200_DUE::readGyro(float *_GyroX, float *_GyroY, float *_GyroZ){
  int16_t x, y, z;
  
  readGyroRawCal(&x, &y, &z); // x,y,z will contain calibrated integer values from the sensor
  *_GyroX =  x / 14.375 * polarities[0] * gains[0];
  *_GyroY =  y / 14.375 * polarities[1] * gains[1];
  *_GyroZ =  z / 14.375 * polarities[2] * gains[2];
}

void ITG3200_DUE::readGyro(float *_GyroXYZ){
  readGyro(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200_DUE::reset() {     
  writemem(PWR_MGM, PWRMGM_HRESET); 
  delay(GYROSTART_UP_DELAY); //gyro startup 
}

bool ITG3200_DUE::isLowPower() {   
  readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_SLEEP) >> 6;
}
  
void ITG3200_DUE::setPowerMode(bool _State) {
  readmem(PWR_MGM, 1, &_buff[0]);
  writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_SLEEP) | _State << 6));  
}

bool ITG3200_DUE::isXgyroStandby() {
  readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_XG) >> 5;
}

bool ITG3200_DUE::isYgyroStandby() {
  readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_YG) >> 4;
}

bool ITG3200_DUE::isZgyroStandby() {
  readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_ZG) >> 3;
}

void ITG3200_DUE::setXgyroStandby(bool _Status) {
  readmem(PWR_MGM, 1, &_buff[0]);
  writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_XG) | _Status << 5));
}

void ITG3200_DUE::setYgyroStandby(bool _Status) {
  readmem(PWR_MGM, 1, &_buff[0]);
  writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_YG) | _Status << 4));
}

void ITG3200_DUE::setZgyroStandby(bool _Status) {
  readmem(PWR_MGM, 1, &_buff[0]);
  writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_ZG) | _Status << 3));
}

byte ITG3200_DUE::getClockSource() {  
  readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_CLK_SEL);
}

void ITG3200_DUE::setClockSource(byte _CLKsource) {   
  readmem(PWR_MGM, 1, &_buff[0]);
  writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_CLK_SEL) | _CLKsource)); 
}

void ITG3200_DUE::writemem(uint8_t _addr, uint8_t _val) {
  Wire.beginTransmission(_dev_address);   // start transmission to device 
  Wire.write(_addr); // send register address
  Wire.write(_val); // send value to write
  Wire.endTransmission(); // end transmission
}

void ITG3200_DUE::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
  Wire.beginTransmission(_dev_address); // start transmission to device 
  Wire.write(_addr); // sends register address to read from
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(_dev_address); // start transmission to device 
  Wire.requestFrom(_dev_address, _nbytes);// send data n-bytes read
  uint8_t i = 0; 
  while (Wire.available()) {
    __buff[i] = Wire.read(); // receive DATA
    i++;
  }
  Wire.endTransmission(); // end transmission
}

