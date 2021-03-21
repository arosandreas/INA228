/*
    INA228 - An arduino library for the INA228 current sensor
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY
 */

#include <Wire.h>
#include <SPI.h>
#include "INA228.h"

/*
INA228::INA228()
{
    address = INA228_ADDRESS;
}
*/


INA228::INA228(int value)
{
    address = value;
    I2C = true;
}


/*
INA228::INA228(int CS)
{
    _CS = CS;
    I2C = false;
    SPI.begin();
    pinMode(_CS, OUTPUT);
    digitalWrite(_CS, HIGH);
}
*/


float INA228::_convert2comp2float(uint64_t twocompdata, uint16_t nrofbit, float factor)
{
    uint64_t isnegative = 1;
    isnegative = (isnegative << (nrofbit - 1));
    //Serial.print(F("isnegative="));
    //Serial.println(isnegative, HEX);
    float dietemp = twocompdata;
    if (dietemp > isnegative)
    {
        dietemp=(dietemp-(2*isnegative))*factor;
    }
    else
    {
        dietemp = (dietemp * factor);
        //Serial.println(F("2comp är positiv:"));
        //Serial.println(dietemp,2);
    }
    return dietemp;
}

uint64_t INA228::_register40(byte reg)
{
    uint64_t regdata = 0;
    uint64_t highByte2 = 0;
    uint64_t highByte = 0;
    uint64_t midByte = 0;
    uint64_t lowByte = 0;
    if (I2C)
    {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(address, 4);
        highByte2 = Wire.read();
        highByte = Wire.read();
        midByte = Wire.read();
        lowByte = Wire.read();
        regdata = regdata | lowByte | (midByte << 8) | (highByte << 16) | (highByte2 << 32);
    }
    else
    {
        reg = (reg << 2);
        reg |= 0x01;
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        highByte2 = SPI.transfer(0);
        highByte = SPI.transfer(0);
        midByte = SPI.transfer(0);
        lowByte = SPI.transfer(0);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
        regdata = regdata | lowByte | (midByte << 8) | (highByte << 16) | (highByte2 << 32);
    }
    return regdata;
}

uint32_t INA228::_register24(byte reg)
{
    uint32_t regdata = 0;
    uint32_t highByte = 0;
    uint32_t midByte = 0;
    uint32_t lowByte = 0;
    if (I2C)
    {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(address, 3);
        highByte = Wire.read();
        midByte = Wire.read();
        lowByte = Wire.read();
        regdata = regdata | lowByte | (midByte << 8) | (highByte << 16);

    }
    else
    {
        reg = (reg << 2);
        reg |= 0x01;
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        highByte = SPI.transfer(0);
        midByte = SPI.transfer(0);
        lowByte = SPI.transfer(0);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
        regdata = regdata |lowByte | (midByte << 8) | (highByte<<16);
    
    }
    return regdata;
}

uint16_t INA228::_register16(byte reg)
{
    uint16_t regdata = 0;
    if (I2C)
    {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(address, 2);
        regdata = (Wire.read() << 8) | Wire.read();
    }
    else{
        //int highByte = 0;
        //int highLow = 0;
        reg= (reg<<2);
        reg|=0x01;
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        regdata = SPI.transfer16(0);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
    }
    return regdata;
}

void INA228::_register16(byte reg, uint16_t regdata)
{

    if(I2C){
        byte msb = (byte)(regdata >> 8);
        byte lsb = (byte)(regdata & 0xFF);

        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(msb);
        Wire.write(lsb);
        Wire.endTransmission();
    }
    else  {
        reg = (reg << 2);
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        SPI.transfer16(regdata);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
    }
}


uint16_t INA228::_register8(byte reg)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(address, 1);
    return Wire.read();
}

void INA228::_register8(byte reg, byte regdata)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(regdata);
    Wire.endTransmission();
}

uint32_t INA228::read24(byte regi)
{
    return (_register24(regi));
}

uint16_t INA228::read16(byte regi)
{
    return (_register16(regi));
}

void INA228::write16(byte regtemp, uint16_t temp)
{
    _register16(regtemp, temp);
}


//CONFIG 0x0
boolean INA228::setADCRANGE(uint8_t value)
{
    INA228_Reg.reg0.all = _register16(CONFIG_REG);
    INA228_Reg.reg0.bit.ADCRANGE=value;
    _register16(CONFIG_REG, INA228_Reg.reg0.all);
    return (1);
}

uint8_t INA228::getADCRANGE(void)
{
    INA228_Reg.reg0.all = _register16(CONFIG_REG);
    return (INA228_Reg.reg0.bit.ADCRANGE);
}

boolean INA228::setCONVDLY(uint8_t value) //
{
    INA228_Reg.reg0.all = _register16(CONFIG_REG);
    INA228_Reg.reg0.bit.CONVDLY = value;
    _register16(CONFIG_REG, INA228_Reg.reg0.all);
    return (1);
}

boolean INA228::setRST(uint8_t value)         //
{
    INA228_Reg.reg0.all = _register16(CONFIG_REG);
    INA228_Reg.reg0.bit.RST = value;
    _register16(CONFIG_REG, INA228_Reg.reg0.all);
    return (1);
}

//ADCCONFIG 0x1
boolean INA228::setAVG(uint8_t value) //
{
    INA228_Reg.reg1.all = _register16(ADCCONFIG_REG);
    INA228_Reg.reg1.bit.AVG = value;
    _register16(ADCCONFIG_REG, INA228_Reg.reg1.all);
    return (1);
}

boolean INA228::setVTCT(uint8_t value) //
{
    INA228_Reg.reg1.all = _register16(ADCCONFIG_REG);
    INA228_Reg.reg1.bit.VTCT = value;
    _register16(ADCCONFIG_REG, INA228_Reg.reg1.all);
    return (1);
}

boolean INA228::setVSHCT(uint8_t value) //
{
    INA228_Reg.reg1.all = _register16(ADCCONFIG_REG);
    INA228_Reg.reg1.bit.VSHCT = value;
    _register16(ADCCONFIG_REG, INA228_Reg.reg1.all);
    return (1);
}

boolean INA228::setVBUSCT(uint8_t value) //
{
    INA228_Reg.reg1.all = _register16(ADCCONFIG_REG);
    INA228_Reg.reg1.bit.VBUSCT = value;
    _register16(ADCCONFIG_REG, INA228_Reg.reg1.all);
    return (1);
}

boolean INA228::setMODE(uint8_t value) //
{
    INA228_Reg.reg1.all = _register16(ADCCONFIG_REG);
    INA228_Reg.reg1.bit.MODE = value;
    _register16(ADCCONFIG_REG, INA228_Reg.reg1.all);
    return (1);
}

// SHUNT_CAL 0x2
boolean INA228::setCURRLSB(uint16_t value)
{
    INA228_Reg.reg2.all = _register16(SHUNT_CAL_REG);
    INA228_Reg.reg2.bit.CURRLSB = value;
    _register16(SHUNT_CAL_REG, INA228_Reg.reg2.all);
    return (1);
}

// VSHUNT 0x4
boolean INA228::setVSHUNT(uint16_t value)
{
    INA228_Reg.reg4.all = _register16(VSHUNT_REG);
    INA228_Reg.reg4.bit.VSHUNT = value;
    _register16(VSHUNT_REG, INA228_Reg.reg4.all);
    return (1);
}

uint32_t INA228::getVSHUNT(void)
{
    INA228_Reg.reg4.all = _register24(VSHUNT_REG);
    return (INA228_Reg.reg4.bit.VSHUNT);
}

float INA228::getVSHUNT_f(void)
{
    return (_convert2comp2float(getVSHUNT(), 20, 312.5e-9));
}

// VBUS 0x5
uint32_t INA228::getVBUS(void)
{
    INA228_Reg.reg5.all = _register24(VBUS_REG);
    return (INA228_Reg.reg5.bit.VBUS);
}

float INA228::getVBUS_f(void)
{

    return (_convert2comp2float(getVBUS(), 20, 195.3e-6));
}

// DIETEMP 0x6
uint16_t INA228::getDIETEMP(void)
{
    INA228_Reg.reg6.all = _register16(DIETEMP_REG);
    return (INA228_Reg.reg6.bit.DIETEMP);
}

float INA228::getDIETEMP_f(void)
{

    return (_convert2comp2float(getDIETEMP(), 16, 7.8125e-3));
}

// CURRENT 0x7
uint32_t INA228::getCURRENT(void)
{
    INA228_Reg.reg7.all = _register24(CURRENT_REG);
    return (INA228_Reg.reg7.bit.CURRENT);
}

float INA228::getCURRENT_f(void)
{

    return (_convert2comp2float(getCURRENT(), 20, current_lsb));
}

// POWER 0x8
float INA228::getPOWER(void)
{
    INA228_Reg.reg8.all = _register24(POWER_REG);
    return (INA228_Reg.reg8.bit.POWER * current_lsb * 3.2);
}

// ENERGY 0x9
float INA228::getENERGY(void)
{
    INA228_Reg.reg9.all = _register40(ENERGY_REG);
    return (INA228_Reg.reg9.bit.ENERGY * current_lsb * 3.2 * 16);
}

// CHARGE 0xa
uint64_t INA228::getCHARGE(void)
{
    INA228_Reg.rega.all = _register40(CHARGE_REG);
    return (INA228_Reg.rega.bit.CHARGE);
}

// DIAG_ALRT 0xb


boolean INA228::getMEMSTAT()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.MEMSTAT);
}

boolean INA228::getCNVRF()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.CNVRF);
}

boolean INA228::getPOL()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.POL);
}

boolean INA228::getBUSUL()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.BUSUL);
}

boolean INA228::getBUSOL()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.BUSOL);
}

boolean INA228::getSHNTUL()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.SHNTUL);
}

boolean INA228::getSHNTOL()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.SHNTOL);
}

boolean INA228::getTMPOL()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.TMPOL);
}

boolean INA228::getMATHOF()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.MATHOF);
}

boolean INA228::getCHARGEOF()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.CHARGEOF);
}

boolean INA228::getENERGYOF()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.ENERGYOF);
}

boolean INA228::getAPOL()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.APOL);
}

boolean INA228::getSLOWALERT()
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA228_Reg.regb.bit.SLOWALERT);
}

boolean INA228::setCNVR(byte value)
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    INA228_Reg.regb.bit.CNVR = value;
    _register16(DIAG_ALRT_REG, INA228_Reg.regb.all);
    return (1);
}

boolean INA228::setALATCH(byte value)
{
    INA228_Reg.regb.all = _register16(DIAG_ALRT_REG);
    INA228_Reg.regb.bit.ALATCH = value;
    _register16(DIAG_ALRT_REG, INA228_Reg.regb.all);
    return (1);
}

// SOVL 0xc
boolean INA228::setSOVL(uint16_t value)
{
    INA228_Reg.regc.all = _register16(SOVL_REG);
    INA228_Reg.regc.bit.SOVL = value;
    _register16(SOVL_REG, INA228_Reg.regc.all);
    return (1);
}

boolean INA228::setSOVL_f(float value)
{
    uint16_t data;
    if (value >= 0)
    {
        data = (value * shunt_res) / (shunt_conv_factor);
    }
    else{
        float value_temp;
        value_temp = value*(-1);
        data = (value_temp * shunt_res) / (shunt_conv_factor);
        data = ~data;
        data+=1;
    }
    setSOVL(data);
    return (1);
}

// SUVL 0xd
boolean INA228::setSUVL(uint16_t value)
{
    INA228_Reg.regd.all = _register16(SUVL_REG);
    INA228_Reg.regd.bit.SUVL = value;
    _register16(SUVL_REG, INA228_Reg.regd.all);
    return (1);
}

boolean INA228::setSUVL_f(float value)
{
    uint16_t data;
    if (value >= 0)
    {
        data = (value * shunt_res) / (shunt_conv_factor);
    }
    else
    {
        float value_temp;
        value_temp = value * (-1);
        data = (value_temp * shunt_res) / (shunt_conv_factor);
        data = ~data;
        data += 1;
    }
    setSUVL(data);
    return (1);
}

// BOVL 0xe
boolean INA228::setBOVL(uint16_t value)
{
    INA228_Reg.rege.all = _register16(BOVL_REG);
    INA228_Reg.rege.bit.BOVL = value;
    _register16(BOVL_REG, INA228_Reg.rege.all);
    return (1);
}

boolean INA228::setBOVL_f(float value)
{
    uint16_t data = value / (16 * 195.3125e-6);
    setBOVL(data);
    return (1);
}

// BUVL 0xf
boolean INA228::setBUVL(uint16_t value)
{
    INA228_Reg.regf.all = _register16(BUVL_REG);
    INA228_Reg.regf.bit.BUVL = value;
    _register16(BUVL_REG, INA228_Reg.regf.all);
    return (1);
}

boolean INA228::setBUVL_f(float value)
{
    uint16_t data = value / (16 * 195.3125e-6);
    setBUVL(data);
    return (1);
}

// TEMP_LIMIT 0x10
boolean INA228::setTOL(uint16_t value)
{
    INA228_Reg.reg10.all = _register16(TEMP_LIMIT_REG);
    INA228_Reg.reg10.bit.TOL = value;
    _register16(TEMP_LIMIT_REG, INA228_Reg.reg10.all);
    return (1);
}

boolean INA228::setTOL_f(float value, float consta)
{
    uint16_t data = value / (16 * consta);
    setTOL(data);
    return (1);
}

// PWR_LIMIT 0x11
boolean INA228::setPOL(uint16_t value)
{
    INA228_Reg.reg11.all = _register16(PWR_LIMIT_REG);
    INA228_Reg.reg11.bit.POL = value;
    _register16(PWR_LIMIT_REG, INA228_Reg.reg11.all);
    return (1);
}

boolean INA228::setPOL_f(float value, float consta)
{
    uint16_t data = value / (16 * consta);
    setPOL(data);
    return (1);
}

// MANUFACTURER_ID 0x3e
uint16_t INA228::getMANFID(void)
{
    INA228_Reg.reg3e.all = _register16(MANUFACTURER_ID_REG);
    return (INA228_Reg.reg3e.bit.MANFID);
}

// DEVICE_ID 0x3f
uint16_t INA228::getREV_ID(void)
{
    INA228_Reg.reg3f.all = _register16(DEVICE_ID_REG);
    return (INA228_Reg.reg3f.bit.REV_ID);
}

uint16_t INA228::getDIEID(void)
{
    INA228_Reg.reg3f.all = _register16(DEVICE_ID_REG);
    return (INA228_Reg.reg3f.bit.DIEID);
}

void INA228::InitINA(float shunt)
{
    shunt_res=shunt;
    float temp=0;
    if (getADCRANGE()){
        temp = 40.96e-3;
        shunt_conv_factor=1.25e-6;
    }
    else
    {
        temp = 163.84e-3;
        shunt_conv_factor = 5.0e-6;
    }
    current_lsb = (temp / shunt) / 524288;
    Serial.print(F("current_lsb:"));
    Serial.println(current_lsb, 10);
    uint16_t shunt_cal = (13107.2e6 * current_lsb * shunt_res);
    Serial.print(F("shunt_cal:"));
    Serial.println(shunt_cal);
    setCURRLSB(shunt_cal);
}
