//Adaptation of Sparkfun's LIS331 Arduino Driver for Ardupilot

#include <assert.h>
#include <utility>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LIS331.h"

extern const AP_HAL::HAL &hal;

//static HAL_ChibiOS::I2CDevice i2c;

Vector3f raw_accel_400g;

//constructor
AP_InertialSensor_LIS331::AP_InertialSensor_LIS331(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_LIS331::~AP_InertialSensor_LIS331()
{
}

/*
//Detect the Sensor
AP_InertialSensor_Backend *AP_InertialSensor_LIS331::probe(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_LIS331 *sensor = new AP_InertialSensor_LIS331(imu, std::move(dev));
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor
        return nullptr;
    }

    return sensor;
}
/*
bool AP_InertialSensor_LIS331::_init_sensor(void) {
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    //Init the accelerometer
    uint8_t data = 0;
    _dev->read_registers(LIS331_ADDRESS, &data, 1);
    if (data != LIS331_DEVID) {
        AP_HAL::panic("AP_InertialSensor_LIS331: could not find LIS331 accelerometer sensor\n");
    }

    _dev->write_register(LIS331_POWER_CTL, 0x00);
    hal.scheduler->delay(5);
    _dev->write_register(LIS331_POWER_CTL, 0xff);
    hal.scheduler->delay(5);
    //Measure mode:
    _dev->write_register(LIS331_POWER_CTL, 0x08);
    hal.scheduler->delay(5);

    //Full resolution, 8g:
    //In full resolution mode, the scale factor need not change
    _dev->write_register(LIS331_DATA_FORMAT, 0x08);
    hal.scheduler->delay(5);

    //Normal power, 800Hz
*/


/*
AP_InertialSensor_LIS331::AP_InertialSensor_LIS331()
{
}
*/
//Needs full rework with chibiOS integration
void AP_InertialSensor_LIS331::begin(comm_mode mode)
{
    hal.console->printf("beginning\n");
    this->mode = mode;
    //this->mode = USE_I2C;
    setPowerMode(NORMAL);
    axesEnable(true);
    uint8_t data = 0;
    for (int i = 0x21; i < 0x25; i++) LIS331_write(i,&data,1);
    for (int i = 0x30; i < 0x37; i++) LIS331_write(i,&data,1);
}

void AP_InertialSensor_LIS331::setI2CAddr(uint8_t address)
{
    this->address = address;
    //_dev->set_device_address(address);

}

void AP_InertialSensor_LIS331::setSPICSPin(uint8_t pin)
{
    this->CSPin = pin;
    //AP_HAL::Device::set_device_address(address);
}

void AP_InertialSensor_LIS331::axesEnable(bool enable)
{
    uint8_t data;
    LIS331_read(CTRL_REG1, &data, 1);
    if (enable)
    {
        data |= 0x07;
    }
    else
    {
        data &= ~0x07;
    }
    LIS331_write(CTRL_REG1, &data, 1);
}

void AP_InertialSensor_LIS331::setPowerMode(power_mode pmode)
{
    uint8_t data;
    LIS331_read(CTRL_REG1, &data, 1);

    // The power mode is the high three bits of CTRL_REG1. The mode
    //  constants are the appropriate bit values left shifted by five, so we
    //  need to right shift them to make them work. We also want to mask off the
    //  top three bits to zero, and leave the others untouched, so we *only*
    //  affect the power mode bits.
    data &= ~0xe0; // Clear the top three bits
    data |= pmode<<5; // set the top three bits to our pmode value
    LIS331_write(CTRL_REG1, &data, 1); // write the new value to CTRL_REG1
}

void AP_InertialSensor_LIS331::setODR(data_rate drate)
{
  uint8_t data;
  LIS331_read(CTRL_REG1, &data, 1);

  // The data rate is bits 4:3 of CTRL_REG1. The data rate constants are the
  //  appropriate bit values; we need to right shift them by 3 to align them
  //  with the appropriate bits in the register. We also want to mask off the
  //  top three and bottom three bits, as those are unrelated to data rate and
  //  we want to only change the data rate.
  data &=~0x18;     // Clear the two data rate bits
  data |= drate<<3; // Set the two data rate bits appropriately.
  LIS331_write(CTRL_REG1, &data, 1); // write the new value to CTRL_REG1
}

void AP_InertialSensor_LIS331::readAxes(int16_t &x, int16_t &y, int16_t &z)
{
  uint8_t data[6]; // create a buffer for our incoming data
  LIS331_read(OUT_X_L, &data[0], 1);
  LIS331_read(OUT_X_H, &data[1], 1);
  LIS331_read(OUT_Y_L, &data[2], 1);
  LIS331_read(OUT_Y_H, &data[3], 1);
  LIS331_read(OUT_Z_L, &data[4], 1);
  LIS331_read(OUT_Z_H, &data[5], 1);
  // The data that comes out is 12-bit data, left justified, so the lower
  //  four bits of the data are always zero. We need to right shift by four,
  //  then typecase the upper data to an integer type so it does a signed
  //  right shift.
  x = data[0] | data[1] << 8;
  y = data[2] | data[3] << 8;
  z = data[4] | data[5] << 8;
  x = x >> 4;
  y = y >> 4;
  z = z >> 4;
  raw_accel_400g = {(float)x,(float)y,(float)z};
}

uint8_t AP_InertialSensor_LIS331::readReg(uint8_t reg_address)
{
  uint8_t data;
  LIS331_read(reg_address, &data, 1);
  return data;
}

float AP_InertialSensor_LIS331::convertToG(int maxScale, int reading)
{
  float result = (float(maxScale) * float(reading))/2047;
  return result;
}

void AP_InertialSensor_LIS331::setHighPassCoeff(high_pass_cutoff_freq_cfg hpcoeff)
{
  // The HPF coeff depends on the output data rate. The cutoff frequency is
  //  is approximately fs/(6*HPc) where HPc is 8, 16, 32 or 64, corresponding
  //  to the various constants available for this parameter.
  uint8_t data;
  LIS331_read(CTRL_REG2, &data, 1);
  data &= ~0xfc;  // Clear the two low bits of the CTRL_REG2
  data |= hpcoeff;
  LIS331_write(CTRL_REG2, &data, 1);
}

void AP_InertialSensor_LIS331::enableHPF(bool enable)
{
  // Enable the high pass filter
  uint8_t data;
  LIS331_read(CTRL_REG2, &data, 1);
  if (enable)
  {
    data |= 1<<5;
  }
  else
  {
    data &= ~(1<<5);
  }
  LIS331_write(CTRL_REG2, &data, 1);
}

void AP_InertialSensor_LIS331::HPFOnIntPin(bool enable, uint8_t pin)
{
  // Enable the hpf on signal to int pins
  uint8_t data;
  LIS331_read(CTRL_REG2, &data, 1);
  if (enable)
  {
    if (pin == 1)
    {
      data |= 1<<3;
    }
    if (pin == 2)
    {
      data |= 1<<4;
    }
  }
  else
  {
    if (pin == 1)
    {
      data &= ~1<<3;
    }
    if (pin == 2)
    {
      data &= ~1<<4;
    }
  }
  LIS331_write(CTRL_REG2, &data, 1);
}

void AP_InertialSensor_LIS331::intActiveHigh(bool enable)
{
  // Are the int pins active high or active low?
  uint8_t data;
  LIS331_read(CTRL_REG3, &data, 1);
  // Setting bit 7 makes int pins active low
  if (!enable)
  {
    data |= 1<<7;
  }
  else
  {
    data &= ~(1<<7);
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void AP_InertialSensor_LIS331::intPinMode(pp_od _pinMode)
{
  uint8_t data;
  LIS331_read(CTRL_REG3, &data, 1);
  // Setting bit 6 makes int pins open drain.
  if (_pinMode == OPEN_DRAIN)
  {
    data |= 1<<6;
  }
  else
  {
    data &= ~(1<<6);
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void AP_InertialSensor_LIS331::latchInterrupt(bool enable, uint8_t intSource)
{
  // Latch mode for interrupt. When enabled, you must read the INTx_SRC reg
  //  to clear the interrupt and make way for another.
  uint8_t data;
  LIS331_read(CTRL_REG3, &data, 1);
  // Enable latching by setting the appropriate bit.
  if (enable)
  {
    if (intSource == 1)
    {
      data |= 1<<2;
    }
    if (intSource == 2)
    {
      data |= 1<<5;
    }
  }
  else
  {
    if (intSource == 1)
    {
      data &= ~1<<2;
    }
    if (intSource == 2)
    {
      data &= ~1<<5;
    }
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void AP_InertialSensor_LIS331::intSrcConfig(int_sig_src src, uint8_t pin)
{

  uint8_t data;
  LIS331_read(CTRL_REG3, &data, 1);
  // Enable latching by setting the appropriate bit.
  if (pin == 1)
  {
    data &= ~0xfc; // clear the low two bits of the register
    data |= src;
  }
  if (pin == 2)
  {
    data &= ~0xe7; // clear bits 4:3 of the register
    data |= src<<4;
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void AP_InertialSensor_LIS331::setFullScale(fs_range range)
{
  uint8_t data;
  LIS331_read(CTRL_REG4, &data, 1);
  data &= ~0xcf;
  data |= range<<4;
  LIS331_write(CTRL_REG4, &data, 1);
}

bool AP_InertialSensor_LIS331::newXData()
{
  uint8_t data;
  LIS331_read(STATUS_REG, &data, 1);
  if (data & 1<<0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool AP_InertialSensor_LIS331::newYData()
{
  uint8_t data;
  LIS331_read(STATUS_REG, &data, 1);
  if (data & 1<<1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool AP_InertialSensor_LIS331::newZData()
{
  uint8_t data;
  LIS331_read(STATUS_REG, &data, 1);
  if (data & 1<<2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void AP_InertialSensor_LIS331::enableInterrupt(int_axis axis, trig_on_level trigLevel,
                     uint8_t interrupt, bool enable)
{
  uint8_t data, reg, mask;
  mask = 0;
  if (interrupt == 1)
  {
    reg = INT1_CFG;
  }
  else
  {
    reg = INT2_CFG;
  }
  LIS331_read(reg, &data, 1);
  if (trigLevel == TRIG_ON_HIGH)
  {
    mask = 1<<1;
  }
  else
  {
    mask = 1;
  }
  if (axis == Z_AXIS) mask = mask<<4;
  if (axis == Y_AXIS) mask = mask<<2;
  if (enable)
  {
    data |= mask;
  }
  else
  {
    data &= ~mask;
  }
  LIS331_write(reg, &data, 1);
}

void AP_InertialSensor_LIS331::setIntDuration(uint8_t duration, uint8_t intSource)
{
  if (intSource == 1)
  {
    LIS331_write(INT1_DURATION, &duration, 1);
  }
  else
  {
    LIS331_write(INT2_DURATION, &duration, 1);
  }
}

void AP_InertialSensor_LIS331::setIntThreshold(uint8_t threshold, uint8_t intSource)
{
  if (intSource == 1)
  {
    LIS331_write(INT1_THS, &threshold, 1);
  }
  else
  {
    LIS331_write(INT2_THS, &threshold, 1);
  }
}

void AP_InertialSensor_LIS331::LIS331_write(uint8_t reg_address, uint8_t *data, uint8_t len)
{
  _dev->write_register(reg_address, *data);
  if (mode == USE_I2C)
  {
    // I2C write handling code
    /*Wire.beginTransmission(address);
    Wire.write(reg_address);
    for(int i = 0; i<len; i++)
    {
      Wire.write(data[i]);
    }
    Wire.endTransmission();*/
    //_dev->write_register(reg_address, *data);
  }
  /*else
  {
    // SPI write handling code
    digitalWrite(CSPin, LOW);
    SPI.transfer(reg_address | 0x40);
    for (int i=0; i<len; i++)
    {
      SPI.transfer(data[i]);
    }
    digitalWrite(CSPin, HIGH);
  }*/
}

void AP_InertialSensor_LIS331::LIS331_read(uint8_t reg_address, uint8_t *data, uint8_t len)
{
  _dev->read_registers(reg_address, data, len);
  if (mode == USE_I2C)
  {
    // I2C read handling code
    /*Wire.beginTransmission(address);
    Wire.write(reg_address);
    Wire.endTransmission();
    Wire.requestFrom(address, len);
    for (int i = 0; i<len; i++)
    {
      data[i] = Wire.read();
    }*/
    //_dev->read_registers(reg_address, data, len);
  }
  /*else
  {
    // SPI read handling code
    digitalWrite(CSPin, LOW);
    SPI.transfer(reg_address | 0xC0);
    for (int i=0; i<len; i++)
    {
      data[i] = SPI.transfer(0);
    }
    digitalWrite(CSPin, HIGH);
  }*/
}

//Testing ChibiOS I2C Drivers from
//http://chibios.sourceforge.net/docs3/hal/group___i2_c.html#ga1bd147f37bd45e003b06f17c0a9b4051

//Needs research on how ardupilot uses this driver and how it can be incorporated here
/*
void AP_inertialSensor_LIS331::LIS331_read(I2CDriver* i2cp, i2caddr_t addr, uint8_t* rxbuf, size_t rxbytes, sysinterval_t timeout){
    i2cMasterReceiveTimeout(i2cp, addr, rxbuf, rxbytes, timeout);
}
*/

//We can use I2CDevice::transfer...somehow
/*
//Try to mimic Invensense's _read_fifo function
void AP_InertialSensor_LIS331::read_fifo(){
  uint8_t n_samples;
  uint16_t bytes_read;
  _fifo_buffer = (uint8_t *)hal.util->malloc_type(MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
  if (_fifo_buffer == nullptr) {
      AP_HAL::panic("Invensense: Unable to allocate FIFO buffer");
  }
  uint8_t *rx = _fifo_buffer;
  bool need_reset = false;

  bytes_read = uint16_val(rx, 0);
  n_samples = bytes_read/MPU_SAMPLE_SIZE;

  if (n_samples > 4) {
      need_reset = true;
      n_samples = 4;
  }

  while (n_samples > 0){
    uint8_t n = MIN(n_samples, MPU_FIFO_BUFFER_LEN);
    uint8_t reg = MPUREG_FIFO_R_W | 0x80;                                       //Probably Needs Changing
    memset(rx, 0, n * MPU_SAMPLE_SIZE);
    if (!_dev->transfer(rx, n * MPU_SAMPLE_SIZE, rx, n * MPU_SAMPLE_SIZE)) {
        hal.console->printf("LIS331: error in fifo read %u bytes\n", n * MPU_SAMPLE_SIZE);
    }
  }
}*/

void AP_InertialSensor_LIS331::fakeStart(){
  hal.console->printf("this does kinda work\n");
  uint8_t ctrlReg = CTRL_REG1;
  uint8_t powerModeData;
  power_mode pmode = NORMAL;
  //_dev->set_device_address(LIS331_ADDRESS);
  _dev->transfer(&ctrlReg, 1, &powerModeData, sizeof(powerModeData));
  powerModeData &= ~0xe0; // Clear the top three bits
  powerModeData |= pmode<<5; // set the top three bits to our pmode value
}

//Start to replace Begin - only need to call this and read_data_transaction_a
void AP_InertialSensor_LIS331::start(){
  hal.console->printf("beginning\n");
  //this->mode = mode;
  //this->mode = USE_I2C;
  //setPowerMode(NORMAL);
  //axesEnable(true);

  //uint8_t ctrlReg = CTRL_REG1;

  //setPowerMode
  uint8_t powerModeData;
  power_mode pmode = NORMAL;
  //i2c.transfer(&ctrlReg, 1, &powerModeData, sizeof(powerModeData));
  _dev->read_registers(CTRL_REG1, &powerModeData, sizeof(powerModeData));

  // The power mode is the high three bits of CTRL_REG1. The mode
  //  constants are the appropriate bit values left shifted by five, so we
  //  need to right shift them to make them work. We also want to mask off the
  //  top three bits to zero, and leave the others untouched, so we *only*
  //  affect the power mode bits.
  powerModeData &= ~0xe0; // Clear the top three bits
  powerModeData |= pmode<<5; // set the top three bits to our pmode value
  //LIS331_write(CTRL_REG1, &powerModeData, 1); // write the new value to CTRL_REG1
  _dev->write_register(CTRL_REG1, powerModeData, 1); // write the new value to CTRL_REG1

  //axesEnable
  uint8_t axesEnableData;
  //i2c.transfer(&ctrlReg, 1, &axesEnableData, sizeof(axesEnableData));
  _dev->read_registers(CTRL_REG1, &axesEnableData, sizeof(axesEnableData));
  axesEnableData |= 0x07;
  //LIS331_write(CTRL_REG1, &axesEnableData, 1);
  _dev->write_register(CTRL_REG1, axesEnableData, 1);

  uint8_t data = 0;
  //for (int i = 0x21; i < 0x25; i++) LIS331_write(i,&data,1);
  for (int i = 0x21; i < 0x25; i++) _dev->write_register(i, data, 1);
  //for (int i = 0x30; i < 0x37; i++) LIS331_write(i,&data,1);
  for (int i = 0x30; i < 0x37; i++) _dev->write_register(i, data, 1);
}

//Try to mimic LSM9DS0's _read_data_transaction_a functions
void AP_InertialSensor_LIS331::read_data_transaction_a(int16_t &x, int16_t &y, int16_t &z){
  //struct sensor_raw_data raw_data = { };
  //const uint8_t reg = OUT_X_L_A | 0xC0;                                         //Probably Needs changing

  //if (!_dev_accel->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data))) {
  //    hal.console->printf("LIS331: error reading accelerometer\n");
  //    return;
  //}

  uint8_t data[6]; // create a buffer for our incoming data
  uint8_t reg[6] = {OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H};
  for (int i = 0; i < 6; i++) {
    //i2c.transfer(&reg[i], 1, &data[i], sizeof(data[i]));
    _dev->read_registers(reg[i], &data[i], sizeof(data[i]));
  }
  // The data that comes out is 12-bit data, left justified, so the lower
  //  four bits of the data are always zero. We need to right shift by four,
  //  then typecase the upper data to an integer type so it does a signed
  //  right shift.
  x = data[0] | data[1] << 8;
  y = data[2] | data[3] << 8;
  z = data[4] | data[5] << 8;
  x = x >> 4;
  y = y >> 4;
  z = z >> 4;
  raw_accel_400g = {(float)x,(float)y,(float)z};
}
