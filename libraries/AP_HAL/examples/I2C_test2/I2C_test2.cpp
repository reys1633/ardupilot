#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ChibiOS/I2CDevice.h>

#define I2C_ADDR 0x19

using namespace ChibiOS;

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

AP_HAL::I2CDevice *i2c_dev;

const size_t msg_len = 4;
uint8_t *to_send_i2c, *to_receive_i2c;

uint8_t j = 0;

void setup() {
    i2c_dev = hal.i2c_mgr->get_device( 0, I2C_ADDR ).leak();
    to_receive_i2c = new uint8_t[ msg_len ];
}

void loop() {
    hal.scheduler->delay( 1000 );
    with_semaphore(i2c_dev->get_semaphore());
    i2c_dev->read_registers( to_send_i2c, msg_len, to_receive_i2c, msg_len );
}

AP_HAL_MAIN();