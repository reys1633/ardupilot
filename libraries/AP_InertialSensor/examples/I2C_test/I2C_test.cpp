#include <AP_InertialSensor/AP_InertialSensor_LIS331.h>

AP_HAL::I2CDevice *device;

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
AP_InertialSensor_LIS331 *lis;
void setup(void);
void loop(void);

void setup() {
    // lis->setI2CAddr(0x19);
    lis->begin(AP_InertialSensor_LIS331::USE_I2C);
    // lis->intSrcConfig(AP_InertialSensor_LIS331::INT_SRC, 1);
    // lis->setIntDuration(50, 1);
    // lis->setIntThreshold(2, 1);
    // lis->enableInterrupt(AP_InertialSensor_LIS331::Z_AXIS, AP_InertialSensor_LIS331::TRIG_ON_HIGH, 1, true);
    hal.console->printf("Starting...");

    // device = hal.i2c_mgr->get_device(1, 0x19).leak();

    // get pointer to i2c bus semaphore
    // if (!device->get_semaphore()->take(5)) return;
}

void loop() {
    int16_t x, y, z;
    // uint8_t data;
    // device->read_registers(0x28, &data, sizeof(data));
    // hal.console->printf("Data: %u\n", data);
    lis->readAxes(x, y, z);
    hal.console->printf("x: %u\ty: %u\tz: %u\n", x, y, z);
}

AP_HAL_MAIN();