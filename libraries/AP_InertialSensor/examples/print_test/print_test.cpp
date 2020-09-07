#include <AP_HAL/AP_HAL.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
void setup(void);
void loop(void);

void setup() {
    hal.console->printf("Starting...");
}

void loop() {
    hal.console->printf("LOOP");
}

AP_HAL_MAIN();