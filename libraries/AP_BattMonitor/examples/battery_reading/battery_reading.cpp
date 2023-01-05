/*
 *  Testing of Coaxial Gimbal Motor library
 *  Code by Yen Cheng Chu (sciyen).
 */

// #define HAL_BATT_MONITOR_DEFAULT ANALOG_VOLTAGE_AND_CURRENT
#define HAL_BATT_MONITOR_DEFAULT 4

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>


/* run with:
    ./waf configure --board pixhawk
    ./waf build --targets examples/battery_reading --upload
*/

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_BoardConfig BoardConfig;
AP_BattMonitor _battmonitor{0, nullptr, nullptr};

bool flag_object_set = false;

void setup(void){
    // AP_Param::setup();
    // // if (AP_Param::set_object_value(&_battmonitor, _battmonitor.var_info, "BATT_MONITOR", 4)){
    // if (AP_Param::set_by_name("BATT_MONITOR", 4)){
    //     flag_object_set = true;
    // }
    // else
    //     hal.console->printf("Failed to set object value\n");
    

    BoardConfig.init();
    _battmonitor.init();
    hal.console->begin(115200);
    hal.console->printf("Battery Monitor Test\n");
}

void loop(void) {
    _battmonitor.read();
    float vol =_battmonitor.voltage();
    hal.console->printf("Voltage: %d, %d, %f\n", AP_Param::initialised(), _battmonitor.num_instances(), vol);
    hal.scheduler->delay(500);
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
