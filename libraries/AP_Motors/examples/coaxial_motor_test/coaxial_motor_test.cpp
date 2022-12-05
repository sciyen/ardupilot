/*
 *  Testing of Coaxial Gimbal Motor library
 *  Code by Yen Cheng Chu (sciyen).
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>

/* run with:
    ./waf configure --board linux
    ./waf build --targets examples/expo_inverse_test
    ./build/linux/examples/expo_inverse_test
*/

void setup();
void loop();
// void motor_order_test();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_BattMonitor _battmonitor{0, nullptr, nullptr};

class Sys{
public:

    AP_MotorsGimbal motors; // loop_rate = 400
    // AP_MotorsCoax  motors;

    Sys():motors(400) {};

    void init(){
        motors.init(AP_Motors::MOTOR_FRAME_COAX_GIMBAL, AP_Motors::MOTOR_FRAME_TYPE_COAXIAL_GIMBAL);
        // motors.init(AP_Motors::MOTOR_FRAME_COAX, AP_Motors::MOTOR_FRAME_TYPE_COAXIAL_GIMBAL);

        motors.output_min();
    };

    void motor_order_test(){
        // hal.console->begin(115200);
        hal.console->printf("testing motor order\n");
        motors.armed(true);

        // Testing servos
        // motor 1 is for servo 1
        // motor 2 is for servo 2
        // motor 3 is for ESC1
        // motor 4 is for ESC2
        hal.rcout->enable_ch(CH_3);
        hal.rcout->enable_ch(CH_4);
        for (int8_t i=3; i <= 4; i++) {
            hal.console->printf("Testing Servo %d\n",(int)i);
            for (int pwm=1000; pwm<2000; pwm+=1){
                motors.output_test_seq(i, pwm);
                SRV_Channels::output_ch_all();
                hal.scheduler->delay(5);
            }
            hal.scheduler->delay(1000);
            for (int pwm=2000; pwm>1000; pwm-=1){
                motors.output_test_seq(i, pwm);
                SRV_Channels::output_ch_all();
                hal.scheduler->delay(5);
            }

            hal.scheduler->delay(2000);
        }
        
        // for (int8_t i=1; i <= 2; i++) {
        //     hal.console->printf("Testing BLDC %d\n",(int)i);
        //     motors.output_test_seq(i+2, 1150);
        //     hal.scheduler->delay(100);
        //     motors.output_test_seq(i+2, 1000);

        //     hal.scheduler->delay(2000);
        // }
        motors.armed(false);
        hal.console->printf("finished test.\n");
    };
};

static AP_BoardConfig BoardConfig;


/*
 *  rotation tests
 */
void setup(void)
{
    BoardConfig.init();
    hal.console->begin(115200);
    hal.console->printf("Coaxial motor library testing\n");
    // SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_3);
    // SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_4);
    SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttleRight);
    SRV_Channels::set_default_function(CH_4, SRV_Channel::k_throttleLeft);
    hal.scheduler->delay(5000);
}

void loop(void) {
    Sys sys;
    int16_t value;

    // display help
    hal.console->printf("Initializing\n");
    _battmonitor.init();
    sys.init();
    hal.console->printf("Initialized\n");
    while(true){
        hal.console->printf("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!\n");
        while( !hal.console->available() ) {
            hal.console->printf("Waiting\n");
            hal.scheduler->delay(100);
        }

        // get character from user
        value = hal.console->read();
        // test motors
        if (value == 't' || value == 'T') {
            hal.console->printf("Running test\n");
            sys.motor_order_test();
        }
    }
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
