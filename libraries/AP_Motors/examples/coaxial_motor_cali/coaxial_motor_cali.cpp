/*
 *  Testing of Coaxial Gimbal Motor library
 *  Code by Yen Cheng Chu (sciyen).
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>

#include <AP_IOMCU/AP_IOMCU.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();


static AP_BoardConfig BoardConfig;
#define SERVO_OUTPUT_RANGE  4500

// #define RC_OUTPUT

/*
 *  rotation tests
 */
void setup(void)
{
    BoardConfig.init();
    hal.console->begin(115200);
    hal.console->printf("Coaxial motor library testing\n");

#ifdef RC_OUTPUT 
    hal.rcout->enable_ch(0);
#else
    SRV_Channels::set_rc_frequency(SRV_Channel::k_tiltMotorLeft, 400);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_1);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, SERVO_OUTPUT_RANGE);
    // SRV_Channels::enable_aux_servos();
#endif

    // hal.scheduler->delay(5000);
}


static uint16_t pwm = 1500;
static int8_t delta = 1;

void loop(void) {
#ifdef RC_OUTPUT
    hal.rcout->write(0, pwm);
#else
    // SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, pwm);
    // SRV_Channels::output_ch_all();
    
    hal.console->printf("Disabled channels: %ld\n", SRV_Channels::get_disabled_channel_mask());
#endif
    pwm += delta;
    if (delta > 0 && pwm >= 2000) {
        delta = -1;
        hal.console->printf("decreasing\n");
    } else if (delta < 0 && pwm <= 1000) {
        delta = 1;
        hal.console->printf("increasing\n");
    }
    hal.scheduler->delay(5);
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
