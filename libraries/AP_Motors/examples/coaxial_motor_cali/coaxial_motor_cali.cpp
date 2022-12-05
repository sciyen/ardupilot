/*
 *  Testing of Coaxial Gimbal Motor library
 *  Code by Yen Cheng Chu (sciyen).
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS_Dummy.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

uint16_t calc_pwm(float scaled_value, uint16_t servo_max, uint16_t servo_min);
uint16_t calc_pwm_x_from_angle(float deg);
uint16_t calc_pwm_y_from_angle(float deg);


// static AP_BoardConfig BoardConfig;
#define SERVO_OUTPUT_RANGE  4500

// #define RC_OUTPUT


// static uint16_t pwm = 1500;
// static int8_t delta = 1;
#define UPPER_MOTOR_CH CH_1
#define LOWER_MOTOR_CH CH_2
#define X_AXIS_SERVO_CH CH_4
#define Y_AXIS_SERVO_CH CH_3

const uint16_t x_servo_min = 700;
const uint16_t x_servo_max = 2300;
const uint16_t y_servo_min = 700;
const uint16_t y_servo_max = 2300;
const float x_gear_ratio = 29.0f / 10.0f;
const float y_gear_ratio = 16.0f / 14.0f;
/*
 *  rotation tests
 */
void setup(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif
    hal.console->begin(115200);
    hal.console->printf("Coaxial motor library testing\n");

#ifdef RC_OUTPUT 
    hal.rcout->enable_ch(UPPER_MOTOR_CH);
    hal.rcout->enable_ch(LOWER_MOTOR_CH);
    hal.rcout->write(UPPER_MOTOR_CH, 1100);
    hal.rcout->write(LOWER_MOTOR_CH, 1100);
    hal.rcout->enable_ch(X_AXIS_SERVO_CH);
    hal.rcout->enable_ch(Y_AXIS_SERVO_CH);
#else
    // SRV_Channels::set_rc_frequency(SRV_Channel::k_aileron, 400);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_aileron, CH_3);
    SRV_Channels::set_output_min_max(SRV_Channel::k_aileron, 1100, 1900);
    SRV_Channels::set_trim_to_pwm_for(SRV_Channel::k_aileron, 1500);
    SRV_Channels::set_angle(SRV_Channel::k_aileron, SERVO_OUTPUT_RANGE);
    // for (uint8_t i = 0; i< 14; i++) {
    //     hal.rcout->enable_ch(i);
    // }
    // hal.rcout->enable_ch(2);
    // SRV_Channels::enable_by_mask(1<<CH_3);
    SRV_Channels::enable_aux_servos();
    // SRV_Channels::set_disabled_channel_mask(0U);
    hal.scheduler->delay(1000);
#endif
}
#ifdef RC_OUTPUT
uint16_t calc_pwm(float scaled_value, uint16_t servo_max, uint16_t servo_min){
    const uint16_t high_out = 4500;
    const uint16_t servo_trim = 1500;
    scaled_value = constrain_float(scaled_value, -high_out, high_out);
    if (scaled_value > 0) {
        return servo_trim + uint16_t( (scaled_value * (float)(servo_max - servo_trim)) / (float)high_out);
    } else {
        return servo_trim - uint16_t( (-scaled_value * (float)(servo_trim - servo_min)) / (float)high_out);
    }
}

uint16_t calc_pwm_x_from_angle(float deg){
    // 90 def -> 4500, i.e. 50/deg
    return calc_pwm(deg * x_gear_ratio * 50.0f, x_servo_max, x_servo_min);
}
uint16_t calc_pwm_y_from_angle(float deg){
    // 90 def -> 4500, i.e. 50/deg
    return calc_pwm(deg * y_gear_ratio * 50.0f, y_servo_max, y_servo_min);
}

static float angle = 0;
static float delta = 0.01f;
const float limit = 10.0f;
// static uint16_t angle = 1500;
// static int16_t delta = 1;
int16_t value = ' ';
#endif

void loop(void) {
#ifdef RC_OUTPUT
    if( hal.console->available() ) {
        // get character from user
        int16_t tmp = hal.console->read();
        if ((tmp=='t') || tmp=='c' || tmp=='m' || tmp=='s' || tmp==' ')
            value = tmp;
    }

    // test motors
    if (value == 't' || value == 'T') {
        hal.rcout->write(UPPER_MOTOR_CH, 1100);
        hal.rcout->write(LOWER_MOTOR_CH, 1100);
        delta = 0.1f;
        angle += delta;
        if (delta > 0 && angle >= limit) {
            delta = -0.1;
            hal.console->printf("decreasing\n");
            hal.scheduler->delay(2000);
        } else if (delta < 0 && angle <= -limit) {
            delta = 0.1;
            hal.console->printf("increasing\n");
            hal.scheduler->delay(2000);
        }
        hal.rcout->write(X_AXIS_SERVO_CH, calc_pwm_x_from_angle(angle));
        hal.rcout->write(Y_AXIS_SERVO_CH, calc_pwm_y_from_angle(angle));
        hal.scheduler->delay(5);
    }
    else if (value == 'c' || value == 'C') {
        delta = 0.01f;
        angle += delta;
        hal.rcout->write(X_AXIS_SERVO_CH, calc_pwm_x_from_angle(limit * cosf(angle)));
        hal.rcout->write(Y_AXIS_SERVO_CH, calc_pwm_y_from_angle(limit * sinf(angle)));
        hal.rcout->write(UPPER_MOTOR_CH, 1150);
        hal.rcout->write(LOWER_MOTOR_CH, 1150);
        hal.scheduler->delay(5);
    }
    else if (value == 'm' || value == 'M') {
        hal.rcout->write(X_AXIS_SERVO_CH, 1500);
        hal.rcout->write(Y_AXIS_SERVO_CH, 1500);
        hal.rcout->write(UPPER_MOTOR_CH, 1100);
        hal.rcout->write(LOWER_MOTOR_CH, 1100);
    }
    else if (value == 's' || value == 'S') {
        hal.console->printf("ESC calibration, please plugin the power\n");
        hal.rcout->write(UPPER_MOTOR_CH, 1900);
        hal.rcout->write(LOWER_MOTOR_CH, 1900);
        while( !(hal.console->available() && hal.console->read() == 's') ) {
            hal.console->printf("Waiting for 'beep beep' and then press any key\n");
            hal.rcout->write(UPPER_MOTOR_CH, 1900);
            hal.rcout->write(LOWER_MOTOR_CH, 1900);
            hal.scheduler->delay(20);
        }
        hal.rcout->write(UPPER_MOTOR_CH, 1100);
        hal.rcout->write(LOWER_MOTOR_CH, 1100);
        value = ' ';
    }
    else{
        hal.rcout->write(UPPER_MOTOR_CH, 1100);
        hal.rcout->write(LOWER_MOTOR_CH, 1100);
        hal.scheduler->delay(20);
    }
#else
    SRV_Channels::set_output_pwm(SRV_Channel::k_aileron, 1500);
    // SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, pwm);
    SRV_Channels::output_ch_all();
    // SRV_Channels::push();
    
    // hal.console->printf("Disabled channels: %ld\n", SRV_Channels::get_disabled_channel_mask());
    hal.console->printf("Runnning\n");
    hal.scheduler->delay(100);
#endif
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
