/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/* Most of the program in this library are migrated from AP_MotorsGimbal.cpp
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsCoaxGimbal.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// init
// motor 1 is for servo 1
// motor 2 is for servo 2
// motor 3 is for ESC1
// motor 4 is for ESC2
void AP_MotorsGimbal::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    hal.console->printf("Adding motor\n");
    // make sure 6 output channels are mapped

#ifdef RC_OUTPUT_TEST 
    hal.rcout->enable_ch(UPPER_MOTOR_CH);   // upper
    hal.rcout->enable_ch(LOWER_MOTOR_CH);   // lower
    // hal.rcout->enable_ch(ECHO_MOTOR_CH);
    hal.rcout->write(UPPER_MOTOR_CH, 1100); // x-axis
    hal.rcout->write(LOWER_MOTOR_CH, 1100); // y-axis
    // hal.rcout->write(ECHO_MOTOR_CH, 1100);
    hal.rcout->enable_ch(X_AXIS_SERVO_CH);
    hal.rcout->enable_ch(Y_AXIS_SERVO_CH);
#else
    // for (uint8_t i = 0; i < 4; i++) {
    //     add_motor_num(CH_1 + i);
    // }
    // SRV_Channels::set_output_min_max(SRV_Channel::k_motor1, x_servo_min, x_servo_max); // CH_1
    // SRV_Channels::set_output_min_max(SRV_Channel::k_motor2, y_servo_min, y_servo_max); // CH_2
    // SRV_Channels::set_output_min_max(SRV_Channel::k_motor3, 1100, 1900);
    // SRV_Channels::set_output_min_max(SRV_Channel::k_motor4, 1100, 1900);
    // SRV_Channels::set_trim_to_pwm_for(SRV_Channel::k_motor1, 1500);
    // SRV_Channels::set_trim_to_pwm_for(SRV_Channel::k_motor2, 1500);
    // SRV_Channels::set_trim_to_pwm_for(SRV_Channel::k_motor3, 1500);
    // SRV_Channels::set_trim_to_pwm_for(SRV_Channel::k_motor4, 1500);

    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, UPPER_MOTOR_CH);

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, LOWER_MOTOR_CH);

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, X_AXIS_SERVO_CH);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, AP_MOTORS_GIMBAL_SERVO_INPUT_RANGE);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, Y_AXIS_SERVO_CH);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, AP_MOTORS_GIMBAL_SERVO_INPUT_RANGE);

    hal.console->printf("Enabling motor\n");
    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[UPPER_MOTOR_CH] = true;
    motor_enabled[LOWER_MOTOR_CH] = true;
    // SRV_Channels::enable_aux_servos();

    set_update_rate(_speed_hz);
#endif

    hal.console->printf("Setting angles\n");
#ifdef RC_OUTPUT_TEST 
    hal.rcout->write(X_AXIS_SERVO_CH, 1500);
    hal.rcout->write(Y_AXIS_SERVO_CH, 1500);
#endif

    _mav_type = MAV_TYPE_COAXIAL;

    hal.console->printf("Initializing\n");
    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_COAX_GIMBAL);
    limit.pitch = false;
    limit.roll  =false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;
    
    _pwm_min.set(1100);
    _pwm_max.set(1900);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsGimbal::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok(frame_class == MOTOR_FRAME_COAX_GIMBAL);
}

// set update rate to motors - a value in hertz
void AP_MotorsGimbal::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    #ifdef RC_OUTPUT_TEST 

    uint32_t mask =
        1U << UPPER_MOTOR_CH |
        1U << LOWER_MOTOR_CH |
        1U << X_AXIS_SERVO_CH |
        1U << Y_AXIS_SERVO_CH ;
    rc_set_freq(mask, _speed_hz);
    #else
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, _speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, _speed_hz);
    #endif
}

void AP_MotorsGimbal::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

#ifdef RC_OUTPUT_TEST
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
        case SpoolState::GROUND_IDLE:
            // sends minimum values out to the motors
            hal.rcout->write(UPPER_MOTOR_CH, 1100); // x-axis
            hal.rcout->write(LOWER_MOTOR_CH, 1100); // y-axis
            hal.rcout->write(X_AXIS_SERVO_CH, 1500);
            hal.rcout->write(Y_AXIS_SERVO_CH, 1500);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            // int x_axis = _calc_pwm_raw(_calc_scaled_x_from_angle(_actuator_out[X_AXIS_SERVO_CH]), x_servo_max, x_servo_min);
            // int y_axis = _calc_pwm_raw(_calc_scaled_y_from_angle(_actuator_out[Y_AXIS_SERVO_CH]), y_servo_max, y_servo_min);
            hal.rcout->write(X_AXIS_SERVO_CH, _calc_pwm_raw(_calc_scaled_x_from_angle(_actuator_out[X_AXIS_SERVO_CH]), x_servo_max, x_servo_min)); // x-axis
            hal.rcout->write(Y_AXIS_SERVO_CH, _calc_pwm_raw(_calc_scaled_y_from_angle(_actuator_out[Y_AXIS_SERVO_CH]), y_servo_max, y_servo_min)); // y-axis
            hal.rcout->write(UPPER_MOTOR_CH, output_to_pwm(_actuator_out[UPPER_MOTOR_CH]));
            hal.rcout->write(LOWER_MOTOR_CH, output_to_pwm(_actuator_out[LOWER_MOTOR_CH]));
            // hal.rcout->write(UPPER_MOTOR_CH, 1300);
            // hal.rcout->write(LOWER_MOTOR_CH, 1100);
            // hal.console->printf("motor command: %f, %f\n", _actuator_out[UPPER_MOTOR_CH], _actuator_out[LOWER_MOTOR_CH]);
            // hal.console->printf("servo pwm: %d, %d, %d, %d\n", x_axis, y_axis, output_to_pwm(_actuator_out[UPPER_MOTOR_CH]), output_to_pwm(_actuator[LOWER_MOTOR_CH]));
            break;
            // TODO: The difference between _actuator_out and _actuator?
    }
#else
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            _actuator_out[UPPER_MOTOR_CH] = 0.0f;
            _actuator_out[LOWER_MOTOR_CH] = 0.0f;
            _actuator_out[X_AXIS_SERVO_CH] = 0.0f;
            _actuator_out[Y_AXIS_SERVO_CH] = 0.0f;
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            set_actuator_with_slew(_actuator[UPPER_MOTOR_CH], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[LOWER_MOTOR_CH], actuator_spin_up_to_ground_idle());
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            set_actuator_with_slew(_actuator[UPPER_MOTOR_CH], thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[LOWER_MOTOR_CH], thrust_to_actuator(_thrust_left));
            break;
    }
    
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(_actuator[UPPER_MOTOR_CH]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(_actuator[LOWER_MOTOR_CH]));

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_left*AP_MOTORS_GIMBAL_SERVO_INPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_right*AP_MOTORS_GIMBAL_SERVO_INPUT_RANGE);
#endif
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsGimbal::get_motor_mask()
{
#ifdef RC_OUTPUT_TEST
    uint32_t motor_mask =
        1U << UPPER_MOTOR_CH |
        1U << LOWER_MOTOR_CH |
        1U << X_AXIS_SERVO_CH |
        1U << Y_AXIS_SERVO_CH;
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);
#else
    uint32_t mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        mask |= 1U << chan;
    }
#endif

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// sends commands to the motors
void AP_MotorsGimbal::output_armed_stabilizing()
{
    // const float C_D;
    // const float C_L;
    // const float k_d = 0.5f; // = k_w * k_w * rho * power(d, 5) * C_D
    // const float k_f = 1.5f * 9.818f; // = k_w * k_w * rho * power(d, 4) * C_L
    const float k_d = 0.2132f; // = k_w * k_w * rho * power(d, 5) * C_D
    const float k_d2 = 0.0043f;
    const float k_f = 10.2645f; // = k_w * k_w * rho * power(d, 4) * C_L
    const float k_f2 = -2.9717f;
    // const float prop_d = 9 * 0.0254;    // meter
    const float max_M_x = 0.1f;         // Nm
    const float max_M_y = 0.1f;         // Nm
    const float l_pg = 0.03f;           // meter
    const float max_T_f = 15.0f;         // kg
    const float servo_angle_limit = 30.0f; // degree

    float T_f;
    float T_d;
    float M_x;
    float M_y;
    // float M_z;

    // const float compensation_gain = get_compensation_gain();
    const float compensation_gain = 1.0f;
    // Determine thrust commands
    // T_f = get_throttle() * compensation_gain;
    T_f = _throttle_in * compensation_gain;

    // Determine moment commands
    // moment commands do not require compensation because the servo motors 
    // is less affected by the power voltage (with the secondary power supply)
    M_x = (_roll_in + _roll_in_ff);
    M_y = (_pitch_in + _pitch_in_ff);
    T_d = (_yaw_in + _yaw_in_ff);

    // hal.console->printf("\n\nInput:\n");
    // hal.console->printf("T_f: %.8f\n", T_f);
    // hal.console->printf("T_d: %.8f\n", T_d);
    // hal.console->printf("M_x: %.8f\n", M_x);
    // hal.console->printf("M_y: %.8f\n", M_y);

    // Bounding for virtual commands
    if (T_f < 0.1){
        limit.throttle_lower = true;
        T_f = 0.1;
    } else if (T_f > max_T_f) {
        limit.throttle_upper = true;
        T_f = max_T_f;
    }

    const float T_d_max = T_f * k_d / k_f;
    if (fabsf(T_d) > T_d_max){
        T_d = constrain_float(T_d, -T_d_max, T_d_max);
        limit.yaw = true;
    }
    if (fabsf(M_x) > max_M_x){
        M_x = constrain_float(M_x, -max_M_x, max_M_x);
        limit.roll = true;
    }
    if (fabsf(M_y) > max_M_y){
        M_y = constrain_float(M_y, -max_M_y, max_M_y);
        limit.pitch = true;
    }

    // hal.console->printf("\n\nAfter Saturation:\n");
    // hal.console->printf("T_f: %.8f\n", T_f);
    // hal.console->printf("T_d: %.8f\n", T_d);
    // hal.console->printf("M_x: %.8f\n", M_x);
    // hal.console->printf("M_y: %.8f\n", M_y);

    // Solve for motor and actuator commands
    const float det_W1 = (l_pg * l_pg * T_f * T_f + T_d * T_d);
    float eta = -RAD_TO_DEG  * (l_pg * T_f * M_x - T_d * M_y) / det_W1;   // in radian
    float xi = RAD_TO_DEG * (T_d * M_x + l_pg * T_f * M_y) / det_W1;    // in radian

    // hal.console->printf("\n\nAfter servo angle calculation:\n");
    // hal.console->printf("det_W1: %.8f\n", det_W1);
    // hal.console->printf("eta: %.8f\n", eta);
    // hal.console->printf("xi: %.8f\n", xi);
    if (fabsf(eta) > servo_angle_limit){
        eta = constrain_float(eta, -servo_angle_limit, servo_angle_limit);
    }
    if (fabsf(xi) > servo_angle_limit){
        xi = constrain_float(xi, -servo_angle_limit, servo_angle_limit);
    }

    // hal.console->printf("\n\nAfter servo angle saturation:\n");
    // hal.console->printf("eta: %.8f\n", eta);
    // hal.console->printf("xi: %.8f\n", xi);

    // Bounding for motor and actuator commands (avoiding imaginary output)
    T_f -= k_f2;
    T_d -= k_d2;
    float u_P1 = (T_f / k_f - T_d / k_d) / 2.0f;
    float u_P2 = (T_f / k_f + T_d / k_d) / 2.0f;
    // float T_P1 = T_f / (2 * C_L) + T_d / (2 * prop_d * C_D);
    // float T_P2 = T_f / (2 * C_L) - T_d / (2 * prop_d * C_D);

    // hal.console->printf("\n\nAfter motor speed calculation:\n");
    // hal.console->printf("u_P1: %.8f\n", u_P1);
    // hal.console->printf("u_P2: %.8f\n", u_P2);

    _actuator_out[X_AXIS_SERVO_CH] = eta;
    _actuator_out[Y_AXIS_SERVO_CH] = xi;
    _actuator_out[UPPER_MOTOR_CH] = u_P1;
    _actuator_out[LOWER_MOTOR_CH] = u_P2;

    // For testing output only
    _eta = eta;
    _xi = xi;
    _Tf = T_f;
    _Td = T_d;
    _u_P1 = u_P1;
    _u_P2 = u_P2;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsGimbal::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
#ifdef RC_OUTPUT_TEST
    switch (motor_seq) {
        case 1:
            // flap servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // flap servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // motor 1
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // motor 2
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        default:
            // do nothing
            break;
    }
#else
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
            break;
        default:
            // do nothing
            break;
    }
#endif
}

float AP_MotorsGimbal::_calc_scaled_x_from_angle(float deg){
    return deg * x_gear_ratio * 50.0f;
}

float AP_MotorsGimbal::_calc_scaled_y_from_angle(float deg){
    return deg * y_gear_ratio * 50.0f;
}

void AP_MotorsGimbal::output_test(){
    // Inputs: (roll, pitch, yaw, thrust), Outputs: (eta, xi, w_p1, w_p2)
    output_armed_stabilizing();
    
    output_logic();
    output_to_motors();
}

void AP_MotorsGimbal::log_test(){
    hal.console->printf("%.4f, %.4f, %.4f, %.4f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f", 
        _roll_in, _pitch_in, _yaw_in, _throttle_in, _eta, _xi, _Tf, _Td, _u_P1, _u_P2);
    // hal.console->printf("safe timer: %f\n", _safe_time.get());
    // hal.console->printf("spin_up_ratio: %f\n", _spin_up_ratio);    
}

# ifdef RC_OUTPUT_TEST
uint16_t AP_MotorsGimbal::_calc_pwm_raw(float scaled_value, uint16_t servo_max, uint16_t servo_min){
    const uint16_t high_out = 4500;
    const uint16_t servo_trim = 1500;
    scaled_value = constrain_float(scaled_value, -high_out, high_out);
    if (scaled_value > 0) {
        return servo_trim + uint16_t( (scaled_value * (float)(servo_max - servo_trim)) / (float)high_out);
    } else {
        return servo_trim - uint16_t( (-scaled_value * (float)(servo_trim - servo_min)) / (float)high_out);
    }
}
#endif
