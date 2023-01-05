/// @file	AP_MotorsGimbal.h
/// @brief	Motor and Servo control class for Co-axial helicopters with two motors and one gimbal at the center
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMulticopter.h"

// feedback direction
#define AP_MOTORS_GIMBAL_POSITIVE      1
#define AP_MOTORS_GIMBAL_NEGATIVE     -1

#define RC_OUTPUT_TEST

// #define NUM_ACTUATORS 2

#define AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS 250 // update rate for digital servos
#define AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS 125  // update rate for analog servos

#define AP_MOTORS_GIMBAL_SERVO_INPUT_RANGE    4500    // roll or pitch input of -4500 will cause servos to their minimum (i.e. radio_min), +4500 will move them to their maximum (i.e. radio_max)

#define UPPER_MOTOR_CH AP_MOTORS_MOT_1
#define LOWER_MOTOR_CH AP_MOTORS_MOT_2
#define X_AXIS_SERVO_CH AP_MOTORS_MOT_4
#define Y_AXIS_SERVO_CH AP_MOTORS_MOT_3

/// @class      AP_MotorsSingle
class AP_MotorsGimbal : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsGimbal(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
    {
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz ) override;

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t            get_motor_mask() override;

    void output_test();

    void log_test();

protected:
    static const uint8_t _num_actuators = 2;
    static const uint8_t _num_motors = 2;

    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    float               _actuator_out[_num_actuators + _num_motors]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
    float               _thrust_yt_ccw;
    float               _thrust_yt_cw;

    const char* _get_frame_string() const override { return "GIMBAL"; }

    const uint16_t x_servo_min = 700;
    const uint16_t x_servo_max = 2300;
    const uint16_t y_servo_min = 700;
    const uint16_t y_servo_max = 2300;
    const float x_gear_ratio = 29.0f / 10.0f;
    const float y_gear_ratio = 16.0f / 14.0f;

    // debug only
    float _eta, _xi, _Tf, _Td, _u_P1, _u_P2;

    // uint16_t _calc_pwm(float scaled_value, uint16_t servo_max, uint16_t servo_min);
    // uint16_t _calc_pwm_x_from_angle(float deg);
    // uint16_t _calc_pwm_y_from_angle(float deg);

    float _calc_scaled_x_from_angle(float deg);
    float _calc_scaled_y_from_angle(float deg);

#ifdef RC_OUTPUT_TEST
    uint16_t _calc_pwm_raw(float scaled_value, uint16_t servo_max, uint16_t servo_min);
#endif

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;
};
