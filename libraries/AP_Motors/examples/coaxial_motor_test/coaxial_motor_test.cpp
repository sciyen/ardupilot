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

// #include <AP_Scheduler/AP_Scheduler.h>

/* run with:
    ./waf configure --board linux
    ./waf build --targets examples/expo_inverse_test
    ./build/linux/examples/expo_inverse_test
*/

void setup();
void loop();
// void motor_order_test();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class UART_Peripheral{
public:
    int updated;
    uint32_t _baudrate;
    UART_Peripheral(uint8_t serial_id, uint32_t baudrate){
        _uart = hal.serial(serial_id);
        _baudrate = baudrate;

        reading = 0.0f;
        updated = false;
        buf_idx = 0;
    }
    void init(){
        if (_uart != nullptr){
            _uart->begin(_baudrate);
        }
        // hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&MPU6000::measure_trampoline, this);
        // hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&UART_FromScale::update, void));
        // hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&UART_FromScale::update, void));
    }

    float read() {return reading; updated=false;}

    void update(){
        if (_uart == nullptr) return;
        // if (AP_HAL::micros() - _last_update_timestamp < 1000 * 10) {
        //     return;
        // }
        int n = _uart->available();
        if (n > 0){
            // hal.console->printf("read\n");
            for (int i=0; i< n; i++){
                char read = _uart->read();
                if (read == '\n'){
                    buf[buf_idx] = read;
                    reading = atof(buf);
                    buf_idx = 0;
                    updated = true;
                }
                else if ((read >= '0' && read <= '9') || read == '.' || read == '-'){
                    buf[buf_idx] = read;
                    buf_idx++;
                }
            }
        }
        _last_update_timestamp = AP_HAL::micros();
    }

protected:
    AP_HAL::UARTDriver *_uart;
    static const int BUF_LEN = 20;
    char buf[BUF_LEN];
    int buf_idx;
    float reading;
    uint32_t _last_update_timestamp;
};

class Sys{
public:

    AP_MotorsGimbal motors; // loop_rate = 400
    // AP_MotorsCoax  motors;

    AP_BattMonitor _battmonitor{0, nullptr, nullptr};
    UART_Peripheral scale;
    UART_Peripheral spin_estimator;

    bool logging;

    Sys():motors(400), scale(4, 9600), spin_estimator(5, 115200)
    {logging = false;};

    void init(){
        _battmonitor.init();
        motors.init(AP_Motors::MOTOR_FRAME_COAX_GIMBAL, AP_Motors::MOTOR_FRAME_TYPE_COAXIAL_GIMBAL);
        // motors.output_min();
        scale.init();
        spin_estimator.init();
        
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&Sys::send_msg, void));
    };

    void send_msg(){
        static bool pre_log = false;
        if (AP_HAL::millis() - _last_send_time < 10) return;

        // Logging experiment info
        if ((!pre_log) && (logging))
            hal.console->printf("_roll_in, _pitch_in, _yaw_in, _throttle_in, _eta, _xi, _Tf, _Td, _u_P1, _u_P2, scale, spin, voltage, current\n");

        if (logging){
            // Logging time info
            hal.console->printf("%ld, ", AP_HAL::millis());

            // Logging motor info
            motors.log_test();

            // Logging scale info
            hal.console->printf(", %.8f", scale.read());

            // Logging spinning speed estimator info
            hal.console->printf(", %.8f", spin_estimator.read());

            // Logging battery info
            float amps;
            _battmonitor.read();
            bool success_read = _battmonitor.current_amps(amps);
            if (!success_read) amps = 0.0f;
            hal.console->printf(", %.8f, %.8f\n", _battmonitor.voltage(), amps);

            _last_send_time = AP_HAL::millis();
            pre_log = logging;
        }
    }

    void update(){
        scale.update();
        spin_estimator.update();
    }

    void motor_order_test(){
        // hal.console->begin(115200);
        hal.console->printf("testing motor order\n");
        motors.armed(true);

        // Testing servos
        // motor 1 is for servo 1
        // motor 2 is for servo 2
        // motor 3 is for ESC1
        // motor 4 is for ESC2
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
    void output_arm_test(){
        // Test with different cases
        hal.console->printf("output testing 1\n");
        motors.set_roll(0.1f);      // M_x, Nm
        motors.set_pitch(0.0f);     // M_y, Nm
        motors.set_yaw(0.0f);       // M_z, Nm
        motors.set_throttle(10.0f);  // T_f, N
        motors.output_test();
        hal.scheduler->delay(100);
        
        hal.console->printf("output testing 2\n");
        motors.set_roll(0.0f);      // M_x, Nm
        motors.set_pitch(0.1f);     // M_y, Nm
        motors.set_yaw(0.0f);       // M_z, Nm
        motors.set_throttle(10.0f);  // T_f, N
        motors.output_test();
        hal.scheduler->delay(100);
        
        hal.console->printf("output testing 3\n");
        motors.set_roll(0.0f);      // M_x, Nm
        motors.set_pitch(0.0f);     // M_y, Nm
        motors.set_yaw(0.01f);       // M_z, Nm
        motors.set_throttle(10.0f);  // T_f, N
        motors.output_test();
        hal.scheduler->delay(100);

        hal.console->printf("output testing 4\n");
        motors.set_roll(0.2f);      // M_x, Nm
        motors.set_pitch(0.0f);     // M_y, Nm
        motors.set_yaw(0.0f);       // M_z, Nm
        motors.set_throttle(10.0f);  // T_f, N
        motors.output_test();
        hal.scheduler->delay(100);
        
        hal.console->printf("output testing 5\n");
        motors.set_roll(0.0f);      // M_x, Nm
        motors.set_pitch(0.2f);     // M_y, Nm
        motors.set_yaw(0.0f);       // M_z, Nm
        motors.set_throttle(10.0f);  // T_f, N
        motors.output_test();
        hal.scheduler->delay(100);
        
        hal.console->printf("output testing 6\n");
        motors.set_roll(0.0f);      // M_x, Nm
        motors.set_pitch(0.0f);     // M_y, Nm
        motors.set_yaw(0.02f);       // M_z, Nm
        motors.set_throttle(10.0f);  // T_f, N
        motors.output_test();
        hal.scheduler->delay(100);
    }


    void output_arm_test2(){
        // Motors warm up
        hal.console->printf("output testing 1\n");
        motors.armed(true);
        motors.set_interlock(true);
        motors.set_spoolup_block(false);
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        while (motors.get_spool_state() <= AP_Motors::SpoolState::GROUND_IDLE){
            motors.output_test();
            update();
            hal.scheduler->delay(10);
        }

        // Testing start
        // Start logging
        logging = true;
        uint32_t current_time = AP_HAL::millis();
        uint32_t  test_start_time = current_time;
        uint32_t  last_update_time = current_time;
        do{
            current_time = AP_HAL::millis();
            if (current_time - last_update_time >= 5){
                float t = (current_time - test_start_time) / 1000.0f;
                // _drag_torque_test(t);
                _thrust_test(t);
                // _power_consume(t);
                motors.output_test();
                last_update_time = current_time;
            }
            update();

            // hal.scheduler->delay(1);
            hal.scheduler->delay_microseconds(500);
            // hal.console->printf("Desired spool: %d, spool: %d\n", (int)motors.get_desired_spool_state(), (int)motors.get_spool_state());
        }
        while (current_time - test_start_time < 8*M_PI * 1000);
        // while (current_time - test_start_time < 20*M_PI * 1000);

        // Motors shut down
        motors.set_yaw(0);       // M_z, Nm
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        while (motors.get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN){
            motors.output_test();
            update();
            hal.scheduler->delay_microseconds(500);
        }
        motors.armed(false);
        // hal.console->printf("shut down\n");
        auto last = AP_HAL::millis();
        while (AP_HAL::millis() - last < 5000){
            motors.output_test();
            scale.update();
            hal.scheduler->delay(10);
        }

        // Stop logging
        logging = false;
    }

protected:
    uint32_t _last_send_time;

private:
    void _drag_torque_test(float t){
        float amp = 0.07f;
        float out = amp * sinf(t) + amp * sinf(1.3 * t);
        motors.set_roll(0.0f);      // M_x, Nm
        motors.set_pitch(0.0f);     // M_y, Nm
        motors.set_yaw(-out);       // M_z, Nm
        motors.set_throttle(10.0f);  // T_f, N
    }

    void _thrust_test(float t){
        float amp = 2.0f;
        float out = 10.0f + amp * sinf(t) + amp * sinf(1.3 * t);
        motors.set_roll(0.0f);      // M_x, Nm
        motors.set_pitch(0.0f);     // M_y, Nm
        motors.set_yaw(0.0f);       // M_z, Nm
        motors.set_throttle(out);  // T_f, N
    }

    void _power_consume(float t){
        float amp = 0.07f;
        float out = amp * sinf(t) + amp * sinf(1.3 * t);
        motors.set_roll(0.0f);      // M_x, Nm
        motors.set_pitch(0.0f);     // M_y, Nm
        motors.set_yaw(-out);       // M_z, Nm
        motors.set_throttle(8);  // T_f, N
    }
};

static AP_BoardConfig BoardConfig;

Sys sys;

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
    // SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttleRight);
    // SRV_Channels::set_default_function(CH_4, SRV_Channel::k_throttleLeft);
    hal.scheduler->delay(5000);
}

void loop(void) {
    int16_t value;

    // display help
    hal.console->printf("Initializing\n");
    sys.init();
    hal.console->printf("Initialized\n");
    while(true){
        hal.console->printf("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!\n");
        while( !hal.console->available() ) {
            hal.scheduler->delay(100);
        }

        // get character from user
        value = hal.console->read();
        // test motors
        if (value == 't' || value == 'T') {
            hal.console->printf("Running test\n");
            // sys.motor_order_test();
            sys.output_arm_test2();
        }
        sys.scale.update();
    }
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
