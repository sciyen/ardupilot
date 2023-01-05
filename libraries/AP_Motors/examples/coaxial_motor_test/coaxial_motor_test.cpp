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

class UART_FromScale{
public:
    int updated;
    UART_FromScale(){
        _uart = hal.serial(4);
        _uart_w = hal.serial(5); 
        scale_read = 0.0f;
        updated = false;
        buf_idx = 0;
    }
    void init(){
        if (_uart != nullptr){
            _uart->begin(9600);
        }
        if (_uart_w != nullptr){
            _uart_w->begin(115200);
        }
        // hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&MPU6000::measure_trampoline, this);
        // hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&UART_FromScale::update, void));
        // hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&UART_FromScale::update, void));
    }

    float get_scale() {return scale_read; updated=false;}
    float get_freq() {return freq_read;}

    void update(){
        if (_uart == nullptr || _uart_w == nullptr) return;
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
                    scale_read = atof(buf);
                    buf_idx = 0;
                    updated = true;
                }
                else if ((read >= '0' && read <= '9') || read == '.' || read == '-'){
                    buf[buf_idx] = read;
                    buf_idx++;
                }
            }
        }
        n = _uart_w->available();
        if (n > 0){
            // hal.console->printf("read\n");
            for (int i=0; i< n; i++){
                char read = _uart_w->read();
                if (read == '\n'){
                    buf_w[buf_idx_w] = read;
                    freq_read = atof(buf_w);
                    buf_idx_w = 0;
                    updated = true;
                }
                else if ((read >= '0' && read <= '9') || read == '.' || read == '-'){
                    buf_w[buf_idx_w] = read;
                    buf_idx_w++;
                }
            }
        }
        _last_update_timestamp = AP_HAL::micros();
    }

protected:
    AP_HAL::UARTDriver *_uart;
    AP_HAL::UARTDriver *_uart_w;
    static const int BUF_LEN = 20;
    char buf[BUF_LEN];
    char buf_w[BUF_LEN];
    int buf_idx;
    int buf_idx_w;
    float scale_read;
    float freq_read;
    uint32_t _last_update_timestamp;
};
class Sys{
public:

    AP_MotorsGimbal motors; // loop_rate = 400
    // AP_MotorsCoax  motors;
    UART_FromScale scale;

    bool logging;

    Sys():motors(400) {logging = false;};

    void init(){
        motors.init(AP_Motors::MOTOR_FRAME_COAX_GIMBAL, AP_Motors::MOTOR_FRAME_TYPE_COAXIAL_GIMBAL);
        // motors.output_min();
        scale.init();
        
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&Sys::send_msg, void));
    };

    void send_msg(){
        if (AP_HAL::micros() - _last_send_time < 1000 * 10) return;
        // hal.console->printf("print\n");
        if (logging){
            hal.console->printf("%ld, ", AP_HAL::millis());
            motors.log_test();
            hal.console->printf(", %.8f, %.8f\n", scale.get_scale(), scale.get_freq());
            // hal.console->write()1
            _last_send_time = AP_HAL::micros();
        }
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
        // Test with different cases
        hal.console->printf("output testing 1\n");
        motors.armed(true);
        motors.set_interlock(true);
        motors.set_spoolup_block(false);
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        while (motors.get_spool_state() <= AP_Motors::SpoolState::GROUND_IDLE){
            motors.output_test();
            scale.update();
            hal.scheduler->delay(10);
        }
        hal.console->printf("output testing 2\n");
        logging = true;
        uint32_t  test_start_time = AP_HAL::millis();
        uint32_t  last_update_time = AP_HAL::millis();
        uint32_t current_time = AP_HAL::millis();
        do{
            current_time = AP_HAL::millis();
            if (current_time - last_update_time >= 5){
                float t = (current_time - test_start_time) / 1000.0f;
                // hal.console->printf("%f\n", t);
                float out = 0.08f * (sinf(t)) + 0.08f * (sinf(1.3 * t));
                motors.set_roll(0.0f);      // M_x, Nm
                motors.set_pitch(0.0f);     // M_y, Nm
                motors.set_yaw(-out);       // M_z, Nm
                motors.set_throttle(10.0f);  // T_f, N
                motors.output_test();
                last_update_time = current_time;
            }
            scale.update();
            _battmonitor.read();
            _battmonitor.voltage();

            // hal.scheduler->delay(1);
            hal.scheduler->delay_microseconds(500);
            // hal.console->printf("Desired spool: %d, spool: %d\n", (int)motors.get_desired_spool_state(), (int)motors.get_spool_state());
        }
        while (current_time - test_start_time < 8*M_PI * 1000);
        motors.set_yaw(0);       // M_z, Nm
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        while (motors.get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN){
            motors.output_test();
            scale.update();
            hal.scheduler->delay_microseconds(500);
        }
        motors.armed(false);
        hal.console->printf("shut down\n");
        auto last = AP_HAL::millis();
        while (AP_HAL::millis() - last < 5000){
            motors.output_test();
            scale.update();
            hal.scheduler->delay(10);
        }
        logging = false;
    }

protected:
    uint32_t _last_send_time;
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
    // SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttleRight);
    // SRV_Channels::set_default_function(CH_4, SRV_Channel::k_throttleLeft);
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
            // hal.console->printf("Waiting\n");
            // sys.scale.update();
            // if (sys.scale.updated)
            // hal.console->printf("Scale read: %f\n", sys.scale.get_scale());
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
