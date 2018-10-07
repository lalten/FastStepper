#pragma once

#include <inttypes.h>

extern "C" void TIMER1_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_OVF_vect(void) __attribute__ ((signal));

class FastStepper_t
{
public:
    // static void attach(int dir_pin);
    static void attach();

    static void run();

    static void set_acceleration(float acceleration);
    static void set_max_speed(int32_t max_speed);
    static void set_target_speed(int32_t target_speed);
    static void set_target_position(int32_t target_position);
    static void quickstop();

    static int32_t get_current_position() { return current_position; }
    static int32_t get_current_speed() { return current_speed; }
    static int32_t get_max_speed() { return max_speed; }
    static float get_acceleration() { return acceleration; }
    static int32_t get_target_speed() { return target_speed; }
    static int32_t get_target_position() { return target_position; }

    friend void TIMER1_COMPA_vect(void);
    friend void TIMER4_OVF_vect(void);

private:
    static void set_speed(int32_t freq);
    static void run_position_target();
    static void run_speed_target();
    static float acceleration;
    static volatile int32_t current_position;
    static volatile float current_position_virtual_offset;
    static volatile bool current_position_virtual_offset_valid;
    static int32_t max_speed;
    static volatile int32_t target_speed;
    static volatile int32_t current_speed;
    static volatile int32_t target_position;
    static volatile int32_t decelerate_at;
    static volatile bool position_mode;
    static volatile bool has_active_target;
    static uint8_t *pin_dir_A_DDR;
    static uint8_t *pin_dir_A_PORT;
    static uint8_t pin_dir_BITMASK;
    static volatile bool is_rising_edge_isr;
    static volatile uint32_t time_of_last_step;
    static volatile int32_t pos_run_initial_pos;
    static volatile int32_t pos_run_initial_speed;
    static uint32_t speed_run_last_acceleration_adjustment_time;
    static float speed_run_current_speed_float;
};

extern FastStepper_t FastStepper;