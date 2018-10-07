#pragma once

#include <inttypes.h>

// forward-declare ISRs
extern "C" void TIMER1_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_COMPA_vect(void) __attribute__ ((signal));

class FastStepper_t
{
public:
    FastStepper_t(uint8_t pin_step, uint8_t pin_direction, uint8_t pin_enable=0xFF);
    ~FastStepper_t();

    void run();

    void set_enabled(bool enable);
    void set_acceleration(float acceleration);
    void set_max_speed(int32_t max_speed);
    void set_target_speed(int32_t target_speed);
    void set_target_position(int32_t target_position);
    void quickstop();

    bool get_enabled();
    int32_t get_current_position() { return _current_position; }
    int32_t get_current_speed() { return _current_speed; }
    int32_t get_max_speed() { return _max_speed; }
    float get_acceleration() { return _acceleration; }
    int32_t get_target_speed() { return _target_speed; }
    int32_t get_target_position() { return _target_position; }

    // Declare ISRs friend of this class so it can access all members
    friend void TIMER1_COMPA_vect(void);
    friend void TIMER3_COMPA_vect(void);

private:
    enum instance_t {INVALID=-1, TIMER_1_INSTANCE, TIMER_3_INSTANCE, NUM_TIMERS};
    static FastStepper_t * instance_ptrs[NUM_TIMERS];

    void set_speed(int32_t freq);
    void run_position_target();
    void run_speed_target();
    void _isr();

    // basic settings
    float _acceleration;
    int32_t _max_speed;
    volatile bool _position_mode = true;

    // basic velocity mode states
    volatile int32_t _current_speed = 0;
    volatile int32_t _target_speed = 0;
    uint32_t _speed_run_last_acceleration_adjustment_time = 0;
    float _speed_run_current_speed_float = 0;

    // basic position mode states
    volatile int32_t _current_position = 0;
    volatile int32_t _target_position = 0;
    volatile bool _has_active_target = false;
    volatile int32_t _decelerate_at;
    volatile float _current_position_virtual_offset = 0;
    volatile bool _current_position_virtual_offset_valid = false;
    volatile uint32_t _time_of_last_step;
    volatile int32_t _pos_run_initial_pos;
    volatile int32_t _pos_run_initial_speed;

    // hardware timer / pins
    const instance_t _timer;
    const uint8_t _pin_enable_bitmask;
    uint8_t * const _pin_enable_port;
    const uint8_t _pin_direction_bitmask;
    uint8_t * const _pin_direction_port;
    volatile uint8_t * const _TCCRnB;
    volatile uint16_t * const _OCRnA;

    // hardware states
    volatile uint8_t _is_rising_edge_isr = false;
    volatile int8_t _stepping_direction = 1;
};
