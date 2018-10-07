#include <Arduino.h>

#include <avr/io.h>
#include <util/atomic.h>

#include "FastStepper.h"
#include "pin_definitions.h"

FastStepper_t * FastStepper_t::instance_ptrs[] = {nullptr, nullptr};

// float FastStepper_t::acceleration = 100;
// int32_t FastStepper_t::max_speed = 8000000;
// volatile int32_t FastStepper_t::target_speed = 0;
// volatile int32_t FastStepper_t::target_position = 0;
// volatile int32_t FastStepper_t::decelerate_at = 0;
// volatile int32_t FastStepper_t::current_position = 0;
// volatile float FastStepper_t::current_position_virtual_offset = 0;
// volatile bool FastStepper_t::current_position_virtual_offset_valid = false;
// volatile int32_t FastStepper_t::current_speed = 0;
// volatile bool FastStepper_t::position_mode = false;
// volatile bool FastStepper_t::has_active_target = false;
// volatile bool FastStepper_t::is_rising_edge_isr = true;
// volatile uint32_t FastStepper_t::time_of_last_step = 0;
// volatile int32_t FastStepper_t::pos_run_initial_pos = 0;
// volatile int32_t FastStepper_t::pos_run_initial_speed = 0;
// float FastStepper_t::speed_run_current_speed_float = 0;
// uint32_t FastStepper_t::speed_run_last_acceleration_adjustment_time = 0;
// volatile int8_t FastStepper_t::stepping_direction = -1;


// Constructor.
// Sets up Stepper. Pulse pin must be A9.
// Only when these are const (or const pointer respectively), the compiler will
// optimize into a single SBI/CBI instruction for setting pins.
// TODO: pin invert options
FastStepper_t::FastStepper_t(uint8_t pin_direction, uint8_t pin_enable) :
    _timer(TIMER_1_INSTANCE),
    _pin_enable_bitmask(pin_enable==0xFF?0x00:digital_pin_to_bit_mask_PGM[pin_enable]),
    _pin_enable_port(pin_enable==0xFF?nullptr:(uint8_t *) port_to_output_PGM[digital_pin_to_port_PGM[pin_enable]]),
    _pin_direction_bitmask(digital_pin_to_bit_mask_PGM[pin_direction]),
    _pin_direction_port((uint8_t *) port_to_output_PGM[digital_pin_to_port_PGM[pin_direction]])
{
    FastStepper_t::instance_ptrs[_timer] = this;

    // reasonable defaults:
    _current_speed = 0;
    _target_speed = 0;
    _position_mode = false;
    set_acceleration(100);
    set_max_speed(40000);

    // Set up pins
    // Set direction pin as output
    pinMode(pin_direction, OUTPUT);
    // We use PB5 (OCR1A) (Arduino D9) as step pin. Set to output.
    bitWrite(DDRB, PB5, OUTPUT);
    // only set up enable pin if it was explicitly defined (default value os 0xFF)
    if(pin_enable != 0xFF)
    {
        pinMode(pin_enable, OUTPUT);
        digitalWrite(pin_enable, LOW);
    }

    // Set up Timer 1, which will control Stepper A's pulses
    // Fast PWM mode (15) with TOP=OCRnA. No clock source.
    // Toggle OC1A on Compare Match, OC1B and OC1C disconnected (normal port operation)
    TCCR1B = _BV(WGM13) | _BV(WGM12);
    TCCR1A = _BV(WGM11) | _BV(WGM10) | _BV(COM1A0);
    // Interrupt on Output Compare Match A
    TIMSK1 = _BV(OCIE1A);

    // Enable interrupts
    sei();
}

FastStepper_t::~FastStepper_t()
{
    TIMSK1 &= ~_BV(OCIE1A);
    FastStepper_t::instance_ptrs[_timer] = nullptr;
}

void FastStepper_t::set_enabled(bool enable)
{
    // If no enable pin was defined, just return
    if(_pin_enable_port == nullptr)
        return;
    
    if(enable)
    {
        // set bit
        *_pin_enable_port |= _pin_enable_bitmask;
    }
    else
    {
        // clear bit
        *_pin_enable_port &= ~_pin_enable_bitmask;
    }
}

bool FastStepper_t::get_enabled()
{
    // If no enable pin was defined, just return true
    if(_pin_enable_port == nullptr)
        return true;

    return *_pin_enable_port & _pin_enable_bitmask;
}

void FastStepper_t::quickstop()
{
    set_speed(0);
    _position_mode = false;
    _has_active_target = false;
    _target_speed = 0;
    _speed_run_last_acceleration_adjustment_time = 0;
}

void FastStepper_t::set_max_speed(int32_t max_speed)
{
    // max speed is saved as positive number
    if(max_speed < 0)
    {
        max_speed = -max_speed;
    }

    // On a 16MHz CPU, we can't go faster than 8MSteps/s (but forget about also counting steps at that speed)
    max_speed = min(max_speed, 8000000);
    
    _max_speed = max_speed;

    // Limit target_speed to -max_speed..max_speed
    if(_target_speed > max_speed)
    {
        _target_speed = max_speed;
    }
    else if(_target_speed < -max_speed)
    {
        _target_speed = -max_speed;
    }
}

// Takes acceleration in unit steps/s/s. 0 means instant speed changes.
void FastStepper_t::set_acceleration(float acceleration)
{
    if(acceleration < 0)
    {
        // We only take positive acceleration values
        acceleration = -acceleration;
    }
    _acceleration = acceleration;
}

// Switch to velocity mode. Target speed is in steps/sec
void FastStepper_t::set_target_speed(int32_t target_speed)
{
    // Switch to target speed mode
    _position_mode = false;
    _has_active_target = false;
    _speed_run_current_speed_float = _current_speed;

    // Limit target_speed to -max_speed..max_speed
    if(target_speed > _max_speed)
    {
        target_speed = _max_speed;
    }
    else if(target_speed < -_max_speed)
    {
        target_speed = -_max_speed;
    }
    _target_speed = target_speed;

    // If no acceleration needed, change immediately
    if(_acceleration == 0)
    {
        set_speed(target_speed);
    }
    // Otherwise, Stepper.run() will take care to set the correct velocities
}

// Switch to position mode. Target position in steps from power-up/reset position
// TODO: handle overflows (what if we want to travel so far that the tick counter wraps around?)
// TODO: handle overshoot (what if we have current vel > 0 and target < current pos, or we can't brake fast enough)
void FastStepper_t::set_target_position(int32_t target_position)
{
    // Switch to target position mode
    _position_mode = true;
    _has_active_target = false;

    // Target is already reached
    if(_current_position == target_position)
    {
        return;
    }

    // Acceleration 0 means instant speed change
    if(_acceleration == 0)
    {
        _decelerate_at = target_position;
    }
    else // we have a max accel/decel
    {
        // Save current position and velocity to be able to calculate desired velocities later
        _pos_run_initial_pos = _current_position;
        _pos_run_initial_speed = _current_speed;

        // Calculate steps needed for deceleration
        float d = abs(target_position - _current_position);
        float v_max = sqrt(_pos_run_initial_speed*_pos_run_initial_speed + _acceleration*d); // 2*a*(d/2) = a*d
        v_max = min(v_max, _max_speed);
        int32_t deceleration_steps = ceil(v_max*v_max / (2 * _acceleration));
        if(target_position > _current_position)
        {
            _decelerate_at = target_position - deceleration_steps;
        }
        else
        {
            _decelerate_at = target_position + deceleration_steps;
        }
    }

    // Start travel towards target position
    _target_speed = (target_position > _current_position)? _max_speed : -_max_speed;
    _target_position = target_position;
    _has_active_target = true;

    // Instant speed change
    if(_acceleration == 0)
    {
        set_speed(_target_speed);
    }
    // Otherwise, Stepper.run() will take care to set the correct velocities
}

void FastStepper_t::set_speed(int32_t new_speed)
{
    uint8_t TCCR1B_tmp = TCCR1B;
    TCCR1B_tmp &= 0b11111000; // clear lower 3 bits

    int32_t new_speed_tmp = new_speed;

    bool forward = new_speed_tmp > 0;
    if(!forward)
    {
        // Calculate with positive speeds.
        new_speed_tmp = -new_speed_tmp;
    }
    // we need two ISRs per step, so double the frequency
    new_speed_tmp *= 2;

    uint16_t top;
    if(new_speed_tmp >= 245) // (16e6/new_speed)/1<65535
    {
        // prescaler=1;
        TCCR1B_tmp |= 0b001;
        top = 16e6/new_speed_tmp - 1; // round would be better, but floor will do
    }
    else if(new_speed >= 31) // (16e6/new_speed)/8<65535
    {
        // prescaler=8;
        TCCR1B_tmp |= 0b010;
        top = 2e6/new_speed_tmp - 1;
    }
    else if(new_speed >= 4) // (16e6/new_speed)/64<65535
    {
        // prescaler=64;
        TCCR1B_tmp |= 0b011;
        top = 250000/new_speed_tmp - 1;
    }
    else if(new_speed >= 1) // (16e6/new_speed)/256<65535
    {
        // prescaler=256;
        TCCR1B_tmp |= 0b100;
        top = 62500/new_speed_tmp - 1;
    }
    else
    {
        // we can't get a speed this low. timer will be stopped (TCCR1B_tmp & 0x7 = 0).
        top = 0xFFFF;
    }

    // Now we actually set some outputs. Temporarily block interrupts to prevent race condition
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        // Only set speed if (1) we're in velocity mode or (2) we're in position mode and are currently moving towards a target
        if(!_position_mode || (_position_mode && _has_active_target))
        {
            // set direction pin
            if(forward)
            {
                *_pin_direction_port |= _pin_direction_bitmask;
                _stepping_direction = 1;
            }
            else
            {
                *_pin_direction_port &= ~_pin_direction_bitmask;
                _stepping_direction = -1;
            }

            // set new timer frequency.
            // set new prescaler setting
            TCCR1B = TCCR1B_tmp;
            // Since we're in Fast PWM mode, OCR1A defines TOP. Setting a new OCR1A
            // that is lower than TCNT1 will thus set TOV1 and reset TCNT1.
            OCR1A = top;
        
            _current_speed = new_speed;

            Serial.print("New TCCR1B = 0b"); Serial.println(TCCR1B, 2);
            Serial.print("New OCR1A = 0x"); Serial.println(OCR1A, 16);
            Serial.print("New _current_speed = "); Serial.println(_current_speed, 10);
        }
    }
}

// This ISR is called on every OC1A toggle
ISR(TIMER1_COMPA_vect)
{
    FastStepper_t * const fs = FastStepper_t::instance_ptrs[FastStepper_t::TIMER_1_INSTANCE];
    // The ISR will execute for every edge, but a step occurs only at a rising edge
    // fs->_is_rising_edge_isr = ~fs->_is_rising_edge_isr;
    if(!(fs->_is_rising_edge_isr++ & 0x01))
    {
        return;
    }

    // Depending on the output state of the dir pin, increment or decrement a step
   fs->_current_position += fs->_stepping_direction;

    // If we're in (active) position mode
    if(fs->_position_mode && fs->_has_active_target)
    {
        fs->_current_position_virtual_offset_valid = false;
        fs->_time_of_last_step = millis();

        if(fs->_current_position == fs->_target_position)
        {
            // the below lines are faster than a call to set_speed(0)
            fs->_target_speed = 0;
            fs->_current_speed = 0;
            fs->_has_active_target = false;
            TCCR1B &= 0b11111000; // stop timer
        }
    }
}

void FastStepper_t::run_position_target()
{
    // save current values in some local temps
    int32_t current_speed;
    float current_position;
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        current_speed = _current_speed;
        current_position = _current_position;
    }

    // Nothing to do if we're at the target
    if(current_position == _target_position)
    {
        return;
    }

    // At low speeds, calc virtual position as if we didn't have discrete steps
    bool use_virtual_position_offset = current_speed != 0 && abs(current_speed) < 1000;
    if(use_virtual_position_offset)
    {
        // If a step ISR happened, reset the virtual position offset
        if(!_current_position_virtual_offset_valid)
        {
            _current_position_virtual_offset = 0;
            _current_position_virtual_offset_valid = true;
        }
        // Adjust virtual position offset
        float t_elapsed = (millis() - _time_of_last_step) / 1000.0;
        _current_position_virtual_offset += _current_speed * t_elapsed;
        current_position += _current_position_virtual_offset;
    }

    // Calculate speed
    float required_speed = 0;
    // Are we past the break point?
    if ((current_speed > 0 && current_position >= _decelerate_at) ||
        (current_speed < 0 && current_position <= _decelerate_at) )
    {
        float d = abs(_target_position - current_position);
        required_speed = sqrt(2.0 * _acceleration * d);
    }
    // We are not at the deceleration point yet, so accelerate further
    else
    {
        float d = abs(current_position - _pos_run_initial_pos);
        required_speed = sqrt(_pos_run_initial_speed * _pos_run_initial_speed + 2.0 * _acceleration * d);
    }
    // Round the speed to an integer steps/sec. Don't go below 1 (or we will never arrive)
    int32_t required_speed_int = (required_speed < 1)? 1 : round(required_speed);
    // Also don't go faster than allowed
    required_speed_int = min(required_speed_int, _max_speed);
    // Give correct direction (value will always be positive after the sqrt)
    required_speed_int *= (_target_position - _current_position > 0)? 1 : -1;
    // Set speed
    set_speed(required_speed_int);
}

void FastStepper_t::run_speed_target()
{
    uint32_t now = millis();
    float t_elapsed = (float) (now - _speed_run_last_acceleration_adjustment_time) / 1000.0;
    _speed_run_last_acceleration_adjustment_time = now;

    // Do we need higher/more positive speed?
    if(_target_speed > _current_speed)
    {
        float new_speed = _speed_run_current_speed_float + _acceleration * t_elapsed;
        // Limit to max positive speed
        new_speed = min(new_speed, _target_speed);
        _speed_run_current_speed_float = new_speed;
        set_speed(round(new_speed));
    }

    // Do we need lower/more negative speed?
    else if(_target_speed < _current_speed)
    {
        float new_speed = _speed_run_current_speed_float - _acceleration * t_elapsed;
        // Limit to max negative speed
        new_speed = max(new_speed, _target_speed);
        _speed_run_current_speed_float = new_speed;
        set_speed(round(new_speed));
    }
}

void FastStepper_t::run()
{
    // Are we in target position mode?
    if(_position_mode && _has_active_target)
    {
        run_position_target();
    }
    else // we are in target speed mode
    {
        run_speed_target();
    }
}
