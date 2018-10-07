#include <Arduino.h>

#include <avr/io.h>
#include <util/atomic.h>

#include "FastStepper.h"

uint8_t *FastStepper_t::pin_dir_A_DDR;
uint8_t *FastStepper_t::pin_dir_A_PORT;
uint8_t FastStepper_t::pin_dir_BITMASK;
float FastStepper_t::acceleration = 100;
int32_t FastStepper_t::max_speed = 8000000;
volatile int32_t FastStepper_t::target_speed = 0;
volatile int32_t FastStepper_t::target_position = 0;
volatile int32_t FastStepper_t::decelerate_at = 0;
volatile int32_t FastStepper_t::current_position = 0;
volatile float FastStepper_t::current_position_virtual_offset = 0;
volatile bool FastStepper_t::current_position_virtual_offset_valid = false;
volatile int32_t FastStepper_t::current_speed = 0;
volatile bool FastStepper_t::position_mode = false;
volatile bool FastStepper_t::has_active_target = false;
volatile bool FastStepper_t::is_rising_edge_isr = true;
volatile uint32_t FastStepper_t::time_of_last_step = 0;
volatile int32_t FastStepper_t::pos_run_initial_pos = 0;
volatile int32_t FastStepper_t::pos_run_initial_speed = 0;
float FastStepper_t::speed_run_current_speed_float = 0;
uint32_t FastStepper_t::speed_run_last_acceleration_adjustment_time = 0;

// Set up Stepper A. Pulse pin must be A9.
// void FastStepper_t::attach(int dir_pin)
void FastStepper_t::attach()
{
    // reasonable defaults:
    FastStepper.current_speed = 0;
    FastStepper.target_speed = 0;
    FastStepper.position_mode = false;
    FastStepper.set_acceleration(100);

    // Set up pins
    // Figure out what the direction pin is
    // uint8_t dir_port_nr = digital_pin_to_port_PGM[dir_pin];
    // FastStepper.pin_dir_BITMASK = digital_pin_to_bit_mask_PGM[dir_pin];
    // FastStepper.pin_dir_A_DDR = (uint8_t *) port_to_mode_PGM[dir_port_nr];
    // FastStepper.pin_dir_A_PORT = (uint8_t *) port_to_output_PGM[dir_port_nr];
    // Set direction pin as output
    // *FastStepper.pin_dir_A_DDR |= FastStepper.pin_dir_BITMASK;
    // We use PB4 (Arduino D8) as direction pin. Set to output.
    DDRB |= _BV(PB4);
    // We use PB5 (OCR1A) (Arduino D9) as step pin. Set to output.
    DDRB |= _BV(PB5);

    // Set up Timer 1, which will control Stepper A's pulses
    // Fast PWM mode (15) with TOP=OCRnA. No clock source.
    // Toggle OC1A on Compare Match, OC1B and OC1C disconnected (normal port operation)
    TCCR1B = _BV(WGM13) | _BV(WGM12);
    TCCR1A = _BV(WGM11) | _BV(WGM10) | _BV(COM1A0);
    // Interrupt on Output Compare Match A
    TIMSK1 = _BV(OCIE1A);

    // Set up Timer 4, which will control accelerations
    // No waveform generation
    TCCR4A = 0x00;
    TCCR4C = 0x00;
    // Normal mode, TOP=OCR4C
    TCCR4B &= 0b11110000; // Stop clock
    // TCCR4B |= 0b00001000; // Select prescaler 128 --> T/C4=125kHz
    // Set Output Compare Register C
    OCR4C = 124; // --> f=1kHz
    // Enable Overflow Interrupt
    TIMSK4 = _BV(TOIE4);

    // Enable interrupts
    sei();
}

void FastStepper_t::quickstop()
{
    FastStepper.set_speed(0);
    FastStepper.position_mode = false;
    FastStepper.has_active_target = false;
    FastStepper.target_speed = 0;
    FastStepper.speed_run_last_acceleration_adjustment_time = 0;
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

    FastStepper.max_speed = max_speed;

    // Limit target_speed to -max_speed..max_speed
    if(FastStepper.target_speed > FastStepper.max_speed)
    {
        FastStepper.target_speed = FastStepper.max_speed;
    }
    else if(FastStepper.target_speed < -FastStepper.max_speed)
    {
        FastStepper.target_speed = -FastStepper.max_speed;
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
    FastStepper.acceleration = acceleration;
}

// Switch to velocity mode. Target speed is in steps/sec
void FastStepper_t::set_target_speed(int32_t target_speed)
{
    // Switch to target speed mode
    FastStepper.position_mode = false;
    FastStepper.has_active_target = false;
    FastStepper.speed_run_current_speed_float = FastStepper.current_speed;

    // Limit target_speed to -max_speed..max_speed
    if(target_speed > FastStepper.max_speed)
    {
        target_speed = FastStepper.max_speed;
    }
    else if(target_speed < -FastStepper.max_speed)
    {
        target_speed = -FastStepper.max_speed;
    }

    // If the target speed changed, act
    if(target_speed != FastStepper.current_speed)
    {
        FastStepper.target_speed = target_speed;
        // No acceleration needed
        if(FastStepper.acceleration == 0)
        {
            FastStepper.set_speed(target_speed);
        }
        else
        {
            // Start Timer 4 to set speeds according to acceleration
            // TCNT4 = OCR4C; // make sure we'll get an interrupt asap
            // TCCR4B |= 0b00001000; // Select prescaler 128 --> T/C4=125kHz
        }
    }
}

// Switch to position mode. Target position in steps from power-up/reset position
// TODO: handle overflows (what if we want to travel so far that the tick counter wraps around?)
// TODO: handle overshoot (what if we have current vel > 0 and target < current pos, or we can't brake fast enough)
void FastStepper_t::set_target_position(int32_t target_position)
{
    // Switch to target position mode
    FastStepper.position_mode = true;
    FastStepper.has_active_target = false;

    // Target is already reached
    if(FastStepper.current_position == target_position)
    {
        return;
    }

    // Acceleration 0 means instant speed change
    if(FastStepper.acceleration == 0)
    {
        FastStepper.decelerate_at = target_position;
    }
    else // we have a max accel/decel
    {
        // Save current position and velocity to be able to calculate desired velocities later
        FastStepper.pos_run_initial_pos = FastStepper.current_position;
        FastStepper.pos_run_initial_speed = FastStepper.current_speed;

        // Calculate steps needed for deceleration
        int32_t deceleration_steps = 0;
        float a = FastStepper.acceleration;
        float v_initial = FastStepper.pos_run_initial_speed;
        int32_t current_position = FastStepper.current_position;
        float d = abs(target_position - current_position);
        float v_max = sqrt(v_initial*v_initial + a*d); // 2*a*(d/2) = a*d
        float v_limit = FastStepper.max_speed;
        v_max = min(v_max, v_limit);
        deceleration_steps = ceil(v_max*v_max / (2 * a));
        if(target_position > current_position)
        {
            FastStepper.decelerate_at = target_position - deceleration_steps;
        }
        else
        {
            FastStepper.decelerate_at = target_position + deceleration_steps;
        }

        // Serial.print("target_position="); Serial.print(target_position);
        // Serial.print("\nv_initial="); Serial.print(v_initial);
        // Serial.print("\nacceleration="); Serial.print(a);
        // Serial.print("\ncurrent_position="); Serial.print(current_position);
        // Serial.print("\nd="); Serial.print(d);
        // Serial.print("\nv_max="); Serial.print(sqrt(v_initial*v_initial + a*d));
        // Serial.print("\nv_limit="); Serial.print(v_limit);
        // Serial.print("\ndeceleration_steps="); Serial.print(deceleration_steps);
        // Serial.print("\ndecelerate_at="); Serial.print(FastStepper.decelerate_at);
        // Serial.print("\n");
    }

    // Start travel towards target position
    FastStepper.target_speed = (target_position > current_position)? FastStepper.max_speed : -FastStepper.max_speed;
    FastStepper.target_position = target_position;
    FastStepper.has_active_target = true;

    // Instant speed change
    if(FastStepper.acceleration == 0)
    {
        FastStepper.set_speed(FastStepper.target_speed);
    }
    // Otherwise, Stepper.run() will take care to set the correct velocities

    // else
    // {
        // Start Timer 4 to set speeds according to acceleration
        // TCNT4 = OCR4C; // make sure we'll get an interrupt asap
        // TCCR4B |= 0b00001000; // Select prescaler 128 --> T/C4=125kHz
    // }
}

void FastStepper_t::set_speed(int32_t new_speed)
{
    // if(FastStepper.position_mode && abs(FastStepper.current_speed) < 1000)
    // {
    //     if(!FastStepper.current_position_virtual_offset_valid)
    //     {
    //         FastStepper.current_position_virtual_offset = 0;
    //         FastStepper.current_position_virtual_offset_valid = true;
    //     }
    //     FastStepper.current_position_virtual_offset += FastStepper.current_speed * (float) (millis() - FastStepper.time_of_last_step) / 1000.0;
    // }
    // FastStepper.time_of_last_step = millis();

    uint8_t TCCR1B_tmp = TCCR1B;
    TCCR1B_tmp &= 0b11111000; // clear lower 3 bits

    // set direction pin
    if(new_speed < 0)
    {
        // *FastStepper.pin_dir_A_PORT &= ~FastStepper.pin_dir_BITMASK;
        PORTB &= ~_BV(PB4);
        // Direction pin is set, calculate with positive speeds now.
        new_speed = -new_speed;
    }
    else if(new_speed > 0)
    {
        // *FastStepper.pin_dir_A_PORT |= FastStepper.pin_dir_BITMASK;
        PORTB |= _BV(PB4);
    }
    else // speed 0
    {
        TCCR1B = TCCR1B_tmp; // stop timer
        return;
    }

    // we need two ISRs per step, so double the frequency
    new_speed *= 2;

    uint16_t top;
    if(new_speed >= 245) // (16e6/new_speed)/1<65535
    {
        // prescaler=1;
        TCCR1B_tmp |= 0b001;
        top = 16e6/new_speed - 1; // round would be better, but floor will do
    }
    else if(new_speed >= 31) // (16e6/new_speed)/8<65535
    {
        // prescaler=8;
        TCCR1B_tmp |= 0b010;
        top = 2e6/new_speed - 1;
    }
    else if(new_speed >= 4) // (16e6/new_speed)/64<65535
    {
        // prescaler=64;
        TCCR1B_tmp |= 0b011;
        top = 250000/new_speed - 1;
    }
    else if(new_speed >= 1) // (16e6/new_speed)/256<65535
    {
        // prescaler=256;
        TCCR1B_tmp |= 0b100;
        top = 62500/new_speed - 1;
    }
    else
    {
        // we can't get a speed this low. stop the timer.        
        TCCR1B = TCCR1B_tmp;
        return;
    }

    // float f_real = 16e6 / prescaler / top;

    // Temporarily block interrupts to prevent race condition
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        // Only set speed if (1) we're in velocity mode or (2) we're in position mode and are currently moving towards a target
        if(!FastStepper.position_mode ^  FastStepper.has_active_target)
        {
            // set new prescaler setting
            TCCR1B = TCCR1B_tmp;
            // set new timer frequency.
            // Since we're in Fast PWM mode, OCR1A defines TOP. Setting a new OCR1A
            // that is lower than TCNT1 will thus set TOV1 and reset TCNT1.
            OCR1A = top;
        
            FastStepper.current_speed = new_speed / 2; // (we multiplied it by 2 before)
        }
    }
}

// This ISR is called on every OC1A toggle
ISR(TIMER1_COMPA_vect)
{
    // The ISR will execute for every edge, but a step occurs only at a rising edge
    FastStepper.is_rising_edge_isr = !FastStepper.is_rising_edge_isr;
    if(!FastStepper.is_rising_edge_isr)
    {
        return;
    }

    // Depending on the output state of the dir pin, increment or decrement a step
    // if(*FastStepper.pin_dir_A_PORT & FastStepper.pin_dir_BITMASK)
    if(PORTB & _BV(PB4))
    {
        FastStepper.current_position++;
    }
    else
    {
        FastStepper.current_position--;
    }

    // If we're in (active) position mode
    if(FastStepper.position_mode && FastStepper.has_active_target)
    {
        // Serial.print("tick "); Serial.print(FastStepper.current_position); Serial.print("\n");
        
        FastStepper.current_position_virtual_offset_valid = false;
        FastStepper.time_of_last_step = millis();

        if(FastStepper.current_position == FastStepper.target_position)// ||
            // (FastStepper.pos_run_initial_pos < FastStepper.target_position && FastStepper.current_position > FastStepper.target_position) ||
            // (FastStepper.pos_run_initial_pos > FastStepper.target_position && FastStepper.current_position < FastStepper.target_position) )
        {
            // the below lines are faster than a call to set_speed(0)
            FastStepper.target_speed = 0;
            FastStepper.current_speed = 0;
            FastStepper.has_active_target = false;
            TCCR1B &= 0b11111000; // stop timer
        }
    }
}

// This ISR is called at 1kHz rate until the target speed has been reached.
ISR(TIMER4_OVF_vect)
{
}

void FastStepper_t::run_position_target()
{
    // Nothing to do if we're at the target
    if(FastStepper.current_position == FastStepper.target_position)// ||
        // (FastStepper.pos_run_initial_pos < FastStepper.target_position && FastStepper.current_position > FastStepper.target_position) ||
        // (FastStepper.pos_run_initial_pos > FastStepper.target_position && FastStepper.current_position < FastStepper.target_position) )
    {
        return;
    }

    // save current values in some local temps
    int32_t current_speed;
    float current_position;
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        current_speed = FastStepper.current_speed;
        current_position = FastStepper.current_position;
    }

    // At low speeds, calc virtual position as if we didn't have discrete steps
    bool use_virtual_position_offset = current_speed != 0 && abs(current_speed) < 1000;
    if(use_virtual_position_offset)
    {
        // If a step ISR happened, reset the virtual position offset
        if(!FastStepper.current_position_virtual_offset_valid)
        {
            FastStepper.current_position_virtual_offset = 0;
            FastStepper.current_position_virtual_offset_valid = true;
        }
        // Adjust virtual position offset
        float t_elapsed = (millis() - FastStepper.time_of_last_step) / 1000.0;
        FastStepper.current_position_virtual_offset += FastStepper.current_speed * t_elapsed;
        current_position += FastStepper.current_position_virtual_offset;
    }

    // Calculate speed
    float required_speed = 0;
    // Are we past the break point?
    if ((current_speed > 0 && current_position >= FastStepper.decelerate_at) ||
        (current_speed < 0 && current_position <= FastStepper.decelerate_at) )
    {
        float d = abs(FastStepper.target_position - current_position);
        required_speed = sqrt(2.0 * FastStepper.acceleration * d);
    }
    // We are not at the deceleration point yet, so accelerate further
    else
    {
        float d = abs(current_position - FastStepper.pos_run_initial_pos);
        float v_initial = FastStepper.pos_run_initial_speed;
        required_speed = sqrt(v_initial * v_initial + 2.0 * FastStepper.acceleration * d);
    }
    // Round the speed to an integer steps/sec. Don't go below 1 (or we will never arrive)
    int32_t required_speed_int = (required_speed < 1)? 1 : round(required_speed);
    // Also don't go faster than allowed
    required_speed_int = min(required_speed_int, FastStepper.max_speed);
    // Give correct direction (value will always be positive after the sqrt)
    required_speed_int *= (FastStepper.target_position - FastStepper.current_position > 0)? 1 : -1;
    // Set speed
    FastStepper.set_speed(required_speed_int);
}

void FastStepper_t::run_speed_target()
{
    uint32_t now = millis();
    float t_elapsed = (float) (now - FastStepper.speed_run_last_acceleration_adjustment_time) / 1000.0;
    FastStepper.speed_run_last_acceleration_adjustment_time = now;

    // Do we need higher/more positive speed?
    if(FastStepper.target_speed > FastStepper.current_speed)
    {
        float new_speed = FastStepper.speed_run_current_speed_float + FastStepper.acceleration * t_elapsed;
        // Limit to max positive speed
        new_speed = min(new_speed, FastStepper.target_speed);
        FastStepper.speed_run_current_speed_float = new_speed;
        FastStepper.set_speed(round(new_speed));

        // Serial.print("++ new speed "); Serial.print(new_speed); Serial.print("\n");
    }
    // Do we need lower/more negative speed?
    else if(FastStepper.target_speed < FastStepper.current_speed)
    {
        float new_speed = FastStepper.speed_run_current_speed_float - FastStepper.acceleration * t_elapsed;
        // Limit to max negative speed
        new_speed = max(new_speed, FastStepper.target_speed);
        FastStepper.speed_run_current_speed_float = new_speed;
        FastStepper.set_speed(round(new_speed));

        // Serial.print("-- new speed "); Serial.print(new_speed); Serial.print("\n");
    }
    // We've already reached target speed. we can stop the acceleration curve.
    else
    {
        // TCCR4B &= 0b11110000; // Stop clock
    }
}

void FastStepper_t::run()
{
    // Are we in target position mode?
    if(FastStepper.position_mode && FastStepper.has_active_target)
    {
        FastStepper.run_position_target();
    }
    else // we are in target speed mode
    {
        FastStepper.run_speed_target();
    }
}
