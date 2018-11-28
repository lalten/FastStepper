# FastStepper
FastStepper is a "hardware-accelerated" stepper driver library.

Advantages:
* Very high step frequencies possible
* Acceleration and deceleration
* Position and speed control

Disadvantages:
* Supports only one or two steppers. The limit comes from the number of hardware-PWM capable pins.
* Targets the SparkFun ProMicro board. It should be easily possible to port to other boards though.
