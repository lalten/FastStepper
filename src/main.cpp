#include <Arduino.h>

#include <FastStepper.h>

const char CMD_WRITE_ACCEL = 'a';
const char CMD_WRITE_MAX_SPEED = 'm';
const char CMD_WRITE_TARGET_ABS_POS = 'T';
const char CMD_WRITE_TARGET_REL_POS = 't';
const char CMD_WRITE_TARGET_ABS_VEL = 'S';
const char CMD_WRITE_TARGET_REL_VEL = 's';
const char CMD_WRITE_QUICKSTOP = 'q';
const char CMD_READ_ABS_POS = 'P';
const char CMD_READ_REL_POS = 'p';
const char CMD_READ_ABS_VEL = 'C';
const char CMD_READ_REL_VEL = 'c';

char serial_buffer[255] = {0};
uint8_t serial_buffer_len = 0;

int32_t last_target_vel = 0;
int32_t last_target_pos = 0;

void interpret_command(char command, int32_t value);

void setup()
{
    Serial.begin(0);
    FastStepper.attach();

    // Set reasonable defaults
    FastStepper.set_max_speed(40000);
    FastStepper.set_acceleration(5000);
}

void loop()
{
    FastStepper.run();

    while(Serial.available())
    {
        char c = Serial.read();
        if(c != '\n')
        {
            serial_buffer[serial_buffer_len++] = c;
        }
        else
        {
            serial_buffer[serial_buffer_len] = '\0';
            serial_buffer_len = 0;
            char command;
            int32_t value;
            sscanf(serial_buffer, "%1c,%ld", &command, &value);
            interpret_command(command, value);
        }
    }
}

void interpret_command(char command, int32_t value)
{
    char outbuf[255];
    switch(command)
    {
        case CMD_WRITE_ACCEL: // write/read acceleration
            FastStepper.set_acceleration(value);
            value = FastStepper.get_acceleration();
            break;
        case CMD_WRITE_MAX_SPEED: // write/read max speed
            FastStepper.set_max_speed(value);
            value = FastStepper.get_max_speed();
            break;
        case CMD_WRITE_QUICKSTOP: // quickstop
            FastStepper.quickstop();
            value =  FastStepper.get_current_position();
            break;
        case CMD_READ_ABS_VEL: // read current speed
            value = FastStepper.get_current_speed();
            break;
        case CMD_READ_REL_VEL: // read current speed
            value = FastStepper.get_current_speed() - last_target_vel;
            break;
        case CMD_READ_ABS_POS: // read current position
            value =  FastStepper.get_current_position();
            break;
        case CMD_READ_REL_POS: // read current speed
            value = FastStepper.get_current_position() - last_target_pos;
            break;
        case CMD_WRITE_TARGET_ABS_VEL: // write/read absolute target speed
            FastStepper.set_target_speed(value);
            last_target_pos = FastStepper.get_target_speed();
            value = last_target_pos;
            break;
        case CMD_WRITE_TARGET_REL_VEL: // write/read target speed relative to current speed
            last_target_vel = FastStepper.get_target_speed();
            FastStepper.set_target_speed(last_target_vel + value);
            value = FastStepper.get_target_speed() - last_target_vel;
            break;
        case CMD_WRITE_TARGET_ABS_POS: // write/read absolute target position
            FastStepper.set_target_position(value);
            last_target_pos = FastStepper.get_target_position();
            value = last_target_pos;
            break;
        case CMD_WRITE_TARGET_REL_POS: // write/read target position relative to current position
            last_target_pos = FastStepper.get_target_position();
            FastStepper.set_target_position(last_target_pos + value);
            value = FastStepper.get_target_position() - last_target_pos;
            break;
        default:
            Serial.print("unknown command \"");
            Serial.print(command);
            Serial.print("\", value \"");
            Serial.print(value);
            Serial.print("\" in string \"");
            Serial.print(serial_buffer);
            Serial.print("\"\n");
            return;
        }
        sprintf(outbuf, "%1c,%ld\n", command, value);
        Serial.print(outbuf);
}