#include "communication.h"
#include <stdlib.h>

#define BUFFER_SIZE 30

bool g_manual_drive = false;
static uint8_t buffer[BUFFER_SIZE];

static bool starts_with(const char *pre, const char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}

static bool contains(const char *substring, const char *str)
{
    return strstr(str, substring) > 0;
}

static uint8_t extract_related_motor(uint8_t *cmd)
{
    uint8_t result = 99;

    if (contains("M1", cmd))
    {
        result = M1;
    }
    else if (contains("M2", cmd))
    {
        result = M2;
    }
    else if (contains("M3", cmd))
    {
        result = M3;
    }
    else if (contains("M4", cmd))
    {
        result = M4;
    }

    return result;
}

/**
 * @brief extract control value from command if available
 *
 * @param cmd given command, with a value in brackets 'MY_COMMAND(value)'
 * @return int
 */
int extract_cmd_value(const char *cmd)
{
    int value = 0;
    char cmd_copy[50];
    memset(cmd_copy, '\0', sizeof(cmd_copy));
    strcpy(cmd_copy, cmd);

    if (contains("(", cmd_copy))
    {
        char *start_char = strchr(cmd_copy, '(');
        char *end_char = strchr(cmd_copy, ')');

        if (start_char != NULL && end_char != NULL)
        {
            char value_str[10] = {'\0'};
            start_char++;
            for (int i = 0; start_char != end_char; i++)
            {
                value_str[i] = *start_char;
                start_char++;
            }

            value = atoi(value_str);

            if (DEBUG_IS_ENABLED)
            {
                char str_buffer[20];
                sprintf(str_buffer, "received value: %s -> %d", value_str, value);
                comm_func_write(str_buffer);
            }
        }
    }
    return value;
}

static void process_cmd(uint8_t *cmd)
{
    if (g_manual_drive)
    {
        uint8_t motor = extract_related_motor(cmd);

        
        if (contains("INIT", cmd))
        {
            // TODO: initialize all motors
            comm_func_write("MOTOR is initialized\n");
        }
        else
        {
            unsigned int speed = 0;
            bool dir = false;
            unsigned int steps = 0xFFFFFFFF;
            if (contains("END", cmd))
            {
                steps = 0;
                speed = 200;
            }
            if (contains("BACKWARD_START", cmd))
            {
                dir = true;
                speed = abs(extract_cmd_value(cmd));
            }
            if (contains("FORWARD_START", cmd))
            {
                dir = false;
                speed = abs(extract_cmd_value(cmd));
            }

            switch (motor)
            {
            case M1:
                setSpeedMotor1(speed);
                setDirectionMotor1(dir);
                moveStepsMotor1(steps, true);
                break;
            
            case M2:
                setSpeedMotor2(speed);
                setDirectionMotor2(dir);
                moveStepsMotor2(steps, true);
                break;
            
            case M3:
                setSpeedMotor3(speed);
                setDirectionMotor3(dir);
                moveStepsMotor3(steps, true);
                break;
            
            case M4:
                setSpeedMotor4(speed);
                setDirectionMotor4(dir);
                moveStepsMotor4(steps, true);
                break;
            
            default:
                break;
            }
            
            char str_buffer[20];
            sprintf(str_buffer, "motor: %d, spd: %d, dir: %d, step: %d", motor, speed, dir, steps);
            comm_func_write(str_buffer);
        }
    }

    if (starts_with("ENABLE_MANUAL", cmd))
    {
        comm_func_write("enable manual drive\n");
        g_manual_drive = true;
    }
    else if (starts_with("DISABLE_MANUAL", cmd))
    {
        comm_func_write("disable manual drive\n");
        g_manual_drive = false;
    }
}

void check_incoming_cmds()
{
    uint16_t data_len = comm_func_read_all_bytes(buffer, BUFFER_SIZE);
    if (data_len > 0)
    {
        buffer[data_len] = '\0';
        comm_func_write("data received: ");
        comm_func_write(buffer);

        process_cmd(buffer);
    }
}