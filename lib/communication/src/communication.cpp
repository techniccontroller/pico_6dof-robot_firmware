#include "communication.h"
#include <stdlib.h>



Communication::Communication(AccelStepper *stepper1, AccelStepper *stepper2, AccelStepper *stepper3, AccelStepper *stepper4, AS5600 *encoder1, AS5600 *encoder2)
{
    _stepper1 = stepper1;
    _stepper2 = stepper2;
    _stepper3 = stepper3;
    _stepper4 = stepper4;
    _encoder1 = encoder1;
    _encoder2 = encoder2;
}

bool Communication::starts_with(const char *pre, const char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}

bool Communication::contains(const char *substring, const char *str)
{
    return strstr(str, substring) != NULL;
}

uint8_t Communication::extract_related_motor(char *cmd)
{
    uint8_t result = 99;

    if (contains("M1", (const char*)cmd))
    {
        result = M1;
    }
    else if (contains("M2", (const char*)cmd))
    {
        result = M2;
    }
    else if (contains("M3", (const char*)cmd))
    {
        result = M3;
    }
    else if (contains("M4", (const char*)cmd))
    {
        result = M4;
    }

    return result;
}


int Communication::extract_cmd_value(const char *cmd)
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

void Communication::process_cmd(char *cmd)
{
    if (_manual_drive)
    {
        uint8_t motor = extract_related_motor(cmd);

        
        if (contains("INIT", cmd))
        {
            // TODO: initialize all motors
            comm_func_write("MOTOR is initialized\n");
            _encoder1->setZero();
            _encoder2->setZero();
        }
        else
        {
            long speed = 0;
            long steps = 0xFFFFFF;
            if (contains("END", cmd))
            {
                steps = 0;
                speed = 200;
            }
            if (contains("BACKWARD_START", cmd))
            {
                speed = abs(extract_cmd_value(cmd));
            }
            if (contains("FORWARD_START", cmd))
            {
                speed = abs(extract_cmd_value(cmd));
                steps *= -1;
            }

            switch (motor)
            {
            case M1:
                if(_stepper1 != NULL){
                    if(steps == 0){
                        _stepper1->stop();
                        _stepper2->stop();
                    }
                    else{
                        _stepper1->setMaxSpeed(speed);
                        _stepper1->setAcceleration(500.0);
                        _stepper1->move(steps);
                        _stepper2->setMaxSpeed(speed);
                        _stepper2->setAcceleration(500.0);
                        _stepper2->move(steps);
                    }
                }
                break;
            
            case M2:
                if(_stepper2 != NULL){
                    if(steps == 0){
                        _stepper2->stop();
                    }
                    else{
                        _stepper2->setMaxSpeed(speed);
                        _stepper2->setAcceleration(500.0);
                        _stepper2->move(steps);
                    }
                }
                break;
            
            case M3:
                if(_stepper3 != NULL){
                    if(steps == 0){
                        _stepper3->stop();
                    }
                    else{
                        _stepper3->setMaxSpeed(speed);
                        _stepper3->setAcceleration(500.0);
                        _stepper3->move(steps);
                    }
                }
                break;
            
            case M4:
                if(_stepper4 != NULL){
                    if(steps == 0){
                        _stepper4->stop();
                    }
                    else{
                        _stepper4->setMaxSpeed(speed);
                        _stepper4->setAcceleration(500.0);
                        _stepper4->move(steps);
                    }
                }
                break;
            
            default:
                break;
            }
            
            char str_buffer[20];
            sprintf(str_buffer, "motor: %d, spd: %d, step: %d", motor, speed, steps);
            comm_func_write(str_buffer);
        }
    }

    if (starts_with("ENABLE_MANUAL", cmd))
    {
        comm_func_write("enable manual drive\n");
        _manual_drive = true;
    }
    else if (starts_with("DISABLE_MANUAL", cmd))
    {
        comm_func_write("disable manual drive\n");
        _manual_drive = false;
    }
}

void Communication::check_incoming_cmds()
{
    uint16_t data_len = comm_func_read_all_bytes((uint8_t*)buffer, BUFFER_SIZE);
    if (data_len > 0)
    {
        buffer[data_len] = '\0';
        comm_func_write("data received: ");
        comm_func_write(buffer);

        process_cmd(buffer);
    }
}