#include "communication.h"
#include <stdlib.h>


Communication::Communication(Controller *controller, StepperConfiguration *stepper_config)
{
    _controller = controller;
    _stepper_config = stepper_config;
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


std::vector<float> Communication::extract_cmd_values(const char *cmd)
{
    std::vector<float> values;
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

            // split value_str by ',' and save to array
            char *token = strtok(value_str, ",");
            char *values_str[2];
            int i = 0;
            while (token != NULL)
            {
                values_str[i] = token;
                token = strtok(NULL, ",");
                i++;
            }

            // convert values_str to float and save to vector
            for (int k = 0; k < i; k++)
            {
                float value = atof(values_str[k]);
                values.push_back(value);
                if (DEBUG_IS_ENABLED)
                {
                    char str_buffer[40];
                    sprintf(str_buffer, "received value: %s -> %f\n", value_str, value);
                    comm_func_write(str_buffer);
                }
            }
        }
    }
    return values;
}

void Communication::process_cmd(char *cmd)
{
    if (_manual_drive)
    {
        uint8_t motor = extract_related_motor(cmd);

        
        if (contains("_INIT", cmd))
        {
            switch(motor){
                case M1:
                    _controller->initializeM1();
                    comm_func_write("Init MOTOR M1\n");
                    break;
                case M2:
                    _controller->initializeM2();
                    comm_func_write("Init MOTOR M2\n");
                    break;
                case M3:
                    _controller->initializeM3();
                    comm_func_write("Init MOTOR M3\n");
                    break;

                default:    
                    break;

            }
        }
        else if (contains("_ZERO", cmd))
        {
            switch(motor){
                case M1:
                    if(_controller->getE1() != NULL){
                        _controller->getE1()->setZero();
                    }
                    comm_func_write("MOTOR M1 is initialized\n");
                    break;
                case M2:
                    if(_controller->getE2() != NULL){
                        _controller->getE2()->setZero();
                    }
                    comm_func_write("MOTOR M2 is initialized\n");
                    break;
                case M3:
                    if(_controller->getE3() != NULL){
                        _controller->getE3()->setZero();
                    }
                    comm_func_write("MOTOR M3 is initialized\n");
                    break;
                default:    
                    break;

            }
        }
        else if (contains("_SET", cmd))
        {
            switch(motor){
                case M1:
                    _controller->setM1Position(extract_cmd_values(cmd)[0]);
                    comm_func_write("MOTOR M1 is set\n");
                    break;
                case M2:
                    _controller->setM2Position(extract_cmd_values(cmd)[0]);
                    comm_func_write("MOTOR M2 is set\n");
                    break;
                case M3:
                    _controller->setM3Position(extract_cmd_values(cmd)[0]);
                    comm_func_write("MOTOR M3 is set\n");
                    break;
                default:    
                    break;
            }
        }
        else
        {
            long speed = 0;
            if (contains("END", cmd))
            {
                speed = 0;
            }
            if (contains("BACKWARD_START", cmd))
            {
                speed = abs(extract_cmd_values(cmd)[0]);
            }
            if (contains("FORWARD_START", cmd))
            {
                speed = abs(extract_cmd_values(cmd)[0]);
                speed *= -1;
            }

            switch (motor)
            {
            case M1:
                _controller->setM1Velocity(speed);
                break;
            
            case M2:
                _controller->setM2Velocity(speed);
                break;
            
            case M3:
                _controller->setM3Velocity(speed);
                break;
            
            default:
                break;
            }
            
            char str_buffer[20];
            sprintf(str_buffer, "motor: %d, spd: %d", motor, speed);
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
    else if (starts_with("COORD", cmd))
    {
        std::vector<float> values = extract_cmd_values(cmd);
        std::vector<float> jointAngles = _stepper_config->inverseKinematics(values[0], values[1], 0);
        printf("jointAngles: %f, %f, %f\n", jointAngles[0], jointAngles[1], jointAngles[2]);

        long absolute[3];
        absolute[0] = _stepper_config->angleRadToSteps(jointAngles[0]);
        absolute[1] = _stepper_config->angleRadToSteps(-1*jointAngles[1]);
        absolute[2] = _stepper_config->angleRadToSteps(jointAngles[2]);

        long distance1 = absolute[0] - _controller->getM1()->currentPosition();
        float time1 = abs(distance1) / 1000.0;
        
        long distance2 = absolute[1] - _controller->getM2()->currentPosition();
        float time2 = abs(distance2) / 1000.0;
        
        long distance3 = absolute[2] - _controller->getM3()->currentPosition();
        float time3 = abs(distance3) / 1000.0;

        float longestTime = time1;
        if (time2 > longestTime){
            longestTime = time2;
        }
        if (time3 > longestTime){
            longestTime = time3;
        }

        if (longestTime > 0.0) {
            // Now work out a new max speed for each stepper so they will all 
            // arrived at the same time of longestTime
            float speed1 = distance1 / longestTime;
            _controller->setM1PositionVelocity(absolute[0], speed1);

            float speed2 = distance2 / longestTime;
            _controller->setM2PositionVelocity(absolute[1], speed2);

            float speed3 = distance3 / longestTime;
            _controller->setM3PositionVelocity(absolute[2], speed3);
        }
        comm_func_write("COORD is set\n");
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