#include "communication.h"
#include <stdlib.h>


Communication::Communication(Controller *controller, 
                                AccelStepper *stepper1, 
                                AccelStepper *stepper2, 
                                AccelStepper *stepper3, 
                                AccelStepper *stepper4, 
                                AS5600 *encoder1, 
                                AS5600 *encoder2,
                                StepperConfiguration *stepper_config)
{
    _stepper1 = stepper1;
    _stepper2 = stepper2;
    _stepper3 = stepper3;
    _stepper4 = stepper4;
    _encoder1 = encoder1;
    _encoder2 = encoder2;
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
                default:    
                    break;

            }
        }
        else if (contains("_ZERO", cmd))
        {
            switch(motor){
                case M1:
                    _encoder1->setZero();
                    comm_func_write("MOTOR M1 is initialized\n");
                    break;
                case M2:
                    _encoder2->setZero();
                    comm_func_write("MOTOR M2 is initialized\n");
                    break;
                default:    
                    break;

            }
        }
        else if(contains("COORD", cmd))
        {
            std::vector<float> values = extract_cmd_values(cmd);
            std::vector<float> jointAngles = _stepper_config->inverseKinematics(values[0], values[1], 0);
            printf("jointAngles: %f, %f\n", jointAngles[0], jointAngles[1]);
            _controller->setM1Position(_stepper_config->angleRadToSteps(jointAngles[0]));
            _controller->setM2Position(_stepper_config->angleRadToSteps(-1*jointAngles[1]));
            comm_func_write("COORD is set\n");
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
                // none
                break;
            
            case M4:
                // none
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