#include "Communication.h"
#include <stdlib.h>


Communication::Communication(Robot *robot)
{
    m_robot = robot;
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
    uint8_t result = Motor::MNONE;

    if (contains("M1", (const char*)cmd))
    {
        result = Motor::M1;
    }
    else if (contains("M2", (const char*)cmd))
    {
        result = Motor::M2;
    }
    else if (contains("M3", (const char*)cmd))
    {
        result = Motor::M3;
    }
    else if (contains("M4", (const char*)cmd))
    {
        result = Motor::M4;
    }
    else if (contains("M5", (const char*)cmd))
    {
        result = Motor::M5;
    }

    return result;
}

uint8_t Communication::extract_related_joint(char *cmd)
{
    uint8_t result = Joint::JNONE;

    if (contains("J1", (const char*)cmd))
    {
        result = Joint::J1;
    }
    else if (contains("J2", (const char*)cmd))
    {
        result = Joint::J2;
    }
    else if (contains("J3", (const char*)cmd))
    {
        result = Joint::J3;
    }
    else if (contains("J4", (const char*)cmd))
    {
        result = Joint::J4;
    }
    else if (contains("J5", (const char*)cmd))
    {
        result = Joint::J5;
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
            char value_str[20] = {'\0'};
            start_char++;
            for (int i = 0; start_char != end_char; i++)
            {
                value_str[i] = *start_char;
                start_char++;
            }

            // split value_str by ',' and save to array
            char *token = strtok(value_str, ",");
            char *values_str[5];
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
                    sprintf(str_buffer, "received value: %s -> %f\n", values_str[k], value);
                    comm_func_write(str_buffer);
                }
            }
        }
    }
    return values;
}

void Communication::process_cmd(char *cmd)
{
    if (starts_with("CMD_", cmd))
    {
        uint8_t joint = extract_related_joint(cmd);
        uint8_t motor = extract_related_motor(cmd);

        if(joint != Joint::JNONE){
            if (contains("_INIT", cmd))
            {
                m_robot->initJoint(joint);
                char str_buffer[40];
                sprintf(str_buffer, "Init Joint J%d\n", joint);
                comm_func_write(str_buffer);
            }
            else if (contains("_ZERO", cmd))
            {
                m_robot->zeroJoint(joint);
                char str_buffer[40];
                sprintf(str_buffer, "Zero Joint J%d\n", joint);
                comm_func_write(str_buffer);
            }
            else if (contains("_SET", cmd))
            {
                m_robot->setJointPosition(joint, extract_cmd_values(cmd)[0]);
            }
            else
            {
                float speed = 0;
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

                m_robot->setJointVelocity(joint, speed);
                
                char str_buffer[20];
                sprintf(str_buffer, "Set Velocity, joint: %d, spd: %d", joint, speed);
                comm_func_write(str_buffer);
            }
        } else if(motor != Motor::MNONE){
            float speed = 0;
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

            m_robot->setMotorVelocity(motor, speed);
            
            char str_buffer[20];
            sprintf(str_buffer, "Set Velocity, motor: %d, spd: %f", motor, speed);
            comm_func_write(str_buffer);
        }
        
    }
    else if(starts_with("SET_MODE_AUTO", cmd))
    {
        comm_func_write("set mode auto\n");
        m_robot->setMode(Robot::RobotMode::AUTO);
    }
    else if(starts_with("SET_MODE_MOTOR", cmd))
    {
        comm_func_write("set mode motor\n");
        m_robot->setMode(Robot::RobotMode::MOTORCONTROL);
    }
    else if(starts_with("SET_MODE_JOINT", cmd))
    {
        comm_func_write("set mode joint\n");
        m_robot->setMode(Robot::RobotMode::JOINTCONTROL);
    }
    else if (starts_with("COORD", cmd))
    {
        std::vector<float> pose = extract_cmd_values(cmd);

        m_robot->moveToPose(pose);
        comm_func_write("COORD is set\n");
    }
    else if (starts_with("PID_J4", cmd))
    {
        std::vector<float> values = extract_cmd_values(cmd);

        m_robot->setPID(Joint::J4, values[0], values[1], values[2]);
        char str_buffer[20];
        sprintf(str_buffer, "J4 PID set: %f, %f, %f", values[0], values[1], values[2]);
        comm_func_write(str_buffer);
    }
    else if (starts_with("PID_J5", cmd))
    {
        std::vector<float> values = extract_cmd_values(cmd);

        m_robot->setPID(Joint::J5, values[0], values[1], values[2]);
        char str_buffer[20];
        sprintf(str_buffer, "J5 PID set: %f, %f, %f", values[0], values[1], values[2]);
        comm_func_write(str_buffer);
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