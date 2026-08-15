#include "communication.h"
#include <stdlib.h>


Communication::Communication(queue_t *commandQueue)
{
    m_command_queue = commandQueue;
}

bool Communication::enqueue_command(const RobotCommand& command)
{
    if (queue_try_add(m_command_queue, &command))
    {
        return true;
    }

    comm_func_write("Command rejected: robot command queue is full\n");
    return false;
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
    else if (contains("M6", (const char*)cmd))
    {
        result = Motor::M6;
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
    else if (contains("J6", (const char*)cmd))
    {
        result = Joint::J6;
    }

    return result;
}

std::vector<float> Communication::extract_cmd_values(const char *cmd)
{
    std::vector<float> values;
    const char *current = strchr(cmd, '(');
    const char *end = current == NULL ? NULL : strchr(current + 1, ')');
    if (current == NULL || end == NULL)
    {
        return values;
    }

    current++;
    while (current < end && values.size() < 15)
    {
        char *value_end = NULL;
        float value = strtof(current, &value_end);
        if (value_end == current || value_end > end)
        {
            values.clear();
            return values;
        }

        values.push_back(value);
        if (DEBUG_IS_ENABLED)
        {
            char str_buffer[48];
            snprintf(str_buffer, sizeof(str_buffer), "received value: %f\n", value);
            comm_func_write(str_buffer);
        }

        if (value_end == end)
        {
            break;
        }
        if (*value_end != ',')
        {
            values.clear();
            return values;
        }
        current = value_end + 1;
    }

    return values;
}

void Communication::process_cmd(char *cmd)
{
    RobotCommand command;

    if (starts_with("CMD_", cmd))
    {
        uint8_t joint = extract_related_joint(cmd);
        uint8_t motor = extract_related_motor(cmd);

        if(joint != Joint::JNONE){
            command.target = joint;
            if (contains("_INIT", cmd))
            {
                command.type = RobotCommandType::INIT_JOINT;
                if (!enqueue_command(command)) return;
                if (DEBUG_IS_ENABLED)
                {
                    char str_buffer[40];
                    snprintf(str_buffer, sizeof(str_buffer), "Init Joint J%d\n", joint);
                    comm_func_write(str_buffer);
                }
            }
            else if (contains("_ZERO", cmd))
            {
                command.type = RobotCommandType::ZERO_JOINT;
                if (!enqueue_command(command)) return;
                if (DEBUG_IS_ENABLED)
                {
                    char str_buffer[40];
                    snprintf(str_buffer, sizeof(str_buffer), "Zero Joint J%d\n", joint);
                    comm_func_write(str_buffer);
                }
            }
            else if (contains("_SET", cmd))
            {
                std::vector<float> values = extract_cmd_values(cmd);
                if(values.size() > 1){
                    command.type = RobotCommandType::SET_JOINT_POSITION_VELOCITY;
                    command.value_count = 2;
                    command.values[0] = values[0];
                    command.values[1] = values[1];
                }
                else if(values.size() == 1){
                    command.type = RobotCommandType::SET_JOINT_POSITION;
                    command.value_count = 1;
                    command.values[0] = values[0];
                }
                else {
                    comm_func_write("Command rejected: joint setpoint is missing\n");
                    return;
                }
                enqueue_command(command);
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
                    std::vector<float> values = extract_cmd_values(cmd);
                    if (values.empty()) return;
                    speed = abs(values[0]);
                }
                if (contains("FORWARD_START", cmd))
                {
                    std::vector<float> values = extract_cmd_values(cmd);
                    if (values.empty()) return;
                    speed = abs(values[0]);
                    speed *= -1;
                }

                command.type = RobotCommandType::SET_JOINT_VELOCITY;
                command.value_count = 1;
                command.values[0] = speed;
                if (!enqueue_command(command)) return;
                
                if (DEBUG_IS_ENABLED)
                {
                    char str_buffer[64];
                    snprintf(str_buffer, sizeof(str_buffer), "Set Velocity, joint: %d, spd: %f\n", joint, speed);
                    comm_func_write(str_buffer);
                }
            }
        } else if(motor != Motor::MNONE){
            float speed = 0;
            if (contains("END", cmd))
            {
                speed = 0;
            }
            if (contains("BACKWARD_START", cmd))
            {
                std::vector<float> values = extract_cmd_values(cmd);
                if (values.empty()) return;
                speed = abs(values[0]);
            }
            if (contains("FORWARD_START", cmd))
            {
                std::vector<float> values = extract_cmd_values(cmd);
                if (values.empty()) return;
                speed = abs(values[0]);
                speed *= -1;
            }

            command.type = RobotCommandType::SET_MOTOR_VELOCITY;
            command.target = motor;
            command.value_count = 1;
            command.values[0] = speed;
            if (!enqueue_command(command)) return;
            
            if (DEBUG_IS_ENABLED)
            {
                char str_buffer[64];
                snprintf(str_buffer, sizeof(str_buffer), "Set Velocity, motor: %d, spd: %f\n", motor, speed);
                comm_func_write(str_buffer);
            }
        }
        
    }
    else if(starts_with("SET_MODE_AUTO", cmd))
    {
        command.type = RobotCommandType::SET_MODE;
        command.target = 2;
        if (!enqueue_command(command)) return;
        comm_func_write("mode auto is set\n");
    }
    else if(starts_with("SET_MODE_MOTOR", cmd))
    {
        command.type = RobotCommandType::SET_MODE;
        command.target = 1;
        if (!enqueue_command(command)) return;
        comm_func_write("mode motor is set\n");
    }
    else if(starts_with("SET_MODE_JOINT", cmd))
    {
        command.type = RobotCommandType::SET_MODE;
        command.target = 0;
        if (!enqueue_command(command)) return;
        comm_func_write("mode joint is set\n");
    }
    else if(starts_with("CONFIG", cmd))
    {
        std::vector<float> config = extract_cmd_values(cmd);
        if (config.size() != 6)
        {
            comm_func_write("CONFIG requires 6 values\n");
            return;
        }
        command.type = RobotCommandType::MOVE_TO_CONFIGURATION;
        command.value_count = 7;
        for (size_t i = 0; i < 6; ++i) command.values[i] = config[i];
        command.values[6] = 10.0f;
        if (!enqueue_command(command)) return;
        comm_func_write("CONFIG is set\n");
    }
    else if(starts_with("VEL_CONFIG", cmd))
    {
        std::vector<float> config_vel = extract_cmd_values(cmd);
        if(config_vel.size() == 7){
            command.type = RobotCommandType::MOVE_TO_CONFIGURATION;
            command.value_count = 7;
            for (size_t i = 0; i < 7; ++i) command.values[i] = config_vel[i];
            if (!enqueue_command(command)) return;
            comm_func_write("VEL_CONFIG is set\n");
        }
        else comm_func_write("VEL_CONFIG requires 7 values\n");
    }
    else if (starts_with("PID_J4", cmd))
    {
        std::vector<float> values = extract_cmd_values(cmd);
        if (values.size() != 3) return;
        command.type = RobotCommandType::SET_PID;
        command.target = Joint::J4;
        command.value_count = 3;
        for (size_t i = 0; i < 3; ++i) command.values[i] = values[i];
        if (!enqueue_command(command)) return;
        if (DEBUG_IS_ENABLED)
        {
            char str_buffer[80];
            snprintf(str_buffer, sizeof(str_buffer), "J4 PID set: %f, %f, %f", values[0], values[1], values[2]);
            comm_func_write(str_buffer);
        }
    }
    else if (starts_with("PID_J5", cmd))
    {
        std::vector<float> values = extract_cmd_values(cmd);
        if (values.size() != 3) return;
        command.type = RobotCommandType::SET_PID;
        command.target = Joint::J5;
        command.value_count = 3;
        for (size_t i = 0; i < 3; ++i) command.values[i] = values[i];
        if (!enqueue_command(command)) return;
        if (DEBUG_IS_ENABLED)
        {
            char str_buffer[80];
            snprintf(str_buffer, sizeof(str_buffer), "J5 PID set: %f, %f, %f", values[0], values[1], values[2]);
            comm_func_write(str_buffer);
        }
    }
    else if (starts_with("PID_J6", cmd))
    {
        std::vector<float> values = extract_cmd_values(cmd);
        if (values.size() != 3) return;
        command.type = RobotCommandType::SET_PID;
        command.target = Joint::J6;
        command.value_count = 3;
        for (size_t i = 0; i < 3; ++i) command.values[i] = values[i];
        if (!enqueue_command(command)) return;
        if (DEBUG_IS_ENABLED)
        {
            char str_buffer[80];
            snprintf(str_buffer, sizeof(str_buffer), "J6 PID set: %f, %f, %f", values[0], values[1], values[2]);
            comm_func_write(str_buffer);
        }
    }
    else if (starts_with("MIX_J56", cmd))
    {
        std::vector<float> values = extract_cmd_values(cmd);
        if(values.size() == 4){
            command.type = RobotCommandType::SET_J5_J6_MIXING;
            command.value_count = 4;
            for (size_t i = 0; i < 4; ++i) command.values[i] = values[i];
            if (!enqueue_command(command)) return;
            comm_func_write("J5/J6 mixing matrix set\n");
        }
        else{
            comm_func_write("MIX_J56 requires 4 values\n");
        }
    }
    else if(starts_with("SAVE_ZEROS", cmd))
    {
        command.type = RobotCommandType::SAVE_ZEROS;
        if (!enqueue_command(command)) return;
        comm_func_write("Zeros saved\n");
    }
    else if(starts_with("LOAD_ZEROS", cmd))
    {
        command.type = RobotCommandType::LOAD_ZEROS;
        if (!enqueue_command(command)) return;
        comm_func_write("Zeros loaded\n");
    }
    else if(starts_with("GRIP_", cmd)){
        if(starts_with("GRIP_OPEN", cmd)){
            command.type = RobotCommandType::OPEN_GRIPPER;
            if (!enqueue_command(command)) return;
            comm_func_write("Gripper opened\n");
        }
        else if(starts_with("GRIP_CLOSE", cmd)){
            command.type = RobotCommandType::CLOSE_GRIPPER;
            if (!enqueue_command(command)) return;
            comm_func_write("Gripper closed\n");
        }
        else if(starts_with("GRIP_SET", cmd)){
            std::vector<float> values = extract_cmd_values(cmd);
            if (values.empty()) return;
            int position = values[0];
            command.type = RobotCommandType::SET_GRIPPER_POSITION;
            command.value_count = 1;
            command.values[0] = position;
            if (!enqueue_command(command)) return;
            if(DEBUG_IS_ENABLED){
                char str_buffer[48];
                snprintf(str_buffer, sizeof(str_buffer), "Gripper position set: %d", position);
                comm_func_write(str_buffer);
            }
        }
    }
}

void Communication::check_incoming_cmds()
{
    uint8_t read_buffer[READ_CHUNK_SIZE];
    uint16_t data_len = 0;

    while ((data_len = comm_func_read_all_bytes(read_buffer, READ_CHUNK_SIZE)) > 0)
    {
        for (uint16_t i = 0; i < data_len; i++)
        {
            const char byte = static_cast<char>(read_buffer[i]);

            if (byte == '\r')
            {
                continue;
            }

            if (discarding_oversized_cmd)
            {
                if (byte == '\n')
                {
                    discarding_oversized_cmd = false;
                }
                continue;
            }

            if (byte == '\n')
            {
                if (buffer_index > 0)
                {
                    buffer[buffer_index] = '\0';
                    comm_func_write("data received: ");
                    comm_func_write(buffer);
                    comm_func_write("\n");
                    process_cmd(buffer);
                    buffer_index = 0;
                }
                continue;
            }

            if (buffer_index < BUFFER_SIZE - 1)
            {
                buffer[buffer_index++] = byte;
            }
            else
            {
                buffer_index = 0;
                discarding_oversized_cmd = true;
                comm_func_write("Command rejected: exceeds 127 bytes\n");
            }
        }
    }
}
