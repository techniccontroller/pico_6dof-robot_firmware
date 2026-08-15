#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"

#include "defines_constants.h"
#include "Robot.h"
#include "communication.h"
#include "robot_command.h"

#define PICO_W 1
#ifdef PICO_W
#include "pico/cyw43_arch.h"
#endif

#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN PICO_DEFAULT_LED_PIN
#else
#define LED_PIN 25
#endif

namespace
{
constexpr uint32_t TELEMETRY_RATE_HZ = 30;
constexpr uint64_t TELEMETRY_INTERVAL_US = 1000000 / TELEMETRY_RATE_HZ;
constexpr uint64_t CONTROL_INTERVAL_US = 10000; // 100 Hz
constexpr uint COMMAND_QUEUE_LENGTH = 16;
constexpr uint TELEMETRY_QUEUE_LENGTH = 2;

queue_t commandQueue;
queue_t telemetryQueue;

void applyCommand(Robot& robot, const RobotCommand& command)
{
    switch (command.type)
    {
    case RobotCommandType::INIT_JOINT:
        robot.initJoint(command.target);
        break;
    case RobotCommandType::ZERO_JOINT:
        robot.zeroJoint(command.target);
        break;
    case RobotCommandType::SET_JOINT_POSITION:
        robot.setJointPosition(command.target, command.values[0]);
        break;
    case RobotCommandType::SET_JOINT_VELOCITY:
        robot.setJointVelocity(command.target, command.values[0]);
        break;
    case RobotCommandType::SET_JOINT_POSITION_VELOCITY:
        robot.setJointPositionVelocity(command.target, command.values[0], command.values[1]);
        break;
    case RobotCommandType::SET_MOTOR_VELOCITY:
        robot.setMotorVelocity(command.target, command.values[0]);
        break;
    case RobotCommandType::SET_MODE:
        robot.setMode(static_cast<Robot::RobotMode>(command.target));
        break;
    case RobotCommandType::MOVE_TO_CONFIGURATION:
        robot.moveToConfiguration(command.values, command.values[6]);
        break;
    case RobotCommandType::SET_PID:
        robot.setPID(command.target, command.values[0], command.values[1], command.values[2]);
        break;
    case RobotCommandType::SET_J5_J6_MIXING:
        robot.setJ5J6Mixing(command.values[0], command.values[1],
                            command.values[2], command.values[3]);
        break;
    case RobotCommandType::SAVE_ZEROS:
        robot.writeAllSensorCalibrationData();
        break;
    case RobotCommandType::LOAD_ZEROS:
        robot.loadAllSensorCalibrationData();
        break;
    case RobotCommandType::OPEN_GRIPPER:
        robot.openGripper();
        break;
    case RobotCommandType::CLOSE_GRIPPER:
        robot.closeGripper();
        break;
    case RobotCommandType::SET_GRIPPER_POSITION:
        robot.setGripperPosition(command.values[0]);
        break;
    }
}

void publishLatestTelemetry(const RobotTelemetry& telemetry)
{
    if (queue_try_add(&telemetryQueue, &telemetry))
    {
        return;
    }

    // Core 0 is behind: discard one stale sample and retain the newest one.
    RobotTelemetry stale;
    queue_try_remove(&telemetryQueue, &stale);
    queue_try_add(&telemetryQueue, &telemetry);
}

void robotCoreMain()
{
    // Static storage keeps the large hardware/controller object off Core 1's stack.
    static Robot robot;

    uint64_t lastControlTimeUs = time_us_64();
    uint64_t lastTelemetryTimeUs = lastControlTimeUs;

    while (true)
    {
        // Pulse generation is serviced continuously on Core 1.
        robot.run();

        // Execute at most one command per pass so a burst cannot starve stepping.
        RobotCommand command;
        if (queue_try_remove(&commandQueue, &command))
        {
            applyCommand(robot, command);
            robot.run();
        }

        uint64_t currentTimeUs = time_us_64();
        if ((currentTimeUs - lastControlTimeUs) >= CONTROL_INTERVAL_US)
        {
            robot.step();
            lastControlTimeUs += CONTROL_INTERVAL_US;

            // Skip stale iterations instead of running a catch-up burst.
            if ((currentTimeUs - lastControlTimeUs) >= CONTROL_INTERVAL_US)
            {
                lastControlTimeUs = currentTimeUs;
            }
            robot.run();
        }

        if ((currentTimeUs - lastTelemetryTimeUs) >= TELEMETRY_INTERVAL_US)
        {
            publishLatestTelemetry(robot.getTelemetrySnapshot());
            lastTelemetryTimeUs = currentTimeUs;
        }
    }
}
}

void setLEDState(bool state)
{
    if(PICO_W){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
    }
    else{
        gpio_put(LED_PIN, state);
    }
}

int main()
{
    stdio_init_all();

    if(PICO_W){
        if (cyw43_arch_init()) {
            printf("Wi-Fi init failed");
            return -1;
        }
    }
    else {
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
    }

    setup_default_uart();

    // Core 0 must participate when Core 1 temporarily locks out flash access.
    multicore_lockout_victim_init();
    queue_init(&commandQueue, sizeof(RobotCommand), COMMAND_QUEUE_LENGTH);
    queue_init(&telemetryQueue, sizeof(RobotTelemetry), TELEMETRY_QUEUE_LENGTH);
    multicore_launch_core1(robotCoreMain);

    Communication comm(&commandQueue);

    puts("Start program...");
    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));

    while (true)
    {
        comm.check_incoming_cmds();

        RobotTelemetry telemetry;
        if (queue_try_remove(&telemetryQueue, &telemetry))
        {
            std::string robotDataJson = Robot::telemetryToJson(telemetry);
            puts(robotDataJson.c_str());
        }
    }

    return 0;
}
