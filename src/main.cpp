#include "robot_controller/control/ControlLoop.hpp"
#include "robot_controller/scheduler/ControlSchedulerFactory.hpp"
#include "robot_controller/core/Robot.hpp"
#include "robot_controller/communication/SimulatedCan.hpp"
#include "robot_controller/communication/CanCommunication.hpp"
#include "robot_controller/communication/JointCommandCodec.hpp"
#include "robot_controller/communication/CanCommandBuffer.hpp"
#include "robot_controller/communication/CanStateBuffer.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    Robot robot;

    auto scheduler =
        createControlScheduler(
            std::chrono::milliseconds{1});

    CanCommandBuffer can_commands;
    CanStateBuffer can_state;

    ControlLoop control_loop{
        robot,
        *scheduler,
        can_commands,
        can_state};

    robot.initializeJointPosition(
        JointIndex{0},
        Angle{0.0});

    SimulatedCan can;

    JointCommand outgoing_command;
    outgoing_command.target_position = Angle{90.0};

    const CanFrame tx =
        JointCommandCodec::encode(outgoing_command);

    can.transmit(tx);

    CanCommunication communication{
        can,
        can_commands,
        can_state};

    constexpr int cycle_count = 2500;

    const auto start = std::chrono::steady_clock::now();

    // Start CAN communication before the control loop.
    communication.start();

    control_loop.run(cycle_count);

    // Wait for the control loop to finish.
    while (control_loop.completedCycles() < cycle_count)
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds{10});
    }

    communication.stop();

    const RobotState state = control_loop.state();

    control_loop.printSnapshot(state);
    control_loop.printTimingStatistics();

    std::cout << "Completed cycles: "
              << control_loop.completedCycles()
              << '\n';

    const auto end = std::chrono::steady_clock::now();

    const auto elapsed =
        std::chrono::duration<double>(end - start);

    std::cout << "\nReal execution time: "
              << elapsed.count()
              << " s\n";

    std::cout << "Simulated time: "
              << cycle_count * 0.001
              << " s\n";
    return 0;
}