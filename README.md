# Robot Controller

A C++20 project for building a simulated 6-DOF robotic arm controller from the ground up.

The goal is to develop a small but realistic robotics control system while learning modern C++20 design, simulation, safety, real-time programming, and software architecture.

## Project Goal

The long-term target is a simulated robotic arm controller capable of running a control loop at **1 kHz (1 ms period)**.

The planned architecture is:

```text
Sensors
   ↓
State Estimator
   ↓
Motion Planner
   ↓
Controller
   ↓
Safety Layer
   ↓
Motor Interface
   ↓
Simulator
```

The project is being developed incrementally, with each stage introducing relevant C++ concepts and robotics/control concepts.

## Current Status

The project currently contains:

* A 6-DOF robot model
* Individual joint objects
* Joint position limits
* Joint velocity limits
* Joint acceleration limits
* Position target commands
* Simulated joint dynamics
* Angular velocity state
* Strong types for physical quantities
* A PD controller
* A configurable PD controller
* A SafetyLayer for validating acceleration commands
* A MotorInterface abstraction
* A simulated motor implementation
* A RobotSimulator responsible for advancing physical state
* A basic simulation update loop at a 1 ms timestep
* C++20 build configuration using CMake and Ninja

### Current Simulation Model

Each joint currently uses:

* Position: degrees
* Velocity: degrees/second
* Acceleration: degrees/second²
* Time step: seconds

The simulator uses semi-implicit Euler integration:

```text
velocity = velocity + acceleration × dt
position = position + velocity × dt
```

Using the newly calculated velocity for the position update gives the current simulation its semi-implicit Euler behavior.

The simulator also enforces each joint's configured velocity and position limits.

The `RobotSimulator` is responsible for advancing the simulated physical state. The `Joint` stores its physical state and exposes the limits and actuator command needed by the simulator.

## Timing and Windows Scheduling

The control loop uses `std::chrono::steady_clock` together with
`std::this_thread::sleep_until()` to schedule a nominal 1 ms control period.

Testing on Windows showed an important limitation of this approach.

A 2.5 second simulation was executed with 2,500 control cycles. The
measured results were:

- Real execution time: 2.50122 s
- Simulated time: 2.5 s
- Average cycle period: 1.00089 ms
- Minimum cycle period: 0.0014 ms
- Maximum cycle period: 28.6843 ms
- Missed 1 ms cycle deadlines: 160
- Maximum control-loop execution time: 0.0978 ms

The results demonstrate that the controller and simulator execute well
within the 1 ms budget, with the measured maximum execution time below
0.1 ms. However, the Windows scheduling environment does not provide deterministic thread wake-up timing at the 1 ms resolution required by a hard real-time control loop.

When the thread wakes later than its scheduled time, `sleep_until()` can return immediately on the following iteration because the next scheduled time is already in the past. This explains the combination of long cycle periods and very short catch-up periods.

### Lesson learned

A nominal 1 kHz loop is not the same as a deterministic 1 kHz real-time
loop.

Using `sleep_until()` provides a useful absolute scheduling mechanism,
and the average period can be very close to 1 ms. However, normal Windows scheduling can introduce significant timing jitter and missed 1 ms deadlines.

The missed deadlines are caused by cycle-start timing jitter rather than the controller or simulator exceeding the 1 ms execution budget.

For this reason, the current implementation should be considered a
**1 kHz scheduled simulation loop**, not a hard real-time control loop.

Achieving deterministic 1 ms behavior requires additional real-time
considerations beyond the scheduling mechanism itself, including thread
scheduling, CPU isolation/affinity, synchronization, memory allocation,
I/O behavior, and ultimately an operating-system/environment capable of
providing appropriate real-time guarantees.

This experiment is intentionally kept as part of the project because it
demonstrates an important distinction between:

- **Execution time** — how long the controller and simulator take to run.
- **Cycle period** — how frequently control cycles actually start.
- **Deadline behavior** — whether each cycle can reliably meet its 1 ms
  timing requirement.

The current implementation demonstrates that the application workload
is fast enough for a 1 ms budget, with a measured maximum execution time of only 0.0978 ms. However, the Windows execution environment introduces scheduling jitter that prevents deterministic 1 ms cycle timing.

## Strong Types

Physical quantities are represented using separate C++ types rather than passing raw `double` values everywhere.

Current types include:

```text
Angle
Duration
AngularVelocity
AngularAcceleration
```

For example:

```cpp
AngularAcceleration{30.0}
```

is intentionally different from:

```cpp
Angle{30.0}
```

This prevents accidental mixing of physically different quantities and makes interfaces more explicit.

## Joint Safety

Each joint has configurable position, velocity, and acceleration limits.

For example, a joint may be configured with:

```text
Minimum position:        -180°
Maximum position:         180°
Maximum velocity:         60 deg/s
Maximum acceleration:     30 deg/s²
```

Commands outside the permitted range are rejected where applicable.

The `Joint` owns its physical limits. Higher-level components such as the controller may request an acceleration, while the `SafetyLayer` validates that request against the limits owned by the `Joint`.

The `SafetyLayer` is integrated into the robot's main control/update path as a separate validation step.

The `RobotSimulator` is responsible for enforcing the simulated velocity and position constraints while advancing the physical state.

## Position Control

The robot supports target positions for its joints.

A target position represents where the controller wants the joint to move, while the joint's current position represents the simulated actual state.

The current controller is a proportional-derivative (PD) controller.

Conceptually:

```text
position error = target position - actual position

acceleration command =
    Kp × position error
    - Kd × velocity
```

The controller generates a requested acceleration command.

The `SafetyLayer` provides a separate validation step that checks the requested command against the Joint's configured acceleration limit.

The PD controller therefore provides both:

* Position-based correction
* Velocity-based damping

The current implementation is intentionally simple and is used to introduce the separation between control logic and joint dynamics.

## Motor Interface

The robot uses a `MotorInterface` abstraction to separate the controller and joint logic from the underlying actuator implementation.

The interface currently allows an acceleration command to be sent to a motor and allows the current commanded acceleration to be read.

The simulator uses `SimulatedMotor` as its motor implementation.

This establishes the separation:

```text
Controller
    ↓
Joint
    ↓
MotorInterface
    ↓
SimulatedMotor
    ↓
RobotSimulator
    ↓
Simulated physical state
```

The interface can later be extended with more realistic actuator behavior without requiring the controller to depend directly on a concrete motor implementation.

## Robot Simulator

The `RobotSimulator` advances the physical state of all six joints.

The simulator currently:

* Reads the acceleration commanded to each motor
* Integrates joint velocity
* Integrates joint position
* Enforces velocity limits
* Enforces position limits
* Updates the simulated joint state

The simulator currently uses a fixed 1 ms timestep supplied by the robot update path.

The simulation is intentionally simple at this stage. More realistic actuator and mechanical behavior can be introduced later.

## Current Robot

The robot contains six joints:

```text
Joint 1: -180° to 180°, max velocity 60 deg/s, max acceleration 30 deg/s²
Joint 2:  -90° to  90°, max velocity 60 deg/s, max acceleration 30 deg/s²
Joint 3: -180° to 180°, max velocity 60 deg/s, max acceleration 30 deg/s²
Joint 4: -180° to 180°, max velocity 60 deg/s, max acceleration 30 deg/s²
Joint 5:  -90° to  90°, max velocity 60 deg/s, max acceleration 30 deg/s²
Joint 6: -180° to 180°, max velocity 60 deg/s, max acceleration 30 deg/s²
```

The robot currently exposes operations for:

* Setting a joint position
* Setting a joint target position
* Setting a joint acceleration
* Updating all joints
* Printing the current robot state

During normal operation, the robot calculates acceleration commands through the PD controller, validates them through the SafetyLayer, sends them to the motors, and then advances the physical simulation.

## Example

A target position can be assigned with:

```cpp
robot.setJointTargetPosition(
    0,
    Angle{90.0}
);
```

The robot's controller then calculates an acceleration command using the position error and current joint velocity.

A manually supplied acceleration command is still available for testing:

```cpp
robot.setJointAcceleration(
    0,
    AngularAcceleration{30.0}
);
```

A command exceeding the configured joint acceleration limit:

```cpp
robot.setJointAcceleration(
    0,
    AngularAcceleration{100.0}
);
```

is rejected.

## Project Structure

```text
RobotController/

├── CMakeLists.txt
├── README.md
├── .gitignore
├── include/
│   ├── Angle.hpp
│   ├── Duration.hpp
│   ├── AngularVelocity.hpp
│   ├── AngularAcceleration.hpp
│   ├── JointTypes.hpp
│   ├── Joint.hpp
│   ├── PDControllerConfig.hpp
│   ├── PDController.hpp
│   ├── SafetyLayer.hpp
│   ├── MotorInterface.hpp
│   ├── SimulatedMotor.hpp
│   ├── RobotSimulator.hpp
│   └── Robot.hpp
│
└── src/
    ├── Joint.cpp
    ├── Robot.cpp
    ├── PDController.cpp
    ├── SafetyLayer.cpp
    ├── SimulatedMotor.cpp
    ├── RobotSimulator.cpp
    └── main.cpp
```

## Building

The project uses:

* C++20
* CMake
* Ninja
* GCC

Configure the project:

```powershell
cmake -S . -B build -G Ninja
```

Build:

```powershell
cmake --build build
```

Run:

```powershell
.\build\robot_controller.exe
```

## Development Approach

The project is intentionally being built in small increments.

Rather than introducing every robotics and C++ concept at once, each stage adds a useful capability and then verifies it through compilation and simulation.

The development process emphasizes:

* Strong type safety
* Clear interfaces
* Encapsulation
* RAII
* Standard library containers and algorithms
* Modern C++20 features
* Deterministic behavior
* Real-time considerations
* Testability
* Performance measurement

## Roadmap

### Completed

* [x] CMake/C++20 project setup
* [x] Basic `Robot` and `Joint` classes
* [x] Six-joint robot model
* [x] Joint position limits
* [x] Joint velocity limits
* [x] Joint acceleration limits
* [x] Simulated joint dynamics
* [x] `Angle` strong type
* [x] `Duration` strong type
* [x] `AngularVelocity` strong type
* [x] `AngularAcceleration` strong type
* [x] Position targets
* [x] Basic PD controller
* [x] Controller acceleration limiting
* [x] SafetyLayer acceleration validation
* [x] Integrate SafetyLayer into robot control path
* [x] Separate commanded state from actual state
* [x] Improve controller architecture
* [x] Motor interface abstraction
* [x] Robot simulator
* [x] Fixed-period control loop


### Next

* [ ] Deterministic 1 kHz control loop
* [ ] Real-time-oriented data structures
* [ ] Threading and `std::jthread`
* [ ] Atomics and synchronization
* [ ] Logging and diagnostics
* [ ] Unit tests
* [ ] Sanitizers
* [ ] Profiling
* [ ] Further C++20 features
* [ ] Communication/network interface

## License

This project is currently for learning and experimentation.