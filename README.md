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
* A SafetyLayer for validating acceleration commands against joint limits
* A basic simulation update loop at a 1 ms timestep
* C++20 build configuration using CMake and Ninja

### Current Simulation Model

Each joint currently uses:

* Position: degrees
* Velocity: degrees/second
* Acceleration: degrees/second²
* Time step: seconds

The joint is updated using semi-implicit Euler integration:

```text
velocity = velocity + acceleration × dt
position = position + velocity × dt
```

Using the newly calculated velocity for the position update gives the current simulation its semi-implicit Euler behavior.

The joint also enforces a maximum velocity. This prevents the simulated joint from accelerating indefinitely and establishes another actuator-level safety boundary.

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

The Joint owns its physical limits. Higher-level components such as the controller may request an acceleration, while the SafetyLayer validates that request against the limits owned by the Joint.

The SafetyLayer validates acceleration commands and rejects requests that exceed the Joint's configured acceleration limit. It is integrated into the robot's main control/update path.

The Joint remains responsible for enforcing limits on its physical state during simulation.

## Position Control

The robot now supports target positions for its joints.

A target position represents where the controller wants the joint to move, while the joint's current position represents the simulated actual state.

The current controller is a proportional-derivative (PD) controller.

Conceptually:

```text
position error = target position - actual position

acceleration command =
    Kp × position error
    - Kd × velocity
```

The controller generates a requested acceleration command. The SafetyLayer provides a separate validation step that checks the requested command against the Joint's configured acceleration limit.

The SafetyLayer is integrated into the robot's main control/update path as a separate validation step.

The PD controller therefore provides both:

* Position-based correction
* Velocity-based damping

The current implementation is intentionally simple and is used to introduce the separation between control logic and joint dynamics.

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

During normal operation, the robot calculates the acceleration command through the PD controller before updating each joint.

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
│   ├── Joint.hpp
│   ├── Robot.hpp
│   ├── PDController.hpp
│   └── SafetyLayer.hpp
└── src/
    ├── Joint.cpp
    ├── Robot.cpp
    ├── PDController.cpp
    ├── SafetyLayer.cpp
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


### Next

* [ ] Motor interface abstraction
* [ ] Robot simulator
* [ ] Fixed-period control loop
* [ ] 1 kHz control loop
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
