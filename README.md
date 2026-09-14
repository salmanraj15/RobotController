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
* Joint acceleration limits
* Position and acceleration commands
* Simulated joint dynamics
* Angular velocity state
* Strong types for physical quantities
* A basic simulation update loop
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

Each joint has configurable position and acceleration limits.

For example, a joint may be configured with:

```text
Minimum position:       -180°
Maximum position:        180°
Maximum acceleration:    30 deg/s²
```

Commands outside the permitted range are rejected.

If a joint reaches a position limit during simulation, the current implementation:

1. Clamps the position to the limit.
2. Sets velocity to zero.
3. Sets acceleration to zero.

This provides a basic safety boundary around the simulated joint state.

## Current Robot

The robot contains six joints:

```text
Joint 1: -180° to 180°, max acceleration 30 deg/s²
Joint 2:  -90° to  90°, max acceleration 30 deg/s²
Joint 3: -180° to 180°, max acceleration 30 deg/s²
Joint 4: -180° to 180°, max acceleration 30 deg/s²
Joint 5:  -90° to  90°, max acceleration 30 deg/s²
Joint 6: -180° to 180°, max acceleration 30 deg/s²
```

The robot currently exposes operations for:

* Setting a joint position
* Setting a joint acceleration
* Updating all joints
* Printing the current robot state

## Example

A valid acceleration command:

```cpp
robot.setJointAcceleration(
    0,
    AngularAcceleration{30.0}
);
```

is accepted.

A command exceeding the configured joint limit:

```cpp
robot.setJointAcceleration(
    0,
    AngularAcceleration{100.0}
);
```

is rejected.

With an acceleration of `30 deg/s²` and a `0.1 s` simulation step, the first few updates produce:

```text
Time: 0.1 s
Position: 0.3 degrees
Velocity: 3

Time: 0.2 s
Position: 0.9 degrees
Velocity: 6

Time: 0.3 s
Position: 1.8 degrees
Velocity: 9

...

Time: 1.0 s
Position: 16.5 degrees
Velocity: 30
```

## Project Structure

```text
RobotController/
├── CMakeLists.txt
├── include/
│   ├── Angle.hpp
│   ├── Duration.hpp
│   ├── AngularVelocity.hpp
│   ├── AngularAcceleration.hpp
│   ├── Joint.hpp
│   └── Robot.hpp
└── src/
    ├── Joint.cpp
    ├── Robot.cpp
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
* [x] Joint acceleration limits
* [x] Simulated joint dynamics
* [x] `Angle` strong type
* [x] `Duration` strong type
* [x] `AngularVelocity` strong type
* [x] `AngularAcceleration` strong type

### Next

* [ ] Position target and feedback control
* [ ] Basic controller
* [ ] Separate commanded state from actual state
* [ ] Safety layer
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
