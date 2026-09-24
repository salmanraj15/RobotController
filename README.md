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

## Cross-Platform Requirement

The application is intended to build and run on multiple operating systems,
including:

- Windows
- Linux

The core robot, controller, simulator, safety, and motor-interface code should
remain platform-independent and use standard C++20 facilities where possible.

Platform-specific behavior should be isolated behind dedicated abstractions
rather than being introduced directly into the core control logic.

This is especially important for real-time timing. The application uses
cross-platform C++ timing and threading facilities such as `std::chrono`,
`std::jthread`, and `std::this_thread::sleep_until()`, but the timing guarantees
provided by the underlying operating system may differ.

The long-term architecture should therefore separate:

- Platform-independent control logic
- Portable timing and threading interfaces
- Operating-system-specific real-time mechanisms, where required

The goal is to keep the application portable while allowing platform-specific
real-time optimizations to be introduced without coupling the core robot
controller to a particular operating system.

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
* A fixed 1 ms simulation timestep
* A dedicated control thread using `std::jthread`
* A dedicated control scheduler abstraction
* Fixed-size atomic state snapshots
* 1 kHz timing instrumentation
* Separate control execution time from scheduling latency
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

The control loop uses `std::chrono::steady_clock` together with `std::this_thread::sleep_until()` to schedule a nominal 1 ms control period.

Testing on Windows showed an important limitation of this approach.

A 2.5 second simulation was executed with 2,500 control cycles. The latest representative run on Windows produced:

- Real execution time: 2.519 s
- Simulated time: 2.5 s
- Average cycle period: 0.99948 ms
- Minimum cycle period: 0.0013 ms
- Maximum cycle period: 21.6456 ms
- Maximum execution time: 0.2882 ms
- Maximum control time: 0.1475 ms
- Maximum snapshot time: 0.0650 ms
- Delayed cycles: 2190
- Deadline misses: 2190
- Maximum scheduling delay: 19.7306 ms
- Maximum schedule backlog: 18.7306 ms
- Cycles with at least 1 ms backlog: 2038
- Minimum deadline margin: -18.7392 ms

Note: These measurements are representative Windows runs and will vary between executions.

The results demonstrate that the controller and simulator execution work remains below the 1 ms cycle budget. The measured maximum control time was 0.1475 ms and the maximum total control-loop execution time was 0.2882 ms.

However, the Windows scheduling environment introduces significant timing variation at the 1 ms resolution required by a deterministic real-time control loop.

A delayed cycle does not necessarily mean that the control work itself exceeded its deadline. A cycle can start late and still complete before its one-millisecond
deadline.

The control loop therefore also measures deadline misses. A deadline miss is recorded when a cycle finishes after its scheduled start time plus the 1 ms control period.

This separates scheduling delay from control execution time and provides a clearer measurement of whether the control cycle completed within its allowed time window.

When the thread wakes later than its scheduled time, `sleep_until()` can return immediately on subsequent iterations because the next scheduled time is already
in the past. This can produce a long cycle period followed by a very short cycle period.

The control loop intentionally executes every requested cycle rather than skipping delayed cycles. The simulation timestep remains fixed at 1 ms, independent of wall-clock scheduling delay.

The `ControlScheduler` now owns the scheduling timeline and returns the scheduled start time for the cycle it releases. Scheduling delay and backlog are calculated explicitly from the actual start and that cycle's scheduled start. Backlog measures lateness beyond one full control period, so a
1.4 ms scheduling delay corresponds to 0.4 ms of schedule backlog.

### Lesson learned

A nominal 1 kHz loop is not the same as a deterministic 1 kHz real-time loop.

Using `sleep_until()` provides an absolute scheduling mechanism, and the average cycle period can remain very close to 1 ms. However, normal Windows thread scheduling can introduce significant delays between the scheduled and actual cycle start times.

The measured controller and simulator execution time is well below the 1 ms cycle budget. The timing variation therefore comes primarily from the execution environment rather than the control workload.

This experiment demonstrates the distinction between:

- **Execution time** — how long the controller and simulator take to run.
- **Cycle period** — the time between actual  control-cycle starts.
- **Scheduling delay** — the difference between the scheduled cycle start and   the actual cycle start.
- **Delayed cycles** — cycles that begin later than their scheduled start.
- **Deadline behavior** — whether each cycle finishes within its 1 ms timing   window.

The current implementation should therefore be considered a **1 kHz scheduled simulation loop**, not a hard real-time control loop.

The control loop maintains an absolute schedule. If Windows delays a cycle, subsequent cycles can start very close together because the next scheduled time
may already be in the past. This preserves the requirement that every requested control cycle is executed while keeping the simulation timestep fixed at 1 ms.

Achieving deterministic 1 ms behavior requires additional real-time considerations beyond the scheduling mechanism itself, including thread scheduling, CPU affinity, synchronization, memory allocation, I/O behavior,
and ultimately an operating-system/environment capable of providing appropriate real-time guarantees.

This does not mean that a 1 kHz control loop cannot run on Windows. The current results show that the loop can maintain an average period close to 1 ms, but the normal Windows scheduling environment does not provide the deterministic timing behavior required for a hard real-time guarantee in this implementation. The next stage is therefore to isolate platform-specific scheduling mechanisms so they can be evaluated independently.

### Threading and `std::jthread`

The control loop uses C++20 `std::jthread` to execute control cycles on a dedicated thread.

The thread receives a `std::stop_token` and checks for a stop request between control cycles, allowing cooperative cancellation.

The control loop owns the worker thread and uses `std::jthread` to manage its lifetime.

The monitoring side can observe control-loop progress while the worker thread is running.

This introduces:

- `std::jthread`
- `std::stop_token`
- Cooperative thread cancellation
- Thread ownership and lifetime management
- Concurrent access to control-loop information
- Atomic synchronization

## Thread Synchronization

The control loop and monitoring code run on separate threads, so shared data requires explicit synchronization.

A completed-cycle counter is stored as:

```cpp
std::atomic<int>
```

This allows the monitoring thread to read the counter while the control thread updates it without a data race.

Robot state contains multiple related values, so it is published as a complete snapshot.

The project initially used a mutex-protected `RobotState` as a correctness baseline. A fixed-size `SnapshotBuffer` has now been introduced as a synchronization experiment.

The snapshot buffer uses:

* A fixed-size array of atomic values
* An atomic sequence counter
* Acquire/release synchronization
* Sequence validation to detect concurrent updates

The sequence counter uses an even value for a stable snapshot and an odd value while the writer is updating the snapshot.

The reader checks the sequence before and after reading the state. If the sequence changes, the reader retries because the snapshot may have been modified during the read.

This removes the mutex from the snapshot publication path while keeping the snapshot bounded and allocation-free.

The snapshot design is currently an evaluated synchronization approach rather than a final hard real-time guarantee.

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
    JointIndex{0},
    Angle{90.0});
```

The robot's controller then calculates an acceleration command using the position error and current joint velocity.

A manually supplied acceleration command is still available for testing:

```cpp
robot.setJointAcceleration(
    JointIndex{0},
    AngularAcceleration{30.0});
```

A command exceeding the configured joint acceleration limit:

```cpp
robot.setJointAcceleration(
    JointIndex{0},
    AngularAcceleration{100.0});
```

is rejected.

## Project Structure

```text
RobotController/
├── CMakeLists.txt
├── README.md
├── .gitignore
│
├── include/
│   ├── Angle.hpp
│   ├── AngularAcceleration.hpp
│   ├── AngularVelocity.hpp
│   ├── ControlLoop.hpp
│   ├── ControlScheduler.hpp
│   ├── Duration.hpp
│   ├── Joint.hpp
│   ├── JointTypes.hpp
│   ├── MotorInterface.hpp
│   ├── PDController.hpp
│   ├── PDControllerConfig.hpp
│   ├── Robot.hpp
│   ├── RobotSimulator.hpp
│   ├── SafetyLayer.hpp
│   ├── SimulatedMotor.hpp
│   └── SnapshotBuffer.hpp
│
└── src/
    ├── Angle.cpp
    ├── ControlLoop.cpp
    ├── ControlScheduler.cpp
    ├── Joint.cpp
    ├── main.cpp
    ├── PDController.cpp
    ├── Robot.cpp
    ├── RobotSimulator.cpp
    ├── SafetyLayer.cpp
    ├── SimulatedMotor.cpp
    └── SnapshotBuffer.cpp

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
* [x] Threading and `std::jthread`
* [x] Portable 1 kHz control-loop architecture
* [x] Bounded/fixed-size control path
* [x] 1 kHz timing instrumentation
* [x] Separate control execution time from scheduling latency
* [x] Atomic counters and synchronized state snapshots
* [x] Snapshot synchronization experiment
* [x] Separate scheduling from control-loop logic
* [x] Explicit scheduled-start, scheduling-delay, and backlog measurements

### Next

* [ ] Deterministic 1 kHz control-loop design
* [ ] Platform scheduling backends
    * [ ] Windows
    * [ ] Linux
* [ ] Real-time verification
    * [ ] Linux/PREEMPT_RT
* [ ] Real-time-oriented data structures
* [ ] Evaluate snapshot design under real-time constraints
* [ ] Logging and diagnostics
* [ ] Unit tests
* [ ] Sanitizers
* [ ] Profiling
* [ ] Further C++20 features
* [ ] Communication/network interface

## License

This project is currently for learning and experimentation.