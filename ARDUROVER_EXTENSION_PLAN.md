# ArduRover Offboard Autonomy Extension Plan

## Scope
This plan turns ArduRover into the safety-critical vehicle controller for an AGV with a Jetson Orin NX companion computer running ROS2, SLAM, perception, planning, task logic, and fleet/application software.

The target end state is:
- ArduRover owns arming, low-level motion control, actuator output, onboard failsafes, and safe fallback behavior.
- The companion owns localization fusion, mapping, perception, planning, task execution, and mission orchestration.
- The interface between them is explicit, heartbeat-driven, and testable.

This plan is based on the current Rover control paths in:
- `Rover/mode.h`
- `Rover/mode.cpp`
- `Rover/mode_guided.cpp`
- `Rover/mode_auto.cpp`
- `Rover/GCS_MAVLink_Rover.cpp`
- `Rover/AP_ExternalControl_Rover.{h,cpp}`
- `Rover/failsafe.cpp`
- `libraries/AR_WPNav/AR_WPNav.{h,cpp}`

## Design Principles
- Do not move safety-critical stop and fallback behavior out of ArduRover.
- Do not make raw steering and throttle streaming the primary autonomy interface.
- Keep the companion interface explicit enough that command freshness, source ownership, and health are visible in logs and telemetry.
- Prefer additive changes around Rover's current `GUIDED` and `AUTO` paths before large refactors.
- Keep ROS2- and Jetson-specific logic outside the ArduPilot tree unless it materially improves the onboard/offboard contract.

## Target Architecture
ArduRover responsibilities:
- arming, disarming, RC override, emergency stop
- steering, speed, yaw-rate, and trajectory tracking
- actuator output and hard failsafes
- offboard heartbeat monitoring and command timeout handling
- fence, battery, RC, GCS, and CPU failsafe handling
- mission fallback if the companion becomes unavailable

Jetson responsibilities:
- ROS2 bringup and lifecycle management
- multi-sensor localization and map alignment
- SLAM, perception, obstacle understanding
- global and local planning
- task execution, behavior tree, docking logic, fleet integration
- translation between ROS2-native goals/actions and ArduRover offboard commands

## Delivery Phases

## Phase 0: Baseline and Instrumentation
Goal: stabilize the current guided/offboard path before introducing a new control mode.

ArduRover work:
- Add logging around all companion-driven setpoint entry points.
- Log command source, command age, rejection reason, and mode state transitions.
- Make current `GUIDED` rejection behavior visible over MAVLink text and dataflash logs.

Files:
- `Rover/GCS_MAVLink_Rover.cpp`
- `Rover/AP_ExternalControl_Rover.cpp`
- `Rover/Log.cpp`
- `Rover/defines.h`

Deliverables:
- new log message for offboard/guided target updates
- new log message for command rejection and stale command timeout
- SITL coverage for setpoint timeout and safe stop

## Phase 1: Introduce a Dedicated OFFBOARD Mode
Goal: separate companion autonomy from generic `GUIDED` behavior.

Why:
- `GUIDED` currently mixes GCS repositioning, scripting, and companion control.
- `mode.h` already reserves mode number `30` for offboard.

Implementation:
- Add `Mode::Number::OFFBOARD = 30`.
- Add `ModeOffboard` with explicit submodes:
- `Position`
- `VelocityHeading`
- `VelocityTurnRate`
- `Trajectory`
- `Stop`
- `Hold`
- Implement explicit entry checks and an internal offboard state machine:
- `Inactive`
- `WaitingForHeartbeat`
- `Active`
- `LostHeartbeat`
- `Failsafe`
- Reuse existing Rover steering and throttle helpers where possible.
- Keep `GUIDED` for current GCS and scripting semantics.

Files:
- `Rover/mode.h`
- `Rover/mode.cpp`
- `Rover/Rover.h`
- new `Rover/mode_offboard.cpp`
- `Rover/system.cpp`

Parameters to add:
- `OFFB_ENABLE`
- `OFFB_TIMEOUT_MS`
- `OFFB_HB_TIMEOUT_MS`
- `OFFB_LOSS_ACT`
- `OFFB_MAX_POS_ERR`
- `OFFB_MAX_LATENCY_MS`

Parameter work:
- `Rover/Parameters.h`
- `Rover/Parameters.cpp`

Telemetry/logging:
- active offboard state
- last command type
- last command age
- heartbeat age
- loss action taken

## Phase 2: Offboard Heartbeat and Deadman Microservice
Goal: ensure the rover stops or falls back safely when the companion stalls, crashes, or partitions from the network.

Implementation:
- Add a dedicated heartbeat path distinct from setpoint arrival.
- Require a valid heartbeat before OFFBOARD becomes active.
- Require fresh commands or explicitly allow hold-last-command for a bounded period.
- On heartbeat loss, trigger a configurable action:
- `STOP`
- `HOLD`
- `LOITER`
- `RTL`
- `AUTO_MISSION_RESUME`

Files:
- `Rover/mode_offboard.cpp`
- `Rover/failsafe.cpp`
- `Rover/GCS_MAVLink_Rover.cpp`
- `Rover/AP_ExternalControl_Rover.{h,cpp}`

Recommended interface:
- Prefer extending `AP_ExternalControl_Rover` with explicit heartbeat APIs.
- Keep MAVLink handling as a parallel path for GCS and companion integration.

Acceptance criteria:
- OFFBOARD cannot arm and drive on stale companion traffic.
- Companion loss produces deterministic safe behavior in SITL.

## Phase 3: External Localization Contract
Goal: make companion-provided localization a first-class, health-aware input.

Implementation:
- Define an explicit external localization interface with pose, velocity, covariance, timestamp, frame source, and health.
- Add validation for stale localization, invalid covariance, and discontinuous pose jumps.
- Add logic for relocalization events and controlled mode fallback when localization health degrades.
- Document supported frame conventions between Rover local frame and ROS2 `map` and `odom`.

ArduRover work:
- add localization health state and telemetry
- add configurable localization timeout and jump detection
- gate OFFBOARD activation on localization health when position or trajectory control is selected

Files likely impacted:
- `Rover/Rover.h`
- `Rover/Rover.cpp`
- `Rover/system.cpp`
- `Rover/GCS_MAVLink_Rover.cpp`
- `Rover/AP_ExternalControl_Rover.{h,cpp}`

Companion work outside this repo:
- create a localization supervisor node that publishes the single approved localization stream to Rover
- own frame alignment, relocalization events, and diagnostics there

## Phase 4: Add a Real Motion Interface
Goal: move beyond single setpoint control to path and trajectory execution.

Current limitation:
- `GUIDED` can track positions and velocity-like commands, but the interface is still setpoint-oriented.

Implementation:
- Add an OFFBOARD trajectory submode that consumes a short rolling horizon.
- Add a path follower that accepts either:
- a sequence of waypoints with timing metadata
- a sequence of curvature plus speed samples
- a sequence of pose plus velocity samples
- Add command IDs and monotonic timestamps so old packets can be dropped.
- Add progress reporting for path index, distance remaining, and tracking error.

Preferred implementation strategy:
- Keep low-level tracking in Rover.
- Let Jetson send a rolling horizon path or trajectory, not raw servo commands.
- Reuse `AR_WPNav` where possible for path segment tracking.
- Add a dedicated offboard trajectory buffer instead of overloading mission items.

Files:
- `libraries/AR_WPNav/AR_WPNav.h`
- `libraries/AR_WPNav/AR_WPNav.cpp`
- `Rover/mode_offboard.cpp`
- `Rover/Rover.h`
- `Rover/GCS_MAVLink_Rover.cpp`
- `Rover/AP_ExternalControl_Rover.{h,cpp}`

Outputs to expose:
- tracking state
- trajectory command ID
- command age
- cross-track error
- along-track progress
- stop reason

## Phase 5: AUTO and Companion Mission Handoff
Goal: support full mission execution where mission ownership can move between ArduRover and the companion.

Current state:
- `AUTO` already supports `MAV_CMD_NAV_GUIDED_ENABLE` and `DO_GUIDED_LIMITS`.

Extend this by:
- adding mission primitives for companion-executed actions
- allowing `AUTO` to call into OFFBOARD instead of only `GUIDED`
- adding clear completion conditions, timeout conditions, and abort semantics

Mission behaviors to add:
- start offboard task
- wait for offboard task complete
- abort offboard task
- pause mission for external event
- resume onboard mission after companion task
- dock and undock hooks

Files:
- `Rover/mode_auto.cpp`
- `Rover/mode.h`
- `Rover/mode_offboard.cpp`
- `Rover/GCS_MAVLink_Rover.cpp`

Implementation note:
- Start with additive mission hooks before creating new MAVLink mission commands.
- Where possible, prototype with existing mission commands and Rover-side state first.

## Phase 6: Dynamic Safety Constraints From the Companion
Goal: allow Jetson to shape motion safely without replacing Rover failsafes.

Features:
- dynamic speed limits
- temporary no-go zones
- keep-out polygons
- virtual bumpers
- stop corridors near humans or machinery

Implementation:
- add an offboard safety input channel independent of motion commands
- merge companion constraints with Rover fence and local controller constraints
- make safety constraints expire automatically if not refreshed

Files:
- `Rover/fence.cpp`
- `Rover/mode_offboard.cpp`
- `Rover/GCS_MAVLink_Rover.cpp`
- `libraries/AR_WPNav/AR_WPNav.cpp`

## Phase 7: Docking and Precision Terminal Behaviors
Goal: make AGV terminal maneuvers first-class.

Features:
- docking approach mode
- fiducial or visual-servo terminal alignment
- retry and abort logic
- charger/contact confirmation hooks
- undock behavior

Existing entry points to review:
- `Rover/mode_dock.cpp`
- `Rover/precision_landing.cpp`

Work:
- determine whether current dock mode is sufficient for AGV docking
- if not, extend `ModeDock` or add OFFBOARD docking submode with tighter terminal constraints

## Phase 8: Telemetry, Diagnostics, and Operator Visibility
Goal: make offboard autonomy debuggable in the field.

Add telemetry for:
- offboard active/inactive/failsafe state
- current control authority owner
- localization health
- last command timestamp and age
- last heartbeat timestamp and age
- active trajectory ID
- rejection reason for the most recent invalid command
- fallback reason when leaving OFFBOARD

Files:
- `Rover/GCS_MAVLink_Rover.cpp`
- `Rover/Log.cpp`
- `Rover/defines.h`

Recommendation:
- keep a compact MAVLink-facing status summary
- keep richer detail in dataflash logs

## Companion Computer Plan
This work belongs in a separate ROS2 workspace, not in the ArduPilot tree.

Create these packages:
- `ardurover_bridge`
- `ardurover_msgs`
- `ardurover_mission_exec`
- `ardurover_safety_supervisor`
- `ardurover_localization_supervisor`

`ardurover_bridge` responsibilities:
- single owner of MAVLink or external-control session
- frame conversion
- heartbeat generation
- command rate limiting
- command ID generation
- telemetry translation into ROS2 topics and actions

Recommended ROS2 APIs:
- action: `NavigateToPose`
- action: `FollowPath`
- action: `Dock`
- service: `PauseAutonomy`
- service: `ResumeAutonomy`
- topic: approved localization
- topic: rover offboard status

Recommended ROS2 runtime design:
- lifecycle nodes for bridge, localization supervisor, and mission executive
- systemd-managed bringup
- fixed DDS config and `ROS_DOMAIN_ID`
- watchdog monitoring for bridge and safety supervisor

## Testing Plan

## Unit and Component Tests
Add tests for:
- OFFBOARD entry and exit checks
- heartbeat timeout
- command age timeout
- invalid command rejection
- localization timeout and jump rejection
- trajectory buffer ordering and stale packet rejection

Likely locations:
- `libraries/*/tests`
- new Rover-specific tests where practical

## SITL Tests
Add Rover SITL autotests for:
- OFFBOARD velocity control
- OFFBOARD position control
- OFFBOARD companion heartbeat loss
- AUTO to OFFBOARD handoff and back
- localization loss while offboard position control is active
- docking terminal behavior

Likely files:
- `Tools/autotest/rover.py`

## ROS2 Integration Tests
Outside this repo, create integration tests that run:
- SITL Rover
- MAVLink bridge
- localization publisher
- path/trajectory publisher
- mission executive

The minimum automated scenario set should include:
- map-localized navigation to pose
- rolling path updates around an obstacle
- companion process death and rover safe stop
- mission resume after offboard task completion

## Suggested Implementation Order
1. Instrument current guided/offboard paths and add tests.
2. Add dedicated `OFFBOARD` mode and parameters.
3. Add heartbeat and deadman behavior.
4. Add localization contract and health gating.
5. Add trajectory/path execution.
6. Extend `AUTO` for companion task handoff.
7. Add dynamic safety constraints.
8. Extend docking and terminal maneuvers.
9. Add richer telemetry and mission/operator tooling.

## Minimal Viable Product
An MVP suitable for early AGV trials should include only:
- OFFBOARD mode
- heartbeat and command timeout
- velocity plus yaw-rate control
- position goal control
- localization health gating
- safe fallback to stop or hold
- SITL tests for loss-of-companion behavior

This is enough to support a Jetson-side ROS2 navigator while keeping ArduRover responsible for safe motion and fallback.

## Non-Goals
- moving SLAM, Nav2, or AI perception into ArduRover
- making raw steering and throttle the primary autonomy interface
- removing Rover-side failsafe ownership
- replacing `AUTO` missions with Jetson-only mission logic in the first phase

## Branch Notes
This document is intended as the concrete implementation plan for the branch created from `origin/Rover-4.5` for AGV-focused ArduRover offboard autonomy work.
