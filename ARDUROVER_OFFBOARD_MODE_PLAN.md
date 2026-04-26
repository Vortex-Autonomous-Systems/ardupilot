# ArduRover DDS Offboard Mode Plan

## Scope
This document plans a new Rover `OFFBOARD` mode designed specifically for AGV autonomy with a companion computer using DDS and ROS 2 for live control, state feedback, system introspection, and runtime control-plane operations.

Parameters and existing broad configuration workflows remain on MAVLink. The new work focuses on moving autonomy-critical live command and feedback interfaces onto DDS in a way that is consistent with the current ArduPilot architecture and the constraints in `AGENTS.md`.

This plan is based on the current implementation in:
- `libraries/AP_DDS/AP_DDS_Client.{h,cpp}`
- `libraries/AP_DDS/AP_DDS_ExternalControl.{h,cpp}`
- `libraries/AP_DDS/AP_DDS_External_Odom.{h,cpp}`
- `libraries/AP_DDS/AP_DDS_Topic_Table.h`
- `libraries/AP_DDS/AP_DDS_Service_Table.h`
- `libraries/AP_DDS/dds_xrce_profile.xml`
- `libraries/AP_ExternalControl/AP_ExternalControl.h`
- `Rover/mode.h`
- `Rover/mode.cpp`
- `Rover/mode_guided.cpp`
- `Rover/mode_auto.cpp`
- `Rover/GCS_MAVLink_Rover.cpp`
- `Rover/AP_ExternalControl_Rover.{h,cpp}`
- `Rover/failsafe.cpp`

## Current DDS Architecture

## What Exists Today
ArduPilot already contains a DDS XRCE client in `libraries/AP_DDS`.

Transport and runtime model:
- ArduPilot runs a Micro XRCE DDS client inside the vehicle process.
- The DDS client talks to a micro-ROS agent over serial or UDP.
- `AP_DDS_Client` runs in its own ArduPilot thread.
- DDS is exposed as an `AP_Vehicle` subgroup parameter block (`DDS_*`).

Current published DDS topics:
- `/ap/time`
- `/ap/clock`
- `/ap/navsat/navsat0`
- `/ap/battery/battery0`
- `/ap/pose/filtered`
- `/ap/twist/filtered`
- `/ap/geopose/filtered`
- `/ap/tf_static`

Current subscribed DDS topics:
- `/ap/joy`
- `/ap/tf`
- `/ap/cmd_vel`
- `/ap/cmd_gps_pose`

Current DDS services:
- `/ap/arm_motors`
- `/ap/mode_switch`

Current control path:
- DDS `cmd_vel` and `cmd_gps_pose` are handled by `AP_DDS_External_Control`.
- `AP_DDS_External_Control` calls the generic `AP_ExternalControl` singleton.
- Rover implements `AP_ExternalControl_Rover`.
- `AP_ExternalControl_Rover` currently only supports:
- linear velocity plus yaw-rate
- global position target
- Rover accepts those commands only when armed and in a guided-capable mode.
- Guided-capable in Rover today means `GUIDED` or guided-within-`AUTO`.

This means DDS already provides a narrow offboard capability, but it is not a dedicated offboard mode. It is a DDS frontend into the generic external-control and `GUIDED` paths.

## Current DDS Extension Points
The important extension points already present are:

1. `AP_DDS_Client`
- owns DDS session lifecycle
- defines topics and services tables
- dispatches incoming topics and service requests

2. `AP_DDS_External_Control`
- translates DDS message types into ArduPilot external-control calls
- currently performs frame conversion and limited semantic validation

3. `AP_ExternalControl`
- generic live control abstraction shared by MAVLink, DDS, and scripting
- currently too small for a real DDS-first offboard design

4. Rover `AP_ExternalControl_Rover`
- vehicle-specific implementation
- currently maps DDS control into `mode_guided`

5. Rover mode system
- already reserves mode number `30` for offboard in `Rover/mode.h`
- currently no dedicated Rover `OFFBOARD` implementation exists

6. `ModeReason::DDS_COMMAND`
- already exists and should continue to be used for DDS-driven mode changes and service actions

## Current Gaps
The current DDS implementation is not sufficient for full AGV offboard autonomy because:
- there is no dedicated `OFFBOARD` mode
- DDS commands are routed through `GUIDED`
- there is no DDS heartbeat or deadman contract
- there is no DDS-native path/trajectory interface
- there is no DDS-native offboard status topic
- there is no explicit DDS command acknowledgement or rejection reason topic
- localization feedback exists, but localization health and control gating do not
- arming and mode switching are available via DDS service, but no broader DDS control-plane API exists

## Design Goals
The new DDS offboard mode should:
- use DDS as the primary live autonomy interface
- keep parameters and general configuration on MAVLink
- keep safety-critical fallback and stop behavior in ArduRover
- expose enough control-plane services and state topics to let ROS 2 drive the robot as a real autonomy endpoint
- avoid moving large ROS 2 or planning logic into ArduPilot
- stay additive and minimally invasive where possible

## Non-Goals
- moving parameter management from MAVLink to DDS in this phase
- moving SLAM, Nav2, or mission/task reasoning into ArduPilot
- using raw steering and throttle as the primary autonomy interface
- replacing Rover failsafes with companion-side logic

## Target DDS Offboard Architecture

## Core Principle
DDS becomes the live autonomy surface.

MAVLink remains responsible for:
- parameters
- traditional GCS workflows
- logs and field tooling compatibility
- mission upload and broad ecosystem support

DDS becomes responsible for:
- live offboard control commands
- live state feedback
- offboard status and command health
- runtime service operations relevant to autonomy
- localization input for offboard control
- trajectory and path ingestion
- offboard diagnostics

## Mode Ownership Model
Add a new Rover mode:
- `OFFBOARD = 30`

This mode should be distinct from `GUIDED`.

Why:
- `GUIDED` today is a broad compatibility mode for MAVLink, scripting, and miscellaneous external commands.
- DDS offboard control needs explicit health gating, ownership semantics, and telemetry.
- Reusing `GUIDED` would keep too much ambiguity in behavior and state transitions.

## Proposed High-Level Stack
DDS layer:
- `AP_DDS_Client`
- DDS topic table and service table
- DDS message profiles in `dds_xrce_profile.xml`

Vehicle abstraction layer:
- extend `AP_ExternalControl`
- add DDS-first control APIs for offboard

Rover implementation layer:
- implement `ModeOffboard`
- extend `AP_ExternalControl_Rover`
- add offboard state storage and health checks in Rover

Navigation/control layer:
- reuse `AR_WPNav` where appropriate
- add offboard-specific control buffer for trajectory/path commands

## Proposed DDS Interfaces

## Services
Keep existing:
- `arm_motors`
- `mode_switch`

Add new DDS services for live autonomy operations:
- `offboard_enable`
- `offboard_disable`
- `offboard_clear_fault`
- `offboard_set_control_mode`
- `offboard_set_limits`
- `offboard_stop`
- `offboard_hold`
- `offboard_resume`

Service design rules:
- services change mode, authority, or static runtime behavior
- topics stream live commands and feedback
- mode changes caused by DDS services should use `ModeReason::DDS_COMMAND`

## Command Topics
Add dedicated DDS topics for offboard live control:
- `offboard_heartbeat`
- `offboard_command_velocity`
- `offboard_command_pose`
- `offboard_command_global_pose`
- `offboard_command_path`
- `offboard_command_trajectory`
- `offboard_command_stop`
- `offboard_safety_constraints`

Recommended semantics:
- heartbeat is independent from command arrival
- commands carry command IDs and timestamps
- path/trajectory topics carry a rolling horizon, not entire missions
- command topics are DDS-first and mapped to OFFBOARD, not GUIDED

## Feedback Topics
Add DDS state topics for autonomy feedback:
- `offboard_status`
- `offboard_command_result`
- `offboard_tracking_status`
- `offboard_localization_status`
- `offboard_safety_status`
- `vehicle_mode_status`

Recommended contents:

`offboard_status`
- enabled
- mode active
- submode active
- heartbeat age
- command age
- control authority owner
- failover state

`offboard_command_result`
- last command ID
- accepted/rejected
- rejection reason enum
- timestamp processed

`offboard_tracking_status`
- active target type
- active trajectory ID
- position error
- heading error
- velocity error
- distance remaining
- progress fraction if available

`offboard_localization_status`
- source valid
- age
- reset counter
- discontinuity detected
- health enum

## Localization Topics
Retain and extend DDS localization support:
- current `/ap/tf` odometry intake exists via `AP_DDS_External_Odom`
- current outgoing pose/twist/geopose are useful and should stay

Add explicit localization ingestion topics if needed:
- `offboard_localization_pose`
- `offboard_localization_twist`
- `offboard_localization_health`

This avoids overloading generic TF for all future offboard localization workflows.

## Offboard Mode Behavior

## OFFBOARD Submodes
Implement these submodes in `ModeOffboard`:
- `Idle`
- `Hold`
- `Stop`
- `VelocityHeading`
- `VelocityTurnRate`
- `Position`
- `GlobalPosition`
- `Path`
- `Trajectory`

Minimal first phase should implement:
- `Hold`
- `Stop`
- `VelocityTurnRate`
- `GlobalPosition`

## OFFBOARD State Machine
Internal state machine should include:
- `Disabled`
- `WaitingForHeartbeat`
- `Active`
- `CommandStale`
- `HeartbeatLost`
- `LocalizationInvalid`
- `Failsafe`

Entry requirements:
- vehicle armed if motion command mode requested
- valid heartbeat present
- fresh control command present or explicit hold request
- localization healthy for position/path/trajectory submodes
- mode switching allowed by Rover and its failsafe state

Exit behavior:
- on explicit disable, go to configured fallback or hold
- on DDS disconnect, go to configured loss action
- on stale commands, stop or hold based on parameters
- on localization invalidation, leave trajectory/position submodes and enter safe fallback

## Heartbeat and Deadman Contract
This is the most important part of the design.

Add a dedicated DDS heartbeat topic and internal watchdog.

Required behavior:
- heartbeat freshness is tracked separately from command freshness
- OFFBOARD cannot become active without recent heartbeats
- if heartbeat is lost, offboard mode triggers a configurable action
- if commands stop but heartbeat remains, use a separate command-stale action

Recommended parameters:
- `OFFB_HB_TIMEOUT_MS`
- `OFFB_CMD_TIMEOUT_MS`
- `OFFB_LOSS_ACT`
- `OFFB_STALE_ACT`

Recommended actions:
- stop
- hold
- loiter
- rtl
- resume auto if the mode was entered from auto-handoff

## Vehicle Authority Rules
Authority model should be explicit:
- RC/manual override always wins if configured to do so
- failsafes always win over offboard
- DDS control owns motion only while OFFBOARD is active and healthy
- GCS or RC mode changes can forcibly revoke offboard authority

Telemetry should indicate who currently owns motion authority.

## Concrete Code Changes

## 1. Extend `AP_ExternalControl`
Current interface only supports:
- `set_linear_velocity_and_yaw_rate`
- `set_global_position`

Extend it to support the offboard DDS design:
- `set_offboard_heartbeat(...)`
- `set_local_position_target(...)`
- `set_global_position_target(...)`
- `set_velocity_heading_target(...)`
- `set_velocity_turn_rate_target(...)`
- `set_path_target(...)`
- `set_trajectory_target(...)`
- `set_offboard_stop(...)`
- `set_offboard_hold(...)`
- `set_offboard_limits(...)`

Files:
- `libraries/AP_ExternalControl/AP_ExternalControl.h`

Rationale:
- keep DDS message translation outside Rover mode logic
- make `AP_DDS_External_Control` a true translation layer

## 2. Extend `AP_DDS_External_Control`
Current responsibilities are minimal translation and frame conversion.

Extend it to:
- translate new DDS topics into the new `AP_ExternalControl` APIs
- validate frame IDs and required fields
- validate command timestamp monotonicity where possible
- provide structured failure results for logging and future feedback topics

Files:
- `libraries/AP_DDS/AP_DDS_ExternalControl.h`
- `libraries/AP_DDS/AP_DDS_ExternalControl.cpp`

## 3. Add `ModeOffboard` to Rover
Implement a new mode and add it to Rover's mode map.

Files:
- `Rover/mode.h`
- `Rover/mode.cpp`
- `Rover/Rover.h`
- new `Rover/mode_offboard.cpp`
- `Rover/system.cpp`

Responsibilities:
- own offboard submode logic
- own heartbeat and command freshness tracking
- own offboard-safe stop/hold fallback inside the mode
- expose status accessors for DDS telemetry

## 4. Extend `AP_ExternalControl_Rover`
Currently Rover external control only works if `in_guided_mode()` is true.

Change this so DDS control can target the new OFFBOARD mode.

Needed changes:
- add helpers for offboard readiness, not just guided readiness
- dispatch commands into `mode_offboard` instead of `mode_guided` when OFFBOARD is active
- preserve existing DDS-through-GUIDED behavior until the new OFFBOARD path is complete

Files:
- `Rover/AP_ExternalControl_Rover.h`
- `Rover/AP_ExternalControl_Rover.cpp`
- `Rover/Rover.cpp`

## 5. Extend `AP_DDS_Client` Topics and Services
Add new topic and service definitions.

Files:
- `libraries/AP_DDS/AP_DDS_Client.h`
- `libraries/AP_DDS/AP_DDS_Client.cpp`
- `libraries/AP_DDS/AP_DDS_Topic_Table.h`
- `libraries/AP_DDS/AP_DDS_Service_Table.h`
- `libraries/AP_DDS/dds_xrce_profile.xml`
- `libraries/AP_DDS/Idl/...`

Implementation notes:
- keep published state topics lightweight and bounded
- choose QoS deliberately per topic class
- command topics should be reliable with bounded history
- high-rate telemetry may use best effort where loss is acceptable

## 6. Add New ROS 2 Messages and Services
Current `ardupilot_msgs` package is too small for a full DDS offboard interface.

Add new types under:
- `Tools/ros2/ardupilot_msgs/msg`
- `Tools/ros2/ardupilot_msgs/srv`

Likely messages:
- `OffboardHeartbeat.msg`
- `OffboardStatus.msg`
- `OffboardCommandResult.msg`
- `OffboardTrackingStatus.msg`
- `OffboardLocalizationStatus.msg`
- `OffboardSafetyConstraints.msg`
- `PathCommand.msg`
- `TrajectoryCommand.msg`

Likely services:
- `OffboardEnable.srv`
- `OffboardSetMode.srv`
- `OffboardStop.srv`
- `OffboardClearFault.srv`

Then add matching IDL under `libraries/AP_DDS/Idl/...`.

## 7. Add Offboard Parameters in Rover
Parameters should remain in MAVLink but control DDS offboard behavior.

Recommended first set:
- `OFFB_ENABLE`
- `OFFB_HB_TO`
- `OFFB_CMD_TO`
- `OFFB_LOSS_ACT`
- `OFFB_STALE_ACT`
- `OFFB_LOC_TO`
- `OFFB_MAXERR`
- `OFFB_MAXLAT`
- `OFFB_PATH_LEN`
- `OFFB_TRJ_LEN`

Files:
- `Rover/Parameters.h`
- `Rover/Parameters.cpp`

Follow ArduPilot parameter rules:
- do not renumber existing indices
- document all relevant annotations
- keep names within the project's parameter length constraints

## 8. Add Navigation Extension for Path and Trajectory Control
The current `AR_WPNav` is waypoint/path-following oriented and can likely be reused for some of this work, but not all.

Recommended approach:
- keep simple velocity and global-position control in `ModeOffboard`
- add a small offboard path/trajectory buffer in Rover first
- reuse `AR_WPNav` for segment following where practical
- avoid large speculative refactors in `AR_WPNav` until the OFFBOARD mode contract is proven

Files likely impacted in later phases:
- `libraries/AR_WPNav/AR_WPNav.h`
- `libraries/AR_WPNav/AR_WPNav.cpp`
- `Rover/mode_offboard.cpp`

## DDS QoS Plan

## Reliable Topics
Use reliable QoS for:
- heartbeat
- mode or authority change commands
- stop and hold commands
- command acknowledgements
- offboard status

## Best-Effort Topics
Use best effort for:
- high-rate pose and twist telemetry
- tracking status if published at high rate

## Transient Local
Use transient local for:
- static capability and metadata topics if added later
- static transforms, already done today

## DDS Profile Work
All topic and service additions require updates to:
- `libraries/AP_DDS/dds_xrce_profile.xml`

This file is the authoritative mapping between DDS object IDs and ROS 2-facing topic/service names. The offboard design should not bypass it.

## Localization Integration Plan
Current DDS localization path uses `tf` to feed `AP_VisualOdom` via `AP_DDS_External_Odom`.

Plan:
- keep current TF ingestion path for compatibility
- add explicit offboard localization health reporting
- add localization age and discontinuity gating for OFFBOARD mode entry and operation
- avoid making path/trajectory submodes active when localization is stale or invalid

This should be additive, not a replacement for the current visual odom integration.

## Mission and AUTO Integration
The first OFFBOARD implementation should be manually selectable via DDS `mode_switch` and GCS mode change.

Later, integrate with `AUTO` using explicit handoff support.

Plan for later phases:
- allow `AUTO` to hand control to `OFFBOARD` instead of only `GUIDED`
- add completion and timeout semantics for offboard mission items
- preserve mission fallback if DDS companion disappears

Files later:
- `Rover/mode_auto.cpp`

## Safety and Failsafe Integration
OFFBOARD mode must integrate with existing Rover failsafes, not replace them.

Required behavior:
- RC failsafe still applies
- battery failsafe still applies
- fence still applies
- EKF/localization failures still apply
- CPU/main loop failsafe still applies

Additional OFFBOARD-specific failsafe conditions:
- DDS disconnected
- DDS heartbeat stale
- DDS command stale
- localization invalid for position-based offboard submodes

Files:
- `Rover/failsafe.cpp`
- `Rover/mode_offboard.cpp`
- possibly `Rover/system.cpp`

## Suggested Implementation Phases

## Phase 1: DDS Offboard MVP
- add `OFFBOARD` mode
- add heartbeat topic
- add offboard status topic
- extend `AP_ExternalControl` and Rover implementation
- support velocity-turn-rate and global-position commands
- add freshness and heartbeat gating
- add stop and hold services

## Phase 2: DDS Feedback and Diagnostics
- add command result topic
- add tracking status topic
- add localization status topic
- improve log messages and dataflash support

## Phase 3: DDS Path and Trajectory
- add path and trajectory command messages
- add bounded command buffers
- add tracking and progress reporting
- add SITL tests for path refresh and stale rejection

## Phase 4: AUTO Handoff and Mission Integration
- add `AUTO` to `OFFBOARD` handoff support
- define mission completion semantics
- add fallback to `AUTO` or safe hold on DDS failure

## Testing Plan

## AP_DDS Unit and Component Tests
Add tests for:
- message translation in `AP_DDS_External_Control`
- frame handling
- stale or malformed command rejection
- localization transform conversion

Likely files:
- `libraries/AP_DDS/tests/*`

## Rover SITL Tests
Add Rover tests for:
- OFFBOARD entry and exit
- heartbeat loss
- command timeout
- localization invalidation during OFFBOARD
- velocity control tracking
- global position control tracking
- forced mode change out of OFFBOARD

Likely location:
- `Tools/autotest/rover.py`

## ROS 2 Integration Tests
Extend `Tools/ros2/ardupilot_dds_tests` with Rover-specific tests:
- mode switch to OFFBOARD
- stream heartbeat and velocity control
- verify status feedback
- drop heartbeat and verify rover fallback

The existing DDS tests are currently demonstration-level and Plane-focused in places, so Rover-specific offboard tests should be added explicitly.

## Risks and Constraints
- DDS XRCE bandwidth and memory are constrained; message sizes must stay bounded.
- Path/trajectory messages must be carefully sized for embedded targets.
- OFFBOARD should be additive so `GUIDED` and MAVLink workflows remain intact.
- New DDS topics and services increase maintenance burden across generated C types, ROS messages, XML profiles, and client code.
- Safety review is required before field use because this changes live motion authority.

## Recommended First Commit Scope
Keep the first implementation narrow:
- add `OFFBOARD` mode scaffold
- add offboard heartbeat topic
- add offboard status topic
- route DDS velocity control to OFFBOARD when active
- add heartbeat timeout fallback to stop or hold

That provides a clean DDS-native offboard backbone without prematurely locking in the full path/trajectory API.

## Summary
The proper way to add DDS-first offboard control to Rover is not to keep stretching DDS into `GUIDED`, but to:
- introduce a dedicated Rover `OFFBOARD` mode
- extend `AP_ExternalControl` into a real live-control abstraction
- expand `AP_DDS` topics and services to cover heartbeat, command, and status
- use DDS for live autonomy control and feedback
- keep MAVLink for parameters, configuration, and existing ecosystem workflows

This approach fits the current ArduPilot architecture, uses the existing DDS stack rather than bypassing it, and keeps the safety-critical fallback behavior inside ArduRover.
