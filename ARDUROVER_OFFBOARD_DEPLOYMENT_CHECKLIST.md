# ArduRover OFFBOARD Deployment Checklist

This checklist is for controlled real-vehicle trials of the Rover OFFBOARD implementation on this branch. OFFBOARD is disabled by default; enable it only after the vehicle passes the checks below.

## Required Vehicle Setup

- Physical emergency stop must be tested before autonomy trials.
- RC mode switch must be configured with a manual recovery mode and HOLD.
- Fence, battery, RC, GCS, and EKF failsafes must remain enabled and tested.
- `OFFB_ENABLE` must stay `0` until the companion-control link is ready for a trial.
- Set `OFFB_HB_TO` and `OFFB_CMD_TO` conservatively for the companion update rate. Start at `1000` to `1500` ms unless there is measured evidence for a different value.
- Keep `OFFB_LOSS_ACT=0` and `OFFB_STALE_ACT=0` for initial trials so heartbeat loss and stale commands stop the rover.

## Companion Control Contract

- Send an onboard-controller heartbeat before sending MAVLink OFFBOARD setpoints.
- Heartbeat and setpoints must come from the same MAVLink sysid, compid, and channel.
- Stream velocity/turn-rate commands at a fixed rate faster than `OFFB_CMD_TO`.
- Do not rely on OFFBOARD heading-only MAVLink commands; use velocity plus yaw-rate or position targets.
- DDS OFFBOARD tests require `microxrceddsgen` and a DDS test environment; do not claim DDS field readiness until those tests run.

## Pre-Field Test Gates

Run these before any powered vehicle trial:

```sh
./waf configure --board sitl
./waf rover
python3 -m py_compile Tools/autotest/rover.py
Tools/autotest/autotest.py test.Rover.OffboardSafety test.Rover.OffboardMode test.Rover.OffboardModeMAVLinkCompat test.Rover.OffboardModePath test.Rover.OffboardAutoMAVLinkCompat test.Rover.Offboard
```

If DDS is part of the trial, also run the AP_DDS tests in an environment with `microxrceddsgen` installed.

## First Vehicle Trial Sequence

1. Run with wheels raised or drive motors mechanically unloaded.
2. Confirm OFFBOARD cannot be entered while `OFFB_ENABLE=0`.
3. Set `OFFB_ENABLE=1`, enter OFFBOARD, and verify no motion occurs without heartbeat and commands.
4. Send low-speed velocity plus yaw-rate commands and verify RC/manual recovery.
5. Stop command streaming while continuing heartbeat and verify the rover stops.
6. Resume motion, stop heartbeat while continuing commands, and verify the rover stops.
7. Repeat at walking speed in a fenced open area with a spotter at the e-stop.

## Remaining Limitations

- This branch has SITL coverage for MAVLink OFFBOARD compatibility and safety fallback, but DDS-in-the-loop testing was not run in this workspace.
- Path, trajectory, docking, and dynamic safety constraints need additional field-specific validation before production use.
- Modified or untracked submodules must be resolved before release packaging.
