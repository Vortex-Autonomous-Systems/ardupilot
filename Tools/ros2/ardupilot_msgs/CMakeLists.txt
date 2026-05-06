cmake_minimum_required(VERSION 3.8)
project(ardupilot_msgs)

# --------------------------------------------------------------------------- #
# Find dependencies.

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------------------------------- #
# Generate and export message interfaces.

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GlobalPosition.msg"
  "msg/Rc.msg"
  "msg/Status.msg"
  "msg/Airspeed.msg"
  "msg/RangeFinder.msg"
  "msg/Proximity.msg"
  "msg/Barometer.msg"
  "msg/Magnetometer.msg"
  "msg/OpticalFlow.msg"
  "msg/EKFStatus.msg"
  "msg/ServoOutputs.msg"
  "msg/GuidedTarget.msg"
  "msg/AttitudeTarget.msg"
  "msg/GuidedState.msg"
  "srv/ArmMotors.srv"
  "srv/ModeSwitch.srv"
  "srv/Takeoff.srv"
  "srv/SetGuidedTarget.srv"
  "srv/Land.srv"
  "srv/ReturnToLaunch.srv"
  "srv/SetHome.srv"
  "srv/ServoControl.srv"
  "srv/FenceControl.srv"
  DEPENDENCIES geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
# Call last.

ament_package()
