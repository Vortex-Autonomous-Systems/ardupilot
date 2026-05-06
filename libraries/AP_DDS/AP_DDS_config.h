#pragma once

#include <AP_GPS/AP_GPS_config.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>
#include <AP_VisualOdom/AP_VisualOdom_config.h>
#include <RC_Channel/RC_Channel_config.h>
#include <AP_RSSI/AP_RSSI_config.h>

#ifndef AP_DDS_ENABLED
#define AP_DDS_ENABLED 1
#endif

// UDP only on SITL for now
#ifndef AP_DDS_UDP_ENABLED
#define AP_DDS_UDP_ENABLED AP_DDS_ENABLED && AP_NETWORKING_ENABLED
#endif

#include <AP_VisualOdom/AP_VisualOdom_config.h>
#ifndef AP_DDS_VISUALODOM_ENABLED
#define AP_DDS_VISUALODOM_ENABLED HAL_VISUALODOM_ENABLED && AP_DDS_ENABLED
#endif

// Whether experimental interfaces are enabled.
#ifndef AP_DDS_EXPERIMENTAL_ENABLED
#define AP_DDS_EXPERIMENTAL_ENABLED 1
#endif

#ifndef AP_DDS_IMU_PUB_ENABLED
#define AP_DDS_IMU_PUB_ENABLED AP_DDS_EXPERIMENTAL_ENABLED
#endif

#ifndef AP_DDS_DELAY_IMU_TOPIC_MS
#define AP_DDS_DELAY_IMU_TOPIC_MS 5
#endif

#ifndef AP_DDS_TIME_PUB_ENABLED
#define AP_DDS_TIME_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_TIME_TOPIC_MS
#define AP_DDS_DELAY_TIME_TOPIC_MS 10
#endif

#ifndef AP_DDS_NAVSATFIX_PUB_ENABLED
#define AP_DDS_NAVSATFIX_PUB_ENABLED AP_GPS_ENABLED
#endif

#ifndef AP_DDS_STATIC_TF_PUB_ENABLED
#define AP_DDS_STATIC_TF_PUB_ENABLED AP_GPS_ENABLED
#endif

#ifndef AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#define AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS
#define AP_DDS_DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS 1000
#endif

#ifndef AP_DDS_GEOPOSE_PUB_ENABLED
#define AP_DDS_GEOPOSE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_GEO_POSE_TOPIC_MS
#define AP_DDS_DELAY_GEO_POSE_TOPIC_MS 33
#endif

#ifndef AP_DDS_LOCAL_POSE_PUB_ENABLED
#define AP_DDS_LOCAL_POSE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS
#define AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS 33
#endif

#ifndef AP_DDS_LOCAL_VEL_PUB_ENABLED
#define AP_DDS_LOCAL_VEL_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_LOCAL_VELOCITY_TOPIC_MS
#define AP_DDS_DELAY_LOCAL_VELOCITY_TOPIC_MS 33
#endif

#ifndef AP_DDS_AIRSPEED_PUB_ENABLED
#define AP_DDS_AIRSPEED_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_AIRSPEED_TOPIC_MS
#define AP_DDS_DELAY_AIRSPEED_TOPIC_MS 33
#endif

#ifndef AP_DDS_RC_PUB_ENABLED
#define AP_DDS_RC_PUB_ENABLED (AP_RSSI_ENABLED && AP_RC_CHANNEL_ENABLED)
#endif

#ifndef AP_DDS_DELAY_RC_TOPIC_MS
#define AP_DDS_DELAY_RC_TOPIC_MS 100
#endif

#ifndef AP_DDS_BATTERY_STATE_PUB_ENABLED
#define AP_DDS_BATTERY_STATE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_BATTERY_STATE_TOPIC_MS
#define AP_DDS_DELAY_BATTERY_STATE_TOPIC_MS 1000
#endif

#ifndef AP_DDS_DELAY_STATUS_TOPIC_MS
#define AP_DDS_DELAY_STATUS_TOPIC_MS 100
#endif

#ifndef AP_DDS_CLOCK_PUB_ENABLED
#define AP_DDS_CLOCK_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_CLOCK_TOPIC_MS
#define AP_DDS_DELAY_CLOCK_TOPIC_MS 10
#endif

#ifndef AP_DDS_GOAL_PUB_ENABLED
#define AP_DDS_GOAL_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_GOAL_TOPIC_MS
#define AP_DDS_DELAY_GOAL_TOPIC_MS  200
#endif
#ifndef AP_DDS_STATUS_PUB_ENABLED
#define AP_DDS_STATUS_PUB_ENABLED 1
#endif

#ifndef AP_DDS_JOY_SUB_ENABLED
#define AP_DDS_JOY_SUB_ENABLED 1
#endif

#ifndef AP_DDS_VEL_CTRL_ENABLED
#define AP_DDS_VEL_CTRL_ENABLED 1
#endif

#ifndef AP_DDS_GLOBAL_POS_CTRL_ENABLED
#define AP_DDS_GLOBAL_POS_CTRL_ENABLED 1
#endif

#ifndef AP_DDS_DYNAMIC_TF_SUB_ENABLED
#define AP_DDS_DYNAMIC_TF_SUB_ENABLED 1
#endif

#ifndef AP_DDS_ARM_SERVER_ENABLED
#define AP_DDS_ARM_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_MODE_SWITCH_SERVER_ENABLED
#define AP_DDS_MODE_SWITCH_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#define AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_PARAMETER_SERVER_ENABLED
#define AP_DDS_PARAMETER_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_ARM_CHECK_SERVER_ENABLED
#define AP_DDS_ARM_CHECK_SERVER_ENABLED 1
#endif

// =====================================================
// Phase 3: RTL/Land Services
// =====================================================

#ifndef AP_DDS_LAND_SERVER_ENABLED
#define AP_DDS_LAND_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_RTL_SERVER_ENABLED
#define AP_DDS_RTL_SERVER_ENABLED 1
#endif

#ifndef AP_DDS_SET_HOME_SERVER_ENABLED
#define AP_DDS_SET_HOME_SERVER_ENABLED 1
#endif

// =====================================================
// Phase 4: Servo/Actuator and Fence Control
// =====================================================

#ifndef AP_DDS_SERVO_CONTROL_SERVER_ENABLED
#define AP_DDS_SERVO_CONTROL_SERVER_ENABLED 1
#endif

#include <AC_Fence/AC_Fence_config.h>
#ifndef AP_DDS_FENCE_CONTROL_SERVER_ENABLED
#define AP_DDS_FENCE_CONTROL_SERVER_ENABLED AP_FENCE_ENABLED
#endif

// =====================================================
// Phase 1: Sensor Data Publishers
// =====================================================

// RangeFinder publisher support
#include <AP_RangeFinder/AP_RangeFinder_config.h>
#ifndef AP_DDS_RANGEFINDER_PUB_ENABLED
#define AP_DDS_RANGEFINDER_PUB_ENABLED AP_RANGEFINDER_ENABLED
#endif

#ifndef AP_DDS_DELAY_RANGEFINDER_TOPIC_MS
#define AP_DDS_DELAY_RANGEFINDER_TOPIC_MS 50
#endif

// Proximity sensor publisher support
#include <AP_Proximity/AP_Proximity_config.h>
#ifndef AP_DDS_PROXIMITY_PUB_ENABLED
#define AP_DDS_PROXIMITY_PUB_ENABLED HAL_PROXIMITY_ENABLED
#endif

#ifndef AP_DDS_DELAY_PROXIMITY_TOPIC_MS
#define AP_DDS_DELAY_PROXIMITY_TOPIC_MS 100
#endif

// Barometer publisher support
#include <AP_Baro/AP_Baro_config.h>
#ifndef AP_DDS_BAROMETER_PUB_ENABLED
#define AP_DDS_BAROMETER_PUB_ENABLED AP_BARO_ENABLED
#endif

#ifndef AP_DDS_DELAY_BAROMETER_TOPIC_MS
#define AP_DDS_DELAY_BAROMETER_TOPIC_MS 50
#endif

// Magnetometer publisher support
#include <AP_Compass/AP_Compass_config.h>
#ifndef AP_DDS_MAGNETOMETER_PUB_ENABLED
#define AP_DDS_MAGNETOMETER_PUB_ENABLED AP_COMPASS_ENABLED
#endif

#ifndef AP_DDS_DELAY_MAGNETOMETER_TOPIC_MS
#define AP_DDS_DELAY_MAGNETOMETER_TOPIC_MS 100
#endif

// OpticalFlow publisher support
#include <AP_OpticalFlow/AP_OpticalFlow_config.h>
#ifndef AP_DDS_OPTICALFLOW_PUB_ENABLED
#define AP_DDS_OPTICALFLOW_PUB_ENABLED AP_OPTICALFLOW_ENABLED
#endif

#ifndef AP_DDS_DELAY_OPTICALFLOW_TOPIC_MS
#define AP_DDS_DELAY_OPTICALFLOW_TOPIC_MS 50
#endif

// EKF Status publisher support
#ifndef AP_DDS_EKFSTATUS_PUB_ENABLED
#define AP_DDS_EKFSTATUS_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_EKFSTATUS_TOPIC_MS
#define AP_DDS_DELAY_EKFSTATUS_TOPIC_MS 100
#endif

// ServoOutputs publisher support
#ifndef AP_DDS_SERVOOUTPUTS_PUB_ENABLED
#define AP_DDS_SERVOOUTPUTS_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_SERVOOUTPUTS_TOPIC_MS
#define AP_DDS_DELAY_SERVOOUTPUTS_TOPIC_MS 50
#endif

// =====================================================
// Phase 2: Guided Mode Extensions
// =====================================================

// Guided Target subscription support (position/velocity/acceleration control)
#ifndef AP_DDS_GUIDED_TARGET_SUB_ENABLED
#define AP_DDS_GUIDED_TARGET_SUB_ENABLED 1
#endif

// Attitude Target subscription support (quaternion/angular rate control)
#ifndef AP_DDS_ATTITUDE_TARGET_SUB_ENABLED
#define AP_DDS_ATTITUDE_TARGET_SUB_ENABLED 1
#endif

// Guided State publisher support
#ifndef AP_DDS_GUIDED_STATE_PUB_ENABLED
#define AP_DDS_GUIDED_STATE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_GUIDED_STATE_TOPIC_MS
#define AP_DDS_DELAY_GUIDED_STATE_TOPIC_MS 100
#endif

// Whether to include Twist support
#define AP_DDS_NEEDS_TWIST AP_DDS_VEL_CTRL_ENABLED || AP_DDS_LOCAL_VEL_PUB_ENABLED

// Whether to include Transform support
#define AP_DDS_NEEDS_TRANSFORMS AP_DDS_DYNAMIC_TF_SUB_ENABLED || AP_DDS_STATIC_TF_PUB_ENABLED

// Whether DDS needs GPS
#define AP_DDS_NEEDS_GPS AP_DDS_NAVSATFIX_PUB_ENABLED || AP_DDS_STATIC_TF_PUB_ENABLED

#ifndef AP_DDS_DEFAULT_UDP_IP_ADDR
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AP_DDS_DEFAULT_UDP_IP_ADDR "192.168.144.2"
#else
#define AP_DDS_DEFAULT_UDP_IP_ADDR "127.0.0.1"
#endif
#endif

#ifndef AP_DDS_PARTICIPANT_NAME
#define AP_DDS_PARTICIPANT_NAME "ap"
#endif
