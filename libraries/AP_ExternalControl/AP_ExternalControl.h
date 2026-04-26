/*
  external control library for MAVLink, DDS and scripting
 */

#pragma once

#include "AP_ExternalControl_config.h"

#if AP_EXTERNAL_CONTROL_ENABLED

#include <stdint.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

class AP_ExternalControl
{
public:

    struct OffboardStatus {
        bool enabled;
        bool active;
        uint8_t state;
        uint8_t submode;
        uint32_t heartbeat_age_ms;
        uint32_t command_age_ms;
        uint8_t failsafe_action;
    };

    struct OffboardCommandResult {
        uint32_t command_id;
        bool accepted;
        uint8_t rejection_reason;
        uint32_t timestamp_ms;
    };

    struct OffboardTrackingStatus {
        uint8_t active_submode;
        float position_error;
        float heading_error;
        float velocity_error;
        float distance_remaining;
        bool target_valid;
    };

    struct OffboardLocalizationStatus {
        bool source_valid;
        uint32_t age_ms;
        uint8_t reset_count;
        bool discontinuity_detected;
        uint8_t health;
    };

    AP_ExternalControl();
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    virtual bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_velocity_turn_rate_target(const Vector3f &linear_velocity, float yaw_rate_rads) WARN_IF_UNUSED {
        return set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate_rads);
    }

    /*
        Sets the target global position with standard guided mode behavior.
    */
    virtual bool set_global_position(const Location& loc) WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_global_position_target(const Location& loc) WARN_IF_UNUSED {
        return set_global_position(loc);
    }

    virtual bool set_offboard_heartbeat() WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_stop() WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_hold() WARN_IF_UNUSED {
        return false;
    }

    virtual bool get_offboard_status(OffboardStatus &status) const WARN_IF_UNUSED {
        status = {};
        return false;
    }

    virtual bool get_offboard_command_result(OffboardCommandResult &result) const WARN_IF_UNUSED {
        result = {};
        return false;
    }

    virtual bool get_offboard_tracking_status(OffboardTrackingStatus &tracking) const WARN_IF_UNUSED {
        tracking = {};
        return false;
    }

    virtual bool get_offboard_localization_status(OffboardLocalizationStatus &loc_status) const WARN_IF_UNUSED {
        loc_status = {};
        return false;
    }

    virtual bool set_offboard_path(uint32_t path_id, float speed, const Location *points, uint8_t num_points) WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_trajectory(uint32_t traj_id, float speed, const Location *points, const float *speeds, uint8_t num_points) WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_pause() WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_resume() WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_abort() WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_safety_constraints(float max_speed, float min_distance) WARN_IF_UNUSED {
        return false;
    }

    virtual bool set_offboard_dock() WARN_IF_UNUSED {
        return false;
    }

    static AP_ExternalControl *get_singleton(void) WARN_IF_UNUSED {
        return singleton;
    }
protected:
    ~AP_ExternalControl() {}

private:
    static AP_ExternalControl *singleton;
};


namespace AP
{
AP_ExternalControl *externalcontrol();
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
