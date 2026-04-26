#include "Rover.h"

bool ModeOffboard::enabled() const
{
    return rover.g2.offboard_enable.get() > 0;
}

bool ModeOffboard::_enter()
{
    if (!enabled()) {
        return false;
    }

    const uint32_t now_ms = millis();
    _submode = SubMode::Hold;
    _state = heartbeat_fresh(now_ms) ? State::Active : State::WaitingForHeartbeat;
    _have_target = false;
    _reached_destination = false;
    _desired_speed = 0.0f;
    _desired_turn_rate_cds = 0.0f;
    _sent_heartbeat_lost_warning = false;
    _sent_command_stale_warning = false;
    _sent_localization_invalid_warning = false;
    _mavlink_owner_set = false;
    _last_cmd_id = 0;
    _last_cmd_accepted = false;
    _last_cmd_reject_reason = 0;
    _last_cmd_result_ms = 0;
    _last_localization_valid_ms = 0;
    _path_count = 0;
    _path_index = 0;
    _path_id = 0;
    _paused = false;
    _dock_complete = false;
    _safety_constraints = {};

    g2.wp_nav.init();
    return true;
}

void ModeOffboard::_exit()
{
    _state = State::Disabled;
    _have_target = false;
    _mavlink_owner_set = false;
}

void ModeOffboard::update()
{
    const uint32_t now_ms = millis();
    update_state(now_ms);

    // Phase 5: if paused, just hold
    if (_paused) {
        stop_vehicle();
        return;
    }

    switch (_state) {
    case State::Active:
        switch (_submode) {
        case SubMode::VelocityTurnRate:
            run_velocity_turn_rate();
            return;
        case SubMode::GlobalPosition:
            run_global_position();
            return;
        case SubMode::Path:
        case SubMode::Trajectory:
            run_path();
            return;
        case SubMode::Dock:
            run_dock();
            return;
        case SubMode::Hold:
            set_hold();
            break;
        case SubMode::Stop:
            set_stop();
            break;
        }
        break;
    case State::WaitingForHeartbeat:
        set_hold();
        break;
    case State::CommandStale:
    case State::HeartbeatLost:
    case State::LocalizationInvalid:
        apply_failsafe();
        break;
    case State::Disabled:
        set_hold();
        break;
    }

    if (_submode == SubMode::Hold) {
        float throttle = 0.0f;

        if (rover.is_balancebot()) {
            rover.balancebot_pitch_control(throttle);
        }

        g2.motors.set_mainsail(100.0f);
        g2.motors.set_wingsail(0.0f);
        g2.motors.set_throttle(throttle);
        g2.motors.set_steering(0.0f);
    } else {
        stop_vehicle();
    }
}

float ModeOffboard::wp_bearing() const
{
    if (_submode == SubMode::GlobalPosition) {
        return g2.wp_nav.wp_bearing_cd() * 0.01f;
    }
    return 0.0f;
}

float ModeOffboard::nav_bearing() const
{
    if (_submode == SubMode::GlobalPosition) {
        return g2.wp_nav.nav_bearing_cd() * 0.01f;
    }
    return 0.0f;
}

float ModeOffboard::crosstrack_error() const
{
    if (_submode == SubMode::GlobalPosition) {
        return g2.wp_nav.crosstrack_error();
    }
    return 0.0f;
}

float ModeOffboard::get_desired_lat_accel() const
{
    if (_submode == SubMode::GlobalPosition) {
        return g2.wp_nav.get_lat_accel();
    }
    return 0.0f;
}

float ModeOffboard::get_distance_to_destination() const
{
    if (_submode == SubMode::GlobalPosition) {
        return _distance_to_destination;
    }
    return 0.0f;
}

bool ModeOffboard::get_desired_location(Location& destination) const
{
    if ((_submode == SubMode::GlobalPosition) && _have_target) {
        destination = _target_location;
        return true;
    }
    return false;
}

bool ModeOffboard::reached_destination() const
{
    switch (_submode) {
    case SubMode::GlobalPosition:
    case SubMode::Path:
    case SubMode::Trajectory:
    case SubMode::Dock:
        return _reached_destination;
    case SubMode::Hold:
    case SubMode::Stop:
    case SubMode::VelocityTurnRate:
        return false;
    }

    return false;
}

bool ModeOffboard::set_desired_speed(float speed)
{
    if (_submode == SubMode::GlobalPosition) {
        return g2.wp_nav.set_speed_max(speed);
    }
    return false;
}

bool ModeOffboard::set_desired_location(const Location &destination, Location next_destination)
{
    if (!enabled()) {
        return false;
    }
    if (!Mode::set_desired_location(destination, next_destination)) {
        return false;
    }

    _target_location = destination;
    _last_command_ms = millis();
    _submode = SubMode::GlobalPosition;
    _state = heartbeat_fresh(_last_command_ms) ? State::Active : State::WaitingForHeartbeat;
    _have_target = true;
    _reached_destination = false;
    _sent_command_stale_warning = false;
    return true;
}

void ModeOffboard::set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed)
{
    _desired_turn_rate_cds = turn_rate_cds;
    _desired_speed = target_speed;
    _last_command_ms = millis();
    _submode = SubMode::VelocityTurnRate;
    _state = heartbeat_fresh(_last_command_ms) ? State::Active : State::WaitingForHeartbeat;
    _have_target = true;
    _sent_command_stale_warning = false;
}

void ModeOffboard::note_heartbeat()
{
    _last_heartbeat_ms = millis();
    _sent_heartbeat_lost_warning = false;
}

void ModeOffboard::note_mavlink_heartbeat(uint8_t sysid, uint8_t compid, uint8_t channel)
{
    if (!_mavlink_owner_set) {
        _mavlink_owner_sysid = sysid;
        _mavlink_owner_compid = compid;
        _mavlink_owner_channel = channel;
        _mavlink_owner_set = true;
    }

    if (!mavlink_source_allowed(sysid, compid, channel)) {
        return;
    }

    note_heartbeat();
}

bool ModeOffboard::mavlink_source_allowed(uint8_t sysid, uint8_t compid, uint8_t channel) const
{
    return _mavlink_owner_set &&
           _mavlink_owner_sysid == sysid &&
           _mavlink_owner_compid == compid &&
           _mavlink_owner_channel == channel;
}

void ModeOffboard::set_heartbeat()
{
    note_heartbeat();
    if (_have_target) {
        _state = State::Active;
    } else {
        _state = State::WaitingForHeartbeat;
    }
}

void ModeOffboard::set_stop()
{
    _submode = SubMode::Stop;
    _desired_speed = 0.0f;
    _desired_turn_rate_cds = 0.0f;
}

void ModeOffboard::set_hold()
{
    _submode = SubMode::Hold;
    _desired_speed = 0.0f;
    _desired_turn_rate_cds = 0.0f;
}

void ModeOffboard::record_command_result(uint32_t cmd_id, bool accepted, uint8_t reject_reason)
{
    _last_cmd_id = cmd_id;
    _last_cmd_accepted = accepted;
    _last_cmd_reject_reason = reject_reason;
    _last_cmd_result_ms = millis();
}

#if AP_EXTERNAL_CONTROL_ENABLED
bool ModeOffboard::get_status(AP_ExternalControl::OffboardStatus &status) const
{
    status.enabled = enabled();
    status.active = rover.control_mode->in_offboard_mode();
    status.state = (uint8_t)_state;
    status.submode = (uint8_t)_submode;
    const uint32_t now_ms = millis();
    status.heartbeat_age_ms = (_last_heartbeat_ms == 0) ? UINT32_MAX : (now_ms - _last_heartbeat_ms);
    status.command_age_ms = (_last_command_ms == 0) ? UINT32_MAX : (now_ms - _last_command_ms);
    status.failsafe_action = (uint8_t)failsafe_action();
    return true;
}

bool ModeOffboard::get_command_result(AP_ExternalControl::OffboardCommandResult &result) const
{
    result.command_id = _last_cmd_id;
    result.accepted = _last_cmd_accepted;
    result.rejection_reason = _last_cmd_reject_reason;
    result.timestamp_ms = _last_cmd_result_ms;
    return true;
}

bool ModeOffboard::get_tracking_status(AP_ExternalControl::OffboardTrackingStatus &tracking) const
{
    tracking.active_submode = (uint8_t)_submode;
    tracking.target_valid = _have_target;

    if ((_submode == SubMode::GlobalPosition || _submode == SubMode::Path || _submode == SubMode::Trajectory) && _have_target) {
        tracking.distance_remaining = _distance_to_destination;
        tracking.position_error = g2.wp_nav.crosstrack_error();
        tracking.heading_error = wrap_180(g2.wp_nav.wp_bearing_cd() * 0.01f - degrees(rover.ahrs.get_yaw()));
    } else if (_submode == SubMode::VelocityTurnRate) {
        float actual_speed = 0.0f;
        g2.attitude_control.get_forward_speed(actual_speed);
        tracking.velocity_error = _desired_speed - actual_speed;
        tracking.distance_remaining = 0.0f;
        tracking.position_error = 0.0f;
        tracking.heading_error = 0.0f;
    } else {
        tracking.distance_remaining = 0.0f;
        tracking.position_error = 0.0f;
        tracking.heading_error = 0.0f;
        tracking.velocity_error = 0.0f;
    }

    return true;
}

bool ModeOffboard::get_localization_status(AP_ExternalControl::OffboardLocalizationStatus &loc_status) const
{
    const uint32_t now_ms = millis();
    const bool data_valid = localization_data_valid();
    const uint32_t age_ms = data_valid ? 0 : ((_last_localization_valid_ms == 0) ? UINT32_MAX : (now_ms - _last_localization_valid_ms));

    loc_status.source_valid = localization_healthy(now_ms);
    loc_status.discontinuity_detected = false;
    loc_status.reset_count = 0;

    if (loc_status.source_valid && age_ms != UINT32_MAX) {
        loc_status.health = 1; // HEALTH_GOOD
        loc_status.age_ms = age_ms;
    } else {
        loc_status.health = 3; // HEALTH_INVALID
        loc_status.age_ms = age_ms;
    }

    return true;
}
#endif

bool ModeOffboard::heartbeat_fresh(uint32_t now_ms) const
{
    return (_last_heartbeat_ms != 0) && ((now_ms - _last_heartbeat_ms) <= heartbeat_timeout_ms());
}

bool ModeOffboard::command_fresh(uint32_t now_ms) const
{
    return (_last_command_ms != 0) && ((now_ms - _last_command_ms) <= command_timeout_ms());
}

void ModeOffboard::update_state(uint32_t now_ms)
{
    if (!heartbeat_fresh(now_ms)) {
        _state = State::HeartbeatLost;
        if (!_sent_heartbeat_lost_warning) {
            _sent_heartbeat_lost_warning = true;
            gcs().send_text(MAV_SEVERITY_WARNING, "Offboard heartbeat lost");
        }
        return;
    }

    _sent_heartbeat_lost_warning = false;

    if (!_have_target) {
        _state = State::WaitingForHeartbeat;
        return;
    }

    if (!command_fresh(now_ms)) {
        _state = State::CommandStale;
        if (!_sent_command_stale_warning) {
            _sent_command_stale_warning = true;
            gcs().send_text(MAV_SEVERITY_WARNING, "Offboard command stale");
        }
        return;
    }

    // check localization health for position-based submodes
    if (localization_data_valid()) {
        _last_localization_valid_ms = now_ms;
        _sent_localization_invalid_warning = false;
    }

    if (submode_needs_localization() && !localization_healthy(now_ms)) {
        _state = State::LocalizationInvalid;
        if (!_sent_localization_invalid_warning) {
            _sent_localization_invalid_warning = true;
            gcs().send_text(MAV_SEVERITY_WARNING, "Offboard localization invalid");
        }
        return;
    }

    _sent_command_stale_warning = false;
    _sent_localization_invalid_warning = false;
    _state = State::Active;
}

void ModeOffboard::run_velocity_turn_rate()
{
    const float steering_out = attitude_control.get_steering_out_rate(radians(_desired_turn_rate_cds / 100.0f),
                                                                      g2.motors.limit.steer_left,
                                                                      g2.motors.limit.steer_right,
                                                                      rover.G_Dt);
    set_steering(steering_out * 4500.0f);
    const float constrained = get_constrained_speed(_desired_speed);
    calc_throttle(calc_speed_nudge(constrained, is_negative(constrained)), true);
}

void ModeOffboard::run_global_position()
{
    if (!g2.wp_nav.reached_destination()) {
        navigate_to_waypoint();
        _reached_destination = false;
    } else {
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
        _reached_destination = true;
        stop_vehicle();
    }
}

bool ModeOffboard::set_path(uint32_t path_id, float speed, const Location *points, uint8_t num_points)
{
    if (!enabled() || num_points == 0 || num_points > OFFBOARD_PATH_MAX) {
        return false;
    }

    for (uint8_t i = 0; i < num_points; i++) {
        _path_points[i] = points[i];
        _path_speeds[i] = speed;
    }
    _path_count = num_points;
    _path_index = 0;
    _path_id = path_id;
    _last_command_ms = millis();
    _submode = SubMode::Path;
    _state = heartbeat_fresh(_last_command_ms) ? State::Active : State::WaitingForHeartbeat;
    _have_target = true;
    _sent_command_stale_warning = false;

    // set speed if provided
    if (!is_zero(speed)) {
        g2.wp_nav.set_speed_max(speed);
    }

    // load first waypoint with look-ahead
    Location next = (_path_count > 1) ? _path_points[1] : Location();
    if (!Mode::set_desired_location(_path_points[0], next)) {
        _path_count = 0;
        _have_target = false;
        return false;
    }
    _target_location = _path_points[0];
    _reached_destination = false;

    return true;
}

bool ModeOffboard::set_trajectory(uint32_t traj_id, float speed, const Location *points, const float *speeds, uint8_t num_points)
{
    if (!enabled() || num_points == 0 || num_points > OFFBOARD_PATH_MAX) {
        return false;
    }

    for (uint8_t i = 0; i < num_points; i++) {
        _path_points[i] = points[i];
        _path_speeds[i] = (speeds != nullptr && !is_zero(speeds[i])) ? speeds[i] : speed;
    }
    _path_count = num_points;
    _path_index = 0;
    _path_id = traj_id;
    _last_command_ms = millis();
    _submode = SubMode::Trajectory;
    _state = heartbeat_fresh(_last_command_ms) ? State::Active : State::WaitingForHeartbeat;
    _have_target = true;
    _sent_command_stale_warning = false;

    if (!is_zero(speed)) {
        g2.wp_nav.set_speed_max(speed);
    }

    Location next = (_path_count > 1) ? _path_points[1] : Location();
    if (!Mode::set_desired_location(_path_points[0], next)) {
        _path_count = 0;
        _have_target = false;
        return false;
    }
    _target_location = _path_points[0];
    _reached_destination = false;

    return true;
}

void ModeOffboard::run_path()
{
    if (_path_index >= _path_count) {
        // path complete
        _reached_destination = true;
        stop_vehicle();
        return;
    }

    if (g2.wp_nav.reached_destination()) {
        // advance to next waypoint
        _path_index++;
        if (_path_index >= _path_count) {
            _reached_destination = true;
            stop_vehicle();
            return;
        }

        // set per-point speed for trajectory mode
        if (_submode == SubMode::Trajectory && !is_zero(_path_speeds[_path_index])) {
            g2.wp_nav.set_speed_max(_path_speeds[_path_index]);
        }

        // load next waypoint with look-ahead
        _target_location = _path_points[_path_index];
        Location next;
        if (_path_index + 1 < _path_count) {
            next = _path_points[_path_index + 1];
        }
        if (!Mode::set_desired_location(_target_location, next)) {
            stop_vehicle();
            return;
        }
        _reached_destination = false;
    }

    navigate_to_waypoint();
}

void ModeOffboard::apply_failsafe()
{
    switch (failsafe_action()) {
    case FailsafeAction::Hold:
        set_hold();
        break;
    case FailsafeAction::Stop:
    default:
        set_stop();
        break;
    }
}

uint32_t ModeOffboard::heartbeat_timeout_ms() const
{
    return MAX((int32_t)g2.offboard_heartbeat_timeout.get(), 100);
}

uint32_t ModeOffboard::command_timeout_ms() const
{
    return MAX((int32_t)g2.offboard_command_timeout.get(), 100);
}

uint32_t ModeOffboard::localization_timeout_ms() const
{
    return MAX((int32_t)g2.offboard_loc_timeout.get(), 100);
}

bool ModeOffboard::localization_data_valid() const
{
    nav_filter_status filter_status;
    rover.ahrs.get_filter_status(filter_status);

    // require horizontal position validity from the EKF
    if (!filter_status.flags.horiz_pos_abs && !filter_status.flags.horiz_pos_rel) {
        return false;
    }

    // check EKF healthy flag
    if (!filter_status.flags.attitude || !filter_status.flags.horiz_vel) {
        return false;
    }

    return true;
}

bool ModeOffboard::localization_healthy(uint32_t now_ms) const
{
    if (localization_data_valid()) {
        return true;
    }

    if (_last_localization_valid_ms == 0) {
        return false;
    }

    return (now_ms - _last_localization_valid_ms) <= localization_timeout_ms();
}

bool ModeOffboard::submode_needs_localization() const
{
    return _submode == SubMode::GlobalPosition ||
           _submode == SubMode::Path ||
           _submode == SubMode::Trajectory;
}

ModeOffboard::FailsafeAction ModeOffboard::failsafe_action() const
{
    const int8_t action = (_state == State::HeartbeatLost) ? g2.offboard_loss_action.get() : g2.offboard_stale_action.get();
    return (action == (int8_t)FailsafeAction::Hold) ? FailsafeAction::Hold : FailsafeAction::Stop;
}

// Phase 5: pause/resume/abort
void ModeOffboard::pause()
{
    if (!_paused) {
        _paused = true;
        _pre_pause_submode = _submode;
        gcs().send_text(MAV_SEVERITY_INFO, "Offboard paused");
    }
}

void ModeOffboard::resume()
{
    if (_paused) {
        _paused = false;
        _submode = _pre_pause_submode;
        _last_command_ms = millis();
        gcs().send_text(MAV_SEVERITY_INFO, "Offboard resumed");
    }
}

void ModeOffboard::abort_task()
{
    _paused = false;
    _have_target = false;
    _path_count = 0;
    _path_index = 0;
    _reached_destination = true;
    set_stop();
    gcs().send_text(MAV_SEVERITY_INFO, "Offboard task aborted");
}

// Phase 6: dynamic safety constraints
bool ModeOffboard::SafetyConstraints::valid() const
{
    return (timestamp_ms != 0) && ((AP_HAL::millis() - timestamp_ms) < 3000);
}

void ModeOffboard::set_safety_constraints(float max_speed, float min_distance)
{
    _safety_constraints.max_speed = max_speed;
    _safety_constraints.min_distance = min_distance;
    _safety_constraints.timestamp_ms = millis();
}

float ModeOffboard::get_constrained_speed(float desired_speed) const
{
    if (!_safety_constraints.valid()) {
        return desired_speed;
    }
    if (!is_zero(_safety_constraints.max_speed)) {
        desired_speed = constrain_float(desired_speed,
                                        -_safety_constraints.max_speed,
                                        _safety_constraints.max_speed);
    }
    return desired_speed;
}

float ModeOffboard::get_safety_speed_limit() const
{
    if (_safety_constraints.valid() && !is_zero(_safety_constraints.max_speed)) {
        return _safety_constraints.max_speed;
    }
    return 0.0f;
}

// Phase 7: docking submode
bool ModeOffboard::start_dock()
{
#if MODE_DOCK_ENABLED == ENABLED
    if (!rover.precland.enabled() || !rover.precland.target_acquired()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Offboard dock: target not acquired");
        return false;
    }
    _submode = SubMode::Dock;
    _dock_complete = false;
    _last_command_ms = millis();
    _have_target = true;
    _sent_command_stale_warning = false;
    gcs().send_text(MAV_SEVERITY_INFO, "Offboard dock started");
    return true;
#else
    gcs().send_text(MAV_SEVERITY_WARNING, "Dock mode not enabled");
    return false;
#endif
}

void ModeOffboard::run_dock()
{
#if MODE_DOCK_ENABLED == ENABLED
    if (_dock_complete) {
        stop_vehicle();
        _reached_destination = true;
        return;
    }

    // use existing dock mode's precision landing target tracking
    Vector2f dock_rel_cm;
    if (!rover.precland.get_target_position_cm(dock_rel_cm)) {
        // lost the dock target, hold position
        stop_vehicle();
        return;
    }
    _distance_to_destination = dock_rel_cm.length() * 0.01f;

    // check if docking is complete
    if (_distance_to_destination <= 0.3f) {
        _dock_complete = true;
        _reached_destination = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Offboard dock complete");
        stop_vehicle();
        return;
    }

    // use position controller to drive toward dock target
    const Vector2p target_pos { dock_rel_cm.topostype() * 0.01 };
    g2.pos_control.input_pos_target(target_pos, rover.G_Dt);
    g2.pos_control.update(rover.G_Dt);

    float desired_speed = g2.pos_control.get_desired_speed();
    const float desired_turn_rate = g2.pos_control.get_desired_turn_rate_rads();

    // apply safety speed constraint
    desired_speed = get_constrained_speed(desired_speed);

    calc_steering_from_turn_rate(desired_turn_rate);
    calc_throttle(desired_speed, true);
#else
    stop_vehicle();
#endif
}
