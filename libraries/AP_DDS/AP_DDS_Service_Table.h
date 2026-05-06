#include "uxr/client/client.h"
#include <AP_DDS/AP_DDS_config.h>

enum class ServiceIndex: uint8_t {
#if AP_DDS_ARM_SERVER_ENABLED
    ARMING_MOTORS,
#endif // #if AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    MODE_SWITCH,
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    PREARM_CHECK,
#endif // AP_DDS_ARM_CHECK_SERVER_ENABLED
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    TAKEOFF,
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
    SET_PARAMETERS,
    GET_PARAMETERS,
#endif // AP_DDS_PARAMETER_SERVER_ENABLED
#if AP_DDS_LAND_SERVER_ENABLED
    LAND,
#endif // AP_DDS_LAND_SERVER_ENABLED
#if AP_DDS_RTL_SERVER_ENABLED
    RTL,
#endif // AP_DDS_RTL_SERVER_ENABLED
#if AP_DDS_SET_HOME_SERVER_ENABLED
    SET_HOME,
#endif // AP_DDS_SET_HOME_SERVER_ENABLED
#if AP_DDS_SERVO_CONTROL_SERVER_ENABLED
    SERVO_CONTROL,
#endif // AP_DDS_SERVO_CONTROL_SERVER_ENABLED
#if AP_DDS_FENCE_CONTROL_SERVER_ENABLED
    FENCE_CONTROL,
#endif // AP_DDS_FENCE_CONTROL_SERVER_ENABLED
};

static inline constexpr uint8_t to_underlying(const ServiceIndex index)
{
    static_assert(sizeof(index) == sizeof(uint8_t));
    return static_cast<uint8_t>(index);
}

constexpr struct AP_DDS_Client::Service_table AP_DDS_Client::services[] = {
#if AP_DDS_ARM_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .rep_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/arm_motorsService",
        .request_type = "ardupilot_msgs::srv::dds_::ArmMotors_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ArmMotors_Response_",
        .request_topic_name = "rq/ap/arm_motorsRequest",
        .reply_topic_name = "rr/ap/arm_motorsReply",
        .qos = {
            .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
#endif // AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .rep_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/mode_switchService",
        .request_type = "ardupilot_msgs::srv::dds_::ModeSwitch_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ModeSwitch_Response_",
        .request_topic_name = "rq/ap/mode_switchRequest",
        .reply_topic_name = "rr/ap/mode_switchReply",
    },
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::PREARM_CHECK),
        .rep_id = to_underlying(ServiceIndex::PREARM_CHECK),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/prearm_checkService",
        .request_type = "std_srvs::srv::dds_::Trigger_Request_",
        .reply_type = "std_srvs::srv::dds_::Trigger_Response_",
        .request_topic_name = "rq/ap/prearm_checkRequest",
        .reply_topic_name = "rr/ap/prearm_checkReply",
    },
#endif // AP_DDS_ARM_CHECK_SERVER_ENABLED    
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::TAKEOFF),
        .rep_id = to_underlying(ServiceIndex::TAKEOFF),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/experimental/takeoffService",
        .request_type = "ardupilot_msgs::srv::dds_::Takeoff_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::Takeoff_Response_",
        .request_topic_name = "rq/ap/experimental/takeoffRequest",
        .reply_topic_name = "rr/ap/experimental/takeoffReply",
    },
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::SET_PARAMETERS),
        .rep_id = to_underlying(ServiceIndex::SET_PARAMETERS),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/set_parametersService",
        .request_type = "rcl_interfaces::srv::dds_::SetParameters_Request_",
        .reply_type = "rcl_interfaces::srv::dds_::SetParameters_Response_",
        .request_topic_name = "rq/ap/set_parametersRequest",
        .reply_topic_name = "rr/ap/set_parametersReply",
    },
    {
        .req_id = to_underlying(ServiceIndex::GET_PARAMETERS),
        .rep_id = to_underlying(ServiceIndex::GET_PARAMETERS),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/get_parameterService",
        .request_type = "rcl_interfaces::srv::dds_::GetParameters_Request_",
        .reply_type = "rcl_interfaces::srv::dds_::GetParameters_Response_",
        .request_topic_name = "rq/ap/get_parametersRequest",
        .reply_topic_name = "rr/ap/get_parametersReply",
    },
#endif // AP_DDS_PARAMETER_SERVER_ENABLED
#if AP_DDS_LAND_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::LAND),
        .rep_id = to_underlying(ServiceIndex::LAND),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/landService",
        .request_type = "ardupilot_msgs::srv::dds_::Land_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::Land_Response_",
        .request_topic_name = "rq/ap/landRequest",
        .reply_topic_name = "rr/ap/landReply",
    },
#endif // AP_DDS_LAND_SERVER_ENABLED
#if AP_DDS_RTL_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::RTL),
        .rep_id = to_underlying(ServiceIndex::RTL),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/rtlService",
        .request_type = "ardupilot_msgs::srv::dds_::ReturnToLaunch_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ReturnToLaunch_Response_",
        .request_topic_name = "rq/ap/rtlRequest",
        .reply_topic_name = "rr/ap/rtlReply",
    },
#endif // AP_DDS_RTL_SERVER_ENABLED
#if AP_DDS_SET_HOME_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::SET_HOME),
        .rep_id = to_underlying(ServiceIndex::SET_HOME),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/set_homeService",
        .request_type = "ardupilot_msgs::srv::dds_::SetHome_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::SetHome_Response_",
        .request_topic_name = "rq/ap/set_homeRequest",
        .reply_topic_name = "rr/ap/set_homeReply",
    },
#endif // AP_DDS_SET_HOME_SERVER_ENABLED
#if AP_DDS_SERVO_CONTROL_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::SERVO_CONTROL),
        .rep_id = to_underlying(ServiceIndex::SERVO_CONTROL),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/servo_controlService",
        .request_type = "ardupilot_msgs::srv::dds_::ServoControl_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ServoControl_Response_",
        .request_topic_name = "rq/ap/servo_controlRequest",
        .reply_topic_name = "rr/ap/servo_controlReply",
    },
#endif // AP_DDS_SERVO_CONTROL_SERVER_ENABLED
#if AP_DDS_FENCE_CONTROL_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::FENCE_CONTROL),
        .rep_id = to_underlying(ServiceIndex::FENCE_CONTROL),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/fence_controlService",
        .request_type = "ardupilot_msgs::srv::dds_::FenceControl_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::FenceControl_Response_",
        .request_topic_name = "rq/ap/fence_controlRequest",
        .reply_topic_name = "rr/ap/fence_controlReply",
    },
#endif // AP_DDS_FENCE_CONTROL_SERVER_ENABLED
};
