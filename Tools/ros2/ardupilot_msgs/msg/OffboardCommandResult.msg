uint32 command_id
bool accepted
uint8 rejection_reason
uint32 timestamp_ms

uint8 REJECT_NONE = 0
uint8 REJECT_NOT_IN_OFFBOARD = 1
uint8 REJECT_HEARTBEAT_STALE = 2
uint8 REJECT_NOT_ARMED = 3
uint8 REJECT_DISABLED = 4
uint8 REJECT_INVALID_COMMAND = 5
