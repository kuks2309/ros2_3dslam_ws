uint8   STATE_IDLE              = 0
uint8   STATE_ALIGNING          = 1
uint8   STATE_STARTING_SLAM     = 2
uint8   STATE_VERIFYING_MAP     = 3
uint8   STATE_MAPPING_MANUAL    = 4
uint8   STATE_MAPPING_AUTO      = 5
uint8   STATE_EXPLORING_UNKNOWN = 6
uint8   STATE_LOOP_CLOSING      = 7
uint8   STATE_COMPLETED         = 8
uint8   STATE_ERROR             = 9
uint8   STATE_PAUSED            = 10
uint8   state
uint8   previous_state
float32 coverage_percent
float64 current_heading_error_deg
string  log_message
