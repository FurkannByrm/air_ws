float64 current_position
float64 current_velocity

float64 target_position
float64 target_velocity

### Driver State Constants:

uint8 NOT_READY_TO_SWITCH_ON = 1
uint8 SWITCH_ON_DISABLED = 2
uint8 SWITCHED_ON = 4
uint8 OPERATION_ENABLED = 5
uint8 FAULT_REACTION_ACTIVE = 6
uint8 FAULT = 7
uint8 QUICK_STOP_ACTIVE = 3


int8 status