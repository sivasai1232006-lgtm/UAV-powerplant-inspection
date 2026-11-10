uint8 IDLE=0
uint8 ASSIGNED=1
uint8 EXECUTING=2
uint8 COMPLETED=3
uint8 RETURNING=4
uint8 FAILED=5
uint8 LANDING=6
uint8 LANDED=7
uint8 RECOVERING=8
uint8 INSPECTING=9
uint8 QUEUED=10

string uav_id
bool connected
bool operational
bool landed
uint8 mission_status
string assigned_task_id
