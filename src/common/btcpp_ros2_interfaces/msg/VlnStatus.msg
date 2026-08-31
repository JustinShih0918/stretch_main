# Live status of the VLN agent (vln_policy/vln_agent_node), published on
# /vln/status on every state transition plus a low-rate heartbeat.

# state values
string STATE_IDLE = IDLE            # waiting for an instruction
string STATE_RESETTING = RESETTING  # backend reset in flight (new episode)
string STATE_THINKING = THINKING    # inference request in flight
string STATE_EXECUTING = EXECUTING  # executor is running the action batch
string STATE_DONE = DONE            # episode finished (model STOP / max_steps)
string STATE_ERROR = ERROR          # backend/executor failure; new instruction recovers

std_msgs/Header header
string state                  # one of the STATE_* values above
string instruction            # active natural-language instruction
string current_action         # action being executed (FORWARD / TURN_LEFT / ...)
string[] pending_actions      # remainder of the current action batch
string backend                # streamvln | dualvln | dummy | navila
string execution_mode         # trajectory | cmd_vel | nav2
uint32 step_count             # discrete actions executed this episode
string detail                 # last error, server latency, etc.

# Episode-correlated fields added in protocol v2.  The fields above remain
# unchanged so existing status consumers continue to compile and run.
string episode_id             # opaque ID, new for every instruction/reset
uint32 model_step_count       # completed model /step calls this episode
bool inference_active         # an HTTP /reset or /step request is in flight
bool motion_active            # an executor currently owns a motion command
string terminal_reason        # model_stop | timeout | backend_error | ...
uint64 controller_tick_count  # trajectory-control ticks (summary/rate input)
