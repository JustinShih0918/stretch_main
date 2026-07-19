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
string backend                # active backend name (streamvln / dummy / navila)
string execution_mode         # cmd_vel | nav2
uint32 step_count             # discrete actions executed this episode
string detail                 # last error, server latency, etc.
