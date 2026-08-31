# One completed remote model update.  This is an event stream (not status).
std_msgs/Header header
builtin_interfaces/Time image_stamp
builtin_interfaces/Time request_stamp
string episode_id
uint32 model_step
string backend
string output_type                 # actions | trajectory | stop
uint32 output_size
bool done
geometry_msgs/Pose2D request_pose
geometry_msgs/Pose2D response_pose
float32 client_latency_ms
float32 server_total_ms
float32 preprocessing_ms
float32 system1_ms
float32 system2_ms
string[] actions
TrajectoryPoint[] trajectory
