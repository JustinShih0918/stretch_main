"""Drive the robot back to spawn (world 0,0, yaw 0) with Nav2 and verify it got there."""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = rclpy.create_node("demo_reset")
buf = Buffer(); TransformListener(buf, n)
ac = ActionClient(n, NavigateToPose, "navigate_to_pose")

def pose():
    for _ in range(50):
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            t = buf.lookup_transform("world", "base_link", Time()).transform
            q = t.rotation
            return t.translation.x, t.translation.y, math.degrees(2 * math.atan2(q.z, q.w))
        except Exception:
            pass
    return None

if not ac.wait_for_server(timeout_sec=30):
    print("[reset] no navigate_to_pose server"); sys.exit(1)
for attempt in range(3):
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "world"
    g.pose.pose.orientation.w = 1.0
    fut = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print(f"[reset] goal rejected (attempt {attempt})"); time.sleep(2); continue
    res = gh.get_result_async(); t0 = time.time()
    while not res.done() and time.time() - t0 < 240:
        rclpy.spin_once(n, timeout_sec=0.2)
    p = pose()
    print(f"[reset] attempt {attempt}: status {res.result().status if res.done() else 'timeout'}, pose {p}")
    if p and math.hypot(p[0], p[1]) < 0.3 and abs(p[2]) < 20:
        print("[reset] OK at spawn"); sys.exit(0)
print("[reset] FAILED to reach spawn"); sys.exit(2)
