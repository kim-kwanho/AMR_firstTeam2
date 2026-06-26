#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from enum import Enum
import random
import time

def is_safe_cell(grid, col, row, radius_cells=10):
    width = grid.info.width
    height = grid.info.height
    for r in range(max(0, row - radius_cells), min(height, row + radius_cells + 1)):
        for c in range(max(0, col - radius_cells), min(width, col + radius_cells + 1)):
            idx = r * width + c
            # If any cell in the neighborhood is not free (0), it is not safe
            if grid.data[idx] != 0:
                return False
    return True

class NodeStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BTNode:
    def __init__(self, name="Node"):
        self.name = name
    def tick(self) -> NodeStatus:
        raise NotImplementedError
    def reset(self):
        pass

class Sequence(BTNode):
    def __init__(self, children, name="Sequence"):
        super().__init__(name)
        self.children = children
    def tick(self) -> NodeStatus:
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.SUCCESS:
                return status
        return NodeStatus.SUCCESS
    def reset(self):
        for child in self.children:
            child.reset()

class Selector(BTNode):
    def __init__(self, children, name="Selector"):
        super().__init__(name)
        self.children = children
    def tick(self) -> NodeStatus:
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.FAILURE:
                return status
        return NodeStatus.FAILURE
    def reset(self):
        for child in self.children:
            child.reset()

# Blackboard for sharing variables among nodes
class Blackboard:
    def __init__(self):
        self.user_goal = None
        self.random_goal = None
        self.costmap = None
        self.nav_client = None
        self.node = None
        self.active_goal_handle = None
        self.llm_active = False
        self.wander_enabled = False  # wandering is off until enabled via /wander_enabled

blackboard = Blackboard()

# --- Condition Nodes ---

class IsUserGoalActive(BTNode):
    def __init__(self):
        super().__init__("IsUserGoalActive")
    def tick(self) -> NodeStatus:
        if blackboard.user_goal is not None:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

class IsWanderEnabled(BTNode):
    def __init__(self):
        super().__init__("IsWanderEnabled")
    def tick(self) -> NodeStatus:
        if blackboard.wander_enabled:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

# --- Action Nodes ---

class GenerateRandomGoal(BTNode):
    def __init__(self):
        super().__init__("GenerateRandomGoal")

    def tick(self) -> NodeStatus:
        if blackboard.random_goal is not None:
            return NodeStatus.SUCCESS

        if blackboard.costmap is None:
            blackboard.node.get_logger().warn(
                "BT: Costmap/Map not received yet. Cannot generate random goal.", 
                throttle_duration_sec=2.0
            )
            return NodeStatus.FAILURE

        grid = blackboard.costmap
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        # Try to find a safe free cell by random sampling (much faster than scanning the whole map)
        chosen_idx = None
        for _ in range(2000):
            idx = random.randint(0, len(grid.data) - 1)
            # 0 means completely free space in the occupancy grid
            if grid.data[idx] == 0:
                col = idx % width
                row = idx // width
                # 10 cells * 0.05m = 0.50m safety margin around the goal
                if is_safe_cell(grid, col, row, radius_cells=10):
                    chosen_idx = idx
                    break

        if chosen_idx is None:
            # Fallback to any free cell if we couldn't find a safe one
            free_indices = [i for i, val in enumerate(grid.data) if val == 0]
            if not free_indices:
                blackboard.node.get_logger().error("BT: No free space found in the costmap!")
                return NodeStatus.FAILURE
            chosen_idx = random.choice(free_indices)
            blackboard.node.get_logger().warn("BT: Could not find a safe cell with 0.5m margin, falling back to any free cell.")

        col = chosen_idx % width
        row = chosen_idx // width

        # Convert to world coordinates
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution

        # Create PoseStamped goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = grid.header.frame_id
        goal_pose.header.stamp = blackboard.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0

        blackboard.random_goal = goal_pose
        blackboard.node.get_logger().info(
            f"BT: Generated random goal at ({x:.2f}, {y:.2f}) using costmap with safety margin."
        )
        return NodeStatus.SUCCESS

class ExecuteGoal(BTNode):
    def __init__(self, goal_type='user'):
        super().__init__(f"ExecuteGoal_{goal_type}")
        self.goal_type = goal_type
        self.goal_handle_future = None
        self.result_future = None
        self.my_goal_handle = None
        self.navigation_started = False
        self.navigation_failed = False
        self.navigation_succeeded = False

    def tick(self) -> NodeStatus:
        goal_pose = blackboard.user_goal if self.goal_type == 'user' else blackboard.random_goal
        
        if goal_pose is None:
            return NodeStatus.FAILURE

        if not self.navigation_started:
            blackboard.node.get_logger().info(f"BT: Starting Nav2 for {self.goal_type.upper()} Goal...")
            if not blackboard.nav_client.wait_for_server(timeout_sec=1.0):
                blackboard.node.get_logger().error("BT: Nav2 Action server not available!")
                return NodeStatus.FAILURE
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            
            self.goal_handle_future = blackboard.nav_client.send_goal_async(goal_msg)
            self.goal_handle_future.add_done_callback(self.goal_response_callback)
            self.navigation_started = True
            return NodeStatus.RUNNING

        if self.navigation_succeeded:
            self.reset()
            self.clear_goal()
            return NodeStatus.SUCCESS

        if self.navigation_failed:
            self.reset()
            self.clear_goal()
            return NodeStatus.FAILURE

        return NodeStatus.RUNNING

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            blackboard.node.get_logger().warn(f"BT: {self.goal_type.upper()} Goal rejected by Nav2.")
            self.navigation_failed = True
            return
        
        self.my_goal_handle = goal_handle
        blackboard.node.get_logger().info(f"BT: {self.goal_type.upper()} Goal accepted by Nav2.")
        blackboard.active_goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        action_result = future.result()
        status = action_result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            blackboard.node.get_logger().info(f"BT: {self.goal_type.upper()} Goal reached successfully!")
            self.navigation_succeeded = True
        elif status == GoalStatus.STATUS_CANCELED:
            blackboard.node.get_logger().warn(f"BT: {self.goal_type.upper()} Goal was cancelled.")
            self.navigation_failed = True
        else:
            blackboard.node.get_logger().error(f"BT: {self.goal_type.upper()} Goal failed with status code: {status}")
            self.navigation_failed = True
            
        if blackboard.active_goal_handle == self.my_goal_handle:
            blackboard.active_goal_handle = None

    def clear_goal(self):
        if self.goal_type == 'user':
            blackboard.user_goal = None
        else:
            blackboard.random_goal = None

    def reset(self):
        self.goal_handle_future = None
        self.result_future = None
        self.my_goal_handle = None
        self.navigation_started = False
        self.navigation_failed = False
        self.navigation_succeeded = False

class WaitAction(BTNode):
    def __init__(self, duration):
        super().__init__(f"Wait_{duration}s")
        self.duration = duration
        self.start_time = None

    def tick(self) -> NodeStatus:
        if self.start_time is None:
            self.start_time = time.time()
            blackboard.node.get_logger().info(f"BT: Waiting for {self.duration} seconds...")
            return NodeStatus.RUNNING

        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.start_time = None
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def reset(self):
        self.start_time = None

# --- ROS 2 Node Wrapper ---

class WanderNavBTNode(Node):
    def __init__(self):
        super().__init__('wander_nav_bt_node')
        
        blackboard.node = self
        blackboard.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions with Transient Local durability (required for static maps / costmaps)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.costmap_callback,
            map_qos
        )
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            goal_qos
        )
        
        # Subscribe to LLM active status
        self.llm_active_sub = self.create_subscription(
            Bool,
            '/llm_active',
            self.llm_active_callback,
            10
        )

        # Subscribe to wander on/off switch (latched so late publishers/subscribers stay in sync)
        wander_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.wander_enabled_sub = self.create_subscription(
            Bool,
            '/wander_enabled',
            self.wander_enabled_callback,
            wander_qos
        )
        
        # Build Behavior Tree
        self.execute_user_node = ExecuteGoal(goal_type='user')
        self.wait_user_node = WaitAction(5.0)
        
        self.generate_random_node = GenerateRandomGoal()
        self.execute_random_node = ExecuteGoal(goal_type='random')
        self.wait_random_node = WaitAction(5.0)

        user_branch = Sequence([
            IsUserGoalActive(),
            self.execute_user_node,
            self.wait_user_node
        ], name="UserGoalBranch")

        wander_branch = Sequence([
            IsWanderEnabled(),
            self.generate_random_node,
            self.execute_random_node,
            self.wait_random_node
        ], name="WanderBranch")

        self.bt_root = Selector([user_branch, wander_branch], name="RootSelector")

        # 10Hz Timer to tick the behavior tree
        self.timer = self.create_timer(0.1, self.tick_bt)
        self.get_logger().info("Wander & Navigation Behavior Tree Node initialized.")

    def costmap_callback(self, msg: OccupancyGrid):
        blackboard.costmap = msg

    def goal_callback(self, msg: PoseStamped):
        # Cancel active random navigation if it is running
        if blackboard.random_goal is not None and blackboard.active_goal_handle is not None:
            self.get_logger().info("BT: Cancelling active random navigation to prioritize user goal...")
            try:
                blackboard.active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"BT: Failed to cancel random goal: {e}")
            
        blackboard.random_goal = None
        blackboard.active_goal_handle = None
        
        # Reset behavior tree execution nodes to clear state
        self.execute_random_node.reset()
        self.wait_random_node.reset()
        self.execute_user_node.reset()
        self.wait_user_node.reset()
            
        blackboard.user_goal = msg
        self.get_logger().info(
            f"BT: Received user goal pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    def wander_enabled_callback(self, msg: Bool):
        if blackboard.wander_enabled == msg.data:
            return
        blackboard.wander_enabled = msg.data
        if msg.data:
            self.get_logger().info("BT: Wandering ENABLED via /wander_enabled.")
            return

        self.get_logger().info("BT: Wandering DISABLED via /wander_enabled.")
        # Cancel an in-flight random goal immediately, mirroring goal_callback
        if blackboard.random_goal is not None and blackboard.active_goal_handle is not None:
            try:
                blackboard.active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"BT: Failed to cancel random goal: {e}")
        blackboard.random_goal = None
        blackboard.active_goal_handle = None
        self.execute_random_node.reset()
        self.wait_random_node.reset()

    def llm_active_callback(self, msg: Bool):
        blackboard.llm_active = msg.data
        if msg.data:
            self.get_logger().info("BT: LLM Agent is active. Pausing random wandering.")
        else:
            self.get_logger().info("BT: LLM Agent is inactive. Resuming random wandering.")

    def tick_bt(self):
        # Pause the BT tick if LLM navigation is running
        if blackboard.llm_active:
            self.get_logger().info("BT: LLM navigation in progress... Ticking paused.", throttle_duration_sec=5.0)
            return
        # Tick the root node
        self.bt_root.tick()

def main(args=None):
    rclpy.init(args=args)
    node = WanderNavBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down BT node.")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
