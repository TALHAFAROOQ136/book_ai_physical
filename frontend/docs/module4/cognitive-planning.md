---
sidebar_position: 3
---

# Cognitive Planning with LLMs

**Translating "Clean the Room" into a Sequence of Robot Actions**

---

## Overview

When a human says "clean the room," they are issuing a high-level instruction that implies dozens of lower-level tasks: identify what is out of place, plan a sequence of pick-and-place operations, navigate between objects, and verify the room is clean when done. Traditional robot programming requires a developer to hardcode every step. **Cognitive planning** uses a Large Language Model as a task planner that bridges natural language instructions and physical robot actions. In this chapter, you will build an LLM-powered task planner, define robot action primitives, implement grounding and safety validation, and handle replanning when actions fail.

## The Task Planning Problem

### Why Hardcoded Plans Fail

Consider the instruction "bring me a drink from the kitchen":

```
HARDCODED APPROACH:
1. go_to(kitchen)               ← What if the kitchen door is closed?
2. find(drink)                  ← What if there are 5 different drinks?
3. grasp(drink)                 ← What if the drink is behind something?
4. go_to(user_position)         ← What if the user moved?
5. handover(drink)              ← What if the user isn't ready?

Each step assumes the previous one succeeded perfectly.
No handling for ambiguity, obstacles, or changed conditions.
```

### Why LLMs Succeed

An LLM can reason about the same task with common-sense understanding:

```
LLM APPROACH:
"Bring me a drink from the kitchen"

LLM reasoning:
- I need to navigate to the kitchen
- I should look for available drinks
- If multiple options exist, I should ask the user's preference
- I need to grasp the chosen drink carefully
- I should verify I'm holding it before moving
- Navigate back to the user
- If the user moved, I should locate them first
- Hand over the drink when they're ready

Generated plan:
1. navigate_to("kitchen")
2. scan_area()
3. IF multiple_drinks_found: say("I see several drinks. Which would you prefer?")
4. detect_object(chosen_drink)
5. navigate_to(drink_position)
6. grasp(chosen_drink)
7. verify_grasp()
8. navigate_to("user")
9. handover(chosen_drink)
10. say("Here you go!")
```

The LLM handles ambiguity, verification steps, and fallback logic that would require hundreds of lines of behavior tree code.

## Defining Action Primitives

The LLM does not generate raw motor commands. It composes **action primitives** — well-tested, self-contained robot capabilities.

### The Action Registry

```python
# humanoid_planning/action_registry.py
"""Registry of robot action primitives available to the LLM planner."""
import json
from dataclasses import dataclass, field


@dataclass
class ActionPrimitive:
    name: str
    description: str
    parameters: dict
    ros2_action: str       # ROS 2 action/service to call
    preconditions: list    # What must be true before this action
    effects: list          # What becomes true after this action
    timeout: float = 30.0  # Maximum execution time in seconds


# Define all available primitives
ACTION_REGISTRY = {
    "navigate_to": ActionPrimitive(
        name="navigate_to",
        description="Move the robot to a named location or coordinates",
        parameters={
            "location": "string — named location (e.g., 'kitchen') or coordinates 'x,y'",
        },
        ros2_action="/navigate_to_pose",
        preconditions=["robot_is_standing", "path_exists_to_location"],
        effects=["robot_at_location"],
        timeout=120.0,
    ),
    "detect_object": ActionPrimitive(
        name="detect_object",
        description="Look for a specific object in the robot's current field of view",
        parameters={
            "object_name": "string — object class (e.g., 'red cup', 'book')",
        },
        ros2_action="/detect_object",
        preconditions=[],
        effects=["object_detected", "object_position_known"],
        timeout=10.0,
    ),
    "scan_area": ActionPrimitive(
        name="scan_area",
        description="Rotate the head to scan the surrounding area for objects",
        parameters={},
        ros2_action="/scan_area",
        preconditions=["robot_is_standing"],
        effects=["area_scanned", "visible_objects_updated"],
        timeout=15.0,
    ),
    "grasp": ActionPrimitive(
        name="grasp",
        description="Pick up an object that has been detected and localized",
        parameters={
            "object_name": "string — the object to grasp",
        },
        ros2_action="/grasp_object",
        preconditions=["object_detected", "object_position_known", "hand_is_empty"],
        effects=["holding_object", "hand_is_not_empty"],
        timeout=30.0,
    ),
    "place": ActionPrimitive(
        name="place",
        description="Place a held object at a specified location",
        parameters={
            "object_name": "string — the object to place",
            "location": "string — where to place it",
        },
        ros2_action="/place_object",
        preconditions=["holding_object"],
        effects=["hand_is_empty", "object_at_location"],
        timeout=30.0,
    ),
    "handover": ActionPrimitive(
        name="handover",
        description="Hand an object to a detected human using compliant control",
        parameters={
            "object_name": "string — the object to hand over",
        },
        ros2_action="/handover_object",
        preconditions=["holding_object", "human_detected_nearby"],
        effects=["hand_is_empty", "object_given_to_human"],
        timeout=20.0,
    ),
    "say": ActionPrimitive(
        name="say",
        description="Speak a message to the user via text-to-speech",
        parameters={
            "text": "string — the message to speak",
        },
        ros2_action="/humanoid/voice/say",
        preconditions=[],
        effects=[],
        timeout=10.0,
    ),
    "wait": ActionPrimitive(
        name="wait",
        description="Wait for a specified duration",
        parameters={
            "seconds": "float — how long to wait",
        },
        ros2_action="/wait",
        preconditions=[],
        effects=[],
        timeout=60.0,
    ),
    "look_at": ActionPrimitive(
        name="look_at",
        description="Point the head camera at a target location or object",
        parameters={
            "target": "string — object name or 'x,y,z' coordinates",
        },
        ros2_action="/look_at",
        preconditions=[],
        effects=["camera_pointing_at_target"],
        timeout=5.0,
    ),
    "verify_grasp": ActionPrimitive(
        name="verify_grasp",
        description="Check force-torque sensor to confirm object is securely held",
        parameters={},
        ros2_action="/verify_grasp",
        preconditions=[],
        effects=["grasp_verified"],
        timeout=5.0,
    ),
}


def get_primitives_description() -> str:
    """Generate a text description of all primitives for the LLM prompt."""
    lines = ["Available robot actions:\n"]
    for name, action in ACTION_REGISTRY.items():
        params = ", ".join(f"{k}: {v}" for k, v in action.parameters.items())
        lines.append(f"- {name}({params})")
        lines.append(f"  Description: {action.description}")
        lines.append(f"  Preconditions: {action.preconditions}")
        lines.append(f"  Effects: {action.effects}")
        lines.append("")
    return "\n".join(lines)
```

## The LLM Task Planner

### Core Planner Node

```python
# humanoid_planning/task_planner.py
"""LLM-powered task planner for humanoid robot actions."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

from openai import OpenAI
from humanoid_planning.action_registry import (
    ACTION_REGISTRY,
    get_primitives_description,
)


class TaskPlanner(Node):
    """Uses an LLM to decompose natural language instructions into action plans."""

    def __init__(self):
        super().__init__('task_planner')

        self.declare_parameter('model', 'gpt-4o')
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('max_plan_steps', 15)

        self.model_name = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value
        self.max_steps = self.get_parameter('max_plan_steps').value

        api_key = os.environ.get('OPENAI_API_KEY', '')
        self.client = OpenAI(api_key=api_key) if api_key else None

        # Subscribe to parsed intents
        self.create_subscription(
            String, '/humanoid/voice/parsed_intent', self.on_intent, 10
        )

        # Subscribe to scene descriptions from perception
        self.create_subscription(
            String, '/humanoid/scene_description', self.on_scene, 10
        )

        # Publish action plans
        self.plan_pub = self.create_publisher(String, '/humanoid/action_plan', 10)

        self.current_scene = "No scene data available yet."

        # Build system prompt
        self.system_prompt = self.build_system_prompt()

        if self.client:
            self.get_logger().info(f'Task planner ready (model: {self.model_name})')
        else:
            self.get_logger().warn('No API key — task planner disabled')

    def build_system_prompt(self) -> str:
        primitives = get_primitives_description()

        return f"""You are a task planner for a humanoid robot. Given a natural language
instruction and a scene description, generate a plan as a JSON array of action steps.

{primitives}

Known locations: kitchen (5,2), living room (0,0), bedroom (-3,4),
front door (8,0), table (2,1), desk (1,-2), charging station (-1,-1).

Rules:
1. Return ONLY a JSON object with "plan" (array of steps) and "reasoning" (brief explanation).
2. Each step must use ONLY actions from the available list above.
3. Each step format: {{"action": "action_name", "params": {{"param": "value"}}, "description": "why"}}
4. Maximum {self.max_steps} steps per plan.
5. Add verification steps (verify_grasp after grasp, scan_area before navigate in unknown areas).
6. Add say() steps for user communication at key moments.
7. If the instruction is ambiguous, add a say() step to ask for clarification BEFORE acting.
8. Never plan actions that could harm humans — always include safety considerations.
9. If a step might fail, note what the fallback should be in the description.
10. Consider the current scene: what objects are visible, where is the robot, what is the state."""

    def on_scene(self, msg: String):
        self.current_scene = msg.data

    def on_intent(self, msg: String):
        intent = json.loads(msg.data)

        # Simple intents can be executed directly without LLM
        if intent.get('type') in ('stop', 'status'):
            self.handle_simple_intent(intent)
            return

        # Complex intents need LLM planning
        self.plan_task(intent)

    def handle_simple_intent(self, intent: dict):
        if intent['type'] == 'stop':
            plan = {
                'plan': [{'action': 'say', 'params': {'text': 'Stopping.'}, 'description': 'Confirm stop'}],
                'reasoning': 'Emergency stop requested',
            }
        elif intent['type'] == 'status':
            plan = {
                'plan': [
                    {'action': 'scan_area', 'params': {}, 'description': 'Look around'},
                    {'action': 'say', 'params': {'text': 'Scanning my surroundings.'}, 'description': 'Report'},
                ],
                'reasoning': 'Status report requested',
            }
        else:
            return

        result = String()
        result.data = json.dumps(plan)
        self.plan_pub.publish(result)

    def plan_task(self, intent: dict):
        if not self.client:
            self.get_logger().error('LLM client not available')
            return

        raw_text = intent.get('raw_text', json.dumps(intent))

        user_message = f"""Instruction: "{raw_text}"

Current scene: {self.current_scene}

Robot state: standing, hands empty, at current position.

Generate an action plan."""

        self.get_logger().info(f'Planning task: "{raw_text}"')

        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_message},
                ],
                response_format={"type": "json_object"},
                temperature=self.temperature,
                max_tokens=1500,
            )

            plan = json.loads(response.choices[0].message.content)

            # Validate the plan
            validated = self.validate_plan(plan)

            result = String()
            result.data = json.dumps(validated)
            self.plan_pub.publish(result)

            self.get_logger().info(
                f'Plan generated: {len(validated["plan"])} steps\n'
                f'Reasoning: {validated.get("reasoning", "N/A")}'
            )
            for i, step in enumerate(validated['plan']):
                self.get_logger().info(
                    f'  Step {i+1}: {step["action"]}({step.get("params", {})}) '
                    f'— {step.get("description", "")}'
                )

        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')

    def validate_plan(self, plan: dict) -> dict:
        """Validate that all actions in the plan are known and parameters are correct."""
        validated_steps = []
        warnings = []

        for i, step in enumerate(plan.get('plan', [])):
            action_name = step.get('action', '')

            if action_name not in ACTION_REGISTRY:
                warnings.append(f'Step {i+1}: Unknown action "{action_name}" — removed')
                continue

            action_def = ACTION_REGISTRY[action_name]
            params = step.get('params', {})

            # Check required parameters
            for param_name in action_def.parameters:
                if param_name not in params:
                    warnings.append(
                        f'Step {i+1}: Missing parameter "{param_name}" for {action_name}'
                    )

            validated_steps.append({
                'action': action_name,
                'params': params,
                'description': step.get('description', ''),
                'timeout': action_def.timeout,
            })

        if warnings:
            for w in warnings:
                self.get_logger().warn(f'Plan validation: {w}')

        plan['plan'] = validated_steps
        plan['validated'] = True
        plan['warnings'] = warnings
        return plan


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TaskPlanner())
    rclpy.shutdown()
```

## Scene Description Builder

The LLM needs a text description of what the robot currently perceives:

```python
# humanoid_planning/scene_builder.py
"""Builds a text scene description from perception data for the LLM planner."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import json
import math


class SceneBuilder(Node):
    """Aggregates perception data into a text scene description for the LLM."""

    def __init__(self):
        super().__init__('scene_builder')

        # Subscribe to perception sources
        self.create_subscription(
            Detection2DArray, '/humanoid/detectnet/detections',
            self.on_detections, 10
        )
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry',
            self.on_odometry, 10
        )
        self.create_subscription(
            PoseArray, '/humanoid/tracked_people',
            self.on_people, 10
        )

        # Publish scene description
        self.scene_pub = self.create_publisher(
            String, '/humanoid/scene_description', 10
        )

        # State
        self.detected_objects = []
        self.robot_position = (0.0, 0.0, 0.0)
        self.robot_heading = 0.0
        self.nearby_people = 0

        # Update scene at 2 Hz
        self.create_timer(0.5, self.publish_scene)

        self.get_logger().info('Scene builder active')

    def on_detections(self, msg: Detection2DArray):
        self.detected_objects = []
        for det in msg.detections:
            for result in det.results:
                if result.hypothesis.score < 0.5:
                    continue
                self.detected_objects.append({
                    'class': result.hypothesis.class_id,
                    'confidence': round(result.hypothesis.score, 2),
                    'bbox_center_x': round(det.bbox.center.position.x),
                    'bbox_center_y': round(det.bbox.center.position.y),
                    'bbox_size': round(det.bbox.size_y),
                })

    def on_odometry(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_position = (round(p.x, 2), round(p.y, 2), round(p.z, 2))
        # Extract yaw from quaternion
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.robot_heading = round(math.degrees(yaw), 1)

    def on_people(self, msg: PoseArray):
        self.nearby_people = len(msg.poses)

    def publish_scene(self):
        # Build human-readable scene description
        lines = []
        lines.append(f"Robot position: ({self.robot_position[0]}, {self.robot_position[1]}), "
                     f"heading: {self.robot_heading} degrees")
        lines.append(f"People nearby: {self.nearby_people}")

        if self.detected_objects:
            lines.append(f"Visible objects ({len(self.detected_objects)}):")
            for obj in self.detected_objects:
                size_desc = "close" if obj['bbox_size'] > 200 else "medium" if obj['bbox_size'] > 100 else "far"
                lines.append(
                    f"  - {obj['class']} (confidence: {obj['confidence']}, "
                    f"distance: {size_desc})"
                )
        else:
            lines.append("No objects currently detected in view.")

        # Nearest known location
        locations = {
            'kitchen': (5.0, 2.0), 'living room': (0.0, 0.0),
            'bedroom': (-3.0, 4.0), 'table': (2.0, 1.0),
        }
        nearest = min(
            locations.items(),
            key=lambda loc: math.sqrt(
                (loc[1][0] - self.robot_position[0]) ** 2 +
                (loc[1][1] - self.robot_position[1]) ** 2
            ),
        )
        dist = math.sqrt(
            (nearest[1][0] - self.robot_position[0]) ** 2 +
            (nearest[1][1] - self.robot_position[1]) ** 2
        )
        lines.append(f"Nearest known location: {nearest[0]} ({dist:.1f}m away)")

        scene = "\n".join(lines)
        msg = String()
        msg.data = scene
        self.scene_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SceneBuilder())
    rclpy.shutdown()
```

## The Plan Executor

### Executing Plans Step by Step

```python
# humanoid_planning/plan_executor.py
"""Executes LLM-generated action plans step by step with monitoring."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import json


class PlanExecutor(Node):
    """Executes a validated action plan, handling failures and replanning."""

    # Map known location names to coordinates
    LOCATIONS = {
        'kitchen': (5.0, 2.0),
        'living room': (0.0, 0.0),
        'bedroom': (-3.0, 4.0),
        'front door': (8.0, 0.0),
        'table': (2.0, 1.0),
        'desk': (1.0, -2.0),
        'charging station': (-1.0, -1.0),
        'user': (0.0, 0.0),  # Updated dynamically
    }

    def __init__(self):
        super().__init__('plan_executor')

        self.create_subscription(
            String, '/humanoid/action_plan', self.on_plan, 10
        )

        self.status_pub = self.create_publisher(
            String, '/humanoid/execution/status', 10
        )
        self.say_pub = self.create_publisher(
            String, '/humanoid/voice/say', 10
        )

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.current_plan = None
        self.current_step = 0
        self.is_executing = False

        self.get_logger().info('Plan executor ready')

    def on_plan(self, msg: String):
        plan_data = json.loads(msg.data)
        self.current_plan = plan_data.get('plan', [])
        self.current_step = 0

        if not self.current_plan:
            self.get_logger().warn('Received empty plan')
            return

        self.get_logger().info(
            f'Executing plan with {len(self.current_plan)} steps'
        )
        self.execute_next_step()

    def execute_next_step(self):
        if self.current_step >= len(self.current_plan):
            self.get_logger().info('Plan execution complete!')
            self.publish_status('COMPLETE', 'All steps executed successfully')
            self.say('Task completed.')
            self.is_executing = False
            return

        step = self.current_plan[self.current_step]
        action = step['action']
        params = step.get('params', {})
        description = step.get('description', '')

        self.get_logger().info(
            f'Step {self.current_step + 1}/{len(self.current_plan)}: '
            f'{action}({params}) — {description}'
        )
        self.publish_status('EXECUTING', f'Step {self.current_step + 1}: {action}')
        self.is_executing = True

        # Dispatch to appropriate handler
        if action == 'navigate_to':
            self.execute_navigate(params)
        elif action == 'say':
            self.execute_say(params)
            self.step_complete(True)
        elif action == 'scan_area':
            self.get_logger().info('Scanning area...')
            self.step_complete(True)  # Simplified
        elif action == 'detect_object':
            self.get_logger().info(f'Detecting: {params.get("object_name", "unknown")}')
            self.step_complete(True)  # Simplified
        elif action == 'grasp':
            self.get_logger().info(f'Grasping: {params.get("object_name", "unknown")}')
            self.step_complete(True)  # Simplified
        elif action == 'verify_grasp':
            self.get_logger().info('Verifying grasp...')
            self.step_complete(True)  # Simplified
        elif action == 'handover':
            self.get_logger().info(f'Handing over: {params.get("object_name", "unknown")}')
            self.step_complete(True)  # Simplified
        elif action == 'place':
            self.get_logger().info(f'Placing: {params.get("object_name", "unknown")}')
            self.step_complete(True)  # Simplified
        elif action == 'look_at':
            self.get_logger().info(f'Looking at: {params.get("target", "unknown")}')
            self.step_complete(True)  # Simplified
        elif action == 'wait':
            seconds = float(params.get('seconds', 1.0))
            self.create_timer(seconds, lambda: self.step_complete(True))
        else:
            self.get_logger().warn(f'Unknown action: {action}')
            self.step_complete(False)

    def execute_navigate(self, params: dict):
        """Execute a navigation action via Nav2."""
        location = params.get('location', '')

        # Resolve location to coordinates
        if location in self.LOCATIONS:
            x, y = self.LOCATIONS[location]
        elif ',' in str(location):
            parts = str(location).split(',')
            x, y = float(parts[0]), float(parts[1])
        else:
            self.get_logger().warn(f'Unknown location: {location}')
            self.step_complete(False)
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to ({x}, {y})')

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server not available')
            self.step_complete(False)
            return

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_nav_goal_response)

    def on_nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.step_complete(False)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        result = future.result()
        success = result.status == 4  # SUCCEEDED
        if success:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().warn(f'Navigation failed with status: {result.status}')
        self.step_complete(success)

    def execute_say(self, params: dict):
        text = params.get('text', '')
        self.say(text)

    def say(self, text: str):
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)

    def step_complete(self, success: bool):
        step = self.current_plan[self.current_step]
        action = step['action']

        if success:
            self.get_logger().info(f'Step {self.current_step + 1} ({action}) succeeded')
            self.current_step += 1
            self.execute_next_step()
        else:
            self.get_logger().error(
                f'Step {self.current_step + 1} ({action}) FAILED'
            )
            self.publish_status(
                'FAILED',
                f'Step {self.current_step + 1} ({action}) failed. Requesting replan.'
            )
            self.say(f'I encountered a problem with {action}. Let me try a different approach.')
            # In a full implementation, trigger replanning here

    def publish_status(self, state: str, message: str):
        msg = String()
        msg.data = json.dumps({'state': state, 'message': message, 'step': self.current_step})
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PlanExecutor())
    rclpy.shutdown()
```

## Grounding: Connecting Words to the Physical World

### The Grounding Pipeline

```python
# humanoid_planning/grounding.py
"""Ground language references to physical objects detected by perception."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import json


class GroundingNode(Node):
    """Resolves language references to detected physical objects."""

    def __init__(self):
        super().__init__('grounding_node')

        self.detections = []

        self.create_subscription(
            Detection2DArray, '/humanoid/detectnet/detections',
            self.on_detections, 10
        )

        # Service-like: receive grounding requests, publish results
        self.create_subscription(
            String, '/humanoid/grounding/request', self.on_request, 10
        )
        self.result_pub = self.create_publisher(
            String, '/humanoid/grounding/result', 10
        )

        self.get_logger().info('Grounding node ready')

    def on_detections(self, msg: Detection2DArray):
        self.detections = []
        for det in msg.detections:
            for result in det.results:
                if result.hypothesis.score < 0.5:
                    continue
                self.detections.append({
                    'class': result.hypothesis.class_id,
                    'confidence': result.hypothesis.score,
                    'center_x': det.bbox.center.position.x,
                    'center_y': det.bbox.center.position.y,
                    'size_x': det.bbox.size_x,
                    'size_y': det.bbox.size_y,
                })

    def on_request(self, msg: String):
        """Resolve a language reference to a detected object."""
        request = json.loads(msg.data)
        query = request.get('object_name', '').lower()
        color = request.get('color', '').lower()

        matches = []
        for det in self.detections:
            det_class = det['class'].lower()

            # Class match
            if query in det_class or det_class in query:
                score = det['confidence']

                # Boost score if color matches (requires color detection)
                if color and color in det_class:
                    score += 0.2

                matches.append({
                    **det,
                    'match_score': min(score, 1.0),
                })

        # Sort by match score
        matches.sort(key=lambda m: m['match_score'], reverse=True)

        result = {
            'query': query,
            'matches': matches,
            'best_match': matches[0] if matches else None,
            'grounded': len(matches) > 0,
        }

        result_msg = String()
        result_msg.data = json.dumps(result)
        self.result_pub.publish(result_msg)

        if matches:
            self.get_logger().info(
                f'Grounded "{query}" → {matches[0]["class"]} '
                f'(confidence: {matches[0]["match_score"]:.2f})'
            )
        else:
            self.get_logger().warn(f'Could not ground "{query}" — no matching detections')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GroundingNode())
    rclpy.shutdown()
```

## Example: "Clean the Table"

Here is how the full pipeline handles a complex instruction:

```
User: "Hey robot, clean the table"

1. WAKE WORD → "hey robot" detected
2. WHISPER → transcript: "clean the table"
3. INTENT PARSER → {type: "complex", raw_text: "clean the table"}
4. SCENE BUILDER →
   "Robot position: (0.0, 0.0), heading: 0 degrees
    Visible objects: cup (close), plate (close), book (medium)
    Nearest location: living room (0.0m away)"

5. LLM PLANNER generates:
   {
     "reasoning": "User wants the table cleared. I see a cup, plate, and book.
                   I need to navigate to the table, pick up each item, and place
                   them in appropriate locations.",
     "plan": [
       {"action": "say", "params": {"text": "I'll clean the table for you."}},
       {"action": "navigate_to", "params": {"location": "table"}},
       {"action": "scan_area", "params": {}},
       {"action": "detect_object", "params": {"object_name": "cup"}},
       {"action": "grasp", "params": {"object_name": "cup"}},
       {"action": "verify_grasp", "params": {}},
       {"action": "navigate_to", "params": {"location": "kitchen"}},
       {"action": "place", "params": {"object_name": "cup", "location": "counter"}},
       {"action": "navigate_to", "params": {"location": "table"}},
       {"action": "detect_object", "params": {"object_name": "plate"}},
       {"action": "grasp", "params": {"object_name": "plate"}},
       {"action": "verify_grasp", "params": {}},
       {"action": "navigate_to", "params": {"location": "kitchen"}},
       {"action": "place", "params": {"object_name": "plate", "location": "counter"}},
       {"action": "say", "params": {"text": "The table is clean!"}}
     ]
   }

6. EXECUTOR runs each step sequentially, monitoring for failures.
```

---

## Exercises

### Exercise 1: Custom Action Primitives
Add 3 new action primitives to the registry: `open_door(door_name)`, `turn_on_light(room)`, and `check_object(object_name)` (inspects an object and reports its state). Update the LLM system prompt and test with the instruction "go to the bedroom and turn on the light."

### Exercise 2: Replanning on Failure
Implement a replanning mechanism: when a step fails (e.g., grasp fails because the object moved), the executor sends the failure context back to the LLM with the instruction "Step N failed because [reason]. The current scene is [updated scene]. Replan the remaining steps." Test by simulating a grasp failure mid-plan.

### Exercise 3: Multi-Object Task
Test the planner with the instruction "set the table for dinner" when the scene contains plates, cups, and silverware at various locations. Verify the LLM generates a reasonable plan that fetches items from their current locations and places them on the table in a logical arrangement.

---

:::tip Key Takeaway
Cognitive planning with LLMs replaces brittle hardcoded behavior with **flexible, common-sense reasoning**. The critical design pattern is the **action primitive registry** — the LLM never generates raw motor commands, only compositions of well-tested primitives. Combined with grounding (matching words to perceived objects) and validation (checking that plans are physically possible), this architecture produces robots that can handle instructions they have never been explicitly programmed for.
:::
