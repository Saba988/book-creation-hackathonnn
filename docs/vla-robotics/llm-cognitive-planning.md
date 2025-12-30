---
sidebar_position: 3
title: 'LLM-based Cognitive Planning'
---

# LLM-based Cognitive Planning

This chapter explores Large Language Model (LLM)-based cognitive planning for robotics applications. You'll learn how to leverage the reasoning capabilities of LLMs to generate intelligent action sequences for robots based on natural language commands and environmental context.

## Introduction to Cognitive Planning

Cognitive planning in robotics involves generating sequences of actions to achieve goals, considering environmental constraints, object properties, and task requirements. Traditional planning approaches often require predefined action libraries and explicit state representations, but LLMs offer a more flexible approach to planning.

### Traditional vs. LLM-based Planning

**Traditional Planning:**
- Requires predefined action schemas
- Needs explicit state representations
- Uses formal logic or search algorithms
- Limited by the designer's foresight

**LLM-based Planning:**
- Leverages world knowledge from training
- Handles natural language directly
- Adapts to novel situations
- Generalizes across domains

### Key Benefits of LLM-based Planning

- **Natural Language Interface**: Direct command interpretation
- **World Knowledge**: Incorporates common sense reasoning
- **Flexibility**: Handles unexpected situations
- **Learning**: Can be fine-tuned for specific domains

## LLM Integration with Robotics

### Architecture Overview

```
Natural Language Command
         ↓
    LLM Planner
         ↓
Action Sequence (Robot-Specific)
         ↓
    Robot Execution
```

### Planning Pipeline Components

1. **Prompt Engineering**: Formulating inputs for the LLM
2. **Context Integration**: Including environmental and robot state
3. **Action Sequencing**: Generating step-by-step plans
4. **Execution Validation**: Ensuring plan feasibility

## Implementing LLM-based Planners

### Basic LLM Integration

```python
import openai
import json
from dataclasses import dataclass
from typing import List, Dict, Any

@dataclass
class PlanStep:
    action: str
    parameters: Dict[str, Any]
    description: str

@dataclass
class RobotPlan:
    steps: List[PlanStep]
    success: bool
    reason: str

class LLMPlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        openai.api_key = api_key
        self.model = model

    def create_plan(self, command: str, robot_state: Dict[str, Any],
                   environment: Dict[str, Any]) -> RobotPlan:
        """Create a plan for a given command using LLM"""

        prompt = self._build_prompt(command, robot_state, environment)

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=1000
            )

            plan_text = response.choices[0].message['content'].strip()
            return self._parse_plan(plan_text)

        except Exception as e:
            return RobotPlan(
                steps=[],
                success=False,
                reason=f"Planning failed: {str(e)}"
            )

    def _build_prompt(self, command: str, robot_state: Dict[str, Any],
                     environment: Dict[str, Any]) -> str:
        """Build the prompt for the LLM"""
        return f"""
        Given the following information:

        Robot State: {json.dumps(robot_state, indent=2)}
        Environment: {json.dumps(environment, indent=2)}

        Please generate a step-by-step plan to execute the command: "{command}"

        Respond with a JSON array of steps, where each step has:
        - "action": the action to take
        - "parameters": any required parameters
        - "description": human-readable description

        Example response format:
        [
            {{
                "action": "navigate_to",
                "parameters": {{"location": "kitchen"}},
                "description": "Move to the kitchen area"
            }},
            {{
                "action": "detect_object",
                "parameters": {{"object_type": "cup"}},
                "description": "Look for a cup in the current location"
            }}
        ]
        """

    def _get_system_prompt(self) -> str:
        """Get the system prompt for the LLM"""
        return """
        You are an expert robotic planning system. Your role is to generate detailed, step-by-step plans for robots to execute tasks.

        Guidelines:
        1. Consider the robot's current state and capabilities
        2. Account for environmental constraints
        3. Generate feasible action sequences
        4. Include error handling and verification steps
        5. Make sure each step is executable by the robot

        Available actions typically include: navigate_to, detect_object, grasp_object, place_object, open_gripper, close_gripper, speak, wait, etc.
        """

    def _parse_plan(self, plan_text: str) -> RobotPlan:
        """Parse the LLM response into a structured plan"""
        try:
            # Try to extract JSON from the response
            start_idx = plan_text.find('[')
            end_idx = plan_text.rfind(']') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = plan_text[start_idx:end_idx]
                steps_data = json.loads(json_str)

                steps = []
                for step_data in steps_data:
                    step = PlanStep(
                        action=step_data['action'],
                        parameters=step_data.get('parameters', {}),
                        description=step_data.get('description', '')
                    )
                    steps.append(step)

                return RobotPlan(
                    steps=steps,
                    success=True,
                    reason="Plan generated successfully"
                )
            else:
                return RobotPlan(
                    steps=[],
                    success=False,
                    reason="Could not extract plan from LLM response"
                )
        except json.JSONDecodeError:
            return RobotPlan(
                steps=[],
                success=False,
                reason="Invalid JSON in LLM response"
            )
```

### Advanced Planning with Context

For more sophisticated planning, we can include additional context and reasoning:

```python
class AdvancedLLMPlanner(LLMPlanner):
    def create_plan_with_reasoning(self, command: str, robot_state: Dict[str, Any],
                                  environment: Dict[str, Any]) -> RobotPlan:
        """Create a plan with explicit reasoning steps"""

        prompt = self._build_reasoning_prompt(command, robot_state, environment)

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_reasoning_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=1500
            )

            response_text = response.choices[0].message['content'].strip()
            return self._parse_reasoned_plan(response_text)

        except Exception as e:
            return RobotPlan(
                steps=[],
                success=False,
                reason=f"Planning failed: {str(e)}"
            )

    def _build_reasoning_prompt(self, command: str, robot_state: Dict[str, Any],
                               environment: Dict[str, Any]) -> str:
        """Build a prompt that encourages reasoning"""
        return f"""
        Task: {command}

        Robot Capabilities: {json.dumps(robot_state.get('capabilities', []), indent=2)}
        Current State: {json.dumps({k: v for k, v in robot_state.items() if k != 'capabilities'}, indent=2)}
        Environment: {json.dumps(environment, indent=2)}

        Please think step-by-step:
        1. What is the goal?
        2. What are the constraints?
        3. What sequence of actions would achieve the goal?
        4. What could go wrong and how to handle it?

        Then provide the action sequence in the JSON format.
        """

    def _get_reasoning_system_prompt(self) -> str:
        """System prompt that encourages reasoning"""
        return """
        You are an expert robotic planning system that thinks step-by-step.
        For each task, first reason about the problem, then provide the action sequence.

        Your reasoning should consider:
        - The robot's physical capabilities and limitations
        - Environmental constraints and obstacles
        - Safety considerations
        - Efficiency of the plan
        - Potential failure modes and recovery strategies

        After reasoning, provide a JSON-formatted action sequence.
        """

    def _parse_reasoned_plan(self, response_text: str) -> RobotPlan:
        """Parse a response that includes reasoning"""
        try:
            # Look for the JSON part after the reasoning
            json_start = response_text.rfind('[')
            json_end = response_text.rfind(']') + 1

            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                steps_data = json.loads(json_str)

                steps = []
                for step_data in steps_data:
                    step = PlanStep(
                        action=step_data['action'],
                        parameters=step_data.get('parameters', {}),
                        description=step_data.get('description', '')
                    )
                    steps.append(step)

                return RobotPlan(
                    steps=steps,
                    success=True,
                    reason="Plan generated successfully with reasoning"
                )
            else:
                return RobotPlan(
                    steps=[],
                    success=False,
                    reason="Could not extract plan from response"
                )
        except json.JSONDecodeError:
            return RobotPlan(
                steps=[],
                success=False,
                reason="Invalid JSON in response"
            )
```

## Planning with Environmental Context

### Object and Scene Understanding

```python
class ContextAwarePlanner(AdvancedLLMPlanner):
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        super().__init__(api_key, model)
        self.object_knowledge = self._load_object_knowledge()
        self.scene_knowledge = self._load_scene_knowledge()

    def _load_object_knowledge(self) -> Dict[str, Any]:
        """Load knowledge about objects and their properties"""
        return {
            "cup": {
                "grasp_method": "top_grasp",
                "typical_locations": ["kitchen_counter", "dining_table", "cabinet"],
                "size": "small",
                "weight": "light"
            },
            "book": {
                "grasp_method": "side_grasp",
                "typical_locations": ["desk", "shelf", "table"],
                "size": "medium",
                "weight": "light"
            },
            "bottle": {
                "grasp_method": "cylindrical_grasp",
                "typical_locations": ["kitchen_counter", "refrigerator", "table"],
                "size": "medium",
                "weight": "light_to_medium"
            }
        }

    def _load_scene_knowledge(self) -> Dict[str, Any]:
        """Load knowledge about scenes and their typical contents"""
        return {
            "kitchen": {
                "common_objects": ["cup", "plate", "bottle", "food"],
                "robot_actions": ["fetch_item", "open_fridge", "pour_liquid"],
                "navigation_points": ["counter", "table", "fridge", "sink"]
            },
            "living_room": {
                "common_objects": ["remote", "book", "pillow", "glass"],
                "robot_actions": ["fetch_remote", "tidy_up"],
                "navigation_points": ["couch", "coffee_table", "tv_stand"]
            }
        }

    def create_contextual_plan(self, command: str, robot_state: Dict[str, Any],
                              environment: Dict[str, Any]) -> RobotPlan:
        """Create a plan considering object and scene knowledge"""

        # Enhance environment with contextual knowledge
        enhanced_env = self._enhance_environment(environment)

        return self.create_plan_with_reasoning(command, robot_state, enhanced_env)

    def _enhance_environment(self, environment: Dict[str, Any]) -> Dict[str, Any]:
        """Add contextual knowledge to environment"""
        enhanced = environment.copy()

        # Add object knowledge
        if 'objects' in enhanced:
            for obj in enhanced['objects']:
                obj_type = obj.get('type', '').lower()
                if obj_type in self.object_knowledge:
                    obj['properties'] = self.object_knowledge[obj_type]

        # Add scene knowledge
        if 'location' in enhanced:
            location = enhanced['location'].lower()
            if location in self.scene_knowledge:
                enhanced['scene_properties'] = self.scene_knowledge[location]

        return enhanced
```

## Integration with ROS 2

### ROS 2 Planning Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import threading

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Initialize LLM planner
        api_key = self.declare_parameter('openai_api_key', '').value
        if not api_key:
            self.get_logger().error("OpenAI API key not provided!")
            return

        self.planner = ContextAwarePlanner(api_key)

        # Publishers and subscribers
        self.plan_publisher = self.create_publisher(String, 'robot_plan', 10)
        self.command_subscription = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )

        # Robot state and environment
        self.robot_state = {
            'position': [0, 0, 0],
            'orientation': [0, 0, 0, 1],
            'battery_level': 0.8,
            'gripper_status': 'open',
            'capabilities': ['navigation', 'manipulation', 'speech']
        }

        self.environment = {
            'objects': [],
            'location': 'unknown',
            'obstacles': []
        }

        # Timer for state updates
        self.state_update_timer = self.create_timer(1.0, self.update_state)

    def command_callback(self, msg):
        """Handle natural language command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Create plan in a separate thread to avoid blocking
        thread = threading.Thread(
            target=self._process_command,
            args=(command,)
        )
        thread.start()

    def _process_command(self, command: str):
        """Process command and generate plan"""
        try:
            plan = self.planner.create_contextual_plan(
                command,
                self.robot_state,
                self.environment
            )

            if plan.success:
                plan_msg = String()
                plan_dict = {
                    'steps': [
                        {
                            'action': step.action,
                            'parameters': step.parameters,
                            'description': step.description
                        }
                        for step in plan.steps
                    ],
                    'command': command
                }
                plan_msg.data = json.dumps(plan_dict)

                self.plan_publisher.publish(plan_msg)
                self.get_logger().info(f'Published plan with {len(plan.steps)} steps')
            else:
                self.get_logger().error(f'Planning failed: {plan.reason}')

        except Exception as e:
            self.get_logger().error(f'Error in planning: {str(e)}')

    def update_state(self):
        """Update robot state and environment"""
        # This would typically come from other ROS nodes
        # For now, we'll just log the current state
        self.get_logger().debug(f'Current robot state: {self.robot_state}')

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Planning Algorithms and Strategies

### Hierarchical Planning

```python
class HierarchicalLLMPlanner(LLMPlanner):
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        super().__init__(api_key, model)
        self.task_library = self._initialize_task_library()

    def _initialize_task_library(self) -> Dict[str, List[str]]:
        """Initialize a library of high-level tasks"""
        return {
            "fetch_object": [
                "navigate_to_object_location",
                "detect_object",
                "grasp_object",
                "navigate_to_destination",
                "place_object"
            ],
            "clean_room": [
                "detect_debris",
                "grasp_debris",
                "navigate_to_trash",
                "dispose_object"
            ],
            "serve_drink": [
                "navigate_to_kitchen",
                "detect_cup",
                "grasp_cup",
                "navigate_to_fridge",
                "open_fridge",
                "grasp_bottle",
                "pour_liquid",
                "place_cup_with_liquid",
                "navigate_to_person",
                "serve_drink"
            ]
        }

    def create_hierarchical_plan(self, command: str, robot_state: Dict[str, Any],
                                environment: Dict[str, Any]) -> RobotPlan:
        """Create a plan using hierarchical task decomposition"""

        # First, identify high-level task
        high_level_task = self._identify_high_level_task(command)

        if high_level_task and high_level_task in self.task_library:
            # Decompose into subtasks
            subtasks = self.task_library[high_level_task]

            # Generate detailed plan for each subtask
            detailed_plan = []
            for subtask in subtasks:
                subtask_plan = self._generate_subtask_plan(
                    subtask, command, robot_state, environment
                )
                detailed_plan.extend(subtask_plan.steps)

            return RobotPlan(
                steps=detailed_plan,
                success=True,
                reason=f"Hierarchical plan created for {high_level_task}"
            )
        else:
            # Fall back to regular planning
            return self.create_plan(command, robot_state, environment)

    def _identify_high_level_task(self, command: str) -> str:
        """Identify the high-level task from the command"""
        command_lower = command.lower()

        for task, keywords in {
            "fetch_object": ["get", "fetch", "bring", "pick up", "take"],
            "clean_room": ["clean", "tidy", "organize", "put away"],
            "serve_drink": ["serve", "get drink", "bring drink", "prepare drink"]
        }.items():
            if any(keyword in command_lower for keyword in keywords):
                return task

        return None

    def _generate_subtask_plan(self, subtask: str, original_command: str,
                              robot_state: Dict[str, Any],
                              environment: Dict[str, Any]) -> RobotPlan:
        """Generate a plan for a specific subtask"""
        # Create a more specific command for the subtask
        subtask_command = f"Perform {subtask} as part of {original_command}"

        return self.create_plan(subtask_command, robot_state, environment)
```

## Practical Exercise: Implementing an LLM-Based Planning System

### Objective
Implement an LLM-based cognitive planning system that generates action sequences for robot tasks based on natural language commands.

### System Architecture

```python
class CompleteLLMPlanningSystem:
    def __init__(self, api_key: str):
        self.planner = HierarchicalLLMPlanner(api_key)
        self.context_manager = ContextAwarePlanner(api_key)

        # Simulated robot state and environment
        self.robot_state = {
            'position': [0, 0, 0],
            'orientation': [0, 0, 0, 1],
            'battery_level': 1.0,
            'gripper_status': 'open',
            'capabilities': ['navigation', 'manipulation', 'object_detection']
        }

        self.environment = {
            'objects': [
                {'type': 'cup', 'name': 'red_cup', 'location': [1, 0, 0]},
                {'type': 'book', 'name': 'blue_book', 'location': [0, 1, 0]},
                {'type': 'bottle', 'name': 'water_bottle', 'location': [2, 1, 0]}
            ],
            'location': 'living_room',
            'obstacles': [],
            'navigation_points': [
                {'name': 'sofa', 'position': [1, 1, 0]},
                {'name': 'table', 'position': [0, 0, 0]},
                {'name': 'door', 'position': [3, 0, 0]}
            ]
        }

    def execute_command(self, command: str) -> RobotPlan:
        """Execute a natural language command"""
        print(f"Processing command: {command}")

        # Try hierarchical planning first
        plan = self.planner.create_hierarchical_plan(
            command,
            self.robot_state,
            self.environment
        )

        if not plan.success:
            # Fall back to contextual planning
            plan = self.context_manager.create_contextual_plan(
                command,
                self.robot_state,
                self.environment
            )

        if plan.success:
            print(f"Generated plan with {len(plan.steps)} steps:")
            for i, step in enumerate(plan.steps, 1):
                print(f"  {i}. {step.description}")
                print(f"     Action: {step.action}, Parameters: {step.parameters}")
        else:
            print(f"Planning failed: {plan.reason}")

        return plan

# Example usage
if __name__ == "__main__":
    # This would require a real OpenAI API key
    # api_key = "your-openai-api-key"
    # system = CompleteLLMPlanningSystem(api_key)
    #
    # # Test with various commands
    # commands = [
    #     "Please bring me the red cup from the living room",
    #     "Clean up the books on the table",
    #     "Go to the kitchen and get me a drink"
    # ]
    #
    # for cmd in commands:
    #     plan = system.execute_command(cmd)
    #     print("-" * 50)
    pass
```

### Expected Outcome

The LLM-based planning system should:
- Generate valid action sequences for natural language commands
- Consider environmental context and robot capabilities
- Handle complex multi-step tasks through hierarchical decomposition
- Demonstrate measurable planning accuracy (target: 85% success rate for common tasks)

## Performance Optimization

### Caching and Efficiency

```python
import functools
import time
from typing import Optional

class OptimizedLLMPlanner(HierarchicalLLMPlanner):
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        super().__init__(api_key, model)
        self.plan_cache = {}
        self.max_cache_size = 100

    def create_plan_with_cache(self, command: str, robot_state: Dict[str, Any],
                              environment: Dict[str, Any]) -> RobotPlan:
        """Create a plan with caching to improve efficiency"""

        # Create a cache key based on command and simplified state/environment
        cache_key = self._create_cache_key(command, robot_state, environment)

        # Check cache first
        if cache_key in self.plan_cache:
            cached_plan, timestamp = self.plan_cache[cache_key]
            # Check if cache is still valid (not too old)
            if time.time() - timestamp < 3600:  # 1 hour validity
                print(f"Using cached plan for: {command}")
                return cached_plan

        # Generate new plan
        plan = self.create_hierarchical_plan(command, robot_state, environment)

        # Store in cache
        if len(self.plan_cache) < self.max_cache_size:
            self.plan_cache[cache_key] = (plan, time.time())

        return plan

    def _create_cache_key(self, command: str, robot_state: Dict[str, Any],
                         environment: Dict[str, Any]) -> str:
        """Create a cache key for the given inputs"""
        # Simplify the state and environment for caching purposes
        simplified_state = {
            'capabilities': sorted(robot_state.get('capabilities', [])),
            'location': environment.get('location', 'unknown')
        }

        return f"{command}||{json.dumps(simplified_state, sort_keys=True)}"
```

## Troubleshooting Common Issues

### Planning Failures

```python
class RobustLLMPlanner(OptimizedLLMPlanner):
    def create_robust_plan(self, command: str, robot_state: Dict[str, Any],
                          environment: Dict[str, Any]) -> RobotPlan:
        """Create a plan with multiple fallback strategies"""

        # Strategy 1: Hierarchical planning
        plan = self.create_hierarchical_plan(command, robot_state, environment)
        if plan.success:
            return plan

        # Strategy 2: Contextual planning
        context_planner = ContextAwarePlanner(self.planner.api_key, self.planner.model)
        plan = context_planner.create_contextual_plan(command, robot_state, environment)
        if plan.success:
            return plan

        # Strategy 3: Simple planning
        plan = self.create_plan(command, robot_state, environment)
        if plan.success:
            return plan

        # All strategies failed
        return RobotPlan(
            steps=[],
            success=False,
            reason="All planning strategies failed"
        )
```

## Summary

LLM-based cognitive planning offers significant advantages over traditional planning approaches by leveraging the world knowledge and reasoning capabilities of large language models. The key components include:

- **Prompt Engineering**: Carefully crafted prompts that guide the LLM to generate appropriate plans
- **Context Integration**: Including environmental and robot state information
- **Hierarchical Decomposition**: Breaking complex tasks into manageable subtasks
- **Caching and Optimization**: Improving efficiency for repeated tasks

The integration with ROS 2 enables real-world robot control, while advanced techniques like contextual planning and hierarchical decomposition enhance the system's capability to handle complex tasks.

In the next chapter, we'll explore how to integrate vision, language, and action components into a complete Vision-Language-Action loop for humanoid systems.