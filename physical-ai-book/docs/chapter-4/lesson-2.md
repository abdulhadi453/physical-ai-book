# Lesson 2: LLM-based Cognitive Planning

## Overview

Welcome to Lesson 2 of the Vision-Language-Action (VLA) Systems module! In this lesson, you will implement the cognitive planning component of the VLA system using Large Language Models (LLMs) to interpret natural language commands and generate executable action sequences. This component serves as the brain of the VLA system, transforming high-level human instructions into structured robotic actions.

## Learning Objectives

By the end of this lesson, you will be able to:
1. Integrate LLMs for natural language understanding and task decomposition
2. Create prompt templates for robotic command interpretation
3. Generate structured action sequences from natural language
4. Implement spatial reasoning for object manipulation
5. Handle complex multi-step commands with dependency tracking

## Prerequisites

Before starting this lesson, ensure you have:
- Completed Lesson 1 (Voice Command Processing)
- Set up OpenAI API access (or alternative LLM provider)
- Familiarized yourself with the VLA system architecture
- Installed required dependencies (openai, transformers, etc.)
- Understood basic concepts of prompt engineering

## Cognitive Planning Architecture

The cognitive planning component follows this architecture:

```
Natural Language Command → LLM Processing → Task Decomposition → Action Sequences → ROS 2 Actions
```

Key considerations include:
- Understanding spatial relationships and object manipulation
- Decomposing complex tasks into simple, executable steps
- Maintaining context across multi-step operations
- Handling ambiguous or underspecified commands

## Implementation Steps

### Step 1: Set Up LLM Client

First, let's create the LLM client that will handle communication with the language model:

```python
# src/vla/llm/llm_client.py

import openai
import os
import time
from typing import Dict, Any, List, Optional
import logging
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)

class LLMClient:
    """
    Client for interacting with Large Language Models for cognitive planning.
    """

    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-4-turbo"):
        """
        Initialize LLM client.

        Args:
            api_key: OpenAI API key (if None, uses OPENAI_API_KEY environment variable)
            model: LLM model to use (default: gpt-4-turbo for best reasoning)
        """
        # Set API key
        if api_key:
            openai.api_key = api_key
        elif os.getenv("OPENAI_API_KEY"):
            openai.api_key = os.getenv("OPENAI_API_KEY")
        else:
            raise ValueError("OpenAI API key not provided and OPENAI_API_KEY environment variable not set")

        self.model = model
        self.max_retries = 3
        self.timeout = 30

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    def generate_response(self, prompt: str, max_tokens: int = 500) -> Dict[str, Any]:
        """
        Generate response from LLM with retry logic.

        Args:
            prompt: Input prompt for the LLM
            max_tokens: Maximum tokens for the response

        Returns:
            Dictionary containing the LLM response and metadata
        """
        try:
            start_time = time.time()

            response = openai.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=max_tokens,
                temperature=0.1,  # Low temperature for consistent planning
                timeout=self.timeout
            )

            end_time = time.time()

            return {
                "success": True,
                "content": response.choices[0].message.content,
                "usage": {
                    "prompt_tokens": response.usage.prompt_tokens,
                    "completion_tokens": response.usage.completion_tokens,
                    "total_tokens": response.usage.total_tokens
                },
                "processing_time": end_time - start_time,
                "model": self.model
            }

        except Exception as e:
            logger.error(f"LLM request failed: {e}")
            raise

    def _get_system_prompt(self) -> str:
        """
        Get the system prompt that guides the LLM's behavior.

        Returns:
            System prompt string
        """
        return """
        You are a robotic cognitive planner. Your role is to interpret natural language commands and convert them into structured action sequences that a robot can execute.

        Rules:
        1. Interpret spatial relationships (left, right, near, far, on top of, under, etc.)
        2. Break down complex commands into simple, executable steps
        3. Identify objects that need to be manipulated
        4. Consider environmental constraints and safety
        5. Return structured JSON responses when possible
        6. If a command is ambiguous, ask for clarification or make reasonable assumptions
        7. Always prioritize safety in your planning
        """

    def validate_response_format(self, response: str) -> bool:
        """
        Validate that the LLM response is in the expected format.

        Args:
            response: Raw response from LLM

        Returns:
            True if response format is valid, False otherwise
        """
        # Basic validation - in production, you might want more sophisticated validation
        required_keywords = ["action", "move", "navigate", "grasp", "place", "go", "pick"]
        response_lower = response.lower()

        # Check if response contains action-related keywords
        has_actions = any(keyword in response_lower for keyword in required_keywords)

        # Check for basic structure (contains steps or sequence)
        has_structure = "step" in response_lower or "sequence" in response_lower or "action" in response_lower

        return has_actions and has_structure
```

### Step 2: Implement Cognitive Planning Module

Now let's create the main cognitive planning module that will interpret commands and generate action sequences:

```python
# src/vla/llm/cognitive_planner.py

import json
import re
from typing import Dict, Any, List, Optional
from enum import Enum
from dataclasses import dataclass
from src.vla.llm.llm_client import LLMClient
from src.vla.models.detected_object import DetectedObject
import logging
import time

logger = logging.getLogger(__name__)

class ActionStepType(Enum):
    """Types of actions that can be executed by the robot."""
    NAVIGATE_TO = "NAVIGATE_TO"
    GRASP_OBJECT = "GRASP_OBJECT"
    RELEASE_OBJECT = "RELEASE_OBJECT"
    LOOK_AT = "LOOK_AT"
    FOLLOW_PATH = "FOLLOW_PATH"
    WAIT = "WAIT"
    REPORT_STATUS = "REPORT_STATUS"

@dataclass
class ActionStep:
    """Represents a single action in an action sequence."""
    id: str
    action_type: ActionStepType
    parameters: Dict[str, Any]
    timeout: int = 10
    required_objects: List[str] = None
    preconditions: List[Dict[str, Any]] = None
    expected_outcomes: List[Dict[str, Any]] = None

    def __post_init__(self):
        if self.required_objects is None:
            self.required_objects = []
        if self.preconditions is None:
            self.preconditions = []
        if self.expected_outcomes is None:
            self.expected_outcomes = []

class IntentType(Enum):
    """Types of intents that can be classified from voice commands."""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    INSPECTION = "inspection"
    COMPLEX_TASK = "complex_task"

@dataclass
class ProcessedIntent:
    """Represents a processed intent with action sequence and metadata."""
    id: str
    original_command_id: str
    intent_type: IntentType
    action_sequence: List[ActionStep]
    context_objects: List[DetectedObject]
    spatial_constraints: Dict[str, Any]
    priority: str
    created_at: float

class CognitivePlanner:
    """
    Main cognitive planning module that interprets natural language commands
    and generates executable action sequences.
    """

    def __init__(self, llm_client: LLMClient):
        """
        Initialize cognitive planner.

        Args:
            llm_client: LLM client for natural language processing
        """
        self.llm_client = llm_client

    def plan_command(self,
                    command_text: str,
                    context_objects: List[DetectedObject] = None,
                    environment_context: str = "") -> ProcessedIntent:
        """
        Plan actions for a given command using LLM-based cognitive planning.

        Args:
            command_text: Natural language command to process
            context_objects: List of objects detected in the environment
            environment_context: Additional context about the environment

        Returns:
            ProcessedIntent with action sequence and metadata
        """
        try:
            # Determine intent type based on command
            intent_type = self._classify_intent(command_text)

            # Generate prompt for LLM
            prompt = self._create_planning_prompt(
                command_text,
                context_objects or [],
                environment_context
            )

            # Get response from LLM
            llm_response = self.llm_client.generate_response(prompt)

            if not llm_response["success"]:
                raise Exception(f"LLM planning failed: {llm_response.get('error', 'Unknown error')}")

            # Parse the LLM response into action sequence
            action_sequence = self._parse_llm_response(llm_response["content"], command_text)

            # Create ProcessedIntent
            processed_intent = ProcessedIntent(
                id=f"intent_{int(time.time())}",
                original_command_id="",
                intent_type=intent_type,
                action_sequence=action_sequence,
                context_objects=context_objects or [],
                spatial_constraints=self._extract_spatial_constraints(command_text),
                priority=self._determine_priority(command_text),
                created_at=time.time()
            )

            return processed_intent

        except Exception as e:
            logger.error(f"Planning failed for command '{command_text}': {e}")
            raise

    def _classify_intent(self, command_text: str) -> IntentType:
        """
        Classify the intent type based on command text.

        Args:
            command_text: Natural language command

        Returns:
            IntentType classification
        """
        command_lower = command_text.lower()

        if any(word in command_lower for word in ["move", "go", "navigate", "turn", "drive", "walk"]):
            return IntentType.NAVIGATION
        elif any(word in command_lower for word in ["pick", "grasp", "take", "place", "put", "lift", "drop"]):
            return IntentType.MANIPULATION
        elif any(word in command_lower for word in ["find", "look", "identify", "see", "show", "locate"]):
            return IntentType.INSPECTION
        else:
            return IntentType.COMPLEX_TASK

    def _create_planning_prompt(self,
                               command_text: str,
                               context_objects: List[DetectedObject],
                               environment_context: str) -> str:
        """
        Create a prompt for the LLM to generate action sequences.

        Args:
            command_text: Natural language command
            context_objects: Objects detected in the environment
            environment_context: Additional environment context

        Returns:
            Formatted prompt string
        """
        objects_str = ""
        if context_objects:
            objects_str = "Available objects in environment:\n"
            for obj in context_objects:
                objects_str += f"- {obj.class_name} (ID: {obj.id}, color: {obj.color}, graspable: {obj.is_graspable})\n"

        environment_str = f"Environment context: {environment_context}" if environment_context else "No specific environment context provided."

        prompt = f"""
        Command: "{command_text}"

        {environment_str}

        {objects_str}

        Please break down this command into a sequence of executable robotic actions. Each action should be simple and specific. Consider:
        1. Spatial relationships and object identification
        2. Feasibility of actions given the environment
        3. Safety considerations
        4. Logical sequence of operations

        Return the action sequence in JSON format with these action types:
        - NAVIGATE_TO: Move to a specific location
        - GRASP_OBJECT: Grasp an identified object
        - RELEASE_OBJECT: Release a grasped object
        - LOOK_AT: Look at a specific location or object
        - FOLLOW_PATH: Follow a specific path
        - WAIT: Wait for a specific duration
        - REPORT_STATUS: Report current status

        Example JSON format:
        {{
            "actions": [
                {{
                    "id": "action_1",
                    "type": "NAVIGATE_TO",
                    "parameters": {{
                        "target_position": {{"x": 1.0, "y": 0.5, "z": 0.0}},
                        "description": "Move to location near the red cube"
                    }},
                    "timeout": 10
                }},
                {{
                    "id": "action_2",
                    "type": "GRASP_OBJECT",
                    "parameters": {{
                        "object_id": "cube_123",
                        "description": "Grasp the red cube"
                    }},
                    "timeout": 5
                }}
            ]
        }}

        Provide the action sequence now:
        """

        return prompt

    def _parse_llm_response(self, response: str, original_command: str) -> List[ActionStep]:
        """
        Parse the LLM response into a list of ActionStep objects.

        Args:
            response: Raw response from LLM
            original_command: Original command for context

        Returns:
            List of ActionStep objects
        """
        try:
            # Try to extract JSON from response
            json_match = re.search(r'\{.*\}', response, re.DOTALL)
            if json_match:
                json_str = json_match.group()
                parsed = json.loads(json_str)

                if "actions" in parsed:
                    actions = parsed["actions"]
                else:
                    # If no "actions" key, assume the whole object is the action list
                    actions = parsed
            else:
                # If no JSON found, try to parse as plain text
                return self._parse_text_response(response, original_command)

            action_steps = []
            for i, action_data in enumerate(actions if isinstance(actions, list) else [actions]):
                try:
                    action_step = ActionStep(
                        id=action_data.get("id", f"action_{i}"),
                        action_type=ActionStepType(action_data.get("type", "WAIT")),
                        parameters=action_data.get("parameters", {}),
                        timeout=action_data.get("timeout", 10),
                        required_objects=action_data.get("required_objects", []),
                        preconditions=action_data.get("preconditions", []),
                        expected_outcomes=action_data.get("expected_outcomes", [])
                    )
                    action_steps.append(action_step)
                except Exception as e:
                    logger.warning(f"Failed to parse action {action_data}: {e}")
                    continue

            return action_steps

        except json.JSONDecodeError:
            logger.warning("Failed to parse JSON from LLM response, falling back to text parsing")
            return self._parse_text_response(response, original_command)
        except Exception as e:
            logger.error(f"Error parsing LLM response: {e}")
            # Return a simple default action if parsing fails
            return [ActionStep(
                id="default_action",
                action_type=ActionStepType.REPORT_STATUS,
                parameters={"message": f"Unable to process command: {original_command}"},
                timeout=5
            )]

    def _parse_text_response(self, response: str, original_command: str) -> List[ActionStep]:
        """
        Fallback method to parse LLM response as plain text.

        Args:
            response: Raw response from LLM
            original_command: Original command for context

        Returns:
            List of ActionStep objects
        """
        # Simple text-based parsing - in production, use more sophisticated NLP
        response_lower = response.lower()

        actions = []

        # Look for navigation commands
        if any(word in response_lower for word in ["navigate", "move to", "go to", "move toward"]):
            actions.append(ActionStep(
                id="nav_action",
                action_type=ActionStepType.NAVIGATE_TO,
                parameters={"description": "Navigate to target location"},
                timeout=10
            ))

        # Look for manipulation commands
        if any(word in response_lower for word in ["grasp", "pick up", "take", "grab"]):
            actions.append(ActionStep(
                id="manip_action",
                action_type=ActionStepType.GRASP_OBJECT,
                parameters={"description": "Grasp target object"},
                timeout=5
            ))

        # Look for placement commands
        if any(word in response_lower for word in ["place", "put", "release", "drop"]):
            actions.append(ActionStep(
                id="place_action",
                action_type=ActionStepType.RELEASE_OBJECT,
                parameters={"description": "Release object at target location"},
                timeout=5
            ))

        # If no specific actions identified, create a status report
        if not actions:
            actions.append(ActionStep(
                id="status_action",
                action_type=ActionStepType.REPORT_STATUS,
                parameters={"message": f"Processed command: {original_command}"},
                timeout=2
            ))

        return actions

    def _extract_spatial_constraints(self, command_text: str) -> Dict[str, Any]:
        """
        Extract spatial constraints from command text.

        Args:
            command_text: Natural language command

        Returns:
            Dictionary of spatial constraints
        """
        constraints = {}
        command_lower = command_text.lower()

        # Extract relative positioning
        if "left of" in command_lower:
            constraints["relative_position"] = "left_of"
        elif "right of" in command_lower:
            constraints["relative_position"] = "right_of"
        elif "near" in command_lower or "close to" in command_lower:
            constraints["relative_position"] = "near"
        elif "on top of" in command_lower:
            constraints["relative_position"] = "on_top_of"
        elif "under" in command_lower or "below" in command_lower:
            constraints["relative_position"] = "under"

        # Extract distances
        distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(meter|m|cm|centimeter)', command_lower)
        if distance_match:
            distance = float(distance_match.group(1))
            unit = distance_match.group(2)
            constraints["distance"] = {"value": distance, "unit": unit}

        return constraints

    def _determine_priority(self, command_text: str) -> str:
        """
        Determine priority level based on command urgency.

        Args:
            command_text: Natural language command

        Returns:
            Priority level (HIGH, MEDIUM, LOW)
        """
        high_priority_keywords = ["emergency", "stop", "danger", "hazard", "urgent", "immediately"]
        command_lower = command_text.lower()

        if any(keyword in command_lower for keyword in high_priority_keywords):
            return "HIGH"
        elif "please" in command_lower or "carefully" in command_lower:
            return "LOW"
        else:
            return "MEDIUM"
```

### Step 3: Create Prompt Templates

Let's create specialized prompt templates for different types of commands:

```python
# src/vla/llm/prompt_templates.py

from typing import Dict, List
from src.vla.models.detected_object import DetectedObject

class PromptTemplates:
    """
    Collection of prompt templates for different types of VLA planning tasks.
    """

    @staticmethod
    def navigation_command_template(command: str, environment_context: str = "") -> str:
        """
        Template for navigation-related commands.
        """
        return f"""
        Command: "{command}"

        Environment context: {environment_context or 'Unknown environment, use general navigation planning.'}

        Plan a navigation sequence for this command. Consider:
        1. Safe path planning around obstacles
        2. Clear destination identification
        3. Appropriate movement speed based on environment

        Return action sequence in JSON format with NAVIGATE_TO and LOOK_AT actions as needed.
        """

    @staticmethod
    def manipulation_command_template(command: str,
                                    objects: List[DetectedObject],
                                    environment_context: str = "") -> str:
        """
        Template for manipulation-related commands.
        """
        objects_str = ""
        if objects:
            objects_str = "Available objects:\n"
            for obj in objects:
                objects_str += f"- {obj.class_name} (ID: {obj.id}, graspable: {obj.is_graspable}, color: {obj.color})\n"

        return f"""
        Command: "{command}"

        Environment context: {environment_context or 'Unknown environment, use general manipulation planning.'}

        {objects_str}

        Plan a manipulation sequence for this command. Consider:
        1. Object identification and verification
        2. Safe approach path to object
        3. Appropriate grasp strategy
        4. Target location for placement

        Return action sequence in JSON format with NAVIGATE_TO, GRASP_OBJECT, RELEASE_OBJECT actions as needed.
        """

    @staticmethod
    def complex_task_template(command: str,
                            objects: List[DetectedObject],
                            environment_context: str = "") -> str:
        """
        Template for complex multi-step tasks.
        """
        objects_str = ""
        if objects:
            objects_str = "Available objects:\n"
            for obj in objects:
                objects_str += f"- {obj.class_name} (ID: {obj.id}, graspable: {obj.is_graspable})\n"

        return f"""
        Command: "{command}"

        Environment context: {environment_context or 'Unknown environment, use general task planning.'}

        {objects_str}

        Decompose this complex task into a sequence of simpler actions. Consider:
        1. Task dependencies and logical order
        2. Object availability and accessibility
        3. Environmental constraints
        4. Safety considerations throughout the sequence

        Return action sequence in JSON format, clearly indicating the order of operations.
        """

    @staticmethod
    def spatial_reasoning_template(command: str,
                                 objects: List[DetectedObject],
                                 environment_context: str = "") -> str:
        """
        Template for commands requiring spatial reasoning.
        """
        objects_str = ""
        if objects:
            objects_str = "Objects in environment:\n"
            for obj in objects:
                objects_str += f"- {obj.class_name} at position ({obj.position_3d.x}, {obj.position_3d.y}, {obj.position_3d.z})\n"

        return f"""
        Command: "{command}"

        Environment context: {environment_context or 'Unknown environment layout.'}

        {objects_str}

        Interpret the spatial relationships in this command. Consider:
        1. Relative positions (left, right, near, far, etc.)
        2. Distance measurements if specified
        3. Spatial configuration of objects
        4. Feasible paths to achieve the spatial goal

        Return action sequence that respects spatial relationships in JSON format.
        """
```

### Step 4: Integrate with VLA System

Now let's create a module that integrates the cognitive planner with the voice processing system:

```python
# src/vla/integration/vla_planning_integrator.py

from typing import Dict, Any, Optional
from src.vla.llm.cognitive_planner import CognitivePlanner, ProcessedIntent
from src.vla.models.detected_object import DetectedObject
import logging

logger = logging.getLogger(__name__)

class VLACognitivePlanner:
    """
    Integrates cognitive planning with the broader VLA system.
    """

    def __init__(self, cognitive_planner: CognitivePlanner):
        """
        Initialize the VLA cognitive planning integrator.

        Args:
            cognitive_planner: Cognitive planner instance
        """
        self.cognitive_planner = cognitive_planner

    def process_command_for_planning(self,
                                   command_text: str,
                                   context_objects: List[DetectedObject] = None,
                                   environment_context: str = "") -> ProcessedIntent:
        """
        Process a command through the cognitive planning system.

        Args:
            command_text: The command text to plan
            context_objects: Objects detected in the environment
            environment_context: Additional environmental context

        Returns:
            ProcessedIntent with action sequence
        """
        logger.info(f"Planning command: '{command_text}'")

        try:
            # Plan the command using the cognitive planner
            intent = self.cognitive_planner.plan_command(
                command_text=command_text,
                context_objects=context_objects,
                environment_context=environment_context
            )

            logger.info(f"Planned {len(intent.action_sequence)} actions for command: {command_text}")
            return intent

        except Exception as e:
            logger.error(f"Planning failed for command '{command_text}': {e}")
            raise

    def validate_action_sequence(self, intent: ProcessedIntent) -> Dict[str, Any]:
        """
        Validate an action sequence for feasibility and safety.

        Args:
            intent: Processed intent with action sequence

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "is_valid": True,
            "issues": [],
            "warnings": [],
            "suggestions": []
        }

        # Check for empty action sequence
        if not intent.action_sequence:
            validation_results["is_valid"] = False
            validation_results["issues"].append("Action sequence is empty")
            return validation_results

        # Check for logical sequence (e.g., can't grasp before navigating to object)
        action_types = [action.action_type for action in intent.action_sequence]

        # Check for grasp before navigate (if there are both)
        if ActionStepType.GRASP_OBJECT in action_types and ActionStepType.NAVIGATE_TO in action_types:
            grasp_idx = action_types.index(ActionStepType.GRASP_OBJECT)
            navigate_idx = action_types.index(ActionStepType.NAVIGATE_TO)

            # If grasping comes before navigating, that might be an issue
            if grasp_idx < navigate_idx and len(action_types) > 1:
                validation_results["warnings"].append("Grasping action before navigation - ensure object is nearby")

        # Check for required objects
        for action in intent.action_sequence:
            if action.required_objects:
                available_object_ids = [obj.id for obj in intent.context_objects]
                missing_objects = [obj_id for obj_id in action.required_objects if obj_id not in available_object_ids]

                if missing_objects:
                    validation_results["warnings"].append(f"Action requires objects not in environment: {missing_objects}")

        # Check spatial constraints make sense
        if intent.spatial_constraints:
            if intent.spatial_constraints.get("distance", {}).get("value", 0) > 10:
                validation_results["warnings"].append("Large distance specified - verify feasibility")

        return validation_results

    def refine_plan(self, intent: ProcessedIntent, feedback: str) -> ProcessedIntent:
        """
        Refine a plan based on feedback or additional constraints.

        Args:
            intent: Original processed intent
            feedback: Feedback or additional constraints

        Returns:
            Refined ProcessedIntent
        """
        # In a real implementation, this would use the feedback to adjust the plan
        # For now, we'll just return the original intent
        logger.info(f"Refining plan with feedback: {feedback}")
        return intent
```

## Practical Exercise

### Exercise 2.1: Cognitive Planning Implementation

1. **Setup**: Create a test script that integrates the LLM client with the cognitive planner.

2. **Implementation**:
   ```python
   # test_cognitive_planning.py
   import os
   import time
   from src.vla.llm.llm_client import LLMClient
   from src.vla.llm.cognitive_planner import CognitivePlanner
   from src.vla.models.detected_object import DetectedObject, Point3D
   from src.vla.integration.vla_planning_integrator import VLACognitivePlanner

   def main():
       # Initialize LLM client (requires OPENAI_API_KEY environment variable)
       llm_client = LLMClient()

       # Initialize cognitive planner
       planner = CognitivePlanner(llm_client)

       # Initialize integrator
       vla_planner = VLACognitivePlanner(planner)

       print("Cognitive planning system initialized")

       # Test commands
       test_commands = [
           "Move to the table",
           "Pick up the red cube",
           "Go to the kitchen and find a cup",
           "Place the object to the left of the blue box"
       ]

       for command in test_commands:
           print(f"\nProcessing command: '{command}'")

           # Create mock detected objects for context
           mock_objects = [
               DetectedObject(
                   id="cube_1",
                   class_name="cube",
                   confidence=0.9,
                   bbox=None,  # Bounding box would come from vision system
                   position_3d=Point3D(x=1.0, y=0.5, z=0.0),
                   dimensions=None,
                   color="red",
                   is_graspable=True
               ),
               DetectedObject(
                   id="box_1",
                   class_name="box",
                   confidence=0.85,
                   bbox=None,
                   position_3d=Point3D(x=1.5, y=0.5, z=0.0),
                   dimensions=None,
                   color="blue",
                   is_graspable=False
               )
           ]

           try:
               # Plan the command
               intent = vla_planner.process_command_for_planning(command, mock_objects)

               print(f"Intent type: {intent.intent_type}")
               print(f"Action sequence:")
               for i, action in enumerate(intent.action_sequence):
                   print(f"  {i+1}. {action.action_type.value}: {action.parameters.get('description', 'No description')}")

               # Validate the plan
               validation = vla_planner.validate_action_sequence(intent)
               print(f"Validation: {validation['is_valid']}")
               if validation['warnings']:
                   print(f"Warnings: {validation['warnings']}")

           except Exception as e:
               print(f"Error processing command: {e}")

   if __name__ == "__main__":
       main()
   ```

3. **Testing**: Run the test script with various commands to see how the cognitive planner interprets them and generates action sequences.

## Key Concepts

### LLM Integration Strategies
- **Function Calling**: Modern LLMs support structured outputs through function calling
- **JSON Mode**: Some models can be constrained to return valid JSON
- **Prompt Engineering**: Carefully crafted prompts guide the LLM toward structured outputs

### Action Sequence Validation
Before executing action sequences, they should be validated for:
- Logical consistency (can't grasp an object before navigating to it)
- Environmental feasibility (is the target location accessible?)
- Safety compliance (no dangerous movements)

### Context Integration
The cognitive planner must effectively use:
- Object detection results from the vision system
- Environmental context from the simulation
- Spatial relationships specified in the command

## Common Challenges and Solutions

### Challenge 1: Ambiguous Commands
**Problem**: Natural language can be ambiguous ("Pick up that thing").
**Solution**: Use visual context to disambiguate references and ask for clarification when needed.

### Challenge 2: Complex Spatial Reasoning
**Problem**: Understanding complex spatial relationships ("between", "surrounding", etc.).
**Solution**: Implement specialized spatial reasoning modules that work with the LLM output.

### Challenge 3: Multi-step Task Decomposition
**Problem**: Breaking down complex tasks into executable steps.
**Solution**: Use hierarchical task networks (HTNs) or similar planning structures.

## Assessment Questions

1. How does the cognitive planner classify different types of commands?
2. What role does the system prompt play in ensuring consistent LLM behavior?
3. How does the system handle spatial relationships in natural language commands?
4. What are the key components of the action sequence generation process?

## Summary

In this lesson, you have implemented the cognitive planning component of the VLA system. You learned how to:
- Integrate LLMs for natural language understanding and task decomposition
- Create structured action sequences from natural language commands
- Handle spatial relationships and object manipulation planning
- Validate action sequences for feasibility and safety

The cognitive planning system serves as the brain of the VLA pipeline, transforming high-level human instructions into structured robotic actions. In the next lesson, you will implement the visual perception component that will provide the spatial context needed for accurate command interpretation and execution.