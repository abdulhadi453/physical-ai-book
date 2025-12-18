# Data Model: Module 4 - Vision-Language-Action (VLA) Systems

**Feature**: Module 4 - Vision-Language-Action (VLA) Systems
**Date**: 2025-12-17
**Status**: Complete

## Entity: VoiceCommand
**Description**: Natural language input from user that specifies desired robot behavior or task to be performed

**Fields**:
- `id`: String (unique identifier for the command)
- `text`: String (the transcribed text from speech input)
- `timestamp`: DateTime (when the command was received)
- `confidence`: Float (confidence score from speech recognition, 0.0-1.0)
- `user_id`: String (identifier for the user issuing the command)
- `language`: String (language code of the spoken command)

**Validation Rules**:
- `text` must not be empty
- `confidence` must be between 0.0 and 1.0
- `timestamp` must be in the past or present

## Entity: ProcessedIntent
**Description**: Structured representation of user intent after speech-to-text and LLM processing, containing actionable elements

**Fields**:
- `id`: String (unique identifier for the processed intent)
- `original_command_id`: String (reference to the original VoiceCommand)
- `intent_type`: Enum (NAVIGATION, MANIPULATION, INSPECTION, COMPLEX_TASK)
- `action_sequence`: Array[ActionStep] (ordered list of actions to execute)
- `context_objects`: Array[ObjectReference] (objects referenced in the command)
- `spatial_constraints`: Object (spatial relationships mentioned in the command)
- `priority`: Enum (HIGH, MEDIUM, LOW)
- `created_at`: DateTime (when the intent was processed)

**Validation Rules**:
- `action_sequence` must contain at least one action step
- `intent_type` must be one of the defined enum values
- All action steps must have valid parameters

## Entity: ActionStep
**Description**: Individual action within an action sequence that can be executed by the robot

**Fields**:
- `id`: String (unique identifier for the action step)
- `action_type`: Enum (NAVIGATE_TO, GRASP_OBJECT, RELEASE_OBJECT, LOOK_AT, FOLLOW_PATH, WAIT, REPORT_STATUS)
- `parameters`: Object (parameters specific to the action type)
- `timeout`: Integer (maximum time in seconds to complete the action)
- `required_objects`: Array[String] (object IDs required for this action)
- `preconditions`: Array[Condition] (conditions that must be met before execution)
- `expected_outcomes`: Array[Condition] (expected results after execution)

**Validation Rules**:
- `action_type` must be one of the defined enum values
- `timeout` must be greater than 0
- Parameters must match the expected format for the action type

## Entity: PerceptionData
**Description**: Visual and sensor information used to contextualize voice commands and identify objects in the environment

**Fields**:
- `id`: String (unique identifier for the perception data)
- `timestamp`: DateTime (when the data was captured)
- `objects`: Array[DetectedObject] (list of objects detected in the scene)
- `camera_pose`: Pose (position and orientation of the camera)
- `field_of_view`: Object (camera field of view parameters)
- `image_data`: String (path to image data or encoded image)
- `confidence_threshold`: Float (minimum confidence for object detection)

**Validation Rules**:
- `timestamp` must be in the past or present
- `objects` array can be empty but must be an array
- `confidence_threshold` must be between 0.0 and 1.0

## Entity: DetectedObject
**Description**: An object detected in the robot's visual field

**Fields**:
- `id`: String (unique identifier for the detected object)
- `class_name`: String (type of object, e.g., "cube", "cup", "chair")
- `confidence`: Float (confidence score of the detection, 0.0-1.0)
- `bbox`: BoundingBox (2D bounding box in image coordinates)
- `position_3d`: Point3D (3D position relative to robot)
- `dimensions`: Object (3D dimensions of the object)
- `color`: String (dominant color of the object)
- `is_graspable`: Boolean (whether the object can be grasped)

**Validation Rules**:
- `confidence` must be between 0.0 and 1.0
- `class_name` must not be empty
- `bbox` must have valid coordinates

## Entity: ExecutionState
**Description**: Current status of command execution including progress, errors, and system readiness for next actions

**Fields**:
- `id`: String (unique identifier for the execution state)
- `current_action_id`: String (ID of currently executing action)
- `progress`: Float (percentage of completion, 0.0-1.0)
- `status`: Enum (PENDING, IN_PROGRESS, SUCCESS, FAILED, CANCELLED, PAUSED)
- `error_message`: String (error message if status is FAILED)
- `executed_actions`: Array[String] (IDs of completed actions)
- `remaining_actions`: Array[String] (IDs of actions yet to be executed)
- `start_time`: DateTime (when execution started)
- `last_update`: DateTime (when state was last updated)

**Validation Rules**:
- `progress` must be between 0.0 and 1.0
- `status` must be one of the defined enum values
- `start_time` must be before or equal to `last_update`

## Entity: BoundingBox
**Description**: 2D bounding box representing an object in image coordinates

**Fields**:
- `x_min`: Integer (minimum x coordinate)
- `y_min`: Integer (minimum y coordinate)
- `x_max`: Integer (maximum x coordinate)
- `y_max`: Integer (maximum y coordinate)

**Validation Rules**:
- `x_min` must be less than `x_max`
- `y_min` must be less than `y_max`
- All coordinates must be non-negative

## Entity: Point3D
**Description**: 3D point representing a position in space

**Fields**:
- `x`: Float (x coordinate)
- `y`: Float (y coordinate)
- `z`: Float (z coordinate)

**Validation Rules**:
- All coordinates must be finite numbers
- Coordinates should be within robot's operational space

## Entity: Pose
**Description**: Position and orientation in 3D space

**Fields**:
- `position`: Point3D (position in 3D space)
- `orientation`: Object (quaternion or euler angles representing orientation)

**Validation Rules**:
- `position` must be a valid Point3D
- `orientation` must represent a valid rotation

## Relationships

- `VoiceCommand` 1 → * `ProcessedIntent` (one command can result in one processed intent)
- `ProcessedIntent` 1 → * `ActionStep` (one intent contains multiple action steps)
- `PerceptionData` 1 → * `DetectedObject` (one perception data contains multiple detected objects)
- `ExecutionState` 1 → 1 `ActionStep` (current action being executed)
- `ProcessedIntent` * → * `DetectedObject` (intent references objects for context)

## State Transitions

### ExecutionState Status Transitions:
- PENDING → IN_PROGRESS (when execution starts)
- IN_PROGRESS → SUCCESS (when action completes successfully)
- IN_PROGRESS → FAILED (when action fails)
- IN_PROGRESS → CANCELLED (when user cancels)
- IN_PROGRESS → PAUSED (when execution is paused)
- PAUSED → IN_PROGRESS (when execution resumes)
- PAUSED → CANCELLED (when paused execution is cancelled)