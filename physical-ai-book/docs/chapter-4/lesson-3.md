# Lesson 4.1.3: Visual Perception Integration

## Overview

Welcome to Lesson 3 of the Vision-Language-Action (VLA) Systems module! In this lesson, you will implement the visual perception component of the VLA system that integrates computer vision techniques with the cognitive planning system to enable object detection, spatial reasoning, and visual context understanding. This component provides critical information that allows the VLA system to understand its environment and execute commands that reference specific objects or locations.

## Learning Objectives

By the end of this lesson, you will be able to:
1. Implement object detection using computer vision libraries
2. Integrate visual perception with the cognitive planning system
3. Perform 3D position estimation for object manipulation
4. Handle spatial relationships between detected objects
5. Create a perception pipeline for real-time processing

## Prerequisites

Before starting this lesson, ensure you have:
- Completed Lesson 1-2 (Voice Processing and Cognitive Planning)
- Installed computer vision dependencies (OpenCV, PyTorch, Ultralytics YOLO)
- Familiarized yourself with the VLA system architecture
- Understood basic concepts of object detection and computer vision
- Set up appropriate camera or simulation environment

## Visual Perception Architecture

The visual perception component follows this architecture:

```
Camera Input → Object Detection → 3D Position Estimation → Object Context → Cognitive Planner
```

Key considerations include:
- Real-time object detection and classification
- 3D position estimation relative to robot
- Spatial relationship computation between objects
- Integration with cognitive planning for command disambiguation

## Implementation Steps

### Step 1: Set Up Object Detection Module

First, let's create the core object detection module:

```python
# src/vla/vision/object_detector.py

import cv2
import numpy as np
import torch
from typing import List, Dict, Any, Optional, Tuple
from src.vla.models.detected_object import DetectedObject, Point3D, BoundingBox
import logging

logger = logging.getLogger(__name__)

class ObjectDetector:
    """
    Object detection module using YOLO or similar computer vision models.
    """

    def __init__(self, model_path: Optional[str] = None, confidence_threshold: float = 0.5):
        """
        Initialize object detector.

        Args:
            model_path: Path to pre-trained model (uses default YOLOv8n if None)
            confidence_threshold: Minimum confidence for detection
        """
        self.confidence_threshold = confidence_threshold
        self.model = self._load_model(model_path)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # COCO dataset class names (first 80 classes)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
            'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
            'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def _load_model(self, model_path: Optional[str]):
        """
        Load the object detection model.

        Args:
            model_path: Path to model file (None for default)

        Returns:
            Loaded model object
        """
        try:
            # Using YOLOv8 as an example (you might need to install ultralytics)
            from ultralytics import YOLO

            if model_path:
                model = YOLO(model_path)
            else:
                # Use default YOLOv8n model
                model = YOLO('yolov8n.pt')

            # Move model to appropriate device
            model.to(self.device)
            logger.info(f"Object detection model loaded on {self.device}")
            return model
        except ImportError:
            logger.warning("Ultralytics not available, using mock detector")
            return None
        except Exception as e:
            logger.error(f"Failed to load object detection model: {e}")
            raise

    def detect_objects(self, image: np.ndarray) -> List[DetectedObject]:
        """
        Detect objects in an image.

        Args:
            image: Input image as numpy array (BGR format from OpenCV)

        Returns:
            List of DetectedObject instances
        """
        if self.model is None:
            # Mock implementation for testing
            return self._mock_detection(image)

        try:
            # Perform detection
            results = self.model(image, conf=self.confidence_threshold)

            detected_objects = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Extract bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])

                        # Create bounding box
                        bbox = BoundingBox(
                            x_min=int(x1),
                            y_min=int(y1),
                            x_max=int(x2),
                            y_max=int(y2)
                        )

                        # Get class name
                        class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"unknown_{class_id}"

                        # Create DetectedObject instance
                        detected_obj = DetectedObject(
                            id=f"obj_{len(detected_objects)}",
                            class_name=class_name,
                            confidence=confidence,
                            bbox=bbox,
                            position_3d=Point3D(x=0.0, y=0.0, z=0.0),  # Will be updated with 3D estimation
                            dimensions=None,  # Will be estimated separately
                            color=self._estimate_color(image, bbox),
                            is_graspable=self._is_graspable(class_name)
                        )

                        detected_objects.append(detected_obj)

            return detected_objects

        except Exception as e:
            logger.error(f"Object detection failed: {e}")
            return []

    def _estimate_color(self, image: np.ndarray, bbox: BoundingBox) -> str:
        """
        Estimate the dominant color in a bounding box region.

        Args:
            image: Input image
            bbox: Bounding box coordinates

        Returns:
            Estimated color name
        """
        try:
            # Extract region of interest
            roi = image[bbox.y_min:bbox.y_max, bbox.x_min:bbox.x_max]

            if roi.size == 0:
                return "unknown"

            # Convert BGR to HSV for better color analysis
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Calculate histogram
            hist = cv2.calcHist([hsv], [0, 1], None, [50, 50], [0, 180, 0, 256])
            hist = cv2.normalize(hist, hist).flatten()

            # Find dominant hue
            dominant_hue = np.argmax(hist) * (180 / 50)  # Scale back to hue range

            # Map hue to color names
            if 0 <= dominant_hue < 15 or dominant_hue >= 165:
                return "red"
            elif 15 <= dominant_hue < 45:
                return "orange"
            elif 45 <= dominant_hue < 75:
                return "yellow"
            elif 75 <= dominant_hue < 105:
                return "green"
            elif 105 <= dominant_hue < 135:
                return "blue"
            elif 135 <= dominant_hue < 165:
                return "purple"
            else:
                return "unknown"

        except Exception as e:
            logger.warning(f"Color estimation failed: {e}")
            return "unknown"

    def _is_graspable(self, class_name: str) -> bool:
        """
        Determine if an object is likely graspable based on class name.

        Args:
            class_name: Object class name

        Returns:
            True if object is likely graspable
        """
        graspable_classes = [
            'bottle', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'orange', 'hot dog', 'pizza', 'donut', 'cake', 'book', 'cell phone',
            'teddy bear', 'scissors', 'remote', 'mouse', 'keyboard', 'laptop'
        ]
        return class_name in graspable_classes

    def _mock_detection(self, image: np.ndarray) -> List[DetectedObject]:
        """
        Mock detection for testing when model is not available.

        Args:
            image: Input image

        Returns:
            List of mock DetectedObject instances
        """
        logger.warning("Using mock object detection - install ultralytics for real detection")

        # Create some mock objects for testing
        mock_objects = [
            DetectedObject(
                id="mock_cube_1",
                class_name="cube",
                confidence=0.85,
                bbox=BoundingBox(x_min=100, y_min=100, x_max=150, y_max=150),
                position_3d=Point3D(x=1.0, y=0.5, z=0.0),
                dimensions=None,
                color="red",
                is_graspable=True
            ),
            DetectedObject(
                id="mock_sphere_1",
                class_name="sphere",
                confidence=0.78,
                bbox=BoundingBox(x_min=200, y_min=120, x_max=250, y_max=170),
                position_3d=Point3D(x=1.2, y=0.8, z=0.0),
                dimensions=None,
                color="blue",
                is_graspable=True
            )
        ]
        return mock_objects
```

### Step 2: Implement Perception Pipeline

Now let's create the perception pipeline that integrates detection with 3D position estimation:

```python
# src/vla/vision/perception_pipeline.py

import cv2
import numpy as np
from typing import List, Dict, Any, Optional
from src.vla.models.detected_object import DetectedObject
from src.vla.models.perception_data import PerceptionData
from src.vla.vision.object_detector import ObjectDetector
import logging

logger = logging.getLogger(__name__)

class PerceptionPipeline:
    """
    Complete perception pipeline that processes camera input and provides
    object detection results with 3D position estimation.
    """

    def __init__(self,
                 object_detector: ObjectDetector,
                 camera_matrix: Optional[np.ndarray] = None,
                 distortion_coeffs: Optional[np.ndarray] = None):
        """
        Initialize perception pipeline.

        Args:
            object_detector: Object detection module
            camera_matrix: Camera intrinsic matrix (if None, uses mock values)
            distortion_coeffs: Camera distortion coefficients (if None, assumes no distortion)
        """
        self.object_detector = object_detector
        self.camera_matrix = camera_matrix or self._get_mock_camera_matrix()
        self.distortion_coeffs = distortion_coeffs or np.zeros((4, 1))

    def _get_mock_camera_matrix(self) -> np.ndarray:
        """
        Create a mock camera matrix for testing.

        Returns:
            Mock camera intrinsic matrix
        """
        # Standard camera matrix for 640x480 image with 60 degree FOV
        return np.array([
            [640, 0, 320],  # fx, 0, cx
            [0, 640, 240],  # 0, fy, cy
            [0, 0, 1]       # 0, 0, 1
        ])

    def process_frame(self,
                     image: np.ndarray,
                     robot_pose: Optional[Dict[str, Any]] = None) -> PerceptionData:
        """
        Process a single camera frame to detect objects and estimate 3D positions.

        Args:
            image: Input camera image (BGR format)
            robot_pose: Current robot pose (position and orientation)

        Returns:
            PerceptionData with detected objects and metadata
        """
        try:
            # Detect objects in the image
            detected_objects = self.object_detector.detect_objects(image)

            # Estimate 3D positions for detected objects
            objects_with_3d = self._estimate_3d_positions(detected_objects, image, robot_pose)

            # Create PerceptionData instance
            perception_data = PerceptionData(
                id=f"perception_{int(time.time())}",
                timestamp=time.time(),
                objects=objects_with_3d,
                camera_pose=robot_pose,  # Simplified - in reality would be separate
                field_of_view={"horizontal": 60, "vertical": 45},  # degrees
                image_data="",  # In practice, this might be a path or encoded data
                confidence_threshold=self.object_detector.confidence_threshold
            )

            return perception_data

        except Exception as e:
            logger.error(f"Perception pipeline processing failed: {e}")
            # Return empty perception data on failure
            return PerceptionData(
                id=f"perception_{int(time.time())}",
                timestamp=time.time(),
                objects=[],
                camera_pose=robot_pose,
                field_of_view={"horizontal": 60, "vertical": 45},
                image_data="",
                confidence_threshold=self.object_detector.confidence_threshold
            )

    def _estimate_3d_positions(self,
                             detected_objects: List[DetectedObject],
                             image: np.ndarray,
                             robot_pose: Optional[Dict[str, Any]] = None) -> List[DetectedObject]:
        """
        Estimate 3D positions of detected objects using camera parameters.

        Args:
            detected_objects: List of 2D detected objects
            image: Original image for size reference
            robot_pose: Robot's current pose in the environment

        Returns:
            List of DetectedObject with updated 3D positions
        """
        # For simplicity, we'll use a basic depth estimation based on object size
        # In a real system, you'd use stereo vision, depth sensors, or other methods

        processed_objects = []

        for obj in detected_objects:
            # Create a copy of the object to modify
            updated_obj = obj

            # Estimate depth based on object size in image
            # Larger objects are likely closer, smaller objects are likely farther
            image_height, image_width = image.shape[:2]
            bbox_width = obj.bbox.x_max - obj.bbox.x_min
            bbox_height = obj.bbox.y_max - obj.bbox.y_min

            # Normalize by image dimensions
            norm_width = bbox_width / image_width
            norm_height = bbox_height / image_height

            # Simple depth estimation (in meters) - this is a simplification
            # In reality, you'd use proper geometric calculations with camera parameters
            depth = max(0.1, 2.0 - (norm_width + norm_height) * 2.0)  # 0.1m to 2.0m range

            # Calculate 3D position based on 2D position and estimated depth
            center_x = (obj.bbox.x_min + obj.bbox.x_max) / 2.0
            center_y = (obj.bbox.y_min + obj.bbox.y_max) / 2.0

            # Convert 2D image coordinates to 3D world coordinates
            # This is a simplified transformation
            x_3d = (center_x - image_width / 2.0) * depth / (image_width / 2.0)  # Scale by depth
            y_3d = (center_y - image_height / 2.0) * depth / (image_height / 2.0)  # Scale by depth

            # Apply robot pose transformation if available
            if robot_pose:
                # This is a simplified transformation
                # In practice, you'd use proper coordinate frame transformations
                robot_x = robot_pose.get("x", 0.0)
                robot_y = robot_pose.get("y", 0.0)
                robot_theta = robot_pose.get("theta", 0.0)  # rotation in radians

                # Rotate and translate the object position relative to robot
                cos_theta = np.cos(robot_theta)
                sin_theta = np.sin(robot_theta)

                world_x = robot_x + x_3d * cos_theta - y_3d * sin_theta
                world_y = robot_y + x_3d * sin_theta + y_3d * cos_theta
            else:
                world_x, world_y = x_3d, y_3d

            # Update the object with 3D position
            updated_obj.position_3d = Point3D(x=world_x, y=world_y, z=depth)

            processed_objects.append(updated_obj)

        return processed_objects

    def get_spatial_relationships(self,
                                objects: List[DetectedObject],
                                reference_object_id: str) -> Dict[str, List[DetectedObject]]:
        """
        Compute spatial relationships between objects.

        Args:
            objects: List of detected objects
            reference_object_id: ID of reference object for spatial relationships

        Returns:
            Dictionary mapping spatial relationships to lists of objects
        """
        relationships = {
            "left_of": [],
            "right_of": [],
            "in_front_of": [],
            "behind": [],
            "above": [],
            "below": [],
            "near": [],
            "far_from": []
        }

        # Find reference object
        ref_obj = None
        for obj in objects:
            if obj.id == reference_object_id:
                ref_obj = obj
                break

        if not ref_obj:
            logger.warning(f"Reference object {reference_object_id} not found")
            return relationships

        # Compute relationships with all other objects
        for obj in objects:
            if obj.id == reference_object_id:
                continue

            # Calculate relative position
            dx = obj.position_3d.x - ref_obj.position_3d.x
            dy = obj.position_3d.y - ref_obj.position_3d.y
            dz = obj.position_3d.z - ref_obj.position_3d.z

            # Calculate distance
            distance = np.sqrt(dx**2 + dy**2 + dz**2)

            # Determine spatial relationships
            if dx < -0.1:  # More than 10cm to the left
                relationships["left_of"].append(obj)
            elif dx > 0.1:  # More than 10cm to the right
                relationships["right_of"].append(obj)

            if dy < -0.1:  # More than 10cm in front
                relationships["in_front_of"].append(obj)
            elif dy > 0.1:  # More than 10cm behind
                relationships["behind"].append(obj)

            if dz > 0.1:  # More than 10cm above
                relationships["above"].append(obj)
            elif dz < -0.1:  # More than 10cm below
                relationships["below"].append(obj)

            if distance < 0.5:  # Within 50cm
                relationships["near"].append(obj)
            else:  # Farther than 50cm
                relationships["far_from"].append(obj)

        return relationships
```

### Step 3: Create Vision Processor

Now let's create the vision processor that integrates with the cognitive planning system:

```python
# src/vla/vision/vision_processor.py

import cv2
import numpy as np
from typing import List, Dict, Any, Optional
from src.vla.models.detected_object import DetectedObject
from src.vla.models.perception_data import PerceptionData
from src.vla.vision.perception_pipeline import PerceptionPipeline
from src.vla.vision.object_detector import ObjectDetector
import logging

logger = logging.getLogger(__name__)

class VisionProcessor:
    """
    High-level vision processing that integrates with the VLA system.
    """

    def __init__(self, perception_pipeline: PerceptionPipeline):
        """
        Initialize vision processor.

        Args:
            perception_pipeline: Perception pipeline for object detection
        """
        self.perception_pipeline = perception_pipeline
        self.last_perception_data = None

    def process_camera_input(self,
                           image: np.ndarray,
                           robot_pose: Optional[Dict[str, Any]] = None) -> PerceptionData:
        """
        Process camera input to detect objects and provide perception data.

        Args:
            image: Camera image input
            robot_pose: Current robot pose information

        Returns:
            PerceptionData with detected objects and relationships
        """
        # Process the frame through the perception pipeline
        perception_data = self.perception_pipeline.process_frame(image, robot_pose)

        # Store for later use
        self.last_perception_data = perception_data

        return perception_data

    def find_object_by_description(self,
                                 description: str,
                                 perception_data: PerceptionData) -> Optional[DetectedObject]:
        """
        Find an object that matches a description in the current perception data.

        Args:
            description: Natural language description of the object
            perception_data: Current perception data to search

        Returns:
            DetectedObject that matches the description, or None if not found
        """
        description_lower = description.lower()

        # Look for matches based on various attributes
        for obj in perception_data.objects:
            obj_desc = f"{obj.color} {obj.class_name}".lower()

            # Direct match
            if description_lower in obj_desc:
                return obj

            # Color match
            if obj.color.lower() in description_lower:
                # If looking for a specific color, return first object with that color
                return obj

            # Class name match
            if obj.class_name.lower() in description_lower:
                return obj

        # If no direct matches, try more sophisticated matching
        for obj in perception_data.objects:
            # Check if this might be what the user meant
            if self._is_possible_match(description_lower, obj):
                return obj

        return None

    def _is_possible_match(self, description: str, obj: DetectedObject) -> bool:
        """
        Determine if an object is a possible match for a description.

        Args:
            description: Natural language description
            obj: Detected object to check

        Returns:
            True if object might match the description
        """
        # Simple heuristics for possible matches
        if "red" in description and obj.color == "red":
            return True
        if "blue" in description and obj.color == "blue":
            return True
        if "green" in description and obj.color == "green":
            return True
        if "small" in description and self._is_small_object(obj):
            return True
        if "large" in description and self._is_large_object(obj):
            return True

        # Class-based matching
        if "box" in description and "box" in obj.class_name:
            return True
        if "cube" in description and "cube" in obj.class_name:
            return True
        if "cylinder" in description and "bottle" in obj.class_name:
            return True  # Bottles are roughly cylindrical

        return False

    def _is_small_object(self, obj: DetectedObject) -> bool:
        """
        Determine if an object is small based on its bounding box size.

        Args:
            obj: Detected object

        Returns:
            True if object is small
        """
        if obj.bbox:
            area = (obj.bbox.x_max - obj.bbox.x_min) * (obj.bbox.y_max - obj.bbox.y_min)
            # Consider objects with area < 1000 pixels as small (adjust as needed)
            return area < 1000
        return False

    def _is_large_object(self, obj: DetectedObject) -> bool:
        """
        Determine if an object is large based on its bounding box size.

        Args:
            obj: Detected object

        Returns:
            True if object is large
        """
        if obj.bbox:
            area = (obj.bbox.x_max - obj.bbox.x_min) * (obj.bbox.y_max - obj.bbox.y_min)
            # Consider objects with area > 5000 pixels as large (adjust as needed)
            return area > 5000
        return False

    def get_object_for_command(self,
                             command: str,
                             perception_data: PerceptionData) -> Optional[DetectedObject]:
        """
        Identify the object referenced in a command based on current perception.

        Args:
            command: Natural language command
            perception_data: Current perception data

        Returns:
            DetectedObject referenced in the command, or None
        """
        # Extract object reference from command
        # This is a simplified approach - in practice, you'd use more sophisticated NLP
        command_lower = command.lower()

        # Look for color + class patterns (e.g., "red cube", "blue bottle")
        for obj in perception_data.objects:
            if obj.color and obj.class_name:
                color_class_pattern = f"{obj.color} {obj.class_name}".lower()
                if color_class_pattern in command_lower:
                    return obj

        # Look for class names only
        for obj in perception_data.objects:
            if obj.class_name.lower() in command_lower:
                return obj

        # Look for color adjectives
        color_keywords = ["red", "blue", "green", "yellow", "orange", "purple", "pink", "brown", "black", "white"]
        for color in color_keywords:
            if color in command_lower:
                # Return first object with matching color
                for obj in perception_data.objects:
                    if obj.color.lower() == color:
                        return obj

        # If no specific object found, return the closest one
        if perception_data.objects:
            # Return object with highest confidence or closest to center
            return max(perception_data.objects, key=lambda o: o.confidence)

        return None

    def visualize_detections(self,
                           image: np.ndarray,
                           perception_data: PerceptionData) -> np.ndarray:
        """
        Draw bounding boxes and labels on the image for visualization.

        Args:
            image: Original image
            perception_data: Perception data with detected objects

        Returns:
            Image with visualized detections
        """
        # Create a copy of the image to draw on
        vis_image = image.copy()

        for obj in perception_data.objects:
            if obj.bbox:
                # Draw bounding box
                cv2.rectangle(
                    vis_image,
                    (obj.bbox.x_min, obj.bbox.y_min),
                    (obj.bbox.x_max, obj.bbox.y_max),
                    (0, 255, 0),  # Green color
                    2
                )

                # Draw label
                label = f"{obj.class_name} ({obj.color}) {obj.confidence:.2f}"
                cv2.putText(
                    vis_image,
                    label,
                    (obj.bbox.x_min, obj.bbox.y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

        return vis_image
```

### Step 4: Integrate with VLA System

Now let's create a module that integrates the vision system with the cognitive planning:

```python
# src/vla/integration/vla_vision_integrator.py

from typing import Dict, Any, Optional
from src.vla.vision.vision_processor import VisionProcessor
from src.vla.models.detected_object import DetectedObject
from src.vla.models.perception_data import PerceptionData
from src.vla.llm.cognitive_planner import ProcessedIntent
import logging

logger = logging.getLogger(__name__)

class VLAVisionIntegrator:
    """
    Integrates visual perception with the broader VLA system.
    """

    def __init__(self, vision_processor: VisionProcessor):
        """
        Initialize the VLA vision integrator.

        Args:
            vision_processor: Vision processor instance
        """
        self.vision_processor = vision_processor

    def process_camera_frame(self,
                           image: np.ndarray,
                           robot_pose: Optional[Dict[str, Any]] = None) -> PerceptionData:
        """
        Process a camera frame through the vision system.

        Args:
            image: Camera image to process
            robot_pose: Current robot pose information

        Returns:
            PerceptionData with detected objects and metadata
        """
        logger.info("Processing camera frame through vision system")

        try:
            # Process the image through the vision pipeline
            perception_data = self.vision_processor.process_camera_input(image, robot_pose)

            logger.info(f"Detected {len(perception_data.objects)} objects in frame")
            return perception_data

        except Exception as e:
            logger.error(f"Camera frame processing failed: {e}")
            raise

    def enhance_intent_with_vision(self,
                                  intent: ProcessedIntent,
                                  perception_data: PerceptionData) -> ProcessedIntent:
        """
        Enhance a processed intent with visual context from perception data.

        Args:
            intent: Original processed intent
            perception_data: Current perception data

        Returns:
            Enhanced ProcessedIntent with visual context
        """
        logger.info(f"Enhancing intent with visual context for {len(perception_data.objects)} objects")

        # Update context objects with perception data
        intent.context_objects = perception_data.objects

        # Update spatial constraints based on detected objects
        if intent.spatial_constraints:
            # If the command mentioned spatial relationships, find corresponding objects
            for obj in perception_data.objects:
                # Example: if command said "left of red cube", find the red cube
                if intent.spatial_constraints.get("relative_position") == "left_of":
                    if obj.color == "red" and "cube" in obj.class_name.lower():
                        # Associate the object with the spatial constraint
                        intent.spatial_constraints["reference_object"] = obj

        return intent

    def resolve_command_ambiguity(self,
                                 command: str,
                                 perception_data: PerceptionData) -> Optional[DetectedObject]:
        """
        Resolve ambiguity in a command by using visual context.

        Args:
            command: Natural language command that may be ambiguous
            perception_data: Current perception data

        Returns:
            Resolved object reference, or None if still ambiguous
        """
        # Look for ambiguous references in the command
        ambiguous_indicators = ["that", "it", "thing", "there", "this"]
        command_lower = command.lower()

        if any(indicator in command_lower for indicator in ambiguous_indicators):
            # Try to disambiguate based on spatial context
            # For example, if command says "pick up that" and there's a red cube nearby
            for obj in perception_data.objects:
                if obj.confidence > 0.7:  # Only consider high-confidence detections
                    # Could implement more sophisticated disambiguation logic here
                    # based on spatial relationships, salience, etc.
                    return obj

        # If no ambiguity, return None
        return None

    def validate_action_feasibility(self,
                                   action_parameters: Dict[str, Any],
                                   perception_data: PerceptionData) -> Dict[str, Any]:
        """
        Validate if an action is feasible given current perception data.

        Args:
            action_parameters: Parameters for the action to validate
            perception_data: Current perception data

        Returns:
            Dictionary with validation results
        """
        validation_result = {
            "is_feasible": True,
            "issues": [],
            "suggestions": [],
            "confidence": 0.0
        }

        # Check if target object exists in environment
        target_object_id = action_parameters.get("object_id")
        if target_object_id:
            target_exists = any(obj.id == target_object_id for obj in perception_data.objects)
            if not target_exists:
                validation_result["is_feasible"] = False
                validation_result["issues"].append(f"Target object {target_object_id} not found in environment")

                # Try to find similar objects
                target_class = action_parameters.get("object_class", "")
                similar_objects = [
                    obj for obj in perception_data.objects
                    if target_class.lower() in obj.class_name.lower()
                ]

                if similar_objects:
                    validation_result["suggestions"].append(
                        f"Similar objects found: {[obj.id for obj in similar_objects]}"
                    )

        # Check navigation feasibility
        target_position = action_parameters.get("target_position")
        if target_position:
            # In a real system, you'd check navigation maps and obstacles
            # For now, just ensure position is reasonable
            if (abs(target_position.get("x", 0)) > 10 or
                abs(target_position.get("y", 0)) > 10 or
                abs(target_position.get("z", 0)) > 10):
                validation_result["issues"].append("Target position seems too far away")

        # Calculate overall confidence based on object detection confidence
        if perception_data.objects:
            avg_confidence = sum(obj.confidence for obj in perception_data.objects) / len(perception_data.objects)
            validation_result["confidence"] = avg_confidence

        return validation_result
```

## Practical Exercise

### Exercise 3.1: Visual Perception Integration

1. **Setup**: Create a test script that integrates the vision components with the cognitive planner.

2. **Implementation**:
   ```python
   # test_vision_integration.py
   import cv2
   import numpy as np
   import time
   from src.vla.vision.object_detector import ObjectDetector
   from src.vla.vision.perception_pipeline import PerceptionPipeline
   from src.vla.vision.vision_processor import VisionProcessor
   from src.vla.integration.vla_vision_integrator import VLAVisionIntegrator

   def main():
       # Initialize vision components
       detector = ObjectDetector(confidence_threshold=0.5)
       pipeline = PerceptionPipeline(detector)
       vision_processor = VisionProcessor(pipeline)
       vision_integrator = VLAVisionIntegrator(vision_processor)

       print("Vision processing system initialized")

       # Create a mock image for testing (in reality, this would come from a camera)
       # Create a simple test image with some shapes
       test_image = np.zeros((480, 640, 3), dtype=np.uint8)

       # Draw some shapes to simulate objects
       cv2.rectangle(test_image, (100, 100), (150, 150), (0, 0, 255), -1)  # Red square
       cv2.circle(test_image, (200, 200), 25, (255, 0, 0), -1)  # Blue circle
       cv2.rectangle(test_image, (300, 100), (350, 180), (0, 255, 0), -1)  # Green rectangle

       # Process the test image
       robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
       perception_data = vision_processor.process_camera_input(test_image, robot_pose)

       print(f"Detected {len(perception_data.objects)} objects:")
       for i, obj in enumerate(perception_data.objects):
           print(f"  {i+1}. {obj.class_name} ({obj.color}) at {obj.position_3d.x:.2f}, {obj.position_3d.y:.2f}, {obj.position_3d.z:.2f}")

       # Test object finding
       target_obj = vision_processor.find_object_by_description("red object", perception_data)
       if target_obj:
           print(f"Found target object: {target_obj.class_name} ({target_obj.color})")

       # Test command-based object selection
       command = "Pick up the red cube"
       command_obj = vision_processor.get_object_for_command(command, perception_data)
       if command_obj:
           print(f"Object for command '{command}': {command_obj.class_name} ({command_obj.color})")

       # Visualize the results
       vis_image = vision_processor.visualize_detections(test_image, perception_data)

       # Save the visualization
       cv2.imwrite("vision_test_output.png", vis_image)
       print("Visualization saved as vision_test_output.png")

   if __name__ == "__main__":
       main()
   ```

3. **Testing**: Run the test script to see how the vision system detects objects, estimates their 3D positions, and relates them to natural language commands.

## Key Concepts

### 3D Position Estimation
- **Monocular Depth Estimation**: Using object size and context to estimate depth
- **Stereo Vision**: Using multiple cameras to calculate depth
- **Depth Sensors**: Using LiDAR or RGB-D cameras for direct depth measurement
- **Coordinate Transformations**: Converting between camera frame and world frame

### Object Recognition and Context
- **Class-based Recognition**: Identifying objects by their category
- **Attribute-based Recognition**: Identifying objects by color, size, shape
- **Contextual Recognition**: Using spatial relationships and scene context

### Real-time Processing
- **Frame Rate Optimization**: Processing images at sufficient frame rates for real-time interaction
- **Detection Confidence**: Balancing accuracy with processing speed
- **Resource Management**: Efficient use of computational resources

## Common Challenges and Solutions

### Challenge 1: Depth Estimation
**Problem**: Estimating 3D positions from 2D images is inherently ambiguous.
**Solution**: Use multiple sensors (stereo, depth cameras) or learn depth from context.

### Challenge 2: Object Occlusion
**Problem**: Objects may be partially hidden or occluded.
**Solution**: Implement temporal consistency tracking and prediction.

### Challenge 3: Lighting Conditions
**Problem**: Performance varies with lighting conditions.
**Solution**: Use illumination-invariant features and preprocessing.

## Assessment Questions

1. How does the system estimate 3D positions from 2D image coordinates?
2. What factors influence the "graspable" property of detected objects?
3. How does the vision processor identify objects referenced in natural language commands?
4. What are the key components of the perception pipeline?

## Summary

In this lesson, you have implemented the visual perception component of the VLA system. You learned how to:
- Implement object detection using computer vision libraries
- Integrate visual perception with the cognitive planning system
- Perform 3D position estimation for object manipulation
- Handle spatial relationships between detected objects
- Create a perception pipeline for real-time processing

The visual perception system provides critical environmental context that enables the VLA system to understand its surroundings and execute commands that reference specific objects or locations. In the next lesson, you will implement the ROS 2 action execution system that will carry out the planned actions in simulation.