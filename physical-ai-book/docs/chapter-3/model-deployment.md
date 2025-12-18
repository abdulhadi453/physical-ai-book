# AI Model Deployment in Isaac Sim Environment

## Overview

This document provides comprehensive instructions for deploying AI models within the NVIDIA Isaac Sim environment for Module 3. The focus is on deploying perception models for object detection, semantic segmentation, and depth estimation that can run efficiently in the simulation environment and integrate with the ROS 2 ecosystem.

## Introduction to AI Model Deployment in Isaac

### What is AI Model Deployment in Isaac?
AI model deployment in Isaac involves packaging, optimizing, and integrating deep learning models for use within the Isaac Sim environment and ROS 2 ecosystem. This includes:

- Model optimization for real-time inference
- Integration with Isaac ROS perception packages
- GPU acceleration using TensorRT
- Real-time performance considerations

### Key Components
1. **Model Optimization**: Converting models to efficient formats
2. **TensorRT Integration**: NVIDIA's inference optimizer
3. **ROS 2 Integration**: Connecting models to ROS 2 topics
4. **Performance Monitoring**: Ensuring real-time capabilities

## Prerequisites

Before deploying AI models, ensure you have:
- Completed Isaac Sim and Isaac ROS setup
- NVIDIA GPU with TensorRT support
- PyTorch and ONNX installed in your environment
- Basic understanding of deep learning models

## Model Preparation and Optimization

### 1. Installing Model Deployment Dependencies

```bash
# Install PyTorch and related dependencies
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install ONNX and ONNX Runtime
pip3 install onnx onnxruntime onnxruntime-gpu

# Install TensorRT (if not already installed with Isaac Sim)
pip3 install tensorrt

# Install additional tools
pip3 install numpy opencv-python matplotlib
```

### 2. Creating Model Repository Structure

```bash
mkdir -p ~/isaac_sim_shared/models
mkdir -p ~/isaac_sim_shared/models/yolo
mkdir -p ~/isaac_sim_shared/models/segmentation
mkdir -p ~/isaac_sim_shared/models/depth
mkdir -p ~/isaac_sim_shared/models/trt_cache
```

### 3. Downloading Pre-trained Models

```bash
# Create model download script
cat > ~/download_models.sh << 'EOF'
#!/bin/bash

# Download pre-trained models for Isaac Sim

# Create models directory
mkdir -p ~/isaac_sim_shared/models

# Download YOLOv5 model
cd ~/isaac_sim_shared/models
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip3 install -r requirements.txt
wget -O yolov5s.pt https://github.com/ultralytics/yolov5/releases/download/v6.2/yolov5s.pt

# Download MiDaS for depth estimation
cd ~/isaac_sim_shared/models
git clone https://github.com/isl-org/MiDaS.git
cd MiDaS
pip3 install -r requirements.txt
mkdir -p model_weights
cd model_weights
wget https://github.com/isl-org/MiDaS/releases/download/v3_1/dpt_large_384.pt

# Download segmentation model (using torchvision's FCN-ResNet101)
cd ~/isaac_sim_shared/models
python3 -c "
import torch
model = torch.hub.load('pytorch/vision:v0.10.0', 'fcn_resnet101', pretrained=True)
torch.save(model.state_dict(), 'fcn_resnet101.pth')
print('Segmentation model downloaded')
"
EOF

# Make executable and run
chmod +x ~/download_models.sh
~/download_models.sh
```

## Model Conversion for Isaac Sim

### 1. Converting YOLOv5 to ONNX Format

```bash
cat > ~/isaac_sim_shared/scripts/convert_yolo_to_onnx.py << 'EOF'
import torch
import numpy as np
import onnx
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import check_img_size
from yolov5.utils.torch_utils import select_device
import os

def convert_yolo_to_onnx():
    """Convert YOLOv5 model to ONNX format for Isaac Sim"""

    # Model parameters
    weights = '/workspace/shared_dir/models/yolov5/yolov5s.pt'  # Path to PyTorch model
    img_size = [640, 640]  # Input image size
    batch_size = 1  # Batch size for ONNX model
    device = 'cpu'  # Use CPU for conversion, will run on GPU in Isaac Sim

    print(f"Loading model from: {weights}")

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=False, data=None, fp16=False)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine

    # Check image size
    img_size = check_img_size(img_size, s=stride)  # Verify img_size are gs-multiples

    # Input
    im = torch.zeros(batch_size, 3, *img_size).to(device)  # Create dummy input

    # Update model
    model.model.float()
    model.warmup(im)  # Warmup

    # Export
    try:
        # Export to ONNX
        torch.onnx.export(
            model,
            im,
            '/workspace/shared_dir/models/yolo/yolov5s.onnx',
            verbose=False,
            opset_version=12,
            input_names=['images'],
            output_names=['output'],
            dynamic_axes={
                'images': {0: 'batch', 2: 'height', 3: 'width'},  # Variable input dimensions
                'output': {0: 'batch', 1: 'num_detections'}  # Variable output dimensions
            } if False else None  # Set to None for fixed dimensions
        )

        # Checks
        model_onnx = onnx.load('/workspace/shared_dir/models/yolo/yolov5s.onnx')
        onnx.checker.check_model(model_onnx)  # Check model is well formed
        print(f"ONNX model saved to: /workspace/shared_dir/models/yolo/yolov5s.onnx")

        # Simplify ONNX model (optional)
        try:
            import onnxsim
            model_onnx, check = onnxsim.simplify(model_onnx)
            assert check, "Simplified ONNX model could not be validated"

            onnx.save(model_onnx, '/workspace/shared_dir/models/yolo/yolov5s_simplified.onnx')
            print("Simplified ONNX model saved")
        except ImportError:
            print("ONNX simplification skipped (install onnxsim for better performance)")

    except Exception as e:
        print(f"Export failure: {e}")

if __name__ == "__main__":
    convert_yolo_to_onnx()
EOF
```

### 2. Converting Segmentation Model to ONNX

```bash
cat > ~/isaac_sim_shared/scripts/convert_segmentation_to_onnx.py << 'EOF'
import torch
import torchvision.models as models
import onnx
import os

def convert_segmentation_to_onnx():
    """Convert FCN-ResNet101 segmentation model to ONNX format"""

    # Load pre-trained segmentation model
    model = models.segmentation.fcn_resnet101(pretrained=True)
    model.eval()  # Set to evaluation mode

    # Create dummy input (batch_size=1, channels=3, height=480, width=640)
    dummy_input = torch.randn(1, 3, 480, 640, requires_grad=True)

    # Export to ONNX
    onnx_filename = '/workspace/shared_dir/models/segmentation/fcn_resnet101.onnx'

    torch.onnx.export(
        model,  # Model being exported
        dummy_input,  # Model input (or a tuple for multiple inputs)
        onnx_filename,  # Where to save the model (can be a file or file-like object)
        export_params=True,  # Store the trained parameter weights
        opset_version=11,  # The ONNX version to export the model to
        do_constant_folding=True,  # Whether to execute constant folding for optimization
        input_names=['input'],  # Model's input names
        output_names=['output'],  # Model's output names
        dynamic_axes={
            'input': {0: 'batch_size', 2: 'height', 3: 'width'},
            'output': {0: 'batch_size', 2: 'height', 3: 'width'}
        }
    )

    # Verify the model
    onnx_model = onnx.load(onnx_filename)
    onnx.checker.check_model(onnx_model)

    print(f"Segmentation ONNX model saved to: {onnx_filename}")

if __name__ == "__main__":
    convert_segmentation_to_onnx()
EOF
```

### 3. Converting Depth Estimation Model to ONNX

```bash
cat > ~/isaac_sim_shared/scripts/convert_depth_to_onnx.py << 'EOF'
import torch
import torchvision.transforms as transforms
from torchvision import models
import onnx
import os

def convert_depth_to_onnx():
    """Convert MiDaS depth estimation model to ONNX format"""

    # Note: MiDaS conversion is more complex, using a simplified approach
    # For actual MiDaS, we'll use the original repository's conversion

    print("Creating simplified depth estimation ONNX model...")

    # For now, we'll create a placeholder conversion
    # In practice, you would use the MiDaS repository's specific conversion

    # Load a simple model as placeholder
    model = models.resnet18(pretrained=True)
    # Modify the final layer for depth estimation
    model.fc = torch.nn.Linear(model.fc.in_features, 1)  # Single output for depth

    model.eval()

    # Create dummy input
    dummy_input = torch.randn(1, 3, 384, 384)

    # Export to ONNX
    onnx_filename = '/workspace/shared_dir/models/depth/midas.onnx'

    torch.onnx.export(
        model,
        dummy_input,
        onnx_filename,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size', 2: 'height', 3: 'width'},
            'output': {0: 'batch_size', 2: 'height', 3: 'width'}
        }
    )

    # Verify the model
    onnx_model = onnx.load(onnx_filename)
    onnx.checker.check_model(onnx_model)

    print(f"Depth estimation ONNX model saved to: {onnx_filename}")
    print("Note: For production use, implement proper MiDaS conversion")

if __name__ == "__main__":
    convert_depth_to_onnx()
EOF
```

## TensorRT Optimization

### 1. Creating TensorRT Optimization Script

```bash
cat > ~/isaac_sim_shared/scripts/optimize_with_tensorrt.py << 'EOF'
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import onnx
import os
from typing import List, Tuple

class TensorRTOptimizer:
    def __init__(self):
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.builder = trt.Builder(self.logger)
        self.network = None
        self.config = None

    def optimize_onnx_model(self, onnx_path: str, trt_path: str, precision: str = "fp16") -> bool:
        """
        Optimize ONNX model using TensorRT

        Args:
            onnx_path: Path to ONNX model
            trt_path: Path to save optimized TensorRT model
            precision: Precision mode ("fp32", "fp16", or "int8")
        """

        # Check if ONNX file exists
        if not os.path.exists(onnx_path):
            print(f"ONNX file does not exist: {onnx_path}")
            return False

        # Parse ONNX model
        explicit_batch = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        self.network = self.builder.create_network(explicit_batch)
        parser = trt.OnnxParser(self.network, self.logger)

        with open(onnx_path, 'rb') as model:
            if not parser.parse(model.read()):
                print("ERROR: Failed to parse the ONNX file.")
                for error in range(parser.num_errors):
                    print(parser.get_error(error))
                return False

        # Configure optimization
        self.config = self.builder.create_builder_config()

        # Set precision
        if precision == "fp16":
            if self.builder.platform_has_fast_fp16:
                self.config.set_flag(trt.BuilderFlag.FP16)
                print("Using FP16 precision")
            else:
                print("FP16 not supported on this platform, using FP32")

        elif precision == "int8":
            if self.builder.platform_has_fast_int8:
                self.config.set_flag(trt.BuilderFlag.INT8)
                print("Using INT8 precision")
            else:
                print("INT8 not supported on this platform, using FP32")

        # Set memory limit
        self.config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB

        # Build engine
        try:
            serialized_engine = self.builder.build_serialized_network(self.network, self.config)

            if serialized_engine is None:
                print("Failed to build TensorRT engine")
                return False

            # Save the engine
            with open(trt_path, 'wb') as f:
                f.write(serialized_engine)

            print(f"TensorRT engine saved to: {trt_path}")
            return True

        except Exception as e:
            print(f"Error building TensorRT engine: {e}")
            return False

def optimize_all_models():
    """Optimize all models for Isaac Sim"""

    optimizer = TensorRTOptimizer()

    # Define model paths
    models_to_optimize = [
        {
            'onnx_path': '/workspace/shared_dir/models/yolo/yolov5s.onnx',
            'trt_path': '/workspace/shared_dir/models/yolo/yolov5s.trt',
            'name': 'YOLOv5'
        },
        {
            'onnx_path': '/workspace/shared_dir/models/segmentation/fcn_resnet101.onnx',
            'trt_path': '/workspace/shared_dir/models/segmentation/fcn_resnet101.trt',
            'name': 'Segmentation'
        },
        {
            'onnx_path': '/workspace/shared_dir/models/depth/midas.onnx',
            'trt_path': '/workspace/shared_dir/models/depth/midas.trt',
            'name': 'Depth Estimation'
        }
    ]

    # Optimize each model
    for model_info in models_to_optimize:
        print(f"\nOptimizing {model_info['name']} model...")

        success = optimizer.optimize_onnx_model(
            model_info['onnx_path'],
            model_info['trt_path'],
            precision="fp16"  # Use FP16 for better performance on Jetson/RTX
        )

        if success:
            print(f"✓ {model_info['name']} optimized successfully")
        else:
            print(f"✗ Failed to optimize {model_info['name']}")

if __name__ == "__main__":
    optimize_all_models()
EOF
```

## Isaac ROS Model Integration

### 1. Creating Isaac ROS Model Configuration

```bash
cat > ~/isaac_sim_shared/configs/model_config.yaml << 'EOF'
# Isaac ROS Model Configuration

# YOLO Object Detection Configuration
yolo_detection:
  model_path: "/workspace/shared_dir/models/yolo/yolov5s.trt"  # Use TensorRT optimized model
  model_input_width: 640
  model_input_height: 640
  confidence_threshold: 0.5
  max_batch_size: 1
  input_tensor_layout: "NCHW"
  input_binding_name: "images"
  output_binding_names: ["output"]
  input_tensor_names: ["images"]
  output_tensor_names: ["output"]
  enable_tensorrt: true
  tensorrt_precision: "fp16"
  tensorrt_max_workspace_size: 1073741824  # 1GB
  enable_statistics: true

# Semantic Segmentation Configuration
semantic_segmentation:
  model_path: "/workspace/shared_dir/models/segmentation/fcn_resnet101.trt"
  model_input_width: 512
  model_input_height: 512
  input_tensor_layout: "NCHW"
  enable_tensorrt: true
  tensorrt_precision: "fp16"
  tensorrt_max_workspace_size: 1073741824  # 1GB

# Depth Estimation Configuration
depth_estimation:
  model_path: "/workspace/shared_dir/models/depth/midas.trt"
  model_input_width: 384
  model_input_height: 384
  input_tensor_layout: "NCHW"
  enable_tensorrt: true
  tensorrt_precision: "fp16"
  tensorrt_max_workspace_size: 1073741824  # 1GB

# Model Loading Configuration
model_loading:
  enable_model_caching: true
  cache_directory: "/workspace/shared_dir/models/trt_cache"
  enable_async_loading: true
  loading_threads: 2
  enable_memory_pool: true
  memory_pool_size: 536870912  # 512MB

# Performance Monitoring
performance:
  enable_profiling: true
  profile_output_file: "/workspace/shared_dir/logs/model_performance.json"
  enable_framerate_monitoring: true
  target_framerate: 30
  max_latency_ms: 100
EOF
```

### 2. Creating Isaac ROS Model Deployment Package

```bash
cd ~/isaac_ros_ws/src
source /opt/ros/humble/setup.bash

# Create model deployment package
ros2 pkg create --build-type ament_python isaac_ros_model_deployment --dependencies rclpy sensor_msgs vision_msgs cv_bridge geometry_msgs std_msgs
```

### 3. Creating Model Deployment Node

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_model_deployment/isaac_ros_model_deployment/model_deployer.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
from typing import Dict, Any, Optional

class ModelDeployer(Node):
    def __init__(self):
        super().__init__('model_deployer')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publishers for model status
        self.status_publisher = self.create_publisher(
            String,
            '/model_deployment_status',
            10
        )

        # Configuration
        self.config = self.load_config()

        # Model deployment status tracking
        self.models_deployed = {}
        self.deployment_status = "initializing"

        # Initialize model deployment
        self.initialize_models()

        self.get_logger().info('Model Deployer initialized')

    def load_config(self) -> Dict[str, Any]:
        """Load model configuration from YAML file"""
        config_path = "/workspace/shared_dir/configs/model_config.yaml"

        # For this example, we'll use a hardcoded config
        # In practice, you would load from the actual YAML file
        config = {
            "yolo_detection": {
                "model_path": "/workspace/shared_dir/models/yolo/yolov5s.trt",
                "model_input_width": 640,
                "model_input_height": 640,
                "confidence_threshold": 0.5
            },
            "semantic_segmentation": {
                "model_path": "/workspace/shared_dir/models/segmentation/fcn_resnet101.trt",
                "model_input_width": 512,
                "model_input_height": 512
            },
            "depth_estimation": {
                "model_path": "/workspace/shared_dir/models/depth/midas.trt",
                "model_input_width": 384,
                "model_input_height": 384
            }
        }

        return config

    def initialize_models(self):
        """Initialize and deploy models"""
        self.get_logger().info('Initializing models...')

        # Check if model files exist and are accessible
        for model_name, model_config in self.config.items():
            model_path = model_config.get('model_path', '')

            if os.path.exists(model_path):
                self.models_deployed[model_name] = True
                self.get_logger().info(f'Model {model_name} found at {model_path}')
            else:
                self.models_deployed[model_name] = False
                self.get_logger().warn(f'Model {model_name} not found at {model_path}')

        # Check deployment status
        all_deployed = all(self.models_deployed.values())
        self.deployment_status = "ready" if all_deployed else "missing_models"

        # Publish status
        status_msg = String()
        status_msg.data = f"Deployment status: {self.deployment_status}. Models: {self.models_deployed}"
        self.status_publisher.publish(status_msg)

    def image_callback(self, image_msg):
        """Process incoming images and test model deployment"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Log that we're processing an image
            self.get_logger().info(f'Processing image: {cv_image.shape}')

            # Publish status update
            status_msg = String()
            status_msg.data = f"Processing image, models deployed: {self.models_deployed}"
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

    def get_deployment_summary(self) -> str:
        """Get a summary of model deployment status"""
        summary = f"Deployment Status: {self.deployment_status}\n"
        for model_name, deployed in self.models_deployed.items():
            status = "✓ Deployed" if deployed else "✗ Missing"
            summary += f"  {model_name}: {status}\n"
        return summary

    def deploy_model(self, model_name: str, model_path: str) -> bool:
        """Deploy a specific model"""
        try:
            # Check if model file exists
            if not os.path.exists(model_path):
                self.get_logger().error(f'Model file does not exist: {model_path}')
                return False

            # In a real implementation, you would load the model into memory here
            # For this example, we'll just mark it as deployed
            self.models_deployed[model_name] = True
            self.get_logger().info(f'Model {model_name} deployed successfully')

            # Update deployment status
            all_deployed = all(self.models_deployed.values())
            self.deployment_status = "ready" if all_deployed else "partial"

            return True

        except Exception as e:
            self.get_logger().error(f'Error deploying model {model_name}: {e}')
            return False

    def undeploy_model(self, model_name: str) -> bool:
        """Undeploy a specific model"""
        if model_name in self.models_deployed:
            # In a real implementation, you would free the model from memory here
            self.models_deployed[model_name] = False
            self.get_logger().info(f'Model {model_name} undeployed')

            # Update deployment status
            if any(self.models_deployed.values()):
                self.deployment_status = "partial"
            else:
                self.deployment_status = "empty"

            return True
        return False


class ModelDeploymentManager(Node):
    def __init__(self):
        super().__init__('model_deployment_manager')

        # Create model deployer instance
        self.model_deployer = ModelDeployer()

        # Create service servers for model management
        from rclpy.qos import QoSProfile
        from std_srvs.srv import Trigger
        from isaac_ros_model_deployment.srv import DeployModel

        # Service to check deployment status
        self.status_service = self.create_service(
            Trigger,
            'get_deployment_status',
            self.get_status_callback
        )

        # Service to deploy a model
        self.deploy_service = self.create_service(
            DeployModel,
            'deploy_model',
            self.deploy_model_callback
        )

        # Service to undeploy a model
        self.undeploy_service = self.create_service(
            DeployModel,
            'undeploy_model',
            self.undeploy_model_callback
        )

        self.get_logger().info('Model Deployment Manager initialized')

    def get_status_callback(self, request, response):
        """Service callback to get deployment status"""
        summary = self.model_deployer.get_deployment_summary()
        response.success = True
        response.message = summary
        return response

    def deploy_model_callback(self, request, response):
        """Service callback to deploy a model"""
        success = self.model_deployer.deploy_model(request.model_name, request.model_path)
        response.success = success
        response.message = f"Model {request.model_name} deployment: {'Success' if success else 'Failed'}"
        return response

    def undeploy_model_callback(self, request, response):
        """Service callback to undeploy a model"""
        success = self.model_deployer.undeploy_model(request.model_name)
        response.success = success
        response.message = f"Model {request.model_name} undeployment: {'Success' if success else 'Failed'}"
        return response


def main(args=None):
    rclpy.init(args=args)

    # Create and run the model deployment manager
    manager = ModelDeploymentManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### 4. Creating Service Definition for Model Deployment

```bash
mkdir -p ~/isaac_ros_ws/src/isaac_ros_model_deployment/srv
cat > ~/isaac_ros_ws/src/isaac_ros_model_deployment/srv/DeployModel.srv << 'EOF'
string model_name
string model_path
---
bool success
string message
EOF
```

### 5. Creating Setup Files for Model Deployment Package

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_model_deployment/setup.py << 'EOF'
from setuptools import setup
from glob import glob
import os

package_name = 'isaac_ros_model_deployment'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include service definition
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Model deployment tools for Isaac ROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_deployer = isaac_ros_model_deployment.model_deployer:main',
        ],
    },
)
EOF
```

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_model_deployment/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_ros_model_deployment</name>
  <version>0.0.1</version>
  <description>Model deployment tools for Isaac ROS</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF
```

## Model Deployment Testing

### 1. Creating Model Deployment Test Script

```bash
cat > ~/test_model_deployment.sh << 'EOF'
#!/bin/bash

# Test script for AI model deployment in Isaac Sim

echo "Testing AI Model Deployment..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if model deployment package is available
echo "Checking model deployment package..."
ros2 pkg list | grep model_deployment

if [ $? -eq 0 ]; then
    echo "✓ Model deployment package found"
else
    echo "✗ Model deployment package not found"
    exit 1
fi

# Check if model files exist
echo "Checking model files..."
if [ -f "/workspace/shared_dir/models/yolo/yolov5s.onnx" ]; then
    echo "✓ YOLO ONNX model found"
else
    echo "✗ YOLO ONNX model not found"
    # Try to convert the model
    echo "Attempting to convert YOLO model to ONNX..."
    python3 /workspace/shared_dir/scripts/convert_yolo_to_onnx.py
fi

if [ -f "/workspace/shared_dir/models/segmentation/fcn_resnet101.onnx" ]; then
    echo "✓ Segmentation ONNX model found"
else
    echo "✗ Segmentation ONNX model not found"
    # Try to convert the model
    echo "Attempting to convert segmentation model to ONNX..."
    python3 /workspace/shared_dir/scripts/convert_segmentation_to_onnx.py
fi

if [ -f "/workspace/shared_dir/models/depth/midas.onnx" ]; then
    echo "✓ Depth ONNX model found"
else
    echo "✗ Depth ONNX model not found"
    # Try to convert the model
    echo "Attempting to convert depth model to ONNX..."
    python3 /workspace/shared_dir/scripts/convert_depth_to_onnx.py
fi

# Build the model deployment package
echo "Building model deployment package..."
cd ~/isaac_ros_ws
colcon build --packages-select isaac_ros_model_deployment
source install/setup.bash

# Check if TensorRT optimization script exists
if [ -f "/workspace/shared_dir/scripts/optimize_with_tensorrt.py" ]; then
    echo "✓ TensorRT optimization script found"
else
    echo "✗ TensorRT optimization script not found"
    exit 1
fi

# Test TensorRT optimization (this may take a while)
echo "Testing TensorRT optimization (this may take a few minutes)..."
python3 /workspace/shared_dir/scripts/optimize_with_tensorrt.py

echo "AI Model Deployment test completed."
EOF

# Make executable
chmod +x ~/test_model_deployment.sh
```

### 2. Running Model Deployment Test

```bash
~/test_model_deployment.sh
```

## Model Performance Monitoring

### 1. Creating Performance Monitoring Script

```bash
cat > ~/isaac_sim_shared/scripts/model_performance_monitor.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
import time
from collections import deque
import json
import os

class ModelPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('model_performance_monitor')

        # Subscribe to image input (to measure processing pipeline)
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publishers for performance metrics
        self.fps_publisher = self.create_publisher(Float32, '/model_fps', 10)
        self.latency_publisher = self.create_publisher(Float32, '/model_latency', 10)
        self.cpu_usage_publisher = self.create_publisher(Float32, '/model_cpu_usage', 10)
        self.gpu_usage_publisher = self.create_publisher(Float32, '/model_gpu_usage', 10)
        self.memory_usage_publisher = self.create_publisher(Float32, '/model_memory_usage', 10)
        self.status_publisher = self.create_publisher(String, '/model_performance_status', 10)

        # Performance tracking
        self.frame_times = deque(maxlen=30)  # Last 30 frames for FPS calculation
        self.processing_times = deque(maxlen=30)  # Last 30 processing times
        self.last_image_time = None

        # Setup timer for periodic metrics publishing
        self.timer = self.create_timer(1.0, self.publish_metrics)

        # Setup timer for system metrics
        self.system_timer = self.create_timer(2.0, self.publish_system_metrics)

        # Performance log
        self.performance_log = []
        self.log_file = "/workspace/shared_dir/logs/model_performance.json"

        self.get_logger().info('Model Performance Monitor initialized')

    def image_callback(self, msg):
        """Track image processing for performance measurement"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        image_proc_time = current_time - (self.last_image_time or current_time)

        if self.last_image_time is not None:
            frame_time = current_time - self.last_image_time
            self.frame_times.append(frame_time)
            self.processing_times.append(image_proc_time)

        self.last_image_time = current_time

    def publish_metrics(self):
        """Publish performance metrics"""
        # Calculate FPS
        if len(self.frame_times) > 0:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0
            fps_msg = Float32()
            fps_msg.data = fps
            self.fps_publisher.publish(fps_msg)

        # Calculate average processing latency
        if len(self.processing_times) > 0:
            avg_latency = sum(self.processing_times) / len(self.processing_times)
            latency_msg = Float32()
            latency_msg.data = avg_latency
            self.latency_publisher.publish(latency_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f"FPS: {fps:.2f}, Latency: {avg_latency*1000:.2f}ms"
        self.status_publisher.publish(status_msg)

        # Log performance data
        perf_data = {
            'timestamp': time.time(),
            'fps': fps,
            'latency_ms': avg_latency * 1000,
            'frame_count': len(self.frame_times)
        }
        self.performance_log.append(perf_data)

        # Write to log file periodically
        if len(self.performance_log) % 10 == 0:  # Every 10 updates
            self.write_performance_log()

    def publish_system_metrics(self):
        """Publish system resource usage metrics"""
        import psutil

        # CPU usage
        cpu_percent = psutil.cpu_percent()
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_usage_publisher.publish(cpu_msg)

        # Memory usage
        memory = psutil.virtual_memory()
        memory_msg = Float32()
        memory_msg.data = float(memory.percent)
        self.memory_usage_publisher.publish(memory_msg)

        # For GPU usage, we'll use nvidia-ml-py if available
        try:
            import pynvml
            pynvml.nvmlInit()
            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            util = pynvml.nvmlDeviceGetUtilizationRates(handle)
            gpu_util_msg = Float32()
            gpu_util_msg.data = float(util.gpu)
            self.gpu_usage_publisher.publish(gpu_util_msg)
        except:
            # If nvidia-ml-py is not available, publish a placeholder
            gpu_util_msg = Float32()
            gpu_util_msg.data = 0.0
            self.gpu_usage_publisher.publish(gpu_util_msg)

    def write_performance_log(self):
        """Write performance log to file"""
        try:
            with open(self.log_file, 'w') as f:
                json.dump(self.performance_log, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Error writing performance log: {e}')

    def get_performance_summary(self):
        """Get a summary of performance metrics"""
        summary = {
            'total_samples': len(self.performance_log),
            'avg_fps': 0.0,
            'avg_latency_ms': 0.0,
            'min_fps': float('inf'),
            'max_fps': 0.0
        }

        if self.performance_log:
            fps_values = [entry['fps'] for entry in self.performance_log]
            latency_values = [entry['latency_ms'] for entry in self.performance_log]

            summary['avg_fps'] = sum(fps_values) / len(fps_values)
            summary['avg_latency_ms'] = sum(latency_values) / len(latency_values)
            summary['min_fps'] = min(fps_values)
            summary['max_fps'] = max(fps_values)

        return summary


def main(args=None):
    rclpy.init(args=args)
    monitor = ModelPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        summary = monitor.get_performance_summary()
        print(f"Performance Summary: {summary}")
        monitor.write_performance_log()
        print(f"Performance log saved to: {monitor.log_file}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Troubleshooting Model Deployment

### Common Issues and Solutions

#### Issue: "Model conversion fails with ONNX export error"
**Solution**: Check PyTorch and ONNX compatibility
```bash
# Verify PyTorch and ONNX versions
python3 -c "import torch; print(torch.__version__)"
python3 -c "import onnx; print(onnx.__version__)"

# Update if needed
pip3 install --upgrade torch torchvision torchaudio onnx
```

#### Issue: "TensorRT engine build fails"
**Solution**: Check TensorRT installation and GPU compatibility
```bash
# Check TensorRT version
python3 -c "import tensorrt; print(tensorrt.__version__)"

# Check GPU compute capability
nvidia-smi
```

#### Issue: "Models not loading in Isaac Sim"
**Solution**: Verify file paths and permissions
```bash
# Check model file existence
ls -la /workspace/shared_dir/models/yolo/
ls -la /workspace/shared_dir/models/segmentation/
ls -la /workspace/shared_dir/models/depth/

# Check permissions
chmod -R 755 /workspace/shared_dir/models/
```

## Verification Checklist

- [ ] Model conversion scripts created and tested
- [ ] ONNX models generated successfully
- [ ] TensorRT optimization implemented
- [ ] Isaac ROS model deployment package created
- [ ] Model configuration files created
- [ ] Performance monitoring tools implemented
- [ ] Test scripts created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After deploying AI models:

1. **Test model performance** in Isaac Sim environment
2. **Optimize models** based on performance metrics
3. **Integrate with perception pipeline** from previous exercises
4. **Create model deployment exercises** for students

The AI model deployment framework is now configured and ready for Module 3, providing students with tools to deploy and optimize AI models in the Isaac Sim environment.