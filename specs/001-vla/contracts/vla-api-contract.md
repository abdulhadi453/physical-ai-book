# API Contract: Vision-Language-Action (VLA) System

**Module 4 - VLA System API Specification**
**Date**: 2025-12-17
**Version**: 1.0

## Overview

This document specifies the API contracts for the Vision-Language-Action (VLA) system as defined in Module 4. The API enables communication between the various components of the VLA pipeline: voice input, LLM processing, vision analysis, and ROS 2 action execution.

## Base URL

The VLA system APIs are accessible through ROS 2 services and topics as well as HTTP endpoints for monitoring and control.

## Voice Processing API

### POST /voice/process

Process incoming voice command and convert to structured intent.

**Request**:
```json
{
  "audio_data": "base64_encoded_audio",
  "user_id": "string",
  "language": "en",
  "timestamp": "2025-12-17T10:30:00Z"
}
```

**Response**:
```json
{
  "command_id": "string",
  "transcribed_text": "string",
  "confidence": 0.85,
  "processed_intent_id": "string",
  "status": "success|error",
  "error_message": "string (optional)"
}
```

**Status Codes**:
- 200: Successfully processed
- 400: Invalid request format
- 401: Authentication required
- 500: Processing error

## Intent Processing API

### POST /intent/plan

Generate action sequence from processed intent using LLM.

**Request**:
```json
{
  "intent_text": "Pick up the red cube and place it on the table",
  "context_objects": [
    {
      "id": "object_123",
      "class_name": "cube",
      "color": "red",
      "position_3d": {"x": 1.0, "y": 0.5, "z": 0.0}
    }
  ],
  "environment_context": "string"
}
```

**Response**:
```json
{
  "intent_id": "string",
  "action_sequence": [
    {
      "id": "action_1",
      "action_type": "NAVIGATE_TO",
      "parameters": {"target_position": {"x": 1.0, "y": 0.5, "z": 0.0}},
      "timeout": 10
    },
    {
      "id": "action_2",
      "action_type": "GRASP_OBJECT",
      "parameters": {"object_id": "object_123"},
      "timeout": 5
    }
  ],
  "status": "success|error"
}
```

## Vision Processing API

### POST /vision/analyze

Analyze camera feed and detect objects relevant to voice command.

**Request**:
```json
{
  "image_data": "base64_encoded_image",
  "command_context": "string (the voice command for context)",
  "timestamp": "2025-12-17T10:30:00Z"
}
```

**Response**:
```json
{
  "analysis_id": "string",
  "detected_objects": [
    {
      "id": "obj_123",
      "class_name": "cube",
      "confidence": 0.92,
      "bbox": {"x_min": 100, "y_min": 150, "x_max": 200, "y_max": 250},
      "position_3d": {"x": 1.0, "y": 0.5, "z": 0.0},
      "color": "red",
      "is_graspable": true
    }
  ],
  "status": "success|error"
}
```

## Action Execution API

### POST /action/execute

Execute a sequence of actions via ROS 2.

**Request**:
```json
{
  "action_sequence": [
    {
      "action_type": "NAVIGATE_TO",
      "parameters": {"target_position": {"x": 1.0, "y": 0.5, "z": 0.0}},
      "timeout": 10
    }
  ],
  "execution_id": "string"
}
```

**Response**:
```json
{
  "execution_id": "string",
  "status": "started|completed|failed",
  "current_action": "string",
  "progress": 0.0,
  "estimated_completion": "timestamp"
}
```

### GET /action/status/{execution_id}

Get current status of action execution.

**Response**:
```json
{
  "execution_id": "string",
  "status": "PENDING|IN_PROGRESS|SUCCESS|FAILED|CANCELLED|PAUSED",
  "current_action_id": "string",
  "progress": 0.8,
  "executed_actions": ["action_1", "action_2"],
  "remaining_actions": ["action_3"],
  "error_message": "string (if status is FAILED)"
}
```

## System Status API

### GET /system/status

Get overall system status and component health.

**Response**:
```json
{
  "timestamp": "2025-12-17T10:30:00Z",
  "system_status": "healthy|degraded|error",
  "components": {
    "voice_input": "active|inactive|error",
    "speech_recognition": "ready|processing|error",
    "llm_planning": "connected|disconnected|error",
    "vision_processing": "active|inactive|error",
    "ros2_execution": "connected|disconnected|error"
  },
  "active_sessions": 1,
  "last_command_time": "2025-12-17T10:29:45Z"
}
```

## Error Response Format

All error responses follow this format:

```json
{
  "error": {
    "code": "string",
    "message": "string",
    "details": "string (optional)",
    "timestamp": "2025-12-17T10:30:00Z"
  }
}
```

## Common Error Codes

- `VOICE_INPUT_ERROR`: Problem with voice input processing
- `LLM_CONNECTION_ERROR`: Unable to connect to LLM service
- `VISION_PROCESSING_ERROR`: Error in vision analysis
- `ACTION_EXECUTION_ERROR`: Problem executing ROS 2 action
- `INVALID_COMMAND`: Command could not be understood or processed
- `OBJECT_NOT_FOUND`: Referenced object not found in environment
- `NAVIGATION_FAILED`: Robot unable to navigate to target location
- `GRASP_FAILED`: Robot unable to grasp the target object

## Authentication

All API endpoints require authentication using the system's authentication token, passed in the `Authorization` header:

```
Authorization: Bearer <token>
```

## Rate Limits

To prevent system overload, API endpoints have the following rate limits:

- Voice processing: 10 requests per minute per user
- Intent planning: 5 requests per minute per user
- Vision analysis: 20 requests per minute per user
- Action execution: 5 requests per minute per user