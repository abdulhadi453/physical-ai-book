# Data Model: Module 1 - Foundations of Physical AI

## Overview

This document defines the data structures and entities for Module 1: Foundations of Physical AI. It captures the key concepts, relationships, and validation rules required for the implementation of the educational module.

## Core Entities

### Module
- **id**: string (required) - Unique identifier for the module (e.g., "module-1-foundations")
- **title**: string (required) - "Foundations of Physical AI"
- **description**: string (required) - Overview of the module content and objectives
- **sections**: array of Section (required) - Collection of sections in the module
- **learningObjectives**: array of string (required) - High-level objectives for the module
- **prerequisites**: array of string - Knowledge required before starting the module
- **estimatedDuration**: number (required) - Estimated time to complete in hours
- **difficultyLevel**: enum (beginner, intermediate) (required) - Target audience level
- **createdAt**: date (required) - Creation timestamp
- **updatedAt**: date (required) - Last updated timestamp

### Section
- **id**: string (required) - Unique identifier for the section
- **title**: string (required) - Section title
- **description**: string (required) - Brief overview of section content
- **learningObjectives**: array of string (required) - Specific objectives for the section
- **content**: SectionContent (required) - The 8-component structure of the section
- **exercises**: array of SimulationExercise (required) - Hands-on exercises
- **assessments**: array of Assessment (required) - Formative and summative assessments
- **prerequisites**: array of string - Knowledge needed before this section
- **duration**: number (required) - Estimated time to complete in minutes
- **order**: number (required) - Position in the module sequence

### SectionContent
- **learningObjectives**: array of string (required) - Clear, measurable goals
- **prerequisites**: array of string (required) - Knowledge needed before starting
- **theoreticalConcepts**: string (required) - Core concepts explained in beginner-friendly language
- **realWorldContext**: string (required) - Practical applications and examples
- **handsOnExercise**: ExerciseContent (required) - Step-by-step practical activity
- **exerciseSolutions**: ExerciseSolutions (required) - Detailed solutions and discussion
- **summary**: string (required) - Key takeaways and review
- **furtherReading**: array of ResourceLink (required) - Additional resources for exploration

### SimulationExercise
- **id**: string (required) - Unique identifier for the exercise
- **title**: string (required) - Exercise title
- **description**: string (required) - Brief description of the exercise
- **simulationEnvironment**: string (required) - ROS 2, Gazebo, or Isaac Sim environment
- **instructions**: string (required) - Step-by-step guide for the exercise
- **expectedOutcome**: string (required) - What users should achieve
- **toolsRequired**: array of string (required) - Software/hardware needed
- **troubleshootingTips**: array of string - Common issues and solutions
- **estimatedTime**: number (required) - Time to complete in minutes
- **difficulty**: enum (beginner, intermediate) (required) - Exercise difficulty level
- **successCriteria**: array of string (required) - How to determine completion

### ExerciseContent
- **instructions**: string (required) - Step-by-step guide for the exercise
- **expectedOutcome**: string (required) - What users should achieve
- **toolsRequired**: array of string (required) - Equipment or software needed
- **troubleshootingTips**: array of string - Common issues and solutions
- **setupInstructions**: string - How to prepare for the exercise

### ExerciseSolutions
- **solutionSteps**: array of string (required) - Detailed solution approach
- **commonMistakes**: array of string - Typical errors and corrections
- **alternativeApproaches**: array of string - Different ways to solve the problem
- **discussionPoints**: array of string (required) - Key insights from the exercise

### Assessment
- **id**: string (required) - Unique identifier for the assessment
- **type**: enum (formative, summative) (required) - Assessment category
- **questions**: array of Question (required) - Collection of questions
- **passingScore**: number (required) - Minimum score required to pass (0-100)
- **timeLimit**: number - Time limit in minutes (null if untimed)
- **feedbackEnabled**: boolean (required) - Whether to provide immediate feedback

### Question
- **id**: string (required) - Unique identifier for the question
- **type**: enum (multiple-choice, short-answer, practical) (required) - Question format
- **content**: string (required) - The question text
- **options**: array of string - For multiple-choice questions
- **correctAnswer**: string (required) - The correct answer
- **explanation**: string - Explanation of the correct answer
- **difficulty**: enum (beginner, intermediate) (required) - Question difficulty level
- **tags**: array of string - Subject tags for the question

### ResourceLink
- **title**: string (required) - Name of the resource
- **url**: string (required) - Link to the resource
- **type**: enum (article, video, code, tool, book, documentation) (required) - Category of resource
- **description**: string (required) - Brief explanation of the resource's value
- **difficulty**: enum (beginner, intermediate, advanced) (required) - Recommended audience level
- **relevance**: string - How this resource relates to the section/module

### UserProgress
- **userId**: string (required) - Identifier for the user
- **moduleId**: string (required) - Module being tracked
- **sectionsCompleted**: array of string - IDs of completed sections
- **exercisesCompleted**: array of string - IDs of completed exercises
- **exerciseScores**: object - Scores for completed exercises
- **assessmentScores**: object - Scores for completed assessments
- **lastAccessed**: date (required) - When the user last accessed content
- **currentSection**: string - Current section the user is working on
- **completionPercentage**: number (required) - Overall module completion percentage
- **timeSpent**: number (required) - Total time spent in minutes

### LearningAnalytics
- **moduleId**: string (required) - Module being tracked
- **sectionId**: string (required) - Section being tracked
- **userId**: string (required) - User identifier
- **startTime**: date (required) - When user started the section
- **endTime**: date - When user completed the section
- **timeSpent**: number (required) - Time spent in seconds
- **exerciseAttempts**: array of ExerciseAttempt - Attempts at exercises
- **assessmentResults**: array of AssessmentResult - Results from assessments
- **engagementMetrics**: EngagementMetrics - User engagement data

### ExerciseAttempt
- **exerciseId**: string (required) - Exercise being attempted
- **attemptNumber**: number (required) - Sequential attempt number
- **startTime**: date (required) - When attempt started
- **endTime**: date - When attempt ended
- **success**: boolean (required) - Whether the attempt was successful
- **score**: number - Score achieved (0-100)
- **actions**: array of string - User actions during the attempt

### AssessmentResult
- **assessmentId**: string (required) - Assessment taken
- **attemptNumber**: number (required) - Sequential attempt number
- **startTime**: date (required) - When assessment started
- **endTime**: date - When assessment ended
- **score**: number (required) - Score achieved (0-100)
- **passed**: boolean (required) - Whether the assessment was passed
- **answers**: object - User's answers to questions

### EngagementMetrics
- **pagesViewed**: number (required) - Number of content pages viewed
- **timeOnPage**: object - Time spent on each page
- **exerciseCompletions**: number (required) - Number of exercises completed
- **assessmentCompletions**: number (required) - Number of assessments completed
- **resourceClicks**: number (required) - Number of resource links clicked
- **helpRequests**: number (required) - Number of help requests made

## Relationships

### Module contains Sections
- One-to-many: A module contains multiple sections
- Each section belongs to exactly one module
- Sections have a defined order within the module

### Section contains Exercises and Assessments
- One-to-many: A section contains multiple exercises and assessments
- Each exercise/assessment belongs to exactly one section
- Exercises and assessments are integral to section completion

### UserProgress tracks User activity
- One-to-many: A user can have progress records for multiple modules
- Each progress record belongs to one user and one module
- Progress records enable personalized learning paths

## Validation Rules

### Module Validation
- Title must be between 10-100 characters
- Description must be between 50-500 characters
- Must have at least 3 sections
- Estimated duration must be between 5-50 hours
- Learning objectives must be specific and measurable

### Section Validation
- Title must be between 5-50 characters
- Description must be between 20-200 characters
- Must have exactly 8 content components as defined
- Duration must be between 30-120 minutes
- Must have at least 1 simulation exercise
- Must have at least 1 assessment

### Exercise Validation
- Title must be between 5-50 characters
- Instructions must be between 100-1000 words
- Expected outcome must be specific and measurable
- Estimated time must be between 15-60 minutes
- Must have at least 1 success criterion

### Assessment Validation
- Must have at least 3 questions
- Passing score must be between 60-90%
- Questions must have correct answers provided
- All questions must be relevant to section content

### UserProgress Validation
- Completion percentage must be between 0-100
- Time spent must be positive
- User ID and module ID must exist in the system
- Section IDs must be valid for the module

## State Transitions

### Section States
- **Not Started**: User hasn't accessed the section
- **In Progress**: User has started but not completed the section
- **Completed**: User has finished all required components
- **Assessed**: User has completed assessments for the section

### Exercise States
- **Not Started**: User hasn't started the exercise
- **In Progress**: User is working on the exercise
- **Submitted**: User has submitted the exercise
- **Completed**: Exercise has been successfully completed
- **Failed**: Exercise was not completed successfully

### Assessment States
- **Not Started**: User hasn't started the assessment
- **In Progress**: User is taking the assessment
- **Submitted**: User has submitted the assessment
- **Graded**: Assessment has been graded
- **Passed/Failed**: Final assessment result