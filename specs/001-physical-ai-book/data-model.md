# Data Model: Physical AI Book in Docusaurus

## Content Entities

### Book
- **name**: string - Title of the Physical AI book
- **description**: string - Overview of the book's content and purpose
- **chapters**: array of Chapter - Collection of chapters in the book
- **metadata**: object - Creation date, last updated, version information

### Chapter
- **id**: string - Unique identifier for the chapter
- **title**: string - Chapter title
- **description**: string - Brief overview of chapter content
- **lessons**: array of Lesson - Collection of lessons in the chapter
- **prerequisites**: array of string - Previous chapters or concepts required
- **learningObjectives**: array of string - Goals for the chapter

### Lesson
- **id**: string - Unique identifier for the lesson
- **title**: string - Lesson title
- **content**: LessonStructure - The 8-component structure of the lesson
- **difficulty**: enum (beginner, intermediate) - Target audience level
- **estimatedTime**: number - Time to complete in minutes
- **prerequisites**: array of string - Required knowledge or lessons
- **relatedLessons**: array of string - Cross-references to related content

### LessonStructure
- **learningObjectives**: array of string - Clear, measurable goals
- **prerequisites**: array of string - Knowledge needed before starting
- **theoreticalConcepts**: string - Core concepts explained in beginner-friendly language
- **realWorldContext**: string - Practical applications and examples
- **handsOnExercise**: ExerciseContent - Step-by-step practical activity
- **exerciseSolutions**: ExerciseSolutions - Detailed solutions and discussion
- **summary**: string - Key takeaways and review
- **furtherReading**: array of ResourceLink - Additional resources for exploration

### ExerciseContent
- **instructions**: string - Step-by-step guide for the exercise
- **expectedOutcome**: string - What users should achieve
- **toolsRequired**: array of string - Equipment or software needed
- **troubleshootingTips**: array of string - Common issues and solutions

### ExerciseSolutions
- **solutionSteps**: array of string - Detailed solution approach
- **commonMistakes**: array of string - Typical errors and corrections
- **alternativeApproaches**: array of string - Different ways to solve the problem
- **discussionPoints**: array of string - Key insights from the exercise

### ResourceLink
- **title**: string - Name of the resource
- **url**: string - Link to the resource
- **type**: enum (article, video, code, tool, book) - Category of resource
- **description**: string - Brief explanation of the resource's value
- **difficulty**: enum (beginner, intermediate, advanced) - Recommended audience level

### UserProgress
- **userId**: string - Identifier for the user
- **lessonsCompleted**: array of string - IDs of completed lessons
- **chaptersCompleted**: array of string - IDs of completed chapters
- **exerciseScores**: object - Scores for completed exercises
- **lastAccessed**: date - When the user last accessed content
- **learningPath**: array of string - Recommended sequence of lessons