# Quickstart Guide: Physical AI Book in Docusaurus

## Prerequisites

- Node.js 18 or higher
- npm or yarn package manager
- Git for version control
- A code editor (VS Code recommended)

## Setup Instructions

### 1. Clone and Initialize the Repository

```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Navigate to the website directory
cd website
```

### 2. Install Dependencies

```bash
# Install Docusaurus and other dependencies
npm install
# or
yarn install
```

### 3. Start Development Server

```bash
# Start the development server
npm run start
# or
yarn start
```

This will start a local development server at `http://localhost:3000` with hot reloading.

### 4. Project Structure Overview

```
website/
├── blog/                 # Optional blog posts
├── docs/                 # Main book content
│   ├── intro.md          # Introduction page
│   ├── chapter-1/        # Chapter directory
│   │   ├── lesson-1.md   # Lesson following 8-component structure
│   │   ├── lesson-2.md   # Lesson following 8-component structure
│   │   └── lesson-3.md   # Lesson following 8-component structure
│   └── ...
├── src/
│   ├── components/       # Custom React components
│   ├── pages/            # Additional static pages
│   └── css/              # Custom styles
├── static/               # Static assets (images, files)
├── docusaurus.config.js  # Main configuration
├── sidebars.js           # Navigation structure
└── package.json          # Dependencies and scripts
```

## Creating New Content

### Adding a New Lesson

1. Create a new MDX file in the appropriate chapter directory:
   ```bash
   # Example: Adding a lesson to chapter-1
   touch docs/chapter-1/my-new-lesson.md
   ```

2. Follow the 8-component lesson structure:

   ```md
   ---
   id: my-new-lesson
   title: Lesson Title
   sidebar_label: Lesson Title
   sidebar_position: 4  # Position in the sidebar
   description: Brief description of the lesson
   ---

   ## Learning Objectives
   - Objective 1
   - Objective 2

   ## Prerequisites
   - What learners should know before starting

   ## Theoretical Concepts
   Explain the core concepts in beginner-friendly language.

   ## Real-World Context
   Show how these concepts apply in practice.

   ## Hands-On Exercise
   Provide step-by-step instructions for practical application.

   ## Exercise Solutions and Discussion
   Detailed solutions and discussion of the exercise.

   ## Summary and Key Takeaways
   Concise review of important concepts.

   ## Further Reading and Resources
   Links to additional resources for deeper exploration.
   ```

### Adding Custom Components

Create new components in `src/components/`:

```jsx
// Example: src/components/ExerciseBox.js
import React from 'react';
import styles from './ExerciseBox.module.css';

export default function ExerciseBox({ children, title = "Exercise" }) {
  return (
    <div className={styles.exerciseBox}>
      <h3>{title}</h3>
      <div className={styles.exerciseContent}>
        {children}
      </div>
    </div>
  );
}
```

## Configuration

### docusaurus.config.js

Key configuration options for the Physical AI book:

- `title`: Site title
- `tagline`: Brief description
- `url`: URL for your site
- `baseUrl`: Base URL for your project
- `organizationName`: GitHub organization/username
- `projectName`: GitHub repository name
- `themes`: Docusaurus themes
- `plugins`: Additional functionality
- `themeConfig`: Theme-specific configuration

### sidebars.js

Define the navigation structure:

```js
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Chapter 1',
      items: ['chapter-1/lesson-1', 'chapter-1/lesson-2', 'chapter-1/lesson-3'],
    },
    // Add more chapters as needed
  ],
};
```

## Building and Deployment

### Build for Production

```bash
# Build the static files
npm run build
# or
yarn build
```

The built files will be in the `build/` directory.

### Deployment

The site can be deployed to various platforms:

- **GitHub Pages**: Use `npm run deploy` if configured
- **Vercel**: Connect your GitHub repository
- **Netlify**: Drag and drop the build folder or connect GitHub
- **Self-hosting**: Serve the contents of the `build/` directory

## Development Tips

1. **Hot Reloading**: Changes to MDX files and components are reflected immediately
2. **Markdown Extensions**: Use Docusaurus' enhanced Markdown features
3. **Frontmatter**: Use YAML frontmatter for page metadata
4. **Versioning**: Plan for content versioning as the book evolves
5. **Accessibility**: Ensure all content meets accessibility standards