# Quickstart Guide: Docusaurus Documentation for ROS 2 Module

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git (for version control)

## Installation

1. **Install Node.js**
   - Download from [nodejs.org](https://nodejs.org/)
   - Verify installation: `node --version`

2. **Create Docusaurus Project**
   ```bash
   npx create-docusaurus@latest website-name classic
   cd website-name
   ```

3. **Install Dependencies**
   ```bash
   npm install
   ```

## Project Structure Setup

1. **Create Module Directory**
   ```bash
   mkdir docs/module-1
   ```

2. **Create Chapter Files**
   ```bash
   touch docs/module-1/index.md
   touch docs/module-1/chapter-1-ros2-basics.md
   touch docs/module-1/chapter-2-rclpy-ai-agents.md
   touch docs/module-1/chapter-3-urdf-modeling.md
   ```

## Configuration

1. **Update `docusaurus.config.js`**
   - Set site title and metadata
   - Configure the docs plugin to recognize your module structure

2. **Update Sidebar (`sidebars.js`)**
   ```javascript
   module.exports = {
     docs: [
       {
         type: 'category',
         label: 'Module 1: The Robotic Nervous System',
         items: [
           'module-1/index',
           'module-1/chapter-1-ros2-basics',
           'module-1/chapter-2-rclpy-ai-agents',
           'module-1/chapter-3-urdf-modeling',
         ],
       },
     ],
   };
   ```

## Content Creation

1. **Add Content to Chapter Files**
   - Edit each `.md` file with appropriate content
   - Follow markdown syntax for headings, code blocks, and formatting
   - Include examples and diagrams as needed

2. **Module Overview (`docs/module-1/index.md`)**
   ```markdown
   # Module 1: The Robotic Nervous System (ROS 2)

   Welcome to Module 1, where you'll learn about ROS 2 as the middleware connecting AI agents to humanoid robot control.
   ```

## Development Server

1. **Start Local Server**
   ```bash
   npm run start
   ```

2. **Preview Changes**
   - Open `http://localhost:3000` in your browser
   - Changes to markdown files will automatically reload

## Build and Deployment

1. **Build Static Site**
   ```bash
   npm run build
   ```

2. **Test Build Locally**
   ```bash
   npm run serve
   ```

3. **Deploy to GitHub Pages**
   - Configure in `docusaurus.config.js`
   - Push to GitHub repository
   - Set up GitHub Actions for automated deployment

## Common Commands

- `npm run start` - Start development server
- `npm run build` - Build static site
- `npm run serve` - Serve built site locally
- `npm run deploy` - Deploy to GitHub Pages (if configured)

## Troubleshooting

- **Build fails**: Check for syntax errors in markdown files
- **Navigation missing**: Verify sidebar configuration
- **Images not showing**: Ensure images are in `static/img/` directory