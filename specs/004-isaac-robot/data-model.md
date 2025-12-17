# Data Model: The AI-Robot Brain (NVIDIA Isaac)

**Feature**: 004-isaac-robot
**Date**: 2025-12-15

## Overview

This data model describes the key entities and relationships for the NVIDIA Isaac educational module. Since this is primarily an educational content feature, the "data" consists of educational components and their relationships.

## Key Entities

### 1. Chapter
- **Name**: String (required)
- **Description**: String (required)
- **Learning Objectives**: Array of strings (3-5 items)
- **Theory Content**: Markdown content
- **Diagrams**: Array of diagram references
- **Runnable Examples**: Array of Example objects
- **Main Project**: Project object
- **Try It Yourself Extensions**: Array of Extension objects
- **Quiz Questions**: Array of Question objects
- **Further Reading**: Array of resource references

### 2. Example
- **Title**: String (required)
- **Description**: String (required)
- **Code**: String (Python/JavaScript)
- **Expected Output**: String or image reference
- **Dependencies**: Array of dependency names
- **Difficulty Level**: Enum (beginner, intermediate)

### 3. Project
- **Title**: String (required)
- **Description**: String (required)
- **Bill of Materials**: Array of material objects
- **Full Code**: String (complete implementation)
- **3D Files**: Array of file references
- **Visual Demonstrations**: Array of GIF/video references
- **Setup Instructions**: String
- **Troubleshooting Guide**: String

### 4. Extension
- **Title**: String (required)
- **Description**: String (required)
- **Difficulty Level**: Enum (beginner, intermediate)
- **Required Knowledge**: Array of prerequisite concepts
- **Implementation Guide**: String

### 5. Question
- **Text**: String (required)
- **Type**: Enum (multiple-choice, short-answer, practical)
- **Options**: Array of option strings (for multiple-choice)
- **Correct Answer**: String
- **Explanation**: String
- **Difficulty Level**: Enum (beginner, intermediate)

## Relationships

```
Chapter 1 -----> * Example
Chapter 1 -----> 1 Project
Chapter 1 -----> * Extension
Chapter 1 -----> * Question
Example 1 -----> * Dependency
Project 1 -----> * Material
Project 1 -----> * 3D File
Project 1 -----> * Visual Demonstration
```

## Validation Rules

### Chapter Validation
- Must have 3-5 learning objectives
- Must follow 7-part structure exactly
- Must include at least 1 runnable example
- Must include a main project
- Must include 3-5 quiz questions

### Example Validation
- Code must be executable and tested
- Must have clear expected output
- Difficulty level must match target audience
- Dependencies must be documented

### Project Validation
- Must include complete BOM
- Code must be fully functional
- Must include visual demonstrations
- Setup instructions must be complete and tested

### Question Validation
- Must have clear, unambiguous answers
- Must test understanding of key concepts
- Difficulty must match chapter level

## State Transitions

### Content Development Lifecycle
```
Draft → Review → Revise → Approved → Published
```

### Chapter Status
- `planning`: Chapter structure defined
- `development`: Content being created
- `testing`: Examples and projects being validated
- `review`: Content under review
- `approved`: Ready for publication
- `published`: Available to learners

## Constraints

- All content must be beginner-friendly with analogies and step-by-step instructions
- All code examples must be tested in Isaac Sim environment
- All projects must be replicable with available documentation
- Content must follow Docusaurus MDX format
- All visual elements must be properly licensed for educational use