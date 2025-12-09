---
id: specs-005-vla-module-data-model
---
# Data Model: Module 4 Content Structure

This document defines the data model for the content of Module 4, which is represented as a structured hierarchy of Markdown files.

## 1. Top-Level Structure

- **Textbook**
  - ...
  - **Module 4**: Vision-Language-Action (VLA)
  - ...

## 2. Module Structure

- **Module Directory (`/docs/module4/`)**
  - `_category_.json`
  - `ch01-voice-to-action.md`
  - `ch02-llm-cognitive-planning.md`
  - `ch03-vision-and-navigation.md`
  - `ch04-capstone-autonomous-humanoid.md`

## 3. Chapter Structure (`.md` file)

- **Front Matter**: YAML metadata.
- **Chapter Content**:
  - Learning Objectives
  - Introduction
  - Main Content Sections
  - Code Examples
  - Conclusion
  - References

## 4. Lab & Code Structure (`/labs/module4/`)

- **`whisper_examples/`**: ROS 2 package for Whisper integration.
- **`llm_planning_examples/`**: ROS 2 package for LLM-based cognitive planning.
- **`vision_navigation_examples/`**: ROS 2 package for vision and navigation.
- **`capstone_project/`**: ROS 2 package for the capstone project.

## 5. Entity Relationship Diagram (Conceptual)

```mermaid
graph TD;
    A[Module 4] --> B1[Chapter 1: Voice-to-Action];
    A --> B2[Chapter 2: LLM Planning];
    A --> B3[Chapter 3: Vision & Nav];
    A --> B4[Chapter 4: Capstone];

    B1 --> C1[Lab: Whisper Examples];
    B2 --> C2[Lab: LLM Planning Examples];
    B3 --> C3[Lab: Vision & Nav Examples];
    B4 --> C4[Lab: Capstone Project];
```
