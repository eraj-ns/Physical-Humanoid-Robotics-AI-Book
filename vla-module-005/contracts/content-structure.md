---
id: specs-005-vla-module-contracts-content-structure
---
# Content Structure Contract: Module 4

This document defines the contract for the structure and content of each chapter in Module 4.

## 1. File Naming and Location

- All chapter files MUST be located in the `/docs/module4/` directory.
- File names MUST be in the format `chXX-<chapter-title>.md`.

## 2. Chapter Front Matter

Each chapter file MUST begin with a YAML front matter block with the following fields:

- `id`
- `title`
- `description`
- `tags`

## 3. Chapter Sections

Each chapter MUST contain the following sections in the specified order:

1.  **Learning Objectives**
2.  **Introduction**
3.  **Main Content**
4.  **Code Examples**
5.  **Conclusion**
6.  **References**

## 4. Citation and Formatting

- **Citations**: In-text citations MUST use the `(Author, Year)` format.
- **Formatting**: All content MUST adhere to standard Markdown syntax.
- **APA Style**: The reference list MUST be formatted according to the APA 7th edition style guide.

## 5. Lab & Code Examples

- All code examples MUST be runnable and tested.
- Code examples MUST be located in the `/labs/module4/` directory, organized by chapter.
- Each code example MUST be accompanied by a clear explanation of its purpose and how to run it.
