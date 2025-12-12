# Specification-Driven Physical AI & Humanoid Robotics Textbook

This project demonstrates **AI-native book creation** using **Spec-Kit Plus**, **Claude Code**, and **Docusaurus** for the Physical AI & Humanoid Robotics course.

## Project Structure

```
.specify/
├── specs/
│   ├── system-physical-ai.spec.yml      # Master specification
│   ├── modules/
│   │   ├── module-1-ros2.spec.yml
│   │   ├── module-2-gazebo-unity.spec.yml
│   │   ├── module-3-isaac.spec.yml
│   │   └── module-4-vla.spec.yml
│   └── prompts/
│       ├── module-1-ros2-generation.prompt.md
│       ├── module-2-gazebo-unity-generation.prompt.md
│       ├── module-3-isaac-generation.prompt.md
│       └── module-4-vla-generation.prompt.md
├── memory/
│   └── constitution.md                  # Project governance
└── README.md

book/
├── docs/
│   ├── module-1-ros2.md                 # Module content (to be generated)
│   ├── module-2-gazebo-unity.md
│   ├── module-3-isaac.md
│   ├── module-4-vla.md
│   ├── references.md                    # 60+ peer-reviewed sources
│   ├── appendix-hardware.md
│   ├── appendix-ros2-commands.md
│   ├── chatbot-guide.md
│   └── capstone-project.md
├── docusaurus.config.ts                 # Site configuration
├── sidebars.ts                          # Navigation structure
└── package.json

.github/
└── workflows/
    └── deploy.yml                       # GitHub Pages CI/CD

.env                                     # API keys (Claude, OpenAI, etc.)
.gitignore
HACKATHON_STATUS.md                      # Progress tracker
```

## Quick Start

### 1. Install Dependencies

```bash
cd book
npm install
```

### 2. Build the Site

```bash
npm run build
```

### 3. Serve Locally

```bash
npm run serve
```

Browse to `http://localhost:3000`

---

## Specification-Driven Workflow

### Step 1: Define Module Specification

Each module has a detailed spec in `.specify/specs/modules/`:

```yaml
id: module-1-ros2
title: 'Module 1: The Robotic Nervous System (ROS 2)'
word_count: 4000
learning_outcomes: [...]
sections:
  - section_id: ros2-intro
    title: 'Introduction to ROS 2'
    word_count: 600
    citations_needed: 3
```

### Step 2: Create Generation Prompt

Guide AI generation with detailed prompts in `.specify/specs/prompts/`:

```markdown
# Prompt Template: Module 1 - ROS 2

## Task

Generate a 4,000-word chapter on ROS 2 fundamentals

## Success Criteria

- [ ] Word count: 4,000 ± 200
- [ ] Citations: 16 [CITATION_N] placeholders
- [ ] Code examples: 5 complete, runnable examples
- [ ] No plagiarism
- [ ] APA 7th edition format

[Detailed instructions...]
```

### Step 3: Generate Module Content

Using Claude Code with Spec-Kit Plus:

```bash
claude generate \
  --spec .specify/specs/modules/module-1-ros2.spec.yml \
  --prompt .specify/specs/prompts/module-1-ros2-generation.prompt.md \
  --output book/docs/module-1-ros2.md
```

### Step 4: Verify Quality

- [ ] Word count matches spec
- [ ] All citations have placeholders [CITATION_N]
- [ ] Code examples are syntactically correct
- [ ] Plagiarism check passed
- [ ] Build with `npm run build` succeeds

### Step 5: Deploy

```bash
npm run deploy
```

---

## Specifications Overview

### System Specification (`system-physical-ai.spec.yml`)

**Defines:** Total word count (15,000–25,000), module structure, success criteria, features (RAG chatbot, auth, personalization, translation)

**Key Requirements:**

- 4 modules × 3,500–5,500 words each
- 60+ citations (40 minimum, 60%+ peer-reviewed)
- 20+ runnable code examples
- 0% plagiarism
- Integrated RAG chatbot with FastAPI + Qdrant
- Better-Auth authentication
- Content personalization (experience level toggles)
- Urdu translation support

### Module Specifications

Each module in `.specify/specs/modules/` defines:

- Learning outcomes
- Section structure and word count targets
- Code example requirements
- Citation counts per section
- Technical prerequisites

**Example:**

```yaml
sections:
  - section_id: ros2-concepts
    title: 'Core Concepts'
    word_count: 800
    code_example_count: 2
    citations_needed: 4
```

### Prompt Templates

Each prompt in `.specify/specs/prompts/` includes:

- Target specifications (word count, citations, code examples)
- Section-by-section breakdown
- Mandatory requirements (APA format, no plagiarism)
- Citation guidelines
- Tone and style expectations
- Example code structure

---

## Bibliography

All 60+ sources are in `book/docs/references.md` including:

- **Peer-reviewed journals:** IJRR, RA-L, T-RO, IEEE publications
- **Conference proceedings:** ICRA, IROS, CoRL, RSS, HRI
- **Foundational textbooks:** Siciliano, Lynch & Park, Spong
- **Official documentation:** ROS 2, Gazebo, NVIDIA Isaac, Jetson
- **Online resources:** Tutorials, courses, standards

---

## Features (Hackathon Requirements)

### ✅ Base: AI-Native Spec-Driven Book

- Docusaurus site with 4 modules (15,000–25,000 words)
- 60+ peer-reviewed references
- All code examples runnable
- GitHub Pages deployment

### 🤖 RAG Chatbot (+50 bonus points)

**Technology:** FastAPI + Qdrant + OpenAI Embeddings

```bash
# Backend (separate deployment)
cd chatbot
pip install -r requirements.txt
uvicorn main:app --reload

# Features
- Answer questions from book content
- Context-aware responses with citations
- User-selected text queries
- Streaming responses
```

**Frontend Integration:** React component in Docusaurus sidebar

### 🔐 Better-Auth (+50 bonus points)

**Technology:** Better-Auth with signup/signin

```typescript
// Signup flow collects:
- Background (Software Engineer / Roboticist / Student)
- ROS 2 experience (Yes / No / Beginner)
- Hardware experience (Jetson / Arduino / None)
- Learning style (Theory / Hands-on / Balanced)
```

### 🎯 Content Personalization (+50 bonus points)

**Implementation:** Toggle button at chapter start

```
[Beginner] [Intermediate] [Advanced]
```

Each level offers:

- Different code complexity
- Example hardware selection
- Adjustable depth of explanation
- Difficulty level indicators

### 🇵🇰 Urdu Translation (+50 bonus points)

**Implementation:** Toggle button at chapter start

```
[English] [Urdu]
```

- Dynamic translation or pre-translated content
- Preserves code blocks and technical terms
- Links to English original for verification

---

## Deployment Checklist

- [ ] All 4 modules generated and verified
- [ ] Bibliography complete (60+ sources)
- [ ] Docusaurus build succeeds (`npm run build`)
- [ ] All links clickable and functional
- [ ] RAG chatbot deployed and responding
- [ ] Better-Auth signup/signin working
- [ ] Personalization toggles functional
- [ ] Urdu translation available
- [ ] GitHub Pages live at custom domain
- [ ] Demo video recorded (<90 seconds)

---

## Submission for Hackathon

**Deadline:** Sunday, November 30, 2025, 6:00 PM

**Submit:**

1. **Public GitHub Repo** - Link to your fork/repo
2. **Published Book Link** - GitHub Pages or Vercel URL
3. **Demo Video** - NotebookLM or screen recording (<90 seconds)
4. **WhatsApp Number** - For live presentation invitation

**Submit Form:** https://forms.gle/CQsSEGM3GeCrL43c8

---

## Live Presentations

**Date & Time:** Sunday, November 30, 2025, 6:00 PM IST  
**Platform:** Zoom

**Meeting Details:**

- Meeting ID: 849 7684 7088
- Passcode: 305850
- Join URL: https://us06web.zoom.us/j/84976847088?pwd=Z7t7NaeXwVmmR5fysCv7NiMbfbhIda.1

Top submissions will be invited to present live. Presentations do not affect scoring but showcase your work to the Panaversity team.

---

## Scoring Rubric

| Category                    | Points  | Criteria                                       |
| --------------------------- | ------- | ---------------------------------------------- |
| **Base Functionality**      | 100     | Book + RAG chatbot                             |
| **Claude Code Subagents**   | 50      | Reusable intelligence in project               |
| **Better-Auth**             | 50      | Signup/signin + personalization                |
| **Content Personalization** | 50      | Level toggles (Beginner/Intermediate/Advanced) |
| **Urdu Translation**        | 50      | Full translation support                       |
| **Total Possible**          | **400** |                                                |

---

## Resources

### Official Documentation

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/)
- [Jetson Documentation](https://docs.nvidia.com/jetson/)

### Learning Resources

- [Modern Robotics Specialization](https://www.coursera.org/specializations/modernrobotics)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Gazebo Tutorials](https://gazebosim.org/docs/latest/tutorials)

### Hackathon Support

- **Panaversity:** https://panaversity.org
- **AI-Native Book Example:** https://ai-native.panaversity.org
- **Spec-Kit Plus:** https://github.com/panaversity/spec-kit-plus

---

## Contact & Community

- **Discord:** [Panaversity Community]
- **GitHub Issues:** Report bugs or suggest features
- **Email:** hackathon@panaversity.org

---

**Last Updated:** December 7, 2025  
**Status:** 🚀 Ready for content generation and hackathon submission
