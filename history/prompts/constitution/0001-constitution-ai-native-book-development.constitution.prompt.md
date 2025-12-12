---
id: 0001
title: Constitution for AI-Native Book Development
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["project-setup", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
null
---

## Prompt

/sp.constitution

Project: AI-Native Software Development — Spec-Driven Book Creation

Objective:
Create a fully AI/spec-driven technical book on AI-native software development, written using Spec-Kit Plus and Claude Code, published using Docusaurus, and deployed to GitHub Pages.

Core Principles:
- Accuracy through primary source verification (papers, official docs, credible research)
- Clarity for a computer science academic audience
- Reproducibility: every claim traceable to cited, high-quality sources
- Rigor: at least 50% peer-reviewed academic sources
- Zero tolerance for plagiarism; all sections must be fully original
- Use Spec-Kit Plus for structure, chapter specs, and reproducible content generation
- Use Claude Code for chapter drafting, fact verification, and iterative refinement

Writing Standards:
- APA citation style for all references
- Flesch-Kincaid readability grade: 10–12
- Word count target: 5,000–7,000 words
- Minimum 15 sources
- 50%+ peer-reviewed journal or conference papers
- All claims must be validated with citations
- Include glossary, introduction, body chapters, conclusion, references

Technical Requirements:
- Build structure using Spec-Kit Plus:
  - /spec/chapters
  - /spec/system
  - /spec/prompts
  - /spec/research
- Use `spec-kit render` to generate Markdown output
- Integrate rendered Markdown into Docusaurus `/docs`
- Deploy final book to GitHub Pages using Docusaurus deployment workflow

Content Quality Requirements:
- Must pass plagiarism screening (0% tolerance)
- Must pass fact-checking review
- All sections written with academic clarity
- Figures or diagrams must be referenced and explained
- Ensure consistency in terminology, definitions, and tone

Success Criteria:
- Book fully generated using Spec-Kit Plus specifications
- All chapters verified with citations and reproducibility standards
- Build passes Docusaurus compile and GitHub Pages deployment
- PDF version generated with embedded APA citations
- Meets all academic, readability, and fact-checking requirements
/sp.constitution

Project: Research Paper on AI-Native Software Development

Mission:
Create a rigorously researched, academically credible, AI/spec-driven book using Docusaurus and deploy it to GitHub Pages. Content will be generated and refined through Spec-Kit Plus and Claude Code, ensuring academic accuracy, reproducibility, and transparent sourcing.

Core Principles:
- Accuracy through primary source verification
- Clarity for an academic audience with a computer science background
- Reproducibility: all claims must be traceable to verifiable sources
- Rigor: prioritize peer-reviewed literature, reputable journals, and official documentation
- Consistency with AI/spec-driven writing workflows

Key Writing Standards:
- All factual claims must cite a valid source
- Minimum 50% peer-reviewed academic sources
- Citation format: APA (7th edition)
- Writing clarity: Flesch-Kincaid grade 10–12
- Plagiarism tolerance: 0%
- Tone: scholarly, precise, and neutral
- Cross-reference concepts between chapters for cohesion

Requirements:
- Word count: 5,000–7,000 words
- Minimum 15 high-quality sources
- All citations embedded in-text, plus full reference list
- Final format: PDF with embedded citations (exported from Docusaurus or Markdown)
- Must include both conceptual explanations and technical examples relevant to AI-native software development

Process Requirements:
- Use Spec-Kit Plus to:
  - Define chapter specs
  - Generate chapter drafts
  - Validate structure and completeness
  - Render markdown for Docusaurus
- Use Claude Code for:
  - Chapter generation
  - Source evaluation
  - Rewriting for clarity and academic tone
  - Fact-checking against primary sources

AI/Spec-Driven Workflow:
1. Use Spec-Kit Plus to define system, chapter, and prompt specs.
2. Generate draft content through Claude Code using the spec files.
3. Validate: ensure all claims have citations.
4. Render markdown using `spec-kit render`.
5. Import into Docusaurus `/docs`.
6. Deploy final book to GitHub Pages.

Constraints:
- Claims without sources must not be included.
- Do not use unverifiable blogs or AI hallucinated citations.
- Avoid overly technical jargon unless necessary; provide definitions.
- Maintain logical structure between chapters.

Success Criteria:
- Zero factual inaccuracies after verification
- Zero plagiarism detection
- All sources valid and traceable
- Meets academic readability standards (grade 10–12)
- Achieves 5,000–7,000 word requirement
- PDF successfully generated and deployed via GitHub Pages
- Passes a final editorial and fact-checking review

## Response snapshot

I've processed the project constitution. I will now create a Prompt History Record (PHR) for this interaction.

## Outcome

- ✅ Impact: Project constitution processed and recorded.
- 🧪 Tests: N/A
- 📁 Files: .specify/memory/constitution.md, history/prompts/constitution/0001-constitution-ai-native-book-development.constitution.prompt.md
- 🔁 Next prompts: Continue with project setup based on the constitution.
- 🧠 Reflection: Successfully recorded the project constitution.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
