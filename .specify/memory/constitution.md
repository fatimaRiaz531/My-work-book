# AI-Native Software Development — Spec-Driven Book Creation Constitution

## Core Principles

### I. Accuracy & Reproducibility

Every claim must be traceable to cited, high-quality primary sources (papers, official docs, credible research). At least 50% peer-reviewed academic sources required.

### II. Clarity & Academic Rigor

Content must be clear for a computer science academic audience. Flesch-Kincaid readability grade: 10–12. Tone: scholarly, precise, and neutral.

### III. Originality & Plagiarism

Zero tolerance for plagiarism; all sections must be fully original.

### IV. Spec-Driven Content Generation

Use Spec-Kit Plus for structure, chapter specifications, and reproducible content generation. Gemini (or another configured LLM agent) for chapter drafting, fact verification, and iterative refinement.

## Writing Standards

- **Citation Style:** APA (7th edition) for all references.
- **Word Count:** 5,000–7,000 words target.
- **Sources:** Minimum 15 high-quality sources; 50%+ peer-reviewed journal or conference papers.
- **Validation:** All claims must be validated with in-text citations.
- **Structure:** Include glossary, introduction, body chapters, conclusion, references.

## Technical Requirements

- **Build Structure:** Use Spec-Kit Plus with `/spec/chapters`, `/spec/system`, `/spec/prompts`, `/spec/research`.
- **Content Generation:** Use `spec-kit render` to generate Markdown output.
- **Publishing:** Integrate rendered Markdown into Docusaurus `/docs`.
- **Deployment:** Deploy final book to GitHub Pages using Docusaurus deployment workflow.
- **Final Format:** PDF version generated with embedded APA citations.

## Content Quality Requirements

- **Plagiarism:** Must pass plagiarism screening (0% tolerance).
- **Fact-Checking:** Must pass fact-checking review.
- **Clarity:** All sections written with academic clarity.
- **Visuals:** Figures or diagrams must be referenced and explained.
- **Consistency:** Ensure consistency in terminology, definitions, and tone.

## Success Criteria

- Book fully generated using Spec-Kit Plus specifications.
- All chapters verified with citations and reproducibility standards.
- Build passes Docusaurus compile and GitHub Pages deployment.
- PDF version generated with embedded APA citations.
- Meets all academic, readability, and fact-checking requirements.

## Governance

This constitution supersedes all other project practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
