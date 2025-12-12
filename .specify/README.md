Spec-Kit Plus configuration for the AI-Native Software Development research book.

Structure:

- `specs/system.spec.yml` — System-level spec that orchestrates rendering, validation, and deployment.
- `specs/chapters/*.spec.yml` — Chapter-level specs used by Gemini (or configured agent) to generate content.
- `specs/prompts/*.prompt.md` — Prompt templates that guide content generation for each chapter.

Usage (local development):

1. Install Spec-Kit Plus and a supported LLM agent such as Gemini (per your environment).
2. Author or edit chapter specs in `specs/chapters/`.
3. Use Gemini (or your configured agent) to generate drafts guided by `specs/prompts/`:

```pwsh
# Example: generate the intro draft (uses local gemini binary or configured agent)
gemini generate --spec .specify/specs/chapters/intro.spec.yml --prompt .specify/specs/prompts/intro-generation.prompt.md --output book/docs/intro.md
```

4. Render Markdown via Spec-Kit Plus

```pwsh
spec-kit render .specify/specs/system.spec.yml --out book/docs
```

5. Build and preview Docusaurus

```pwsh
cd book
npm run start
```

Notes:

- Replace `yourusername` in `system.spec.yml` with your GitHub username before running deploy steps.
  -- All generated drafts require agent-based source verification and manual review to ensure APA citations and 0% plagiarism.
