---
id: 0007
title: ADR — Search & i18n Strategy
stage: adr
date: 2025-12-01
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: unknown
command: /sp.adr Justify gating local search to production and per-locale dev servers; avoid MDX-only widgets.
labels: [adr, decision, search, i18n]
links:
  spec: D:/Hackathon/hackathon-project/specs/001-ai-robotics-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
- D:/Hackathon/hackathon-project/my-book1/docusaurus.config.ts
- D:/Hackathon/hackathon-project/my-book1/src/css/custom.css
tests:

---

## Prompt

```text
Context
- Windows dev exhibited MODULE_NOT_FOUND with local search theme in dev
- Docusaurus dev serves only one locale; dropdown causes 404 for others
- UI regressions from MDX/video widgets (white-on-white, broken arrows)

Decision
1) Gate local search theme to production builds only
2) Run dev servers per-locale when testing (`--locale hi`, `--locale ur`)
3) Use plain Markdown + tables; avoid MDX-only widgets until later

Consequences
- Dev more stable; prod retains full search and i18n switch
- Simpler content reduces styling bugs and complexity
```

## Response snapshot

Captured decisions to stabilize local development and ensure a smooth production experience. Prioritized content clarity and correctness over interactive widgets.

## Outcome

- ✅ ADR recorded; config updated accordingly
- 🔧 CSS and i18n settings aligned with decisions

## Evaluation notes (flywheel)

- Revisit enabling search in dev after resolving module resolution on Windows
- Consider MDX components later with stricter styling guards

